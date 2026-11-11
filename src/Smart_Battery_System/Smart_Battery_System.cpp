// Smart_Battery_System.cpp
// SbsBattery - the core: construction and watchdog-hook plumbing, wake-up notification, the cache snapshot and the cache getters,
// state-of-charge estimation (OCV lookup, coulomb-counter seeding / resync, Kalman predict + update in updateStateAndPublish()),
// processFSM() (EMA classification, full-charge log, CUTOFF / re-enable) and the state / statistics accessors.
// The acquisition side (init, fetch*WithRetry, pollHardwareAndUpdateCache, publishError) is in Smart_Battery_Acquisition.cpp.

#include "Smart_Battery_System.h"
#include "Persistent_Configuration_System.h"
#include "Power_Management_System.h" 
#include "Retry_Policy.h"
#include "Fsm_classifier.h"
#include <zephyr/logging/log.h>
#include <zephyr/device.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/kernel.h>
#include <array>
#include <algorithm>

LOG_MODULE_REGISTER(BATTERY_SYS, LOG_LEVEL_INF);

extern "C" void daly_watchdog_feed_hook(void) __attribute__((weak));

#ifndef IS_TEST_ENVIRONMENT
extern "C" void daly_watchdog_feed_hook(void) {
    return;
}
#endif

namespace {
// Same helper as in Smart_Battery_Acquisition.cpp; kept file-local so no shared internal header is needed.
    template <typename T>
    constexpr T absolute(T val) { return (val < 0) ? -val : val; }
}

SbsBattery::SbsBattery(I2CManager* i2c_bus, DeviceContext* context, WatchdogFeedHook hook)
    : ina226(i2c_bus), sys_context(context), current_state(BatteryFSM::IDLE),
      full_charge_logged(false), watchdog_feed_hook((hook != nullptr) ? hook : daly_watchdog_feed_hook),
      cache_mutex{}, last_valid_comm_time(k_uptime_get_32()), consecutive_comm_failures(0U),
      consecutive_mutex_failures(0U),
      cache{}, stats{}, soc_initialized(false), accumulated_uAh(0), last_poll_time_ms(0U),
      consecutive_jump_rejects(0U), rest_period_start_ms(0U),
      kf_soc_pct(0.0f), kf_p_covariance(1.0f) {}

void SbsBattery::setWatchdogFeedHook(WatchdogFeedHook hook) {
    watchdog_feed_hook.store((hook != nullptr) ? hook : daly_watchdog_feed_hook);
}

void SbsBattery::feedWatchdog() const {
    WatchdogFeedHook hook = watchdog_feed_hook.load();
    if (hook) hook();
}

void SbsBattery::notifySystemWakeup() {
    if (k_mutex_lock(&cache_mutex, K_NO_WAIT) == 0) {
        const uint32_t now = k_uptime_get_32();
        last_valid_comm_time = now;
        last_poll_time_ms = now;
        if (cache.valid) cache.timestamp_ms = now;
        k_mutex_unlock(&cache_mutex);
    } else {
        LOG_WRN("Could not lock cache_mutex during system wakeup.");
    }
}

uint8_t SbsBattery::estimateSocFromVoltage(uint16_t pack_mv) const {
    return CurveFitting::interpolateOcv(CurveFitting::OCV_LUT, pack_mv);
}

void SbsBattery::seedOrResyncCoulombCounter(uint16_t pack_mv, int32_t shunt_uv, bool force_seed) {
    const bool at_rest = (shunt_uv > -BatteryLimits::IDLE_SHUNT_UV_THRESHOLD) &&
                         (shunt_uv < BatteryLimits::IDLE_SHUNT_UV_THRESHOLD);

    const uint32_t now = k_uptime_get_32();

    bool long_rest_resync = false;
    if (at_rest) {
        if (rest_period_start_ms == 0U) {
            rest_period_start_ms = now;
        } else if ((now - rest_period_start_ms) >= BatteryLimits::REST_RESYNC_DURATION_MS) {
            long_rest_resync = true;
            rest_period_start_ms = now;
        }
    } else {
        rest_period_start_ms = 0U;
    }

    const bool full_charge = (pack_mv >= (BatteryLimits::PACK_MAX_VOLTAGE_MV - 100)) && at_rest;
    const bool full_discharge = (pack_mv <= (BatteryLimits::PACK_MIN_VOLTAGE_MV + 100)) && at_rest;

    if (force_seed || full_charge || full_discharge || long_rest_resync) {
        const uint8_t ocv_soc_pct = estimateSocFromVoltage(pack_mv);
        accumulated_uAh = (static_cast<int64_t>(ocv_soc_pct) * BatteryLimits::NOMINAL_CAPACITY_MAH * 1000LL) / 100LL;
        
        kf_soc_pct = static_cast<float>(ocv_soc_pct);
        kf_p_covariance = 1.0f; 
        
        soc_initialized = true;
    }
}

void SbsBattery::updateStateAndPublish(uint16_t pack_mv, int32_t current_ma, int16_t temp_tenths, int32_t shunt_uv) {
    if (k_mutex_lock(&cache_mutex, K_MSEC(BatteryLimits::MUTEX_TIMEOUT_MS)) != 0) {
        atomic_inc(&stats.validation_errors);
        publishError(CommFault::MUTEX_TIMEOUT);
        return;
    }

    const uint32_t now = k_uptime_get_32();
    const int64_t max_uAh = static_cast<int64_t>(BatteryLimits::NOMINAL_CAPACITY_MAH) * 1000LL;

    if (!soc_initialized) {
        seedOrResyncCoulombCounter(pack_mv, shunt_uv, true);
        last_poll_time_ms = now;
    } else {
        const uint32_t delta_ms = now - last_poll_time_ms;
        const float dt_hours = static_cast<float>(delta_ms) / 3600000.0f;
        const float capacity_mah = static_cast<float>(BatteryLimits::NOMINAL_CAPACITY_MAH);

        // --- 1. KALMAN PREDICT (Coulomb Counting) ---
        KalmanFilter::predict(kf_soc_pct, kf_p_covariance,
                              static_cast<float>(current_ma), dt_hours, capacity_mah,
                              BatteryLimits::KF_PROCESS_NOISE);

        // --- 2. MEASUREMENT (OCV Compensation) ---
        int32_t ir_drop_mv = (current_ma * BatteryLimits::ESTIMATED_PACK_IR_MILLIOHMS) / 1000;
        int32_t estimated_ocv_mv = static_cast<int32_t>(pack_mv) - ir_drop_mv;
        estimated_ocv_mv = std::clamp(estimated_ocv_mv, 0, static_cast<int32_t>(UINT16_MAX));
        float z_measured_soc = static_cast<float>(estimateSocFromVoltage(static_cast<uint16_t>(estimated_ocv_mv)));

        // --- 3. KALMAN UPDATE (Sensor Fusion) ---
        float r_noise = (absolute(shunt_uv) < BatteryLimits::IDLE_SHUNT_UV_THRESHOLD) 
                        ? BatteryLimits::KF_MEAS_NOISE_REST 
                        : BatteryLimits::KF_MEAS_NOISE_ACTIVE;

        KalmanFilter::update(kf_soc_pct, kf_p_covariance, z_measured_soc, r_noise);

        // Sync back to traditional Coulomb accumulator
        accumulated_uAh = static_cast<int64_t>((kf_soc_pct / 100.0f) * static_cast<float>(max_uAh));
        
        last_poll_time_ms = now;
        seedOrResyncCoulombCounter(pack_mv, shunt_uv, false);
    }

    accumulated_uAh = std::clamp(accumulated_uAh, int64_t{0}, max_uAh);
    const uint8_t soc_pct = static_cast<uint8_t>((accumulated_uAh * 100LL) / max_uAh);

    cache.voltage = Millivolts{pack_mv};
    cache.current = Milliamps{current_ma};
    cache.shunt_voltage = Microvolts{shunt_uv};
    cache.soc = Percent{soc_pct};
    cache.temperature = Kelvin{static_cast<uint16_t>(temp_tenths + Thermistor::KELVIN_OFFSET_TENTHS)};
    const int64_t mah = accumulated_uAh / 1000LL;

    cache.capacity = MilliAmpHours{static_cast<uint32_t>(std::clamp(mah, int64_t{0}, static_cast<int64_t>(UINT32_MAX)))};
    cache.valid = true;
    cache.last_error = CommFault::NONE;
    cache.timestamp_ms = now;

    consecutive_comm_failures = 0U;
    consecutive_mutex_failures = 0U;
    last_valid_comm_time = now;

    k_mutex_unlock(&cache_mutex);
}

BmsCache SbsBattery::getCacheSnapshot() const {
    BmsCache snapshot{};
    if (k_mutex_lock(&cache_mutex, K_MSEC(BatteryLimits::MUTEX_TIMEOUT_MS)) == 0) {
        snapshot = cache;
        k_mutex_unlock(&cache_mutex);
    } else {
        snapshot.valid = false;
        snapshot.last_error = CommFault::MUTEX_TIMEOUT;
    }
    return snapshot;
}

static bool isCacheFresh(const BmsCache& c) {
    return (c.timestamp_ms != 0U) && ((k_uptime_get_32() - c.timestamp_ms) <= BatteryLimits::CACHE_STALE_MS);
}

static CommFault cacheFailureReason(const BmsCache& c) {
    return (c.last_error != CommFault::NONE) ? c.last_error : CommFault::CACHE_INVALID;
}

static bool isCacheValid(const BmsCache& c) {
    return c.valid && (c.last_error == CommFault::NONE) && isCacheFresh(c);
}

result<Millivolts> SbsBattery::getVoltage() const {
    const BmsCache snapshot = getCacheSnapshot();
    return isCacheValid(snapshot) ? result<Millivolts>::Ok(snapshot.voltage) : result<Millivolts>::Err(cacheFailureReason(snapshot));
}

result<Milliamps> SbsBattery::getCurrent() const {
    const BmsCache snapshot = getCacheSnapshot();
    return isCacheValid(snapshot) ? result<Milliamps>::Ok(snapshot.current) : result<Milliamps>::Err(cacheFailureReason(snapshot));
}

result<Microvolts> SbsBattery::getShuntVoltage() const {
    const BmsCache snapshot = getCacheSnapshot();
    return isCacheValid(snapshot) ? result<Microvolts>::Ok(snapshot.shunt_voltage) : result<Microvolts>::Err(cacheFailureReason(snapshot));
}

result<Percent> SbsBattery::getStateOfCharge() const {
    const BmsCache snapshot = getCacheSnapshot();
    return isCacheValid(snapshot) ? result<Percent>::Ok(snapshot.soc) : result<Percent>::Err(cacheFailureReason(snapshot));
}

result<Kelvin> SbsBattery::getTemperature() const {
    const BmsCache snapshot = getCacheSnapshot();
    return isCacheValid(snapshot) ? result<Kelvin>::Ok(snapshot.temperature) : result<Kelvin>::Err(cacheFailureReason(snapshot));
}

result<MilliAmpHours> SbsBattery::getCapacity() const {
    const BmsCache snapshot = getCacheSnapshot();
    return isCacheValid(snapshot) ? result<MilliAmpHours>::Ok(snapshot.capacity) : result<MilliAmpHours>::Err(cacheFailureReason(snapshot));
}

void SbsBattery::processFSM() {
    const BmsCache snapshot = getCacheSnapshot();
    if (!isCacheValid(snapshot)) {
        return;
    }

    if (snapshot.voltage.value < BatteryLimits::MIN_VALID_VOLTAGE_MV) {
        if (current_state.load() != BatteryFSM::IDLE) {
            current_state.store(BatteryFSM::IDLE);
        }
        FsmClassifier::reset();
        return; 
    }

    const int32_t shunt_uv = snapshot.shunt_voltage.value;
    const uint8_t soc_pct = snapshot.soc.value;
    const BatteryFSM current_fsm_state = current_state.load();
    BatteryFSM next_fsm_state = current_fsm_state;

    if (current_fsm_state != BatteryFSM::CUTOFF) {
        const float shunt_ema = FsmClassifier::update(shunt_uv);
        next_fsm_state = FsmClassifier::classify(current_fsm_state, shunt_ema,
                                                 BatteryLimits::IDLE_SHUNT_UV_THRESHOLD);

        if (current_fsm_state != next_fsm_state) {
            LOG_INF("Battery state changed (%d -> %d)", (int)current_fsm_state, (int)next_fsm_state);
        } 
        
        current_state.store(next_fsm_state);
        const BatteryFSM active_state = current_state.load();
        if (active_state == BatteryFSM::CHARGING || active_state == BatteryFSM::CUTOFF|| active_state == BatteryFSM::IDLE|| active_state == BatteryFSM::DISCHARGING) {
            PowerManager::getInstance().reportActivity();
        }
    }

    if (soc_pct >= 100U) {
        bool expected = false;
        if (full_charge_logged.compare_exchange_strong(expected, true)) {
            LOG_INF("BATTERY FULLY CHARGED. Logging event to NVS.");
            const ConfigStore::LogEntry entry{k_uptime_get_32(), 0x01U, static_cast<int32_t>(soc_pct)};
            (void)ConfigStore::getInstance().set(ConfigKey::FULL_CHARGE_LOG, entry);
        }
    } else if (soc_pct < 95U) {
        full_charge_logged.store(false);
    }

    if (soc_pct < BatteryLimits::CUTOFF_SOC_PCT) {
        if (current_fsm_state != BatteryFSM::CUTOFF) {
            if (sys_context != nullptr) {
                LOG_WRN("Battery Critically Low. Requesting SAFE_HALT state.");
                sys_context->requestTransition(SystemState::SAFE_HALT);
            }
            current_state.store(BatteryFSM::CUTOFF);
        }
    } else if (soc_pct > BatteryLimits::REENABLE_SOC_PCT) {
        if (current_fsm_state == BatteryFSM::CUTOFF) {
            if (sys_context != nullptr && sys_context->getState() == SystemState::SAFE_HALT) {
                LOG_INF("Battery recovered to %u%%. System safe to restart.", soc_pct);
                sys_context->requestTransition(SystemState::INIT);
                current_state.store((shunt_uv > BatteryLimits::IDLE_SHUNT_UV_THRESHOLD) ? BatteryFSM::CHARGING : BatteryFSM::IDLE);
                FsmClassifier::reset();
            }
        }
    }
}

BatteryFSM SbsBattery::getState() const { return current_state.load(); }

CommStatistics SbsBattery::getStats() const {
    CommStatistics snapshot{};
    snapshot.reads = static_cast<uint32_t>(atomic_get(&stats.reads));
    snapshot.i2c_faults = static_cast<uint32_t>(atomic_get(&stats.i2c_faults));
    snapshot.thermistor_faults = static_cast<uint32_t>(atomic_get(&stats.thermistor_faults));
    snapshot.retries = static_cast<uint32_t>(atomic_get(&stats.retries));
    snapshot.validation_errors = static_cast<uint32_t>(atomic_get(&stats.validation_errors));
    snapshot.successful_publishes = static_cast<uint32_t>(atomic_get(&stats.successful_publishes));
    return snapshot;
}
