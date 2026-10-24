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
    template <typename T>
    constexpr T absolute(T val) { return (val < 0) ? -val : val; }

    CommFault mapI2CFault(I2CFault fault) {
        if (fault == I2CFault::NACK) {
            return CommFault::I2C_NACK;
        } else if (fault == I2CFault::TIMEOUT) {
            return CommFault::I2C_TIMEOUT;
        } else if (fault == I2CFault::BUS_BUSY) {
            return CommFault::I2C_BUS_BUSY;
        } else if (fault == I2CFault::ARBITRATION_LOST) {
            return CommFault::I2C_ARBITRATION_LOST;
        } else if (fault == I2CFault::DEVICE_NOT_READY) {
            return CommFault::DEVICE_NOT_READY;
        } else {
            return CommFault::I2C_NACK;
        }
    }
}

SbsBattery::SbsBattery(I2CManager* i2c_bus, DeviceContext* context, WatchdogFeedHook hook)
    : ina226(i2c_bus), sys_context(context), current_state(BatteryFSM::IDLE),
      full_charge_logged(false), watchdog_feed_hook((hook != nullptr) ? hook : daly_watchdog_feed_hook),
      cache_mutex{}, last_valid_comm_time(k_uptime_get_32()), consecutive_comm_failures(0U),
      consecutive_mutex_failures(0U),
      cache{}, stats{}, soc_initialized(false), accumulated_uAh(0), last_poll_time_ms(0U),
      consecutive_jump_rejects(0U), rest_period_start_ms(0U),
      kf_soc_pct(0.0f), kf_p_covariance(1.0f) {}

bool SbsBattery::init() {
    k_mutex_init(&cache_mutex);
    const bool ina_ok = ina226.init();
    if (!ina_ok) LOG_ERR("INA226 init failed");
    const bool therm_ok = Thermistor::init();
    if (!therm_ok) LOG_ERR("Thermistor ADC init failed");
    return ina_ok && therm_ok;
}

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

result<int16_t> SbsBattery::fetchI2cRegisterWithRetry(Result<int16_t> (INA226::Driver::*reader)()) {
    const RetryPolicy::Config cfg{MAX_RETRIES, INITIAL_BACKOFF_MS, MAX_BACKOFF_MS};

    Result<int16_t> response = RetryPolicy::run(
        cfg, Result<int16_t>::Err(I2CFault::TIMEOUT),
        [this, reader]() { return (ina226.*reader)(); },
        [](const Result<int16_t>& r) { return r.isOk(); },
        [this]() { feedWatchdog(); },
        [this]() { atomic_inc(&stats.retries); },
        [](uint32_t ms) { k_msleep(ms); });

    if (response.isOk()) {
        atomic_inc(&stats.reads);
        return result<int16_t>::Ok(response.unwrap());
    }
    atomic_inc(&stats.i2c_faults);
    return result<int16_t>::Err(mapI2CFault(response.error));
}

result<int16_t> SbsBattery::fetchBusVoltageRawWithRetry() {
    return fetchI2cRegisterWithRetry(&INA226::Driver::readBusVoltageRaw);
}

result<int16_t> SbsBattery::fetchShuntVoltageRawWithRetry() {
    return fetchI2cRegisterWithRetry(&INA226::Driver::readShuntVoltageRaw);
}

result<int16_t> SbsBattery::fetchCurrentRawWithRetry() {
    return fetchI2cRegisterWithRetry(&INA226::Driver::readCurrentRaw);
}

result<int16_t> SbsBattery::fetchTemperatureTenthsWithRetry() {
    using TempReading = Thermistor::Reading<int16_t>;
    const RetryPolicy::Config cfg{MAX_RETRIES, INITIAL_BACKOFF_MS, MAX_BACKOFF_MS};

    const TempReading response = RetryPolicy::run(
        cfg, TempReading::Err(Thermistor::Fault::ADC_READ_ERROR),
        []() { return Thermistor::readCelsius(); },
        [](const TempReading& r) { return r.success; },
        [this]() { feedWatchdog(); },
        [this]() { atomic_inc(&stats.retries); },
        [](uint32_t ms) { k_msleep(ms); });

    if (response.success) {
        atomic_inc(&stats.reads);
        return result<int16_t>::Ok(response.value);
    }
    atomic_inc(&stats.thermistor_faults);
    return result<int16_t>::Err(CommFault::THERMISTOR_FAULT);
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

void SbsBattery::pollHardwareAndUpdateCache() {

    const result<int16_t> voltage_raw = fetchBusVoltageRawWithRetry();
    if (!voltage_raw.success) {
        publishError(voltage_raw.error);
        return;
    }

    const uint32_t pack_mv_32 = static_cast<uint32_t>(voltage_raw.value) + (static_cast<uint32_t>(voltage_raw.value) / 4U);
    const BmsCache snapshot = getCacheSnapshot();
    const bool was_connected = snapshot.valid && (snapshot.voltage.value >= BatteryLimits::MIN_VALID_VOLTAGE_MV);

    // Disconnect detection logic
    bool is_disconnected = (pack_mv_32 < BatteryLimits::MIN_VALID_VOLTAGE_MV);

    if (was_connected) {
        const int32_t v_drop = static_cast<int32_t>(snapshot.voltage.value) - static_cast<int32_t>(pack_mv_32);
        if ((v_drop > BatteryLimits::MAX_VOLTAGE_DELTA_MV) && (pack_mv_32 < BatteryLimits::PACK_MIN_VOLTAGE_MV)) {
            is_disconnected = true;
        }
    } else {
        if (pack_mv_32 < BatteryLimits::PACK_MIN_VOLTAGE_MV) {
            is_disconnected = true;
        }
    }

    if (is_disconnected) {
        if (k_mutex_lock(&cache_mutex, K_MSEC(BatteryLimits::MUTEX_TIMEOUT_MS)) == 0) {
            cache.voltage = Millivolts{0};
            cache.current = Milliamps{0};
            cache.shunt_voltage = Microvolts{0};
            cache.soc = Percent{0};
            cache.valid = true;
            cache.last_error = CommFault::NONE;
            cache.timestamp_ms = k_uptime_get_32();

            consecutive_comm_failures = 0U;
            consecutive_mutex_failures = 0U;
            last_valid_comm_time = cache.timestamp_ms;
            k_mutex_unlock(&cache_mutex);
        }
        current_state.store(BatteryFSM::IDLE);
        soc_initialized = false;
        consecutive_jump_rejects = 0U;
        FsmClassifier::reset(); // clear stale direction bias so a fresh connection starts clean
        feedWatchdog();
        return;
    }

    if (pack_mv_32 > BatteryLimits::MAX_VALID_VOLTAGE_MV) {
        atomic_inc(&stats.validation_errors);
        publishError(CommFault::VALIDATION_ERROR);
        return;
    }
    const uint16_t pack_mv = static_cast<uint16_t>(pack_mv_32);

    const result<int16_t> shunt_raw = fetchShuntVoltageRawWithRetry();
    if (!shunt_raw.success) {
        publishError(shunt_raw.error);
        return;
    }
    const int32_t shunt_uv = (static_cast<int32_t>(shunt_raw.value) * 5) / 2;

    const result<int16_t> current_raw = fetchCurrentRawWithRetry();
    if (!current_raw.success) {
        publishError(current_raw.error);
        return;
    }

    const int32_t current_ma = (static_cast<int32_t>(current_raw.value) * static_cast<int32_t>(INA226::CURRENT_LSB_UA)) / 1000;
    if (absolute(current_ma) > BatteryLimits::MAX_VALID_CURRENT_MA) {
        atomic_inc(&stats.validation_errors);
        publishError(CommFault::VALIDATION_ERROR);
        return;
    }

    const result<int16_t> temp_tenths = fetchTemperatureTenthsWithRetry();
    if (!temp_tenths.success) {
        publishError(temp_tenths.error);
        return;
    }

    if (snapshot.timestamp_ms != 0 && (snapshot.valid || snapshot.last_error == CommFault::VALIDATION_ERROR) && was_connected) {
        const int32_t v_delta = absolute(static_cast<int32_t>(pack_mv) - static_cast<int32_t>(snapshot.voltage.value));
        const int32_t c_delta = absolute(current_ma - snapshot.current.value);
        const int32_t prev_temp_tenths_c = static_cast<int32_t>(snapshot.temperature.value) - Thermistor::KELVIN_OFFSET_TENTHS;
        const int32_t t_delta = absolute(static_cast<int32_t>(temp_tenths.value) - prev_temp_tenths_c);

        const bool jump_detected = (v_delta > BatteryLimits::MAX_VOLTAGE_DELTA_MV) ||
                                    (c_delta > BatteryLimits::MAX_CURRENT_DELTA_MA) ||
                                    (t_delta > BatteryLimits::MAX_TEMP_DELTA_TENTHS);

        if (jump_detected) {
            atomic_inc(&stats.validation_errors);
            ++consecutive_jump_rejects;
            if (consecutive_jump_rejects >= BatteryLimits::MAX_CONSECUTIVE_JUMP_REJECTS) {
                publishError(CommFault::VALIDATION_ERROR);
            }
            return;
        }
    }
    consecutive_jump_rejects = 0U;

    updateStateAndPublish(pack_mv, current_ma, temp_tenths.value, shunt_uv);
    atomic_inc(&stats.successful_publishes);
    feedWatchdog();
}

void SbsBattery::publishError(CommFault fault) {
    bool threshold_reached = false;
    bool timeout_reached = false;
    const bool is_mutex_fault = (fault == CommFault::MUTEX_TIMEOUT);

    if (k_mutex_lock(&cache_mutex, K_MSEC(BatteryLimits::MUTEX_TIMEOUT_MS)) == 0) {
        cache.valid = false;
        cache.last_error = fault;

        if (is_mutex_fault) {
            ++consecutive_mutex_failures;
            threshold_reached = (consecutive_mutex_failures >= WATCHDOG_MUTEX_FAILURE_THRESHOLD);
        } else {
            ++consecutive_comm_failures;
            consecutive_mutex_failures = 0U;
            threshold_reached = (consecutive_comm_failures >= WATCHDOG_FAILURE_THRESHOLD);
        }

        timeout_reached = ((k_uptime_get_32() - last_valid_comm_time) > COMM_TIMEOUT_MS);
        k_mutex_unlock(&cache_mutex);
    } else {
        ++consecutive_mutex_failures;
        threshold_reached = (consecutive_mutex_failures >= WATCHDOG_MUTEX_FAILURE_THRESHOLD);
    }

    if ((threshold_reached || timeout_reached) && (current_state.load() != BatteryFSM::CUTOFF)) {
        LOG_ERR("CRITICAL: BMS watchdog triggered. Fault:%d (mutex-related:%d)",
                static_cast<int>(fault), static_cast<int>(is_mutex_fault));
        if (sys_context != nullptr) {
            sys_context->triggerFault("BMS Communication Watchdog");
        }
        current_state.store(BatteryFSM::CUTOFF);
    }
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
            const uint32_t event_timestamp = k_uptime_get_32();
            (void)ConfigStore::getInstance().set(ConfigKey::FULL_CHARGE_LOG, event_timestamp);
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

#ifdef IS_TEST_ENVIRONMENT
CommFault test_mapI2CFault(I2CFault fault) { return mapI2CFault(fault); }
#endif
