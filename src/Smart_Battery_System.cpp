#include "Smart_Battery_System.h"
#include <zephyr/logging/log.h>
#include <zephyr/device.h>
#include <zephyr/sys/byteorder.h>
#include <array>
#include <algorithm>

#ifdef IS_TEST_ENVIRONMENT
    __attribute__((weak)) int test_iterations_remaining = 0;
    #define THREAD_LOOP_CONDITION (test_iterations_remaining > 0 ? (test_iterations_remaining--, true) : false)
#else
    #define THREAD_LOOP_CONDITION true
#endif

extern DeviceContext sys_context;
#ifndef IS_TEST_ENVIRONMENT
extern I2CManager i2c_manager;
#endif

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
        switch (fault) {
            case I2CFault::NONE:              return CommFault::NONE;
            case I2CFault::NACK:              return CommFault::I2C_NACK;
            case I2CFault::TIMEOUT:           return CommFault::I2C_TIMEOUT;
            case I2CFault::BUS_BUSY:          return CommFault::I2C_BUS_BUSY;
            case I2CFault::ARBITRATION_LOST:  return CommFault::I2C_ARBITRATION_LOST;
            case I2CFault::DEVICE_NOT_READY:  return CommFault::DEVICE_NOT_READY;
            default:                          return CommFault::I2C_NACK;
        }
    }
}

namespace CurveFitting {
    struct OcvPoint { uint16_t mv; uint8_t soc_pct; };
    struct NtcPoint { int32_t mv; int16_t temp_tenths; };

    static constexpr OcvPoint OCV_LUT[] = {
        { 8700, 0 },    { 9600, 5 },    { 10200, 10 },
        { 10800, 25 },  { 11100, 40 },  { 11400, 60 },
        { 11700, 75 },  { 12000, 85 },  { 12300, 95 },
        { 12600, 100 }
    };

    static constexpr NtcPoint NTC_LUT[] = {
        { 3220, -400 }, { 3187, -350 }, { 3143, -300 }, { 3086, -250 },
        { 3014, -200 }, { 2925, -150 }, { 2816, -100 }, { 2689,  -50 },
        { 2543,    0 }, { 2381,   50 }, { 2206,  100 }, { 2023,  150 },
        { 1836,  200 }, { 1650,  250 }, { 1470,  300 }, { 1301,  350 },
        { 1143,  400 }, { 1000,  450 }, {  871,  500 }, {  757,  550 },
        {  657,  600 }, {  570,  650 }, {  494,  700 }, {  428,  750 },
        {  372,  800 }, {  323,  850 }, {  282,  900 }, {  246,  950 },
        {  215, 1000 }, {  189, 1050 }, {  166, 1100 }, {  146, 1150 },
        {  129, 1200 }, {  114, 1250 }
    };

    template <size_t N>
    uint8_t interpolateOcv(const OcvPoint (&lut)[N], uint16_t mv) {
        if (mv <= lut[0].mv) return lut[0].soc_pct;
        if (mv >= lut[N-1].mv) return lut[N-1].soc_pct;
        
        for (size_t i = 0; i < N - 1; ++i) {
            if (mv >= lut[i].mv && mv <= lut[i+1].mv) {
                uint32_t v_range = lut[i+1].mv - lut[i].mv;
                uint32_t s_range = lut[i+1].soc_pct - lut[i].soc_pct;
                uint32_t v_offset = mv - lut[i].mv;
                return lut[i].soc_pct + static_cast<uint8_t>((v_offset * s_range) / v_range);
            }
        }
        return 0;
    }

    template <size_t N>
    int16_t interpolateNtc(const NtcPoint (&lut)[N], int32_t mv) {
        if (mv >= lut[0].mv) return lut[0].temp_tenths;
        if (mv <= lut[N-1].mv) return lut[N-1].temp_tenths;
        
        for (size_t i = 0; i < N - 1; ++i) {
            if (mv <= lut[i].mv && mv >= lut[i+1].mv) {
                int32_t v_range = lut[i].mv - lut[i+1].mv;
                int32_t t_range = lut[i+1].temp_tenths - lut[i].temp_tenths;
                int32_t v_offset = lut[i].mv - mv;
                return lut[i].temp_tenths + static_cast<int16_t>((v_offset * t_range) / v_range);
            }
        }
        return 250;
    }
}

namespace Thermistor {
#ifdef CONFIG_BOARD_QEMU_CORTEX_M3
bool init() { return true; }
Reading<int16_t> readCelsius() { return Reading<int16_t>::Ok(250); }
#else
const struct adc_dt_spec thermistor_adc_chan = ADC_DT_SPEC_GET_BY_IDX(DT_NODELABEL(zephyr_user), 0);

bool init() {
    if (!adc_is_ready_dt(&thermistor_adc_chan)) {
        LOG_ERR("Thermistor ADC channel not ready");
        return false;
    }
    return adc_channel_setup_dt(&thermistor_adc_chan) == 0;
}

Reading<int16_t> readCelsius() {
    if (!adc_is_ready_dt(&thermistor_adc_chan)) return Reading<int16_t>::Err(Fault::ADC_NOT_READY);

    int32_t sum_mv = 0;
    for (uint8_t i = 0; i < OVERSAMPLE_COUNT; ++i) {
        int16_t raw = 0;
        struct adc_sequence sequence{};
        sequence.buffer = &raw;
        sequence.buffer_size = sizeof(raw);

        if (adc_sequence_init_dt(&thermistor_adc_chan, &sequence) != 0 || 
            adc_read(thermistor_adc_chan.dev, &sequence) != 0) {
            return Reading<int16_t>::Err(Fault::ADC_READ_ERROR);
        }

        int32_t mv = raw;
        if (adc_raw_to_millivolts_dt(&thermistor_adc_chan, &mv) != 0) return Reading<int16_t>::Err(Fault::ADC_READ_ERROR);
        sum_mv += mv;
    }

    int32_t avg_mv = sum_mv / static_cast<int32_t>(OVERSAMPLE_COUNT);

    if ((avg_mv <= 0) || (avg_mv >= SUPPLY_MILLIVOLTS)) return Reading<int16_t>::Err(Fault::OUT_OF_RANGE);
    const int16_t celsius_tenths = CurveFitting::interpolateNtc(CurveFitting::NTC_LUT, avg_mv);

    if ((celsius_tenths < (MIN_VALID_CELSIUS * 10)) || (celsius_tenths > (MAX_VALID_CELSIUS * 10))) {
        return Reading<int16_t>::Err(Fault::OUT_OF_RANGE);
    }
    return Reading<int16_t>::Ok(celsius_tenths);
}
#endif 
}

namespace INA226 {

Driver::Driver(I2CManager* i2c) : i2c(i2c) {}

bool Driver::init() {
    if (i2c == nullptr) return false;
    const Result<bool> cfg = i2c->writeWord(I2C_ADDR, REG_CONFIG, sys_cpu_to_be16(CONFIG_VALUE));
    if (!cfg.isOk()) return false;
    return i2c->writeWord(I2C_ADDR, REG_CALIBRATION, sys_cpu_to_be16(CALIBRATION_VALUE)).isOk();
}

Result<int16_t> Driver::readBusVoltageRaw() {
    const Result<uint16_t> r = i2c->readWord(I2C_ADDR, REG_BUS_VOLT);
    if (!r.isOk()) return Result<int16_t>::Err(r.error);
    
    // Explicit manual byte-swap to guarantee endianness translation regardless of I2CManager's pointer casting
    uint16_t raw_val = r.unwrap();
    return Result<int16_t>::Ok(static_cast<int16_t>((raw_val << 8) | (raw_val >> 8)));
}

Result<int16_t> Driver::readCurrentRaw() {
    const Result<uint16_t> r = i2c->readWord(I2C_ADDR, REG_CURRENT);
    if (!r.isOk()) return Result<int16_t>::Err(r.error);
    
    // Explicit manual byte-swap
    uint16_t raw_val = r.unwrap();
    return Result<int16_t>::Ok(static_cast<int16_t>((raw_val << 8) | (raw_val >> 8)));
}

}

SbsBattery::SbsBattery(I2CManager* i2c_bus, DeviceContext* context, WatchdogFeedHook hook)
    : ina226(i2c_bus), sys_context(context), current_state(BatteryFSM::IDLE),
      full_charge_logged(false), watchdog_feed_hook((hook != nullptr) ? hook : daly_watchdog_feed_hook),
      cache_mutex{}, last_valid_comm_time(k_uptime_get_32()), consecutive_comm_failures(0U),
      consecutive_mutex_failures(0U),
      cache{}, stats{}, soc_initialized(false), accumulated_uAh(0), last_poll_time_ms(0U),
      consecutive_jump_rejects(0U), rest_period_start_ms(0U) {}

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

result<int16_t> SbsBattery::fetchBusVoltageRawWithRetry() {
    Result<int16_t> response = Result<int16_t>::Err(I2CFault::TIMEOUT);
    uint32_t backoff_ms = INITIAL_BACKOFF_MS;

    for (uint32_t attempt = 0U; attempt <= MAX_RETRIES; ++attempt) {
        feedWatchdog();
        response = ina226.readBusVoltageRaw();
        if (response.isOk()) {
            atomic_inc(&stats.reads);
            return result<int16_t>::Ok(response.unwrap());
        }
        atomic_inc(&stats.retries);
        if (attempt < MAX_RETRIES) {
            k_msleep(backoff_ms);
            backoff_ms = (backoff_ms >= (MAX_BACKOFF_MS / 2U)) ? MAX_BACKOFF_MS : (backoff_ms * 2U);
        }
    }
    atomic_inc(&stats.i2c_faults);
    return result<int16_t>::Err(mapI2CFault(response.error));
}

result<int16_t> SbsBattery::fetchCurrentRawWithRetry() {
    Result<int16_t> response = Result<int16_t>::Err(I2CFault::TIMEOUT);
    uint32_t backoff_ms = INITIAL_BACKOFF_MS;

    for (uint32_t attempt = 0U; attempt <= MAX_RETRIES; ++attempt) {
        feedWatchdog();
        response = ina226.readCurrentRaw();
        if (response.isOk()) {
            atomic_inc(&stats.reads);
            return result<int16_t>::Ok(response.unwrap());
        }
        atomic_inc(&stats.retries);
        if (attempt < MAX_RETRIES) {
            k_msleep(backoff_ms);
            backoff_ms = (backoff_ms >= (MAX_BACKOFF_MS / 2U)) ? MAX_BACKOFF_MS : (backoff_ms * 2U);
        }
    }
    atomic_inc(&stats.i2c_faults);
    return result<int16_t>::Err(mapI2CFault(response.error));
}

result<int16_t> SbsBattery::fetchTemperatureTenthsWithRetry() {
    Thermistor::Reading<int16_t> response = Thermistor::Reading<int16_t>::Err(Thermistor::Fault::ADC_READ_ERROR);
    uint32_t backoff_ms = INITIAL_BACKOFF_MS;

    for (uint32_t attempt = 0U; attempt <= MAX_RETRIES; ++attempt) {
        feedWatchdog();
        response = Thermistor::readCelsius();
        if (response.success) {
            atomic_inc(&stats.reads);
            return result<int16_t>::Ok(response.value);
        }
        atomic_inc(&stats.retries);
        if (attempt < MAX_RETRIES) {
            k_msleep(backoff_ms);
            backoff_ms = (backoff_ms >= (MAX_BACKOFF_MS / 2U)) ? MAX_BACKOFF_MS : (backoff_ms * 2U);
        }
    }
    atomic_inc(&stats.thermistor_faults);
    return result<int16_t>::Err(CommFault::THERMISTOR_FAULT);
}

uint8_t SbsBattery::estimateSocFromVoltage(uint16_t pack_mv) const {
    return CurveFitting::interpolateOcv(CurveFitting::OCV_LUT, pack_mv);
}

void SbsBattery::seedOrResyncCoulombCounter(uint16_t pack_mv, int32_t current_ma, bool force_seed) {
    const bool at_rest = (current_ma > -BatteryLimits::REST_CURRENT_THRESHOLD_MA) &&
                         (current_ma < BatteryLimits::REST_CURRENT_THRESHOLD_MA);

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
        soc_initialized = true;
    }
}

void SbsBattery::updateStateAndPublish(uint16_t pack_mv, int32_t current_ma, int16_t temp_tenths) {
    if (k_mutex_lock(&cache_mutex, K_MSEC(BatteryLimits::MUTEX_TIMEOUT_MS)) != 0) {
        atomic_inc(&stats.validation_errors);
        publishError(CommFault::MUTEX_TIMEOUT);
        return;
    }

    const uint32_t now = k_uptime_get_32();

    if (!soc_initialized) {
        seedOrResyncCoulombCounter(pack_mv, current_ma, true);
        last_poll_time_ms = now;
    } else {
        const uint32_t delta_ms = now - last_poll_time_ms;
        const int64_t uAh_change = (static_cast<int64_t>(current_ma) * static_cast<int64_t>(delta_ms)) / 3600LL;
        accumulated_uAh += uAh_change;
        last_poll_time_ms = now;
        seedOrResyncCoulombCounter(pack_mv, current_ma, false);
    }

    const int64_t max_uAh = static_cast<int64_t>(BatteryLimits::NOMINAL_CAPACITY_MAH) * 1000LL;
    accumulated_uAh = std::clamp(accumulated_uAh, int64_t{0}, max_uAh);
    const uint8_t soc_pct = static_cast<uint8_t>((accumulated_uAh * 100LL) / max_uAh);

    cache.voltage = Millivolts{pack_mv};
    cache.current = Milliamps{current_ma};
    cache.soc = Percent{soc_pct};
    cache.temperature = Kelvin{static_cast<uint16_t>(temp_tenths + Thermistor::KELVIN_OFFSET_TENTHS)};
    cache.capacity = MilliAmpHours{static_cast<uint32_t>(accumulated_uAh / 1000LL)};
    cache.valid = true;
    cache.last_error = CommFault::NONE;
    cache.timestamp_ms = now;

    consecutive_comm_failures = 0U;
    consecutive_mutex_failures = 0U;
    last_valid_comm_time = now;

    k_mutex_unlock(&cache_mutex);
}

void SbsBattery::pollHardwareAndUpdateCache() {
#ifdef CONFIG_BOARD_QEMU_CORTEX_M3
    updateStateAndPublish(11100U, -150, 250);
    feedWatchdog();
    return;
#else
    const result<int16_t> voltage_raw = fetchBusVoltageRawWithRetry();
    if (!voltage_raw.success) {
        publishError(voltage_raw.error);
        return;
    }
    
    const uint32_t pack_mv_32 = static_cast<uint32_t>(voltage_raw.value) + (static_cast<uint32_t>(voltage_raw.value) / 4U);
    if (pack_mv_32 > BatteryLimits::MAX_VALID_VOLTAGE_MV) {
        atomic_inc(&stats.validation_errors);
        publishError(CommFault::VALIDATION_ERROR);
        return;
    }
    const uint16_t pack_mv = static_cast<uint16_t>(pack_mv_32);

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

    const BmsCache snapshot = getCacheSnapshot();
    // Maintain checks if we are actively guarding a jump fault
    if (snapshot.valid || snapshot.last_error == CommFault::VALIDATION_ERROR) {
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

    updateStateAndPublish(pack_mv, current_ma, temp_tenths.value);
    atomic_inc(&stats.successful_publishes);
    feedWatchdog();
#endif
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
        LOG_ERR("Battery cache unavailable. Error Code:%d", static_cast<int>(cacheFailureReason(snapshot)));
        return;
    }

    const int32_t current_ma = snapshot.current.value;
    const uint8_t soc_pct = snapshot.soc.value;
    const BatteryFSM current_fsm_state = current_state.load();

    if (current_fsm_state != BatteryFSM::CUTOFF) {
        if (current_ma > 0) current_state.store(BatteryFSM::CHARGING);
        else if (current_ma < 0) current_state.store(BatteryFSM::DISCHARGING);
        else current_state.store(BatteryFSM::IDLE);
    }

    if (soc_pct >= 100U) {
        bool expected = false;
        if (full_charge_logged.compare_exchange_strong(expected, true)) {
            LOG_INF("BATTERY FULLY CHARGED. Triggering NVS Log entry.");
        }
    } else if (soc_pct < 95U) {
        full_charge_logged.store(false);
    }

    if (soc_pct <= BatteryLimits::CUTOFF_SOC_PCT) {
        if (current_fsm_state != BatteryFSM::CUTOFF) {
            LOG_ERR("DISCHARGE GUARD TRIGGERED! SoC:%u%%. Halting System.", soc_pct);
            if (sys_context != nullptr) sys_context->triggerFault("Battery Critically Low");
            current_state.store(BatteryFSM::CUTOFF);
        }
    } else if (soc_pct >= BatteryLimits::REENABLE_SOC_PCT) {
        if (sys_context != nullptr && sys_context->getState() == SystemState::SAFE_HALT) {
            LOG_INF("Battery recovered to %u%%. System safe to restart.", soc_pct);
            sys_context->requestTransition(SystemState::INIT);
            current_state.store((current_ma > 0) ? BatteryFSM::CHARGING : BatteryFSM::IDLE);
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

SbsBattery* smart_battery = nullptr;

namespace {
    static bool bms_objects_initialized = false;
#ifndef IS_TEST_ENVIRONMENT
    static SbsBattery static_smart_battery(&i2c_manager, &sys_context, daly_watchdog_feed_hook);
#endif

    class BmsPowerObserver final : public IPowerObserver {
    private:
        atomic_t is_sleeping;
    public:
        BmsPowerObserver() { atomic_set(&is_sleeping, 0); }
        void beforeSleep() override { atomic_set(&is_sleeping, 1); }
        void afterWakeup() override {
            atomic_set(&is_sleeping, 0);
            if (smart_battery != nullptr) smart_battery->notifySystemWakeup();
        }
        void sleepAborted() override { atomic_set(&is_sleeping, 0); }
        bool isSleeping() const noexcept { return atomic_get(&is_sleeping) != 0; }
    };
    
    static BmsPowerObserver g_bmsPowerObserver;
    K_SEM_DEFINE(bms_objects_ready_sem, 0, 1);

    static void initializeBmsObjects() {
        if (!bms_objects_initialized) {
#ifndef IS_TEST_ENVIRONMENT
            smart_battery = &static_smart_battery;
#endif
            bms_objects_initialized = true;
        }
    }
}

SbsBattery* getSmartBatteryInstance() {
    initializeBmsObjects();
    return smart_battery;
}

#ifndef IS_TEST_ENVIRONMENT
I2CManager* getI2cBusManagerInstance() {
    initializeBmsObjects();
    return &i2c_manager;
}
#endif

void bms_comm_thread(void) {
    initializeBmsObjects();

    if (smart_battery == nullptr) {
        LOG_ERR("Smart battery instance is null.");
        k_sem_give(&bms_objects_ready_sem); 
        return; 
    }

    uint32_t init_attempts = 0U;
    while (!smart_battery->init()) {
        ++init_attempts;
        LOG_ERR("BMS sensors (INA226 / thermistor) init failed (attempt %u/%u).",
                init_attempts, BatteryLimits::MAX_INIT_RETRIES);

        if (init_attempts >= BatteryLimits::MAX_INIT_RETRIES) {
            LOG_ERR("BMS sensor init failed %u times -- escalating fault instead of retrying forever.", init_attempts);
            sys_context.triggerFault("BMS Sensor Init Failure");
            k_sem_give(&bms_objects_ready_sem);
            return;
        }

        k_msleep(5000);
    }

    k_sem_give(&bms_objects_ready_sem);
    PowerManager::getInstance().registerObserver(&g_bmsPowerObserver);

    do {
       if (!g_bmsPowerObserver.isSleeping()) {
           smart_battery->pollHardwareAndUpdateCache();
       }
       k_msleep(1000);
    } while(THREAD_LOOP_CONDITION);
}

void battery_monitor_thread(void) {
    k_sem_take(&bms_objects_ready_sem, K_FOREVER);
    
    do {
        if (smart_battery != nullptr) {
            if (!g_bmsPowerObserver.isSleeping()) {
                smart_battery->processFSM();

                if (smart_battery->getState() == BatteryFSM::DISCHARGING) {
                    const auto soc = smart_battery->getStateOfCharge();
                    if (soc.success) {
                        LOG_DBG("Battery Discharging: %u%% remaining", soc.value.value);
                    }
                }
            }
        }
        k_msleep(1000);
    } while(THREAD_LOOP_CONDITION);
}

K_THREAD_DEFINE(bms_comm_tid, 1536, bms_comm_thread, NULL, NULL, NULL, BatteryLimits::BMS_THREAD_PRIO, 0, 0);
K_THREAD_DEFINE(battery_tid, 1024, battery_monitor_thread, NULL, NULL, NULL, BatteryLimits::MONITOR_THREAD_PRIO, 0, 0);
