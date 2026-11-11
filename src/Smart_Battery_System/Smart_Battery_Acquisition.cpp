// Smart_Battery_Acquisition.cpp
// SbsBattery - hardware acquisition and its fault handling: init() (INA226 + thermistor), the retry-with-backoff register reads
// (fetch*WithRetry via Retry_Policy.h, I2C fault -> CommFault mapping), pollHardwareAndUpdateCache() (disconnect detection, range and
// jump validation, then hands the sample to updateStateAndPublish()), and publishError() (failure counting, watchdog / CUTOFF).
// The SoC estimation, cache getters and the FSM are in Smart_Battery_System.cpp.

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

LOG_MODULE_DECLARE(BATTERY_SYS, LOG_LEVEL_INF);

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

bool SbsBattery::init() {
    k_mutex_init(&cache_mutex);
    const bool ina_ok = ina226.init();
    if (!ina_ok) LOG_ERR("INA226 init failed");
    const bool therm_ok = Thermistor::init();
    if (!therm_ok) LOG_ERR("Thermistor ADC init failed");
    return ina_ok && therm_ok;
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

#ifdef IS_TEST_ENVIRONMENT
CommFault test_mapI2CFault(I2CFault fault) { return mapI2CFault(fault); }
#endif
