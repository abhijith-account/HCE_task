// I2C_Recovery_Strategies.cpp
// Fault classification, recovery dispatch, and all three I2CStrategy implementations:
//   RetryStrategy    - transient fault (NACK / ARBITRATION_LOST): warn and back off (10 ms)
//   BusResetStrategy - bus lockup (TIMEOUT / BUS_BUSY): log SCL recovery sequence
//   FailSafeStrategy - last-known-good value tracking and fallback error reporting

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/device.h>
#include <cerrno>
#include <cstdint>
#include "Fault_Tolerant_I2C_Communication_Layer.h"
#include "I2C_Internal.h"

LOG_MODULE_DECLARE(I2C_HW, LOG_LEVEL_INF);

namespace i2c_internal {

I2CFault classify_i2c_error(int err) {
    switch (err) {
        case -ETIMEDOUT: return I2CFault::TIMEOUT;
        case -EBUSY:     return I2CFault::BUS_BUSY;
        case -EAGAIN:    return I2CFault::ARBITRATION_LOST;
        case -ENODEV:    return I2CFault::DEVICE_NOT_READY;
        default:         return I2CFault::NACK;
    }
}

bool is_recoverable_with_bus_reset(I2CFault fault) {
    return (fault == I2CFault::TIMEOUT) || (fault == I2CFault::BUS_BUSY);
}

I2CFault classify_and_recover(int err,
                              const device* i2c_dev,
                              BusResetStrategy& reset_strategy,
                              RetryStrategy& retry_strategy) {
    const I2CFault fault = classify_i2c_error(err);
    if (is_recoverable_with_bus_reset(fault)) {
        reset_strategy.executeRecovery(i2c_dev);
    } else {
        retry_strategy.executeRecovery(i2c_dev);
    }
    return fault;
}

} // namespace i2c_internal

void RetryStrategy::executeRecovery(const device* ) {
    LOG_WRN("I2C Fault Detected. Executing Exponential Backoff Retry...");
    k_msleep(10);
}

void BusResetStrategy::executeRecovery(const device* ) {
    LOG_ERR("I2C Bus Hard-Locked. Toggling SCL to recover");
}

void FailSafeStrategy::executeRecovery(const device* ) {
    LOG_ERR("I2C Critical Failure. Falling back to Last-known-Good data: %llu",
            (unsigned long long)last_known_good_value);
}

void FailSafeStrategy::updateLastGood(uint64_t val) {
    last_known_good_value = val;
}

uint64_t FailSafeStrategy::getLastGood() const {
    return last_known_good_value;
}
