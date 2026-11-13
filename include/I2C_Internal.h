#pragma once

#include <cstdint>
#include "Fault_Tolerant_I2C_Communication_Layer.h"

namespace i2c_internal {

/* Map a negative Zephyr driver errno to an I2CFault enum value. */
I2CFault classify_i2c_error(int err);

/* Return true when the fault requires a physical SCL bus reset (TIMEOUT or BUS_BUSY). */
bool is_recoverable_with_bus_reset(I2CFault fault);

/* Classify `err` and execute either `reset_strategy` or `retry_strategy`. */
I2CFault classify_and_recover(int err,
                              const device* i2c_dev,
                              BusResetStrategy& reset_strategy,
                              RetryStrategy& retry_strategy);

/* Store / refresh the value for `key`: resets the reliability score to the maximum, bumps the sample count. */
void update_cache(uint32_t key, uint64_t value, bool calibrated = true);

/* Fallback read: succeeds only for a fresh (<= max_age_ms), reliable (score >= minimum) and calibrated entry,
 * and decays the entry's reliability on every successful use. */
bool get_cached_value(uint32_t key, uint64_t* out_val, uint32_t max_age_ms);

} // namespace i2c_internal
