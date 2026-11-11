// I2C_Internal.h
// Private to the Fault_Tolerant_I2C sources (I2C_Manager.cpp, I2C_Fallback_Cache.cpp). Not part of the public API
// (that is Fault_Tolerant_I2C_Communication_Layer.h).
//
// The fallback cache lives in I2C_Fallback_Cache.cpp; I2CManager (I2C_Manager.cpp) stores every good read in it and falls back
// to it on a bus error. update_cache() and get_cached_value() used to be file-local (static) in the single source file; they are
// exported here, inside a namespace so the names cannot collide with anything else.
#pragma once

#include <cstdint>

namespace i2c_internal {

/* Store / refresh the value for `key`: resets the reliability score to the maximum, bumps the sample count. */
void update_cache(uint32_t key, uint64_t value, bool calibrated = true);

/* Fallback read: succeeds only for a fresh (<= max_age_ms), reliable (score >= minimum) and calibrated entry,
 * and decays the entry's reliability on every successful use. */
bool get_cached_value(uint32_t key, uint64_t* out_val, uint32_t max_age_ms);

} // namespace i2c_internal
