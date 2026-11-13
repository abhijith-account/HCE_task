// I2C_Fallback_Cache.cpp
// The fallback side of the fault-tolerant I2C layer:
//   - the register cache (StaticPool<CacheEntry, 64>): update_cache() on every good read, get_cached_value() on a bus error
//     with staleness, reliability (100 -> 50, -10 per use) and calibration checks
//   - FailSafeStrategy: remembers the last-known-good value and reports the fall-back
//   - the host-test hooks (IS_TEST_ENVIRONMENT): resetI2CCacheForTests(), exhaustI2CCachePool(), test_* shims
// update_cache() / get_cached_value() are declared in I2C_Internal.h and called from I2C_Manager.cpp.

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/device.h>
#include "Fault_Tolerant_I2C_Communication_Layer.h"
#include "Static_Memory+MISRA_Compliance_Layer.h"
#include "Power_Management_System.h"
#include "Device_State_Machine+Watchdog.h" 
#include <zephyr/sys/atomic.h>
#include <cerrno>
#include <cstdlib>
#include <new>
#include <cstdint>
#include "I2C_Internal.h"

LOG_MODULE_DECLARE(I2C_HW, LOG_LEVEL_INF);

constexpr size_t MAX_CACHED_REGISTERS = 64U;
constexpr uint8_t CACHE_RELIABILITY_MAX = 100U;
constexpr uint8_t CACHE_RELIABILITY_MIN_FOR_FALLBACK = 50U;
constexpr uint8_t CACHE_RELIABILITY_DECAY_PER_USE = 10U;

struct CacheEntry {
    uint32_t key;
    uint64_t raw_value;
    int64_t timestamp;
    uint8_t reliability_score;
    bool is_calibrated;
    uint32_t sample_count;
};

static StaticPool<CacheEntry, MAX_CACHED_REGISTERS> cache_pool;
static CacheEntry* active_entries[MAX_CACHED_REGISTERS] = {nullptr};

static struct k_mutex cache_tracker_mutex;
static bool cache_mutex_init = false;

void ensure_mutex_initialized() {
    if (!cache_mutex_init) {
        k_mutex_init(&cache_tracker_mutex);
        cache_mutex_init = true;
    }
}

namespace i2c_internal {

void update_cache(uint32_t key, uint64_t value, bool calibrated) {
    ensure_mutex_initialized();
    k_mutex_lock(&cache_tracker_mutex, K_FOREVER);
    int64_t now = k_uptime_get();

    for (size_t i = 0; i < MAX_CACHED_REGISTERS; i++) {
        if (active_entries[i] != nullptr && active_entries[i]->key == key) {
            active_entries[i]->raw_value = value;
            active_entries[i]->timestamp = now;
            active_entries[i]->reliability_score = CACHE_RELIABILITY_MAX;
            active_entries[i]->is_calibrated = calibrated;

            if (active_entries[i]->sample_count < UINT32_MAX) {
                ++active_entries[i]->sample_count;
            }
            k_mutex_unlock(&cache_tracker_mutex);
            return;
        }
    }

    size_t free_index = MAX_CACHED_REGISTERS;
    for (size_t i = 0U; i < MAX_CACHED_REGISTERS; i++) {
        if (active_entries[i] == nullptr) {
            free_index = i;
            break;
        }
    }

    if (free_index == MAX_CACHED_REGISTERS) {
        LOG_ERR("I2C cache table full. Key 0x%08X not cached.", key);
        k_mutex_unlock(&cache_tracker_mutex);
        return;
    }

    void* mem = cache_pool.allocate();
    if (mem != nullptr) {
        auto* new_entry = new(mem) CacheEntry;
        new_entry->key = key;
        new_entry->raw_value = value;
        new_entry->timestamp = now;
        new_entry->reliability_score = CACHE_RELIABILITY_MAX;
        new_entry->is_calibrated = calibrated;
        new_entry->sample_count = 1U;
        active_entries[free_index] = new_entry;
    }

    k_mutex_unlock(&cache_tracker_mutex);
}

bool get_cached_value(uint32_t key, uint64_t* out_val, uint32_t max_age_ms) {
    if (out_val == nullptr) {
        return false;
    }

    ensure_mutex_initialized();
    k_mutex_lock(&cache_tracker_mutex, K_FOREVER);
    const int64_t now = k_uptime_get();
    
    for (size_t i = 0; i < MAX_CACHED_REGISTERS; i++) {
       if ((active_entries[i] != nullptr) && (active_entries[i]->key == key)) {
          CacheEntry* const entry = active_entries[i];

          // Check if data is stale - but don't print error logs while in SAFE_HALT
          if ((now - entry->timestamp) > max_age_ms) {
              if (sys_context.getState() == SystemState::RUNNING) {
                  LOG_ERR("Fallback Rejected: Data Stale (>%d ms)", max_age_ms);
              }
              k_mutex_unlock(&cache_tracker_mutex);
              return false;
          }

          if (entry->reliability_score < CACHE_RELIABILITY_MIN_FOR_FALLBACK) {
              if (sys_context.getState() == SystemState::RUNNING) {
                  LOG_ERR("Fallback Rejected: Reliability Score too low");
              }
              k_mutex_unlock(&cache_tracker_mutex);
              return false;
          }

          if (!entry->is_calibrated) {
              if (sys_context.getState() == SystemState::RUNNING) {
                  LOG_ERR("Fallback Rejected: Data is uncalibrated.");
              }
              k_mutex_unlock(&cache_tracker_mutex);
              return false;
          }

          entry->reliability_score -= CACHE_RELIABILITY_DECAY_PER_USE;

          *out_val = entry->raw_value;
          k_mutex_unlock(&cache_tracker_mutex);
          return true;
       }
    }
    k_mutex_unlock(&cache_tracker_mutex);
    return false;
}

} // namespace i2c_internal

#ifdef IS_TEST_ENVIRONMENT
void resetI2CCacheForTests() noexcept {
    for (size_t i = 0; i < MAX_CACHED_REGISTERS; i++) {
        if (active_entries[i] != nullptr) {
            active_entries[i]->~CacheEntry();
            cache_pool.deallocate(active_entries[i]);
            active_entries[i] = nullptr;
        }
    }
}

static void* exhausted_blocks[MAX_CACHED_REGISTERS] = {nullptr};

void exhaustI2CCachePool() {
    for (size_t i = 0; i < MAX_CACHED_REGISTERS; i++) {
        exhausted_blocks[i] = cache_pool.allocate();
    }
}

void resetI2CCachePool() {
    resetI2CCacheForTests();

    for (size_t i = 0; i < MAX_CACHED_REGISTERS; i++) {
        if (exhausted_blocks[i] != nullptr) {
            cache_pool.deallocate(exhausted_blocks[i]);
            exhausted_blocks[i] = nullptr;
        }
    }
}

void test_update_cache(uint32_t key, uint64_t value, bool calibrated) {
    i2c_internal::update_cache(key, value, calibrated);
}

bool test_get_cached_value(uint32_t key, uint64_t* out_val, uint32_t max_age_ms) {
    return i2c_internal::get_cached_value(key, out_val, max_age_ms);
}

void test_set_reliability(uint32_t key, uint8_t score) {
    k_mutex_lock(&cache_tracker_mutex, K_FOREVER);
    bool found = false;
    for (size_t i = 0; i < MAX_CACHED_REGISTERS; i++) {
        if (active_entries[i] != nullptr && active_entries[i]->key == key) {
            active_entries[i]->reliability_score = score;
            found = true;
            break;
        }
    }
    k_mutex_unlock(&cache_tracker_mutex);
    if (!found) {
        fprintf(stderr, "Shim: Key 0x%08X not found in active_entries!\n", key);
    }
}

void test_set_sample_count(uint32_t key, uint32_t count)
{
    k_mutex_lock(&cache_tracker_mutex, K_FOREVER);

    for(size_t i = 0; i < MAX_CACHED_REGISTERS; ++i)
    {
        if(active_entries[i] &&
           active_entries[i]->key == key)
        {
            active_entries[i]->sample_count = count;
            break;
        }
    }

    k_mutex_unlock(&cache_tracker_mutex);
}
#endif
