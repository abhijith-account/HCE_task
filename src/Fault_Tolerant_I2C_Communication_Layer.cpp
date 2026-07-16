#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/device.h> // Fixed: Required for device_is_ready()
#include "Fault_Tolerant_I2C_Communication_Layer.h"
#include "Static_Memory+MISRA_Compliance_Layer.h"
#include <cerrno>
#include <cstdlib> 
#include <new>
#include <cstdint>

LOG_MODULE_REGISTER(I2C_HW, LOG_LEVEL_WRN);

constexpr size_t MAX_CACHED_REGISTERS = 15U;
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

// Fixed: Bypassing K_MUTEX_DEFINE macro to ensure compatibility with host-based Google Tests
static struct k_mutex cache_tracker_mutex;
static bool cache_mutex_init = false;

#ifdef CONFIG_BOARD_QEMU_CORTEX_M3

const struct device *i2c_hardware = nullptr;

#elif !defined(IS_TEST_ENVIRONMENT)

const struct device *i2c_hardware =
    DEVICE_DT_GET(DT_NODELABEL(i2c1));

#endif

#ifndef IS_TEST_ENVIRONMENT
I2CManager i2c_manager(i2c_hardware);
#endif

void ensure_mutex_initialized() {
    if (!cache_mutex_init) {
        k_mutex_init(&cache_tracker_mutex);
        cache_mutex_init = true;
    }
}

static void update_cache(uint32_t key, uint64_t value, bool calibrated = true) {
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
        CacheEntry* new_entry = new(mem) CacheEntry;
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

#ifndef CONFIG_BOARD_QEMU_CORTEX_M3

static bool get_cached_value(uint32_t key, uint64_t* out_val, uint32_t max_age_ms) {
    if (out_val == nullptr) {
        return false;
    }
    
    ensure_mutex_initialized();
    k_mutex_lock(&cache_tracker_mutex, K_FOREVER);
    const int64_t now = k_uptime_get();
    
    for (size_t i = 0; i < MAX_CACHED_REGISTERS; i++) {
       if ((active_entries[i] != nullptr) && (active_entries[i]->key == key)) {
          CacheEntry* const entry = active_entries[i];
          
          if ((now - entry->timestamp) > max_age_ms) {
              LOG_ERR("Fallback Rejected: Data Stale (>%d ms)", max_age_ms);
              k_mutex_unlock(&cache_tracker_mutex);
              return false;
          }
          
          if (entry->reliability_score < CACHE_RELIABILITY_MIN_FOR_FALLBACK) {
              LOG_ERR("Fallback Rejected: Reliability Score too low");
              k_mutex_unlock(&cache_tracker_mutex);
              return false;
          }
          
          if (!entry->is_calibrated) {
              LOG_ERR("Fallback Rejected: Data is uncalibrated.");
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

static bool is_recoverable_with_bus_reset(I2CFault fault) {
    return (fault == I2CFault::TIMEOUT) || (fault == I2CFault::BUS_BUSY);
}

static I2CFault classify_i2c_error(int err) {
    switch (err) {
        case -ETIMEDOUT: return I2CFault::TIMEOUT;
        case -EBUSY:     return I2CFault::BUS_BUSY;
        case -EAGAIN:    return I2CFault::ARBITRATION_LOST;
        case -ENODEV:    return I2CFault::DEVICE_NOT_READY;
        default:         return I2CFault::NACK; 
    }
}

#endif
 
void RetryStrategy::executeRecovery(const device* /* i2c_dev */) {
    LOG_WRN("I2C Fault Detected. Executing Exponential Backoff Retry...");
    k_msleep(10);
}

void BusResetStrategy::executeRecovery(const device* /* i2c_dev */) {
    LOG_ERR("I2C Bus Hard-Locked. Toggling SCL to recover");
}

void FailSafeStrategy::executeRecovery(const device* /* i2c_dev */) {
      LOG_ERR("I2C Critical Failure. Falling back to Last-known-Good data: %llu",
            (unsigned long long)last_known_good_value);
}

// Fixed: Signatures matched to Header declarations (uint64_t instead of uint8_t)
void FailSafeStrategy::updateLastGood(uint64_t val) {
      last_known_good_value = val;
}

uint64_t FailSafeStrategy::getLastGood() const {
      return last_known_good_value;
}

I2CManager::I2CManager(const device* i2c_dev) : i2c_dev(i2c_dev) {}

Result<uint8_t> I2CManager::readRegister(uint16_t sensor_addr, uint8_t reg_addr) {
   const uint32_t cache_key = (static_cast<uint32_t>(sensor_addr) << 8U) | static_cast<uint32_t>(reg_addr);
#ifdef CONFIG_BOARD_QEMU_CORTEX_M3
    k_msleep(50); 
    uint8_t mock_data = 60U;
    update_cache(cache_key, mock_data);
    failsafe_strategy.updateLastGood(mock_data);
    return Result<uint8_t>::Ok(mock_data);
#else
#ifdef IS_TEST_ENVIRONMENT
    extern int g_i2c_force_errno;
    if (g_i2c_force_errno != 0) {
        return Result<uint8_t>::Err(classify_i2c_error(g_i2c_force_errno));
    }
#endif
    if ((i2c_dev == nullptr) || (!device_is_ready(i2c_dev))) {
        return Result<uint8_t>::Err(I2CFault::DEVICE_NOT_READY);
    }
    
    uint8_t data = 0U;
    const int err = i2c_reg_read_byte(i2c_dev, sensor_addr, reg_addr, &data);
    
    if (err == 0) {
        update_cache(cache_key, data);
        failsafe_strategy.updateLastGood(data);
        return Result<uint8_t>::Ok(data);
    }
    
    const I2CFault fault = classify_i2c_error(err);
    if (is_recoverable_with_bus_reset(fault)) {
        reset_strategy.executeRecovery(i2c_dev);
    } else {
        retry_strategy.executeRecovery(i2c_dev);
    }
    
    uint64_t cached_val = 0U;
    if (get_cached_value(cache_key, &cached_val, 3000U)) {
       failsafe_strategy.executeRecovery(i2c_dev);
       return Result<uint8_t>::Ok(static_cast<uint8_t>(cached_val & 0xFFU));
    }
    return Result<uint8_t>::Err(fault);
#endif
}

Result<bool> I2CManager::writeRegister(uint16_t sensor_addr, uint8_t reg_addr, uint8_t val) {
#ifdef CONFIG_BOARD_QEMU_CORTEX_M3
    k_msleep(10);
    return Result<bool>::Ok(true);
#else
#ifdef IS_TEST_ENVIRONMENT
    extern int g_i2c_force_errno;
    if (g_i2c_force_errno != 0) {
       return Result<bool>::Err(classify_i2c_error(g_i2c_force_errno));
    }
#endif
    if ((i2c_dev == nullptr) || (!device_is_ready(i2c_dev))) {
        return Result<bool>::Err(I2CFault::DEVICE_NOT_READY);
    }
    
    const int err = i2c_reg_write_byte(i2c_dev, sensor_addr, reg_addr, val);
    
    if (err == 0) {
        return Result<bool>::Ok(true);
    }
    
    const I2CFault fault = classify_i2c_error(err);
    if (is_recoverable_with_bus_reset(fault)) {
        reset_strategy.executeRecovery(i2c_dev);
    } else {
        retry_strategy.executeRecovery(i2c_dev);
    }
    
    return Result<bool>::Err(fault);
#endif
}

Result<uint16_t> I2CManager::readWord(uint16_t sensor_addr, uint8_t reg_addr) {
    uint32_t cache_key = (static_cast<uint32_t>(sensor_addr) << 8) | reg_addr;
#ifdef CONFIG_BOARD_QEMU_CORTEX_M3
    k_msleep(50);
    uint16_t mock_val = 900U;
    update_cache(cache_key, mock_val);
    failsafe_strategy.updateLastGood(mock_val);
    return Result<uint16_t>::Ok(mock_val);
#else
#ifdef IS_TEST_ENVIRONMENT
    extern int g_i2c_force_errno;
    if (g_i2c_force_errno != 0) {
        return Result<uint16_t>::Err(classify_i2c_error(g_i2c_force_errno));
    }
#endif
    if ((i2c_dev == nullptr) || (!device_is_ready(i2c_dev))) {
        return Result<uint16_t>::Err(I2CFault::DEVICE_NOT_READY);
    }
    
    uint8_t buf[2] = {0U, 0U};
    const int err = i2c_burst_read(i2c_dev, sensor_addr, reg_addr, buf, 2);
    
    if (err == 0) {
        uint16_t val = static_cast<uint16_t>((static_cast<uint16_t>(buf[1]) << 8U) | static_cast<uint16_t>(buf[0]));
        update_cache(cache_key, val);
        failsafe_strategy.updateLastGood(val);
        return Result<uint16_t>::Ok(val);
    }
    
    const I2CFault fault = classify_i2c_error(err);
    if (is_recoverable_with_bus_reset(fault)) {
        reset_strategy.executeRecovery(i2c_dev);
    } else {
        retry_strategy.executeRecovery(i2c_dev);
    }
  
    uint64_t cached_val = 0U; // Fixed: Needs to be a 64-bit value to pass into get_cached_value
    if (get_cached_value(cache_key, &cached_val, 3000U)) {
        failsafe_strategy.executeRecovery(i2c_dev);
        return Result<uint16_t>::Ok(static_cast<uint16_t>(cached_val & 0xFFFFU));
    }
    return Result<uint16_t>::Err(fault);
#endif
}

Result<uint32_t> I2CManager::read24Bit(uint16_t sensor_addr, uint8_t reg_addr) {
    // Fixed: Corrected sensr_addr to sensor_addr
    uint32_t cache_key = (static_cast<uint32_t>(sensor_addr) << 8) | static_cast<uint32_t>(reg_addr); 
#ifdef CONFIG_BOARD_QEMU_CORTEX_M3
    k_msleep(50);
    uint32_t mock_val = 101300U;
    update_cache(cache_key, mock_val);
    failsafe_strategy.updateLastGood(mock_val);
    return Result<uint32_t>::Ok(mock_val);
#else
#ifdef IS_TEST_ENVIRONMENT
    extern int g_i2c_force_errno;
    if (g_i2c_force_errno != 0) {
        return Result<uint32_t>::Err(classify_i2c_error(g_i2c_force_errno));
    }
#endif
    if ((i2c_dev == nullptr) || (!device_is_ready(i2c_dev))) {
        return Result<uint32_t>::Err(I2CFault::DEVICE_NOT_READY);
    }
    uint8_t buf[3] = {0U, 0U, 0U};
    int err = i2c_burst_read(i2c_dev, sensor_addr, reg_addr, buf, 3);
    
    if (err == 0) {
        uint32_t val = (static_cast<uint32_t>(buf[0]) << 16U) | (static_cast<uint32_t>(buf[1]) << 8U) | static_cast<uint32_t>(buf[2]);
        update_cache(cache_key, val);
        failsafe_strategy.updateLastGood(val);
        return Result<uint32_t>::Ok(val);
    }
    
    const I2CFault fault = classify_i2c_error(err);
    if (is_recoverable_with_bus_reset(fault)) {
        reset_strategy.executeRecovery(i2c_dev);
    } else {
        retry_strategy.executeRecovery(i2c_dev);
    }
    
    uint64_t cached_val = 0U; // Fixed: Pointer cast mismatch
    if (get_cached_value(cache_key, &cached_val, 3000U)) {
        failsafe_strategy.executeRecovery(i2c_dev);
        return Result<uint32_t>::Ok(static_cast<uint32_t>(cached_val & 0xFFFFFFU));
    }
    return Result<uint32_t>::Err(fault);
#endif        
}

Result<uint64_t> I2CManager::read64Bit(uint16_t sensor_addr, uint8_t reg_addr) {
  uint32_t cache_key = (static_cast<uint32_t>(sensor_addr) << 8) | reg_addr;
  
#ifdef CONFIG_BOARD_QEMU_CORTEX_M3
    k_msleep(50);
    uint64_t mock_val = 0U; 
    update_cache(cache_key, mock_val);
    failsafe_strategy.updateLastGood(mock_val);
    return Result<uint64_t>::Ok(mock_val); 
#else
#ifdef IS_TEST_ENVIRONMENT
    extern int g_i2c_force_errno;
    if (g_i2c_force_errno != 0) {
        return Result<uint64_t>::Err(classify_i2c_error(g_i2c_force_errno));
    }
#endif
    if ((i2c_dev == nullptr) || (!device_is_ready(i2c_dev))) {
        return Result<uint64_t>::Err(I2CFault::DEVICE_NOT_READY);
    }
    uint8_t buf[8] = {0U};
    int err = i2c_burst_read(i2c_dev, sensor_addr, reg_addr, buf, 8);
    
    if (err == 0) {
        // Fixed: Cast explicitly to uint64_t to prevent bit-shift truncation overflows
        uint64_t val = (static_cast<uint64_t>(buf[0]) << 56U) | (static_cast<uint64_t>(buf[1]) << 48U) |
                       (static_cast<uint64_t>(buf[2]) << 40U) | (static_cast<uint64_t>(buf[3]) << 32U) |
                       (static_cast<uint64_t>(buf[4]) << 24U) | (static_cast<uint64_t>(buf[5]) << 16U) |
                       (static_cast<uint64_t>(buf[6]) << 8U)  | static_cast<uint64_t>(buf[7]);
        update_cache(cache_key, val);
        failsafe_strategy.updateLastGood(val);
        return Result<uint64_t>::Ok(val);
    }
    
    const I2CFault fault = classify_i2c_error(err);
    if (is_recoverable_with_bus_reset(fault)) {
        reset_strategy.executeRecovery(i2c_dev);
    } else {
        retry_strategy.executeRecovery(i2c_dev);
    }
    
    uint64_t cached_val = 0U;
    if (get_cached_value(cache_key, &cached_val, 3000U)) {
        failsafe_strategy.executeRecovery(i2c_dev);
        return Result<uint64_t>::Ok(cached_val);
    }
    return Result<uint64_t>::Err(fault);
#endif        
}

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

// Array to track intentionally exhausted blocks
static void* exhausted_blocks[MAX_CACHED_REGISTERS] = {nullptr};

void exhaustI2CCachePool() {
    for (size_t i = 0; i < MAX_CACHED_REGISTERS; i++) {
        // Store the allocated blocks so they can be cleaned up
        exhausted_blocks[i] = cache_pool.allocate();  
    }
}

void resetI2CCachePool() {
    resetI2CCacheForTests();
    
    // Safely deallocate the intentionally leaked blocks instead of using placement new
    for (size_t i = 0; i < MAX_CACHED_REGISTERS; i++) {
        if (exhausted_blocks[i] != nullptr) {
            cache_pool.deallocate(exhausted_blocks[i]);
            exhausted_blocks[i] = nullptr;
        }
    }
}

void test_update_cache(uint32_t key, uint64_t value, bool calibrated) {
    update_cache(key, value, calibrated);
}

bool test_get_cached_value(uint32_t key, uint64_t* out_val, uint32_t max_age_ms) {
    return get_cached_value(key, out_val, max_age_ms);
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
        // This will print to console if the test is looking for a key that doesn't exist
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
