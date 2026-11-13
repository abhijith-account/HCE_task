// I2C_Manager.cpp
// I2CManager: the fault-tolerant bus transactions - readRegister / writeRegister / readWord / writeWord / read24Bit / read64Bit.
// Each transaction: power-observer + SAFE_HALT/FAULT lock-out, device check, the I2C call, then on error classify the fault
// (classify_i2c_error), pick a recovery strategy (I2C_Recovery_Strategies.cpp) and fall back to the cache (I2C_Fallback_Cache.cpp).
// Also owns the I2C hardware handle and the global i2c_manager instance on target builds.

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

LOG_MODULE_REGISTER(I2C_HW, LOG_LEVEL_INF);

using namespace i2c_internal;

#ifndef IS_TEST_ENVIRONMENT
#ifdef CONFIG_BOARD_MPS2_AN386
const struct device *i2c_hardware = DEVICE_DT_GET(DT_ALIAS(i2c1));
#else
const struct device *i2c_hardware = DEVICE_DT_GET(DT_NODELABEL(i2c1));
#endif
#endif

#ifndef IS_TEST_ENVIRONMENT
I2CManager i2c_manager(i2c_hardware);
#endif

namespace {
    class I2CPowerObserver final : public IPowerObserver {
    private:
        atomic_t is_sleeping{};
    public:
        I2CPowerObserver() { atomic_set(&is_sleeping, 0); }
        void beforeSleep() override { atomic_set(&is_sleeping, 1); }
        void afterWakeup() override { atomic_set(&is_sleeping, 0); }
        void sleepAborted() override { atomic_set(&is_sleeping, 0); }
        bool isSleeping() const noexcept { return atomic_get(&is_sleeping) != 0; }
    };

    I2CPowerObserver g_i2cPowerObserver;
    atomic_t g_i2cObserverRegistered = ATOMIC_INIT(0);

    void ensure_power_observer_registered() {
        if (atomic_cas(&g_i2cObserverRegistered, 0, 1)) {
            PowerManager::getInstance().registerObserver(&g_i2cPowerObserver);
        }
    }
    
    constexpr uint32_t make_cache_key(uint16_t sensor_addr, uint8_t reg_addr) noexcept {
        return (static_cast<uint32_t>(sensor_addr) << 8U) | static_cast<uint32_t>(reg_addr);
    }

    bool is_bus_locked_out() {
        ensure_power_observer_registered();
        const SystemState st = sys_context.getState();
        return g_i2cPowerObserver.isSleeping() || (st == SystemState::FAULT) || (st == SystemState::SAFE_HALT);
    }

    bool is_device_unavailable(const device* i2c_dev) {
        return (i2c_dev == nullptr) || !device_is_ready(i2c_dev);
    }
}

I2CManager::I2CManager(const device* i2c_dev) : i2c_dev(i2c_dev) {}

Result<uint8_t> I2CManager::readRegister(uint16_t sensor_addr, uint8_t reg_addr) {
    const uint32_t cache_key = (static_cast<uint32_t>(sensor_addr) << 8U) | static_cast<uint32_t>(reg_addr);
    ensure_power_observer_registered();
    
    // SAFE HALT: Complete lock out of physical bus. No polling.
    if (g_i2cPowerObserver.isSleeping() || sys_context.getState() == SystemState::FAULT || sys_context.getState() == SystemState::SAFE_HALT) {
        return Result<uint8_t>::Err(I2CFault::DEVICE_NOT_READY);
    }

    const uint32_t cache_key = make_cache_key(sensor_addr, reg_addr);
    if (is_bus_locked_out()||is_device_unavailable(i2c_dev)) {
        return Result<uint8_t>::Err(I2CFault::DEVICE_NOT_READY);
    }
#ifndef IS_TEST_ENVIRONMENT
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

    const I2CFault fault = classify_and_recover(err, i2c_dev, reset_strategy, retry_strategy);

    uint64_t cached_val = 0U;
    if (get_cached_value(cache_key, &cached_val, 3000U)) {
       failsafe_strategy.executeRecovery(i2c_dev);
       return Result<uint8_t>::Ok(static_cast<uint8_t>(cached_val & 0xFFU));
    }
    return Result<uint8_t>::Err(fault);
}

Result<bool> I2CManager::writeRegister(uint16_t sensor_addr, uint8_t reg_addr, uint8_t val) {
    ensure_power_observer_registered();
    
    if (g_i2cPowerObserver.isSleeping() || sys_context.getState() == SystemState::FAULT || sys_context.getState() == SystemState::SAFE_HALT) {
        return Result<bool>::Err(I2CFault::DEVICE_NOT_READY);
    }

    const uint32_t cache_key = make_cache_key(sensor_addr, reg_addr);
    if (is_bus_locked_out()||is_device_unavailable(i2c_dev)) {
        return Result<uint8_t>::Err(I2CFault::DEVICE_NOT_READY);
    }
#ifndef IS_TEST_ENVIRONMENT
    extern int g_i2c_force_errno;
    if (g_i2c_force_errno != 0) {
        return Result<uint8_t>::Err(classify_i2c_error(g_i2c_force_errno));
    }
#endif

    if ((i2c_dev == nullptr) || (!device_is_ready(i2c_dev))) {
        return Result<bool>::Err(I2CFault::DEVICE_NOT_READY);
    }

    const int err = i2c_reg_write_byte(i2c_dev, sensor_addr, reg_addr, val);

    if (err == 0) {
        return Result<bool>::Ok(true);
    }

    const I2CFault fault = classify_and_recover(err, i2c_dev, reset_strategy, retry_strategy);

    return Result<bool>::Err(fault);
}

Result<uint16_t> I2CManager::readWord(uint16_t sensor_addr, uint8_t reg_addr) {
    uint32_t cache_key = (static_cast<uint32_t>(sensor_addr) << 8) | reg_addr;
    ensure_power_observer_registered();
    
    if (g_i2cPowerObserver.isSleeping() || sys_context.getState() == SystemState::FAULT || sys_context.getState() == SystemState::SAFE_HALT) {
        return Result<uint16_t>::Err(I2CFault::DEVICE_NOT_READY);
    }

    const uint32_t cache_key = make_cache_key(sensor_addr, reg_addr);
    if (is_bus_locked_out()||is_device_unavailable(i2c_dev)) {
        return Result<uint8_t>::Err(I2CFault::DEVICE_NOT_READY);
    }
#ifndef IS_TEST_ENVIRONMENT
    extern int g_i2c_force_errno;
    if (g_i2c_force_errno != 0) {
        return Result<uint8_t>::Err(classify_i2c_error(g_i2c_force_errno));
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

    const I2CFault fault = classify_and_recover(err, i2c_dev, reset_strategy, retry_strategy);

    uint64_t cached_val = 0U;
    if (get_cached_value(cache_key, &cached_val, 3000U)) {
        failsafe_strategy.executeRecovery(i2c_dev);
        return Result<uint16_t>::Ok(static_cast<uint16_t>(cached_val & 0xFFFFU));
    }
    return Result<uint16_t>::Err(fault);
}

Result<bool> I2CManager::writeWord(uint16_t sensor_addr, uint8_t reg_addr, uint16_t val) {
    uint8_t high = static_cast<uint8_t>((val >> 8) & 0xFF);
    uint8_t low  = static_cast<uint8_t>(val & 0xFF);

    Result<bool> res1 = writeRegister(sensor_addr, reg_addr, high);
    if (!res1.isOk()) return res1;
    return writeRegister(sensor_addr, reg_addr + 1, low);
}

Result<uint32_t> I2CManager::read24Bit(uint16_t sensor_addr, uint8_t reg_addr) {
    uint32_t cache_key = (static_cast<uint32_t>(sensor_addr) << 8) | static_cast<uint32_t>(reg_addr);
    ensure_power_observer_registered();
    
    if (g_i2cPowerObserver.isSleeping() || sys_context.getState() == SystemState::FAULT || sys_context.getState() == SystemState::SAFE_HALT) {
        return Result<uint32_t>::Err(I2CFault::DEVICE_NOT_READY);
    }

    const uint32_t cache_key = make_cache_key(sensor_addr, reg_addr);
    if (is_bus_locked_out()||is_device_unavailable(i2c_dev)) {
        return Result<uint8_t>::Err(I2CFault::DEVICE_NOT_READY);
    }
#ifndef IS_TEST_ENVIRONMENT
    extern int g_i2c_force_errno;
    if (g_i2c_force_errno != 0) {
        return Result<uint8_t>::Err(classify_i2c_error(g_i2c_force_errno));
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

    const I2CFault fault = classify_and_recover(err, i2c_dev, reset_strategy, retry_strategy);

    uint64_t cached_val = 0U;
    if (get_cached_value(cache_key, &cached_val, 3000U)) {
        failsafe_strategy.executeRecovery(i2c_dev);
        return Result<uint32_t>::Ok(static_cast<uint32_t>(cached_val & 0xFFFFFFU));
    }
    return Result<uint32_t>::Err(fault);
}

Result<uint64_t> I2CManager::read64Bit(uint16_t sensor_addr, uint8_t reg_addr) {
    uint32_t cache_key = (static_cast<uint32_t>(sensor_addr) << 8) | reg_addr;
    ensure_power_observer_registered();
    
    if (g_i2cPowerObserver.isSleeping() || sys_context.getState() == SystemState::FAULT || sys_context.getState() == SystemState::SAFE_HALT) {
        return Result<uint64_t>::Err(I2CFault::DEVICE_NOT_READY);
    }
    
    const uint32_t cache_key = make_cache_key(sensor_addr, reg_addr);
    if (is_bus_locked_out()||is_device_unavailable(i2c_dev)) {
        return Result<uint8_t>::Err(I2CFault::DEVICE_NOT_READY);
    }
#ifndef IS_TEST_ENVIRONMENT
    extern int g_i2c_force_errno;
    if (g_i2c_force_errno != 0) {
        return Result<uint8_t>::Err(classify_i2c_error(g_i2c_force_errno));
    }
#endif

    if ((i2c_dev == nullptr) || (!device_is_ready(i2c_dev))) {
        return Result<uint64_t>::Err(I2CFault::DEVICE_NOT_READY);
    }
    uint8_t buf[8] = {0U};
    int err = i2c_burst_read(i2c_dev, sensor_addr, reg_addr, buf, 8);

    if (err == 0) {
        uint64_t val = (static_cast<uint64_t>(buf[0]) << 56U) | (static_cast<uint64_t>(buf[1]) << 48U) |
                       (static_cast<uint64_t>(buf[2]) << 40U) | (static_cast<uint64_t>(buf[3]) << 32U) |
                       (static_cast<uint64_t>(buf[4]) << 24U) | (static_cast<uint64_t>(buf[5]) << 16U) |
                       (static_cast<uint64_t>(buf[6]) << 8U)  | static_cast<uint64_t>(buf[7]);
        update_cache(cache_key, val);
        failsafe_strategy.updateLastGood(val);
        return Result<uint64_t>::Ok(val);
    }

    const I2CFault fault = classify_and_recover(err, i2c_dev, reset_strategy, retry_strategy);

    uint64_t cached_val = 0U;
    if (get_cached_value(cache_key, &cached_val, 3000U)) {
        failsafe_strategy.executeRecovery(i2c_dev);
        return Result<uint64_t>::Ok(cached_val);
    }
    return Result<uint64_t>::Err(fault);
}
