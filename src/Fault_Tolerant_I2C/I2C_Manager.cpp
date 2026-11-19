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
    constexpr uint8_t MAX_I2C_ATTEMPTS = 3U;

    class I2CScopedLock {
    public:
        explicit I2CScopedLock(struct k_mutex* mutex) : m_mutex(mutex) {
            (void)k_mutex_lock(m_mutex, K_FOREVER);
        }
        ~I2CScopedLock() {
            (void)k_mutex_unlock(m_mutex);
        }
        I2CScopedLock(const I2CScopedLock&) = delete;
        I2CScopedLock& operator=(const I2CScopedLock&) = delete;
    private:
        struct k_mutex* m_mutex;
    };

    enum class TransferOp : uint8_t { ReadByte, WriteByte, WriteWord, BurstRead };

    int execute_transfer_once(TransferOp op, const device* dev, uint16_t addr,
                              uint8_t reg, uint8_t* buf, uint32_t len, uint8_t write_val) {
        if (op == TransferOp::ReadByte) {
            return i2c_reg_read_byte(dev, addr, reg, buf);
        }
        if (op == TransferOp::WriteByte) {
            return i2c_reg_write_byte(dev, addr, reg, write_val);
        }
        if (op == TransferOp::WriteWord) {
            return i2c_write(dev, buf, len, addr);
        }
        return i2c_burst_read(dev, addr, reg, buf, len);
    }

    int execute_with_retry(TransferOp op, const device* dev, uint16_t addr,
                           uint8_t reg, uint8_t* buf, uint32_t len, uint8_t write_val,
                           BusResetStrategy& reset_strategy, RetryStrategy& retry_strategy,
                           I2CFault& out_fault) {
        retry_strategy.resetBackoff();
        for (uint8_t attempt = 0U; attempt < MAX_I2C_ATTEMPTS; ++attempt) {
            const int err = execute_transfer_once(op, dev, addr, reg, buf, len, write_val);
            if (err == 0) {
                retry_strategy.resetBackoff();
                out_fault = I2CFault::NONE;
                return 0;
            }
            out_fault = classify_and_recover(err, dev, reset_strategy, retry_strategy);
        }
        return -EIO;
    }

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

    K_MUTEX_DEFINE(g_i2c_observer_registration_mutex);

    bool g_i2cObserverRegistered = false;

    bool ensure_power_observer_registered() {

        k_mutex_lock(
            &g_i2c_observer_registration_mutex,
            K_FOREVER
        );

        if (!g_i2cObserverRegistered) {

            g_i2cObserverRegistered =
                PowerManager::getInstance().registerObserver(
                    &g_i2cPowerObserver
                );
        }

        const bool registered = g_i2cObserverRegistered;

        k_mutex_unlock(&g_i2c_observer_registration_mutex);

        return registered;
    }

    constexpr uint32_t make_cache_key(uint16_t sensor_addr, uint8_t reg_addr) noexcept {
        return (static_cast<uint32_t>(sensor_addr) << 8U) | static_cast<uint32_t>(reg_addr);
    }

    bool is_bus_locked_out() {

        if (!ensure_power_observer_registered()) {
            LOG_ERR("I2C observer unavailable; refusing bus access");
            return true;
        }

        const SystemState st = sys_context.getState();

        return g_i2cPowerObserver.isSleeping() ||
               (st == SystemState::FAULT) ||
               (st == SystemState::SAFE_HALT);
    }

    bool is_device_unavailable(const device* i2c_dev) {
        return (i2c_dev == nullptr) || !device_is_ready(i2c_dev);
    }

    Result<bool> execute_write_transaction(TransferOp op, const device* i2c_dev,
                                           uint16_t sensor_addr, uint8_t reg_addr,
                                           uint8_t* tx_buf, uint32_t len, uint8_t write_val,
                                           BusResetStrategy& reset_strategy,
                                           RetryStrategy& retry_strategy) {
        if (is_bus_locked_out() || is_device_unavailable(i2c_dev)) {
            return Result<bool>::Err(I2CFault::DEVICE_NOT_READY);
        }
#ifdef IS_TEST_ENVIRONMENT
        if (g_i2c_force_errno != 0) {
            return Result<bool>::Err(classify_i2c_error(g_i2c_force_errno));
        }
#endif
        I2CFault fault = I2CFault::NONE;
        const int err = execute_with_retry(op, i2c_dev, sensor_addr, reg_addr,
                                           tx_buf, len, write_val, reset_strategy, retry_strategy, fault);
        if (err == 0) {
            return Result<bool>::Ok(true, k_uptime_get());
        }
        return Result<bool>::Err(fault);
    }
}

I2CManager::I2CManager(const device* i2c_dev) : i2c_dev(i2c_dev) {
    (void)k_mutex_init(&bus_mutex);
}

Result<uint8_t> I2CManager::readRegister(uint16_t sensor_addr, uint8_t reg_addr) {
    I2CScopedLock lock(&bus_mutex);
    const uint32_t cache_key = make_cache_key(sensor_addr, reg_addr);
    if (is_bus_locked_out() || is_device_unavailable(i2c_dev)) {
        return Result<uint8_t>::Err(I2CFault::DEVICE_NOT_READY);
    }
#ifdef IS_TEST_ENVIRONMENT
    if (g_i2c_force_errno != 0) {
        return Result<uint8_t>::Err(classify_i2c_error(g_i2c_force_errno));
    }
#endif

    uint8_t data = 0U;
    I2CFault fault = I2CFault::NONE;
    const int err = execute_with_retry(TransferOp::ReadByte, i2c_dev, sensor_addr, reg_addr,
                                       &data, 1U, 0U, reset_strategy, retry_strategy, fault);

    if (err == 0) {
        update_cache(cache_key, data);
        failsafe_strategy.updateLastGood(data);
        return Result<uint8_t>::Ok(data, k_uptime_get());
    }

    uint64_t cached_val = 0U;
    CacheReadMeta meta{};
    if (get_cached_value(cache_key, &cached_val, 3000U, meta)) {
        failsafe_strategy.executeRecovery(i2c_dev);
        return Result<uint8_t>::Degraded(static_cast<uint8_t>(cached_val & 0xFFU),
                                         fault, meta.timestamp_ms, meta.reliability_score);
    }
    return Result<uint8_t>::Err(fault);
}

Result<bool> I2CManager::writeRegister(uint16_t sensor_addr, uint8_t reg_addr, uint8_t val) {
    I2CScopedLock lock(&bus_mutex);
    return execute_write_transaction(TransferOp::WriteByte, i2c_dev, sensor_addr, reg_addr,
                                     nullptr, 1U, val, reset_strategy, retry_strategy);
}

Result<uint16_t> I2CManager::readWord(uint16_t sensor_addr, uint8_t reg_addr) {
    I2CScopedLock lock(&bus_mutex);
    const uint32_t cache_key = make_cache_key(sensor_addr, reg_addr);
    if (is_bus_locked_out() || is_device_unavailable(i2c_dev)) {
        return Result<uint16_t>::Err(I2CFault::DEVICE_NOT_READY);
    }
#ifdef IS_TEST_ENVIRONMENT
    if (g_i2c_force_errno != 0) {
        return Result<uint16_t>::Err(classify_i2c_error(g_i2c_force_errno));
    }
#endif
    uint8_t buf[2] = {0U, 0U};
    I2CFault fault = I2CFault::NONE;
    const int err = execute_with_retry(TransferOp::BurstRead, i2c_dev, sensor_addr, reg_addr,
                                       buf, 2U, 0U, reset_strategy, retry_strategy, fault);

    if (err == 0) {
        uint16_t val = static_cast<uint16_t>((static_cast<uint16_t>(buf[1]) << 8U) | static_cast<uint16_t>(buf[0]));
        update_cache(cache_key, val);
        failsafe_strategy.updateLastGood(val);
        return Result<uint16_t>::Ok(val, k_uptime_get());
    }

    uint64_t cached_val = 0U;
    CacheReadMeta meta{};
    if (get_cached_value(cache_key, &cached_val, 3000U, meta)) {
        failsafe_strategy.executeRecovery(i2c_dev);
        return Result<uint16_t>::Degraded(static_cast<uint16_t>(cached_val & 0xFFFFU),
                                          fault, meta.timestamp_ms, meta.reliability_score);
    }
    return Result<uint16_t>::Err(fault);
}

Result<bool> I2CManager::writeWord(uint16_t sensor_addr, uint8_t reg_addr, uint16_t val) {
    I2CScopedLock lock(&bus_mutex);
    uint8_t tx_buf[3] = {
        reg_addr,
        static_cast<uint8_t>((val >> 8U) & 0xFFU),
        static_cast<uint8_t>(val & 0xFFU)
    };
    return execute_write_transaction(TransferOp::WriteWord, i2c_dev, sensor_addr, reg_addr,
                                     tx_buf, 3U, 0U, reset_strategy, retry_strategy);
}

Result<uint32_t> I2CManager::read24Bit(uint16_t sensor_addr, uint8_t reg_addr) {
    I2CScopedLock lock(&bus_mutex);
    const uint32_t cache_key = make_cache_key(sensor_addr, reg_addr);
    if (is_bus_locked_out() || is_device_unavailable(i2c_dev)) {
        return Result<uint32_t>::Err(I2CFault::DEVICE_NOT_READY);
    }
#ifdef IS_TEST_ENVIRONMENT
    if (g_i2c_force_errno != 0) {
        return Result<uint32_t>::Err(classify_i2c_error(g_i2c_force_errno));
    }
#endif
    uint8_t buf[3] = {0U, 0U, 0U};
    I2CFault fault = I2CFault::NONE;
    const int err = execute_with_retry(TransferOp::BurstRead, i2c_dev, sensor_addr, reg_addr,
                                       buf, 3U, 0U, reset_strategy, retry_strategy, fault);

    if (err == 0) {
        uint32_t val = (static_cast<uint32_t>(buf[0]) << 16U) | (static_cast<uint32_t>(buf[1]) << 8U) | static_cast<uint32_t>(buf[2]);
        update_cache(cache_key, val);
        failsafe_strategy.updateLastGood(val);
        return Result<uint32_t>::Ok(val, k_uptime_get());
    }

    uint64_t cached_val = 0U;
    CacheReadMeta meta{};
    if (get_cached_value(cache_key, &cached_val, 3000U, meta)) {
        failsafe_strategy.executeRecovery(i2c_dev);
        return Result<uint32_t>::Degraded(static_cast<uint32_t>(cached_val & 0xFFFFFFU),
                                          fault, meta.timestamp_ms, meta.reliability_score);
    }
    return Result<uint32_t>::Err(fault);
}

Result<uint64_t> I2CManager::read64Bit(uint16_t sensor_addr, uint8_t reg_addr) {
    I2CScopedLock lock(&bus_mutex);
    const uint32_t cache_key = make_cache_key(sensor_addr, reg_addr);
    if (is_bus_locked_out() || is_device_unavailable(i2c_dev)) {
        return Result<uint64_t>::Err(I2CFault::DEVICE_NOT_READY);
    }
#ifdef IS_TEST_ENVIRONMENT
    if (g_i2c_force_errno != 0) {
        return Result<uint64_t>::Err(classify_i2c_error(g_i2c_force_errno));
    }
#endif
    uint8_t buf[8] = {0U};
    I2CFault fault = I2CFault::NONE;
    const int err = execute_with_retry(TransferOp::BurstRead, i2c_dev, sensor_addr, reg_addr,
                                       buf, 8U, 0U, reset_strategy, retry_strategy, fault);

    if (err == 0) {
        uint64_t val = (static_cast<uint64_t>(buf[0]) << 56U) | (static_cast<uint64_t>(buf[1]) << 48U) |
                       (static_cast<uint64_t>(buf[2]) << 40U) | (static_cast<uint64_t>(buf[3]) << 32U) |
                       (static_cast<uint64_t>(buf[4]) << 24U) | (static_cast<uint64_t>(buf[5]) << 16U) |
                       (static_cast<uint64_t>(buf[6]) << 8U)  | static_cast<uint64_t>(buf[7]);
        update_cache(cache_key, val);
        failsafe_strategy.updateLastGood(val);
        return Result<uint64_t>::Ok(val, k_uptime_get());
    }

    uint64_t cached_val = 0U;
    CacheReadMeta meta{};
    if (get_cached_value(cache_key, &cached_val, 3000U, meta)) {
        failsafe_strategy.executeRecovery(i2c_dev);
        return Result<uint64_t>::Degraded(cached_val, fault, meta.timestamp_ms, meta.reliability_score);
    }
    return Result<uint64_t>::Err(fault);
}
#ifdef IS_TEST_ENVIRONMENT
void resetI2CObserverRegistrationForTests() noexcept {
    k_mutex_lock(&g_i2c_observer_registration_mutex, K_FOREVER);
    g_i2cObserverRegistered = false;
    k_mutex_unlock(&g_i2c_observer_registration_mutex);
}
#endif
