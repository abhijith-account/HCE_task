#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#if !defined(CONFIG_BOARD_MPS2_AN386) && !defined(IS_TEST_ENVIRONMENT)
#include <stm32f4xx_hal.h>
#endif
#include <cerrno>
#include <cstdint>
#include "Fault_Tolerant_I2C_Communication_Layer.h"
#include "I2C_Internal.h"

LOG_MODULE_DECLARE(I2C_HW, LOG_LEVEL_INF);

namespace i2c_internal {
#ifdef IS_TEST_ENVIRONMENT
MockGpioPort g_mock_gpiob{};

void reset_mock_gpiob() noexcept {
    g_mock_gpiob = MockGpioPort{};
}
#endif

I2CFault classify_i2c_error(int err) {
    switch (err) {
        case -ETIMEDOUT: return I2CFault::TIMEOUT;
        case -EBUSY: return I2CFault::BUS_BUSY;
        case -EAGAIN: return I2CFault::ARBITRATION_LOST;
        case -ENODEV: return I2CFault::DEVICE_NOT_READY;
        default: return I2CFault::NACK;
    }
}

bool is_recoverable_with_bus_reset(I2CFault fault) {
    return (fault == I2CFault::TIMEOUT) || (fault == I2CFault::BUS_BUSY);
}

I2CFault classify_and_recover(int err, const device* i2c_dev, BusResetStrategy& reset_strategy, RetryStrategy& retry_strategy) {
    const I2CFault fault = classify_i2c_error(err);
    if (is_recoverable_with_bus_reset(fault)) {
        reset_strategy.executeRecovery(i2c_dev);
    } else {
        retry_strategy.executeRecovery(i2c_dev);
    }
    return fault;
}
} // namespace i2c_internal

#if !defined(CONFIG_BOARD_MPS2_AN386)
namespace {
inline void gpiob_enable_clock() noexcept {
#ifdef IS_TEST_ENVIRONMENT
    i2c_internal::g_mock_gpiob.rcc_gpiob_clk_enabled = true;
#else
    __HAL_RCC_GPIOB_CLK_ENABLE();
#endif
}

inline auto* gpiob_regs() noexcept {
#ifdef IS_TEST_ENVIRONMENT
    return &i2c_internal::g_mock_gpiob;
#else
    return GPIOB;
#endif
}

inline void gpiob_write_bsrr(uint32_t mask) noexcept {
    gpiob_regs()->BSRR = mask;
#ifdef IS_TEST_ENVIRONMENT
    i2c_internal::g_mock_gpiob.bsrr_log[i2c_internal::g_mock_gpiob.bsrr_count & 31U] = mask;
    ++i2c_internal::g_mock_gpiob.bsrr_count;
#endif
}
} // namespace
#endif

void RetryStrategy::executeRecovery(const device*) {
    const uint32_t current = backoff_ms.load();
    LOG_WRN("I2C Fault Detected. Executing Exponential Backoff Retry... (%u ms)", current);
    k_msleep(static_cast<int32_t>(current));
    if (current < MAX_BACKOFF_MS) {
        backoff_ms.store(current * 2U);
    }
}

void RetryStrategy::resetBackoff() noexcept { backoff_ms.store(INITIAL_BACKOFF_MS); }
uint32_t RetryStrategy::getBackoffMs() const noexcept { return backoff_ms.load(); }

void BusResetStrategy::executeRecovery(const device* i2c_dev) {
    LOG_ERR("I2C Bus Hard-Locked. Toggling SCL to recover");
#if !defined(CONFIG_BOARD_MPS2_AN386)
    /* Hardware 9-clock open-drain SCL recovery on PB8 (SCL) / PB9 (SDA) + STOP condition */
    gpiob_enable_clock();
    auto* const port = gpiob_regs();
    port->OTYPER |= (1U << 8) | (1U << 9);
    port->PUPDR = (port->PUPDR & ~((3U << 16) | (3U << 18))) | (1U << 16) | (1U << 18);
    gpiob_write_bsrr((1U << 8) | (1U << 9));
    port->MODER = (port->MODER & ~((3U << 16) | (3U << 18))) | (1U << 16);
    for (int pulse = 0; pulse < 9; ++pulse) {
        gpiob_write_bsrr(1U << (8 + 16));
        k_busy_wait(5);
        gpiob_write_bsrr(1U << 8);
        k_busy_wait(5);
    }
    port->MODER = (port->MODER & ~(3U << 18)) | (1U << 18);
    gpiob_write_bsrr(1U << (9 + 16));
    k_busy_wait(5);
    gpiob_write_bsrr(1U << 8);
    k_busy_wait(5);
    gpiob_write_bsrr(1U << 9);
    k_busy_wait(5);
    port->MODER = (port->MODER & ~((3U << 16) | (3U << 18))) | (2U << 16) | (2U << 18);
#endif
    (void)i2c_recover_bus(i2c_dev);
}

void FailSafeStrategy::executeRecovery(const device*) {
    LOG_ERR("I2C Critical Failure. Falling back to Last-known-Good data: %llu",
            static_cast<unsigned long long>(last_known_good_value));
}

void FailSafeStrategy::updateLastGood(uint64_t val) { last_known_good_value = val; }
uint64_t FailSafeStrategy::getLastGood() const { return last_known_good_value; }
