#pragma once
// Private to the Power_*.cpp files. Not part of the public API.
// Prologue is identical to the original single .cpp so every file sees the same environment.

#include "Power_Management_System.h"
#include "Device_State_Machine+Watchdog.h"
#include "RTOS_Synchronization_Layer.h"
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/pm/pm.h>
#include <zephyr/pm/policy.h>
#include <zephyr/pm/device.h>
#include <zephyr/drivers/rtc.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <cerrno>
#include <cstring>

#ifdef IS_TEST_ENVIRONMENT
#ifndef CONFIG_BOARD_MPS2_AN386
#define CONFIG_BOARD_MPS2_AN386 1
#endif
#endif

#ifndef CONFIG_BOARD_MPS2_AN386
#include <soc.h>
#include <stm32f4xx.h>
#include <stm32f4xx_hal.h>
#endif

extern "C" {
    #include <zephyr/drivers/usb/usb_dc.h>
    #include <zephyr/usb/usb_device.h>
}

// ---- timing / board constants ----
constexpr uint32_t ACTIVE_TIMEOUT_MS = 30000; // 30s 
constexpr uint32_t IDLE_TIMEOUT_MS   = 5000;  // 5s
constexpr uint32_t STOP_TOTAL_TIME_US    = 60000000; // 60 seconds total
constexpr uint32_t STOP_WAKE_INTERVAL_US = 25000000; // 25 seconds wake interval (IWDG feed + alarm re-arm, ~0.6 ms @4 MHz)
constexpr uint32_t THREAD_PERIOD_MS  = 1000;
constexpr uint32_t USB_RECONNECT_DELAY_MS = 250;

constexpr uint32_t PM_LOG_FLUSH_MS = 120;

constexpr uint32_t IDLE_HCLK_DIV = 1;

#ifndef CONFIG_BOARD_MPS2_AN386
constexpr uint32_t APB1_STOP_KEEP_MASK = RCC_APB1ENR_PWREN;
constexpr uint32_t APB2_STOP_KEEP_MASK = 0U;

constexpr uint32_t IDLE_APB1_EXTRA_CLOCKS =
    RCC_APB1ENR_TIM2EN  | RCC_APB1ENR_TIM3EN  | RCC_APB1ENR_TIM4EN  | RCC_APB1ENR_TIM5EN  |   // 4 timers
    RCC_APB1ENR_TIM6EN  | RCC_APB1ENR_TIM7EN  | RCC_APB1ENR_TIM12EN | RCC_APB1ENR_TIM13EN |   // 4 timers
    RCC_APB1ENR_TIM14EN |                                                                      // 1 timer
    RCC_APB1ENR_SPI2EN  | RCC_APB1ENR_SPI3EN  |                                               // 2 SPI
    RCC_APB1ENR_USART2EN | RCC_APB1ENR_USART3EN | RCC_APB1ENR_UART4EN | RCC_APB1ENR_UART5EN | // 4 UART
    RCC_APB1ENR_I2C1EN  | RCC_APB1ENR_I2C2EN  | RCC_APB1ENR_I2C3EN;                           // 3 I2C
constexpr uint32_t IDLE_APB2_EXTRA_CLOCKS = 0U;   // e.g. RCC_APB2ENR_TIM1EN | RCC_APB2ENR_TIM8EN | RCC_APB2ENR_SPI1EN

// Active-state load. Burn share is already at its maximum (100 %), so the remaining headroom comes from
//  - the bus clocks below, now including OTG FS/HS(+ULPI), DAC and WWDG (clock only, never configured;
//    roughly 0.8 mA each at 180 MHz, DS10693 Table 32), and
//  - ACTIVE_BURN_HEAVY: false = plain spin (k_busy_wait), true = multiply/load/store loop on SRAM, which
//    toggles far more core logic than a spin (about +1..3 mA). Switch it on only if still short of 50 mA.
constexpr uint32_t ACTIVE_BURN_PERCENT   = 100;   // 0..100
constexpr uint32_t ACTIVE_BURN_PERIOD_MS = 10;
constexpr bool     ACTIVE_BURN_HEAVY     = true;

constexpr uint32_t ACTIVE_AHB1_EXTRA_CLOCKS =
    RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIOCEN | RCC_AHB1ENR_GPIODEN |
    RCC_AHB1ENR_GPIOEEN | RCC_AHB1ENR_GPIOFEN | RCC_AHB1ENR_GPIOGEN | RCC_AHB1ENR_GPIOHEN |
    RCC_AHB1ENR_CRCEN   | RCC_AHB1ENR_BKPSRAMEN | RCC_AHB1ENR_DMA1EN | RCC_AHB1ENR_DMA2EN |
    RCC_AHB1ENR_OTGHSEN | RCC_AHB1ENR_OTGHSULPIEN;
constexpr uint32_t ACTIVE_AHB2_EXTRA_CLOCKS = RCC_AHB2ENR_DCMIEN | RCC_AHB2ENR_OTGFSEN;
constexpr uint32_t ACTIVE_AHB3_EXTRA_CLOCKS = RCC_AHB3ENR_FMCEN | RCC_AHB3ENR_QSPIEN;
constexpr uint32_t ACTIVE_APB1_EXTRA_CLOCKS =
    RCC_APB1ENR_TIM2EN   | RCC_APB1ENR_TIM3EN   | RCC_APB1ENR_TIM4EN  | RCC_APB1ENR_TIM5EN  |
    RCC_APB1ENR_TIM6EN   | RCC_APB1ENR_TIM7EN   | RCC_APB1ENR_TIM12EN | RCC_APB1ENR_TIM13EN |
    RCC_APB1ENR_TIM14EN  | RCC_APB1ENR_SPI2EN   | RCC_APB1ENR_SPI3EN  | RCC_APB1ENR_SPDIFRXEN |
    RCC_APB1ENR_USART2EN | RCC_APB1ENR_USART3EN | RCC_APB1ENR_UART4EN | RCC_APB1ENR_UART5EN |
    RCC_APB1ENR_I2C1EN   | RCC_APB1ENR_I2C2EN   | RCC_APB1ENR_I2C3EN  | RCC_APB1ENR_FMPI2C1EN |
    RCC_APB1ENR_CAN1EN   | RCC_APB1ENR_CAN2EN   | RCC_APB1ENR_CECEN   |
    RCC_APB1ENR_DACEN    | RCC_APB1ENR_WWDGEN;
constexpr uint32_t ACTIVE_APB2_EXTRA_CLOCKS =
    RCC_APB2ENR_TIM1EN   | RCC_APB2ENR_TIM8EN   | RCC_APB2ENR_USART1EN | RCC_APB2ENR_USART6EN |
    RCC_APB2ENR_ADC1EN   | RCC_APB2ENR_ADC2EN   | RCC_APB2ENR_ADC3EN   | RCC_APB2ENR_SDIOEN   |
    RCC_APB2ENR_SPI1EN   | RCC_APB2ENR_SPI4EN   | RCC_APB2ENR_SYSCFGEN | RCC_APB2ENR_TIM9EN   |
    RCC_APB2ENR_TIM10EN  | RCC_APB2ENR_TIM11EN  | RCC_APB2ENR_SAI1EN   | RCC_APB2ENR_SAI2EN;
#else
constexpr uint32_t APB1_STOP_KEEP_MASK = 0U;
constexpr uint32_t APB2_STOP_KEEP_MASK = 0U;
constexpr uint32_t IDLE_APB1_EXTRA_CLOCKS = 0U;
constexpr uint32_t IDLE_APB2_EXTRA_CLOCKS = 0U;
constexpr uint32_t ACTIVE_BURN_PERCENT   = 0;
constexpr uint32_t ACTIVE_BURN_PERIOD_MS = 10;
constexpr bool     ACTIVE_BURN_HEAVY     = false;
#endif

#define STOP_APB_GATING_ENABLED 1

[[maybe_unused]] constexpr uint16_t STOP_AF_DRIVE_PIN_MASK[8] = {
    0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000
};

// ---- globals defined elsewhere ----
extern DeviceContext sys_context;   // application-owned
extern bool g_usb_connected;        // application-owned
#ifdef IS_TEST_ENVIRONMENT
extern bool g_force_init_fail;        // weak, defined in Power_Management_System.cpp
extern bool g_force_idle_enter_fail;  // weak, defined in Power_Management_System.cpp
#endif

// Defined in Power_Hardware.cpp (global names: the gtest file declares them extern).
extern struct k_work_delayable usb_reconnect_work;
#ifdef CONFIG_BOARD_MPS2_AN386
bool pm_device_suspend_tolerant(const struct device* dev, const char* name);
bool pm_device_resume_tolerant(const struct device* dev, const char* name);
#endif

namespace pm_internal {

// ---- register snapshots ----
struct GpioSnapshot {
    uint32_t moder[8];
    uint32_t pupdr[8];
    uint32_t otyper[8];
    uint32_t ospeedr[8];
    uint32_t odr[8];
    uint32_t afr[8][2];   // AFRL / AFRH, replayed through HAL_GPIO_Init() on restore
};

struct PeripheralClockSnapshot {
    uint32_t apb1enr;
    uint32_t apb2enr;
    uint32_t apb1lpenr;
    uint32_t apb2lpenr;
};

struct ClockSnapshot {
    uint32_t cr;
    uint32_t cfgr;
    uint32_t systick_ctrl;
    uint32_t systick_load;
    uint32_t flash_acr;
    uint32_t pwr_vos;
    uint32_t pwr_od;     // PWR_CR ODEN | ODSWEN as found before the downscale (0 if Over-drive was off)
};

struct AhbClockSnapshot {
    uint32_t ahb1enr;
    uint32_t ahb2enr;
    uint32_t ahb3enr;
    uint32_t ahb1lpenr;
    uint32_t ahb2lpenr;
    uint32_t ahb3lpenr;
};

struct AdcSnapshot {
    uint32_t cr1_1;
    uint32_t cr2_1;
#ifdef ADC2
    uint32_t cr1_2;
    uint32_t cr2_2;
#endif
#ifdef ADC3
    uint32_t cr1_3;
    uint32_t cr2_3;
#endif
    uint32_t ccr;
};

// ---- Power_Hardware.cpp: clocks, GPIO, ADC, peripheral gating, USB, thread suspension ----
#ifndef CONFIG_BOARD_MPS2_AN386
void gpio_save_and_set_all_analog(GpioSnapshot &snap, uint32_t &prev_ahb1enr);
void gpio_restore_after_wake(const GpioSnapshot &snap, uint32_t prev_ahb1enr);
void adc_save_and_disable(AdcSnapshot &snap);
void adc_restore_after_wake(const AdcSnapshot &snap);
void usb_disable_hardware_and_release_pins();
void usb_restore_pins_and_reenable();
void stop_configure_power(uint32_t &prev_pwr_cr);
void stop_restore_power(uint32_t prev_pwr_cr);
void stop_reassert_power();                              // re-apply STOP power bits right before each WFI
void stop_dbg_disable();                                 // HAL_DBGMCU_Disable*(): debug off in Sleep/Stop/Standby
HAL_StatusTypeDef stop_enter_wfi(uint32_t &out_pwr_csr_on_wake, uint32_t &out_icsr_on_wake); // STOP entry (executes WFI with PRIMASK=1)
void dac_save_and_disable(uint32_t &prev_cr);            // DAC output buffers keep drawing current in STOP
void dac_restore_after_wake(uint32_t prev_cr);
#endif
[[maybe_unused]] void periph_clocks_save_and_gate(PeripheralClockSnapshot &snap, uint32_t apb1_keep_mask, uint32_t apb2_keep_mask);
[[maybe_unused]] void periph_clocks_restore(const PeripheralClockSnapshot &snap);
void clock_save_state(ClockSnapshot &snap);
void clock_downscale_for_idle(ClockSnapshot &snap, uint32_t hclk_div = 4);   // HCLK = 16 MHz HSI / hclk_div (1, 2 or 4)
bool clock_restore_from_idle(const ClockSnapshot &snap);
void suspend_background_threads();
void resume_background_threads();

// ---- state shared between files ----
extern uint32_t s_idle_entry_time_ms;   // written by IdleState, read by the monitor thread
extern bool     s_in_idle_state;        // written by IdleState, read by the monitor thread
#ifndef CONFIG_BOARD_MPS2_AN386
extern uint32_t s_last_alraf_cycles;    // written by Power_RTC.cpp, logged by StopState::exit
extern uint32_t s_last_alraf_timeout;

extern uint16_t s_stop_od_af_mask[8];
extern uint16_t s_stop_od_low_mask[8];

extern uint32_t s_stop_applied_moder[8];
extern uint32_t s_stop_applied_pupdr[8];
// DBGMCU_CR read back right after the firmware cleared it (see stop_dbg_disable()). Logged on wake.
extern uint32_t s_stop_dbgmcu_after_write;
#endif

// Set by PowerManager::transitionTo() right before it calls old_state->exit(), so the exit
// logs can name the destination state.
extern const char* s_transition_target_name;
// True only while transitionTo() runs IdleState::exit() on the Idle -> STOP handoff.
extern bool        s_idle_exit_to_stop;

inline void pm_log_flush() {
#ifndef IS_TEST_ENVIRONMENT
    k_msleep(PM_LOG_FLUSH_MS);
#endif
}

} // namespace pm_internal
