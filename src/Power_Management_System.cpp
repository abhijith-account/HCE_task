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

LOG_MODULE_REGISTER(PWR_SYS, LOG_LEVEL_INF);

extern bool g_usb_connected;
bool s_real_hw_sleep_active = false;
uint32_t s_elapsed_sleep_time_us = 0;
uint32_t s_current_sleep_interval_us = 0;
atomic_t s_activity_pending = ATOMIC_INIT(0);
volatile StopWakeReason s_stop_wake_reason = StopWakeReason::WAKE_NONE;
K_SEM_DEFINE(pm_wake_sem, 0, 1);
K_SEM_DEFINE(stop_wake_sem, 0, 1);
#ifdef IS_TEST_ENVIRONMENT
    extern bool run_thread_once;
    __attribute__((weak)) bool g_force_init_fail = false;
    __attribute__((weak)) bool g_force_idle_enter_fail = false;
    #define THREAD_LOOP_CONDITION (run_thread_once ? (run_thread_once = false, true) : false)
#else
    #define THREAD_LOOP_CONDITION true
#endif

constexpr uint32_t ACTIVE_TIMEOUT_MS = 30000; // 30s 
constexpr uint32_t IDLE_TIMEOUT_MS   = 5000;  // 5s
constexpr uint32_t STOP_TOTAL_TIME_US    = 60000000; // 60 seconds total
constexpr uint32_t STOP_WAKE_INTERVAL_US = 25000000; // 25 seconds wake interval (real-HW/RTC path only)
constexpr uint32_t THREAD_PERIOD_MS  = 1000;
constexpr uint32_t USB_RECONNECT_DELAY_MS = 250;

#ifndef CONFIG_BOARD_MPS2_AN386
constexpr uint32_t APB1_STOP_KEEP_MASK = RCC_APB1ENR_PWREN;
constexpr uint32_t APB2_STOP_KEEP_MASK = 0U;
#else
constexpr uint32_t APB1_STOP_KEEP_MASK = 0U;
constexpr uint32_t APB2_STOP_KEEP_MASK = 0U;
#endif

#define STOP_APB_GATING_ENABLED 1

#ifdef CONFIG_BOARD_MPS2_AN386

static void simulated_stop_timer_handler(struct k_timer *timer_id)
{
    ARG_UNUSED(timer_id);

    auto& pm = PowerManager::getInstance();

    atomic_set(&pm.wake_pending, 1);

    k_sem_give(&pm_wake_sem);
    k_sem_give(&stop_wake_sem);
}
#else

static void simulated_rtc_timer_handler(struct k_timer *timer_id)
{
    ARG_UNUSED(timer_id);
    auto& pm = PowerManager::getInstance();
    atomic_set(&pm.wake_pending, 1);
    k_sem_give(&pm_wake_sem);
    k_sem_give(&stop_wake_sem);
}
#endif

namespace {
#ifndef CONFIG_BOARD_MPS2_AN386

void usb_disable_hardware_and_release_pins() {
    __HAL_RCC_USB_OTG_FS_CLK_ENABLE();
    
    // 0 = Transceiver powered DOWN. 
    // This is the active-low bit I misled you on. This MUST be 0.
    USB_OTG_FS->GCCFG = 0; 
    
    // Force a dummy read so the bus matrix completes the write before the clock dies.
    volatile uint32_t dummy = USB_OTG_FS->GCCFG; 
    (void)dummy;
    
    __HAL_RCC_USB_OTG_FS_CLK_DISABLE();
}

void usb_restore_pins_and_reenable() {
    __HAL_RCC_USB_OTG_FS_CLK_ENABLE();
    
    // 1 = Transceiver Active (ON). We must re-enable it before the stack resumes.
    SET_BIT(USB_OTG_FS->GCCFG, USB_OTG_GCCFG_PWRDWN);

    __HAL_RCC_GPIOA_CLK_ENABLE();

    GPIOA->AFR[1]   = (GPIOA->AFR[1] & ~(0xFFU << 12)) | (0xAAU << 12);
    GPIOA->MODER    = (GPIOA->MODER & ~(0xFU << 22)) | (0xAU << 22);
    GPIOA->OSPEEDR |= (0xFU << 22);  // Very High Speed
    GPIOA->PUPDR   &= ~(0xFU << 22); // No Pull
    
    k_msleep(100);
}

#endif

void usb_reconnect_work_handler(struct k_work *work) {
    ARG_UNUSED(work);

#ifndef CONFIG_BOARD_MPS2_AN386
    usb_restore_pins_and_reenable();
#ifdef CONFIG_USB_DEVICE_STACK
    int err = usb_dc_attach();
    if (err == 0) {
        LOG_INF("USB re-attached after STOP wake; awaiting host re-enumeration");
    } else {
        LOG_ERR("Failed to re-attach USB after STOP wake (err %d)", err);
    }
#endif
#endif
}

struct GpioSnapshot {
    uint32_t moder[8];
    uint32_t pupdr[8];
};

#ifndef CONFIG_BOARD_MPS2_AN386
GPIO_TypeDef* const GPIO_PORTS[8] = {GPIOA, GPIOB, GPIOC, GPIOD, GPIOE, GPIOF, GPIOG, GPIOH};

void gpio_save_and_set_all_analog(GpioSnapshot &snap, uint32_t &prev_ahb1enr) {
    prev_ahb1enr = RCC->AHB1ENR;

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOE_CLK_ENABLE();
    __HAL_RCC_GPIOF_CLK_ENABLE();
    __HAL_RCC_GPIOG_CLK_ENABLE();
    __HAL_RCC_GPIOH_CLK_ENABLE();
    __DSB();

    for (int port = 0; port < 8; ++port) {
        GPIO_TypeDef* gpio = GPIO_PORTS[port];
        
        snap.moder[port] = gpio->MODER;
        snap.pupdr[port] = gpio->PUPDR;
        
        uint32_t new_moder = snap.moder[port];
        uint32_t new_pupdr = 0U;

        for (int pin = 0; pin < 16; ++pin) {
            // 1. Preserve SWD debug pins
            if (port == 0 && (pin == 13 || pin == 14)) {
                new_pupdr |= (snap.pupdr[port] & (0x3U << (pin * 2)));
                continue; 
            }
            
            // 2. Force USB pins to Analog to prevent host fights
            if (port == 0 && (pin == 11 || pin == 12)) {
                new_moder |= (0x3U << (pin * 2)); 
                continue;
            }

            uint32_t pin_mode = (snap.moder[port] >> (pin * 2)) & 0x3U;

            if (pin_mode == 0x01U) {
                // OUTPUT: Preserve exactly
                new_pupdr |= (snap.pupdr[port] & (0x3U << (pin * 2)));
            } 
            else if (pin_mode == 0x02U) {
                // ALTERNATE FUNCTION: The IDR Freeze
                // Read the actual physical idle voltage currently on the pin.
                uint32_t current_level = (gpio->IDR >> pin) & 0x1U;
                
                // Write that exact state to ODR so it holds strongly during sleep
                if (current_level) {
                    gpio->ODR |= (1U << pin);
                } else {
                    gpio->ODR &= ~(1U << pin);
                }
                
                // Change mode to Output (01)
                new_moder &= ~(0x3U << (pin * 2));
                new_moder |=  (0x1U << (pin * 2));
                
                // Preserve pull resistors
                new_pupdr |= (snap.pupdr[port] & (0x3U << (pin * 2)));
            } 
            else {
                // INPUT / ANALOG: Force to Analog (11)
                new_moder |= (0x3U << (pin * 2));
            }
        }

        gpio->PUPDR = new_pupdr;
        gpio->MODER = new_moder;
    }

    __DSB();

#ifdef CONFIG_DEBUG
    RCC->AHB1ENR = (RCC->AHB1ENR & ~0x000000FFU) | (prev_ahb1enr & 0x000000FFU) | RCC_AHB1ENR_GPIOAEN;
#else
    RCC->AHB1ENR = (RCC->AHB1ENR & ~0x000000FFU) | (prev_ahb1enr & 0x000000FFU);
#endif
}

void gpio_restore_after_wake(const GpioSnapshot &snap, uint32_t prev_ahb1enr) {
    RCC->AHB1ENR |= 0x000000FFU;
    __DSB();

    for (int i = 0; i < 8; ++i) {
        GPIO_PORTS[i]->MODER = snap.moder[i];
        GPIO_PORTS[i]->PUPDR = snap.pupdr[i];
    }
    
    RCC->AHB1ENR = prev_ahb1enr;
}

void adc_save_and_disable(uint32_t &prev_cr2, uint32_t &prev_ccr) {
    __HAL_RCC_ADC1_CLK_ENABLE();
    prev_cr2 = ADC1->CR2;
    prev_ccr = ADC123_COMMON->CCR;

    CLEAR_BIT(ADC1->CR2, ADC_CR2_ADON);           // Clear ADON
    CLEAR_BIT(ADC123_COMMON->CCR, ADC_CCR_TSVREFE); // Clear TSVREFE (VREFINT + temp sensor buffer)
    
    volatile uint32_t dummy = ADC1->CR2; // Flush APB write
    (void)dummy;
}

void adc_restore_after_wake(uint32_t prev_cr2, uint32_t prev_ccr) {
    ADC123_COMMON->CCR = prev_ccr;
    ADC1->CR2 = prev_cr2;
}
#endif

struct PeripheralClockSnapshot {
    uint32_t apb1enr;
    uint32_t apb2enr;
};

[[maybe_unused]] void periph_clocks_save_and_gate(PeripheralClockSnapshot &snap, uint32_t apb1_keep_mask, uint32_t apb2_keep_mask) {
#ifndef CONFIG_BOARD_MPS2_AN386
    snap.apb1enr = RCC->APB1ENR;
    snap.apb2enr = RCC->APB2ENR;

    RCC->APB1ENR &= apb1_keep_mask;
    RCC->APB2ENR &= apb2_keep_mask;
    __DSB();
#endif
}

[[maybe_unused]] void periph_clocks_restore(const PeripheralClockSnapshot &snap) {
#ifndef CONFIG_BOARD_MPS2_AN386
    RCC->APB1ENR = snap.apb1enr;
    RCC->APB2ENR = snap.apb2enr;
    __DSB();
#endif
}

struct ClockSnapshot {
    uint32_t cr;
    uint32_t cfgr;
    uint32_t systick_ctrl;
    uint32_t flash_acr;
    uint32_t pwr_vos;
};

void clock_save_state(ClockSnapshot &snap) {
#ifndef CONFIG_BOARD_MPS2_AN386
    snap.cr           = RCC->CR;
    snap.cfgr         = RCC->CFGR;
    snap.systick_ctrl = SysTick->CTRL;
    snap.flash_acr    = FLASH->ACR;
    snap.pwr_vos      = PWR->CR & PWR_CR_VOS;
#endif
}

void clock_downscale_for_idle(ClockSnapshot &snap) {
    clock_save_state(snap);

#ifndef CONFIG_BOARD_MPS2_AN386
    __HAL_RCC_HSI_ENABLE();
    uint32_t timeout = 100000;
    while (!__HAL_RCC_GET_FLAG(RCC_FLAG_HSIRDY) && --timeout) {}
    
    MODIFY_REG(RCC->CFGR, RCC_CFGR_HPRE, RCC_CFGR_HPRE_DIV4);
    
    __HAL_RCC_SYSCLK_CONFIG(RCC_SYSCLKSOURCE_HSI);
    timeout = 100000;
    while (__HAL_RCC_GET_SYSCLK_SOURCE() != RCC_SYSCLKSOURCE_STATUS_HSI && --timeout) {}

    __HAL_FLASH_SET_LATENCY(FLASH_LATENCY_0);
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);
    __HAL_RCC_PLL_DISABLE();
    
    if (snap.cr & RCC_CR_HSEON) {
        __HAL_RCC_HSE_CONFIG(RCC_HSE_OFF);
    }
#endif
}

bool clock_restore_from_idle(const ClockSnapshot &snap)
{
#ifndef CONFIG_BOARD_MPS2_AN386
    MODIFY_REG(PWR->CR, PWR_CR_VOS, snap.pwr_vos);
    MODIFY_REG(RCC->CFGR, RCC_CFGR_HPRE, snap.cfgr & RCC_CFGR_HPRE);
    FLASH->ACR = snap.flash_acr;

    if (snap.cr & RCC_CR_HSEON) {
        __HAL_RCC_HSE_CONFIG(RCC_HSE_ON);

        uint32_t timeout = 1000000;
        while (!__HAL_RCC_GET_FLAG(RCC_FLAG_HSERDY) && --timeout) {
        }

        if (timeout == 0) {
            return false;
        }
    }

    if (snap.cr & RCC_CR_PLLON) {
        __HAL_RCC_PLL_ENABLE();

        uint32_t timeout = 1000000;
        while (!__HAL_RCC_GET_FLAG(RCC_FLAG_PLLRDY) && --timeout) {
        }

        if (timeout == 0) {
            return false;
        }

        uint32_t vos_timeout = 1000000;
        while (!(PWR->CSR & PWR_CSR_VOSRDY) && --vos_timeout) {
        }

        if (vos_timeout == 0) {
            return false;
        }
    }

    uint32_t sw = snap.cfgr & RCC_CFGR_SW;
    __HAL_RCC_SYSCLK_CONFIG(sw);
    uint32_t expected_sws = sw << 2;

    uint32_t timeout2 = 1000000;
    while (__HAL_RCC_GET_SYSCLK_SOURCE() != expected_sws && --timeout2) {
    }

    if (timeout2 == 0) {
        return false;
    }

    return true;
#else
    return true;
#endif
}

void suspend_background_threads() {
    k_thread_suspend(processor_tid);
    k_thread_suspend(producer_tid);
    k_thread_suspend(logger_tid);
    k_thread_suspend(battery_tid);
    k_thread_suspend(shell_tid);
    k_thread_suspend(bms_comm_tid);
    k_thread_suspend(hr_prod_tid);
    k_thread_suspend(disp_cons_tid);
    k_thread_suspend(mem_mon_tid);
    k_thread_suspend(trace_tid);
}

void resume_background_threads() {
    k_thread_resume(processor_tid);
    k_thread_resume(producer_tid);
    k_thread_resume(logger_tid);
    k_thread_resume(battery_tid);
    k_thread_resume(shell_tid);
    k_thread_resume(bms_comm_tid);
    k_thread_resume(hr_prod_tid);
    k_thread_resume(disp_cons_tid);
    k_thread_resume(mem_mon_tid);
    k_thread_resume(trace_tid);
}

#ifndef CONFIG_BOARD_MPS2_AN386

uint32_t s_last_alarm_interval_us = 0;
uint32_t s_last_alarm_seconds = 0;
int s_last_alarm_pending = 0;
struct rtc_time s_last_rtc_now{};
struct rtc_time s_last_rtc_alarm{};
uint32_t s_last_alraf_cycles = 0;
uint32_t s_last_alraf_timeout = 0;
uint32_t s_last_full_wake_cycles = 0;

uint32_t s_wfi_icsr = 0;
uint32_t s_wfi_pwr_csr = 0;
uint32_t s_wfi_pwr_cr = 0;
uint32_t s_wfi_nvic_pending0 = 0;
uint32_t s_wfi_nvic_active0 = 0;
#endif
uint32_t s_wfi_wake_count = 0;

#ifndef CONFIG_BOARD_MPS2_AN386
uint32_t s_pre_wfi_exti_pr = 0;
uint32_t s_pre_wfi_nvic_pending0 = 0;

GpioSnapshot s_stop_gpio_snap{};
uint32_t s_stop_prev_ahb1enr = 0;
uint32_t s_stop_prev_adc_cr2 = 0;
uint32_t s_stop_prev_adc_ccr = 0;
ClockSnapshot s_stop_clock_snap{};
uint32_t s_stop_prev_pwr_cr = 0;
#endif
[[maybe_unused]] PeripheralClockSnapshot s_stop_periph_snap{};
[[maybe_unused]] PeripheralClockSnapshot s_idle_periph_snap{};

ClockSnapshot s_idle_clock_snap{};
uint32_t s_idle_entry_time_ms = 0;
bool s_in_idle_state = false;
}
#ifndef CONFIG_BOARD_MPS2_AN386

void advance_rtc_time(struct rtc_time& time, uint32_t seconds_to_add)
{
    uint32_t total_seconds =
        static_cast<uint32_t>(time.tm_hour) * 3600U +
        static_cast<uint32_t>(time.tm_min) * 60U +
        static_cast<uint32_t>(time.tm_sec) +
        seconds_to_add;

    time.tm_hour = (total_seconds / 3600U) % 24U;
    time.tm_min  = (total_seconds / 60U) % 60U;
    time.tm_sec  = total_seconds % 60U;

    if (total_seconds >= 24U * 3600U) {
        time.tm_mday += total_seconds / (24U * 3600U);
    }
}

constexpr uint32_t RTC_ALARM_A_EXTI_LINE = (1UL << 17);

bool clear_rtc_alarm_a_pending() {
    RTC->WPR = 0xCA;
    RTC->WPR = 0x53;
    RTC->ISR &= ~RTC_ISR_ALRAF; // clear RTC Alarm A flag

    // Enable DWT Cycle Counter hardware
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    
    uint32_t start_cycles = DWT->CYCCNT;
    uint32_t timeout = 100000;
    
    while ((RTC->ISR & RTC_ISR_ALRAF) && --timeout) {
        // wait
    }
    
    uint32_t end_cycles = DWT->CYCCNT;
    
    // Store silently instead of logging
    s_last_alraf_cycles = (end_cycles - start_cycles);
    s_last_alraf_timeout = timeout;

    RTC->WPR = 0xFF; // re-lock
    EXTI->PR = RTC_ALARM_A_EXTI_LINE; // write-1-to-clear EXTI line 17
    NVIC_ClearPendingIRQ(RTC_Alarm_IRQn);
    return timeout != 0;
}

int set_rtc_alarm(const struct device* rtc_dev, uint32_t interval_us, void* user_data)
{
    if (!rtc_dev) {
        return -ENODEV;
    }

    int ret = rtc_alarm_set_time(rtc_dev, 0, 0, nullptr);
    if (ret != 0) {
        return ret;
    }

    rtc_alarm_set_callback(rtc_dev, 0, nullptr, nullptr);

    struct rtc_time now;
    if (rtc_get_time(rtc_dev, &now) != 0) {
        return -EIO;
    }

    uint32_t seconds_to_add = (interval_us + 999999U) / 1000000U;
    if (seconds_to_add == 0) {
        seconds_to_add = 1;
    }

    struct rtc_time alarm = now;
    advance_rtc_time(alarm, seconds_to_add);

    s_last_alarm_interval_us = interval_us;
    s_last_alarm_seconds = seconds_to_add;
    s_last_rtc_now = now;
    s_last_rtc_alarm = alarm;

    rtc_alarm_set_callback(
        rtc_dev,
        0,
        PowerManager::rtc_alarm_handler,
        user_data
    );

    uint16_t mask =
        RTC_ALARM_TIME_MASK_SECOND |
        RTC_ALARM_TIME_MASK_MINUTE |
        RTC_ALARM_TIME_MASK_HOUR;

    if (!clear_rtc_alarm_a_pending()) {
        return -EIO;
    }

    ret = rtc_alarm_set_time(
        rtc_dev,
        0,
        mask,
        &alarm
    );

    if (ret == 0) {
        s_last_alarm_pending = 0;
    }

    return ret;
}

void clear_rtc_alarm(const struct device* rtc_dev) {
    if (!rtc_dev) return;
    rtc_alarm_set_callback(rtc_dev, 0, nullptr, nullptr);
    rtc_alarm_set_time(rtc_dev, 0, 0, nullptr);
}

#endif // !CONFIG_BOARD_MPS2_AN386

#ifdef CONFIG_BOARD_MPS2_AN386

bool pm_device_suspend_tolerant(const struct device* dev, const char* name) {
    if (!dev) {
        return true;
    }

    int rc = pm_device_action_run(dev, PM_DEVICE_ACTION_SUSPEND);
    if (rc == 0 || rc == -EALREADY || rc == -ENOSYS || rc == -ENOTSUP) {
        return true;
    }

    LOG_ERR("%s suspend failed: %d", name, rc);
    return false;
}

bool pm_device_resume_tolerant(const struct device* dev, const char* name) {
    if (!dev) {
        return true;
    }

    int rc = pm_device_action_run(dev, PM_DEVICE_ACTION_RESUME);
    if (rc == 0 || rc == -EALREADY || rc == -ENOSYS || rc == -ENOTSUP) {
        return true;
    }

    LOG_ERR("%s resume failed: %d", name, rc);
    return false;
}
#endif


extern const struct device* i2c_hardware;
extern const struct device* uart_hardware;
extern const struct device* usb_hardware;
extern const struct device* adc_hardware;
extern DeviceContext sys_context;

#ifdef IS_TEST_ENVIRONMENT
    __attribute__((weak)) const struct device* rtc_hardware = nullptr;
    __attribute__((weak)) const struct device* uart_hardware = nullptr;
    __attribute__((weak)) const struct device* usb_hardware = nullptr;
    __attribute__((weak)) const struct device* adc_hardware = nullptr;
#else
    #if defined(CONFIG_RTC) && DT_NODE_HAS_STATUS(DT_ALIAS(rtc0), okay)
        const struct device* rtc_hardware = DEVICE_DT_GET(DT_ALIAS(rtc0));
    #else
        const struct device* rtc_hardware = nullptr;
    #endif

    #ifdef CONFIG_BOARD_MPS2_AN386
        const struct device* adc_hardware = nullptr;
    #else
        #if defined(CONFIG_ADC) && DT_NODE_HAS_STATUS(DT_NODELABEL(adc1), okay)
            const struct device* adc_hardware = DEVICE_DT_GET(DT_NODELABEL(adc1));
        #else
            const struct device* adc_hardware = nullptr;
        #endif
    #endif

    #if defined(CONFIG_USB_DEVICE_STACK) && DT_NODE_HAS_STATUS(DT_ALIAS(cdc_acm_uart0), okay)
        const struct device* usb_hardware = DEVICE_DT_GET(DT_ALIAS(cdc_acm_uart0));
    #else
        const struct device* usb_hardware = nullptr;
    #endif

    #if DT_NODE_EXISTS(DT_CHOSEN(zephyr_console))
        const struct device* uart_hardware = DEVICE_DT_GET(DT_CHOSEN(zephyr_console));
    #else
        const struct device* uart_hardware = nullptr;
    #endif
#endif

K_MUTEX_DEFINE(pm_state_mutex);
K_MUTEX_DEFINE(pm_observer_mutex);

#ifdef CONFIG_BOARD_MPS2_AN386
K_TIMER_DEFINE(mps2_stop_timer, simulated_stop_timer_handler, nullptr);
#else
K_TIMER_DEFINE(sim_rtc_timer, simulated_rtc_timer_handler, nullptr);
#endif

K_WORK_DELAYABLE_DEFINE(usb_reconnect_work, usb_reconnect_work_handler);

void PowerManager::rtc_alarm_handler(const struct device*, uint16_t, void* user_data) {
    if (!user_data) return;
    auto* self = static_cast<PowerManager*>(user_data);
    atomic_set(&self->wake_pending, 1);
    
    k_sem_give(&pm_wake_sem); 
    k_sem_give(&stop_wake_sem);
}

bool PowerManager::consumeWakePending() {
    return atomic_cas(&wake_pending, 1, 0);
}

PowerManager::PowerManager()
    : current_state(nullptr),
      last_sleep_time(0),
      expected_wake_time(0),
      rtc_dev(nullptr),
      i2c_dev(nullptr),
      uart_dev(nullptr),
      usb_dev(nullptr),
      adc_dev(nullptr),
      observer_count(0),
      fault_context(nullptr),
      consecutive_pm_failures(0)
{
    atomic_set(&wake_pending, 0);
}

PowerManager& PowerManager::getInstance() {
    static PowerManager instance;
    return instance;
}

bool PowerManager::init(const struct device* rtc, const struct device* i2c,
                        const struct device* uart, const struct device* usb,
                        const struct device* adc, DeviceContext* fault_ctx) {
    #ifdef IS_TEST_ENVIRONMENT
    if (g_force_init_fail) return false;
    #endif

#ifndef CONFIG_BOARD_MPS2_AN386
    uint32_t reset_flags = RCC->CSR;

    if (reset_flags & RCC_CSR_IWDGRSTF) {
        LOG_ERR("RESET REASON: IWDG RESET");
    }

    if (reset_flags & RCC_CSR_WWDGRSTF) {
        LOG_ERR("RESET REASON: WWDG RESET");
    }

    /* Clear reset flags using HAL macros */
    __HAL_RCC_CLEAR_RESET_FLAGS();
    
#ifdef CONFIG_DEBUG
    HAL_DBGMCU_EnableDBGStopMode();
#else
    HAL_DBGMCU_DisableDBGStopMode();
#endif
#endif

    rtc_dev = rtc;
    i2c_dev = i2c;
    uart_dev = uart;
    usb_dev = usb;
    adc_dev = adc;
    fault_context = fault_ctx;
    last_activity_time.store(k_uptime_get_32());

    if (rtc_dev == nullptr || !device_is_ready(rtc_dev)) {
        LOG_WRN("RTC device unavailable; RTC-based wake disabled");
        rtc_dev = nullptr;
    }
    if (i2c_dev != nullptr && !device_is_ready(i2c_dev)) {
        LOG_ERR("I2C device bound but not ready!");
        i2c_dev = nullptr; 
    }
    if (adc_dev != nullptr && !device_is_ready(adc_dev)) {
        LOG_ERR("ADC device bound but not ready!");
        adc_dev = nullptr; 
    }
    if (rtc_dev) {
        struct rtc_time time;
        bool need_set = (rtc_get_time(rtc_dev, &time) != 0);
        if (need_set) {
            memset(&time, 0, sizeof(time));
            time.tm_mday = 1;
            time.tm_year = 126;
            rtc_set_time(rtc_dev, &time);
        }
    }

    LOG_INF("Power Manager initialized. Automatically starting FSM.");

#ifdef CONFIG_BOARD_MPS2_AN386
#else
    pm_policy_state_lock_get(PM_STATE_SUSPEND_TO_RAM, PM_ALL_SUBSTATES);
#endif

    k_mutex_lock(&pm_state_mutex, K_FOREVER);
    transitionTo(ActiveState::getInstance());
    k_mutex_unlock(&pm_state_mutex);

    return true;
}

bool PowerManager::registerObserver(IPowerObserver* obs) {
    k_mutex_lock(&pm_observer_mutex, K_FOREVER);
    for (size_t i = 0; i < observer_count; ++i) {
        if (observers[i] == obs) {
            k_mutex_unlock(&pm_observer_mutex);
            return true;
        }
    }
    if (observer_count < MAX_OBSERVERS) {
        observers[observer_count++] = obs;
        k_mutex_unlock(&pm_observer_mutex);
        return true;
    }
    k_mutex_unlock(&pm_observer_mutex);
    LOG_ERR("Observer limit reached");
    return false;
}

size_t PowerManager::captureObservers(std::array<IPowerObserver*, MAX_OBSERVERS>& out) {
    k_mutex_lock(&pm_observer_mutex, K_FOREVER);
    size_t count = (observer_count > MAX_OBSERVERS) ? MAX_OBSERVERS : observer_count;
    for (size_t i = 0; i < count; ++i) out[i] = observers[i];
    k_mutex_unlock(&pm_observer_mutex);
    return count;
}

void PowerManager::notifyBeforeSleep() {
    std::array<IPowerObserver*, MAX_OBSERVERS> local_obs{};
    size_t count = captureObservers(local_obs);
    for (size_t i = 0; i < count; ++i) {
        if (local_obs[i]) local_obs[i]->beforeSleep();
    }
}

void PowerManager::notifyAfterWakeup() {
    std::array<IPowerObserver*, MAX_OBSERVERS> local_obs{};
    size_t count = captureObservers(local_obs);
    for (size_t i = 0; i < count; ++i) {
        if (local_obs[i]) local_obs[i]->afterWakeup();
    }
}

void PowerManager::notifySleepAborted() {
    std::array<IPowerObserver*, MAX_OBSERVERS> local_obs{};
    size_t count = captureObservers(local_obs);
    for (size_t i = 0; i < count; ++i) {
        if (local_obs[i]) local_obs[i]->sleepAborted();
    }
}

void PowerManager::reportActivity() {
    last_activity_time.store(k_uptime_get_32());
    atomic_set(&s_activity_pending, 1);
    
    k_sem_give(&pm_wake_sem);
    k_sem_give(&stop_wake_sem);
}

void PowerManager::reportPmFailure() {
    ++consecutive_pm_failures;
    if (consecutive_pm_failures == PM_FAILURE_FAULT_THRESHOLD) {
        if (fault_context) {
            fault_context->triggerFault("Power Management Failure");
        }
    }
}

void PowerManager::transitionTo(IPowerState& next_state) {
    if (current_state == &next_state) return;

    IPowerState* old_state = current_state;

    if (old_state) {
        old_state->exit(*this);
    }

    LOG_INF("Transition: %s -> %s",
            old_state ? old_state->getName() : "NONE",
            next_state.getName());
            
    if (&next_state == &ActiveState::getInstance()) {
        last_activity_time.store(k_uptime_get_32());
    }
    
    if (next_state.enter(*this)) {
        current_state = &next_state;
        return;
    }

    LOG_WRN("State %s aborted entry. Evaluating cascaded fallback.", next_state.getName());

    if (IdleState::getInstance().enter(*this)) {
        current_state = &IdleState::getInstance();
        return;
    }

    LOG_ERR("Power manager halted. Idle fallback failed.");
    if (fault_context) {
        fault_context->triggerFault("Power Manager FSM Halted");
    }

    current_state = nullptr;
}

void PowerManager::processFSM() {
    bool activity_woke = atomic_cas(&s_activity_pending, 1, 0);

    IPowerState* local_state = nullptr;

    k_mutex_lock(&pm_state_mutex, K_FOREVER);
    local_state = current_state;
    k_mutex_unlock(&pm_state_mutex);

    if (activity_woke && local_state != nullptr) {
        last_activity_time.store(k_uptime_get_32());

        if (local_state != &ActiveState::getInstance()) {
            LOG_WRN(">>> HARDWARE WAKEUP DETECTED! Resuming from Sleep. <<<");

            k_mutex_lock(&pm_state_mutex, K_FOREVER);
            transitionTo(ActiveState::getInstance());
            local_state = current_state;
            k_mutex_unlock(&pm_state_mutex);
        }
    }

    if (local_state) {
        IPowerState& next_state = local_state->execute(*this);
        if (&next_state != local_state) {
            k_mutex_lock(&pm_state_mutex, K_FOREVER);
            if (current_state == local_state) {
                transitionTo(next_state);
            }
            k_mutex_unlock(&pm_state_mutex);
        }
    }
}

#ifdef IS_TEST_ENVIRONMENT
void PowerManager::resetForTest() {
    k_mutex_lock(&pm_state_mutex, K_FOREVER);
    current_state = nullptr;
    k_mutex_unlock(&pm_state_mutex);

    last_activity_time.store(0);
    last_sleep_time = 0;
    expected_wake_time = 0;
    consecutive_pm_failures = 0;
    atomic_set(&wake_pending, 0);
    atomic_set(&s_activity_pending, 0);

    k_mutex_lock(&pm_observer_mutex, K_FOREVER);
    observer_count = 0;
    observers.fill(nullptr);
    k_mutex_unlock(&pm_observer_mutex);

    StopState::getInstance().resetForTest();
}
#endif

ActiveState& ActiveState::getInstance() {
    static constinit ActiveState instance;
    return instance;
}

bool ActiveState::enter(PowerManager& pm) {
    LOG_INF("Entering PM_STATE_ACTIVE");
    pm_policy_state_lock_get(PM_STATE_SUSPEND_TO_IDLE, PM_ALL_SUBSTATES);
    return true;
}

IPowerState& ActiveState::execute(PowerManager& pm) {
    uint32_t elapsed = k_uptime_get_32() - pm.getLastActivityTime();
    if (elapsed >= ACTIVE_TIMEOUT_MS) {
        return IdleState::getInstance();
    }
    return *this;
}

void ActiveState::exit(PowerManager&) {
    pm_policy_state_lock_put(PM_STATE_SUSPEND_TO_IDLE, PM_ALL_SUBSTATES);
}

IdleState& IdleState::getInstance() {
    static constinit IdleState instance;
    return instance;
}

bool IdleState::enter(PowerManager& pm) {
#ifdef IS_TEST_ENVIRONMENT
    if (g_force_idle_enter_fail) return false;
#endif
    LOG_INF("Entering PM_STATE_SUSPEND_TO_IDLE");
    s_idle_entry_time_ms = k_uptime_get_32();
    s_in_idle_state = true;
    suspend_background_threads();
#ifdef CONFIG_BOARD_MPS2_AN386
    if (!pm_device_suspend_tolerant(pm.getUartDev(), "UART")) {
        resume_background_threads();
        return false;
    }
    if (!pm_device_suspend_tolerant(pm.getAdcDev(), "ADC")) {
        (void)pm_device_resume_tolerant(pm.getUartDev(), "UART");
        resume_background_threads();
        return false;
    }
    if (!pm_device_suspend_tolerant(pm.getI2cDev(), "I2C")) {
        (void)pm_device_resume_tolerant(pm.getAdcDev(), "ADC");
        (void)pm_device_resume_tolerant(pm.getUartDev(), "UART");
        resume_background_threads();
        return false;
    }
#else
    if (pm.getUartDev()) {
        int rc = pm_device_action_run(pm.getUartDev(), PM_DEVICE_ACTION_SUSPEND);
        if (rc != 0 && rc != -EALREADY) {
            LOG_ERR("UART suspend failed: %d", rc);
            resume_background_threads();
            return false;
        }
    }
    if (pm.getAdcDev()) {
        int rc = pm_device_action_run(pm.getAdcDev(), PM_DEVICE_ACTION_SUSPEND);
        if (rc != 0 && rc != -EALREADY) {
            LOG_ERR("ADC suspend failed: %d", rc);
            resume_background_threads();
            return false;
        }
    }
    if (pm.getI2cDev()) {
        int rc = pm_device_action_run(pm.getI2cDev(), PM_DEVICE_ACTION_SUSPEND);
        if (rc != 0 && rc != -EALREADY) {
            LOG_ERR("I2C suspend failed: %d", rc);
            resume_background_threads();
            return false;
        }
    }
#endif

#ifndef CONFIG_BOARD_MPS2_AN386
    __HAL_RCC_ADC1_CLK_DISABLE();

#ifdef CONFIG_DEBUG
    HAL_DBGMCU_EnableDBGStopMode();
#else
    HAL_DBGMCU_DisableDBGStopMode();
#endif
#endif

    clock_downscale_for_idle(s_idle_clock_snap);

#if STOP_APB_GATING_ENABLED
    periph_clocks_save_and_gate(s_idle_periph_snap, APB1_STOP_KEEP_MASK, APB2_STOP_KEEP_MASK);
#endif

    return true;
}

IPowerState& IdleState::execute(PowerManager& pm) {
    uint32_t idle_elapsed = k_uptime_get_32() - s_idle_entry_time_ms;
    if (idle_elapsed >= IDLE_TIMEOUT_MS) {
        return StopState::getInstance();
    }
    return *this;
}

void IdleState::exit(PowerManager& pm) {
s_in_idle_state = false;
#ifndef CONFIG_BOARD_MPS2_AN386
#ifdef CONFIG_DEBUG
    HAL_DBGMCU_EnableDBGStopMode();
#else
    HAL_DBGMCU_DisableDBGStopMode();
#endif
#endif

#if STOP_APB_GATING_ENABLED
    periph_clocks_restore(s_idle_periph_snap);
#endif

    bool clock_ok = clock_restore_from_idle(s_idle_clock_snap);

#ifndef CONFIG_BOARD_MPS2_AN386
    __HAL_RCC_ADC1_CLK_ENABLE();
#endif

#ifdef CONFIG_BOARD_MPS2_AN386
    (void)pm_device_resume_tolerant(pm.getUartDev(), "UART");
    (void)pm_device_resume_tolerant(pm.getI2cDev(), "I2C");
    (void)pm_device_resume_tolerant(pm.getAdcDev(), "ADC");
#else
    if (pm.getUartDev()) {
        int rc = pm_device_action_run(pm.getUartDev(), PM_DEVICE_ACTION_RESUME);
        if (rc != 0 && rc != -EALREADY) {
            LOG_ERR("UART resume failed: %d", rc);
        }
    }
    if (pm.getI2cDev()) {
        int rc = pm_device_action_run(pm.getI2cDev(), PM_DEVICE_ACTION_RESUME);
        if (rc != 0 && rc != -EALREADY) {
            LOG_ERR("I2C resume failed: %d", rc);
        }
    }
    if (pm.getAdcDev()) {
        int rc = pm_device_action_run(pm.getAdcDev(), PM_DEVICE_ACTION_RESUME);
        if (rc != 0 && rc != -EALREADY) {
            LOG_ERR("ADC resume failed: %d", rc);
        }
    }
#endif

    if (!clock_ok) {
        LOG_ERR("Idle clock restore failed");
        pm.reportPmFailure();
    }
    resume_background_threads();
}

StopState& StopState::getInstance() {
    static constinit StopState instance;
    return instance;
}

bool StopState::enter(PowerManager& pm) {
    LOG_WRN("Preparing for Deep Sleep (PM_STATE_SUSPEND_TO_RAM)");

    s_stop_wake_reason = StopWakeReason::WAKE_NONE;
    atomic_set(&s_activity_pending, 0);
    pm.clearWakePending();

#ifdef CONFIG_BOARD_MPS2_AN386
    k_timer_stop(&mps2_stop_timer);
#endif

    while (k_sem_take(&stop_wake_sem, K_NO_WAIT) == 0) {}

    sleep_prepared = false;
    s_real_hw_sleep_active = false;
    s_elapsed_sleep_time_us = 0;
    s_wfi_wake_count = 0;

    pm_policy_state_lock_get(PM_STATE_SUSPEND_TO_IDLE, PM_ALL_SUBSTATES);

    pm.notifyBeforeSleep();
    k_msleep(50);

    suspend_background_threads();

#ifdef CONFIG_BOARD_MPS2_AN386
    LOG_INF("MPS2/QEMU: entering simulated STOP state");
    if (!pm_device_suspend_tolerant(pm.getUartDev(), "UART")) {
        resume_background_threads();
        pm_policy_state_lock_put(PM_STATE_SUSPEND_TO_IDLE, PM_ALL_SUBSTATES);
        return false;
    }
    if (!pm_device_suspend_tolerant(pm.getAdcDev(), "ADC")) {
        (void)pm_device_resume_tolerant(pm.getUartDev(), "UART");
        resume_background_threads();
        pm_policy_state_lock_put(PM_STATE_SUSPEND_TO_IDLE, PM_ALL_SUBSTATES);
        return false;
    }
    if (!pm_device_suspend_tolerant(pm.getI2cDev(), "I2C")) {
        (void)pm_device_resume_tolerant(pm.getAdcDev(), "ADC");
        (void)pm_device_resume_tolerant(pm.getUartDev(), "UART");
        resume_background_threads();
        pm_policy_state_lock_put(PM_STATE_SUSPEND_TO_IDLE, PM_ALL_SUBSTATES);
        return false;
    }

    s_current_sleep_interval_us = STOP_TOTAL_TIME_US;
    k_timer_start(&mps2_stop_timer, K_USEC(s_current_sleep_interval_us), K_NO_WAIT);

    sleep_prepared = true;
    s_real_hw_sleep_active = false;

    pm.recordSleepTime();
    pm.setExpectedWakeTime(pm.getSleepTime() + (STOP_TOTAL_TIME_US / 1000U));

    return true;

#else
    /* Existing STM32 implementation */

    const bool have_real_rtc = (pm.getRtcDev() != nullptr);

    if (!have_real_rtc) {
        k_timer_start(&sim_rtc_timer, K_USEC(STOP_TOTAL_TIME_US), K_NO_WAIT);
        sleep_prepared = true;
        s_real_hw_sleep_active = false;
        LOG_WRN("No RTC: using simulated STOP wake timer");
        return true;
    }

    s_current_sleep_interval_us = STOP_WAKE_INTERVAL_US;

    sys_context.feedWatchdog();

    int err = set_rtc_alarm(pm.getRtcDev(), s_current_sleep_interval_us, &pm);
    if (err) {
        LOG_ERR("Failed to set RTC alarm (err: %d). Aborting STOP entry.", err);
        pm.reportPmFailure();
        resume_background_threads();
        pm_policy_state_lock_put(PM_STATE_SUSPEND_TO_IDLE, PM_ALL_SUBSTATES);
        return false;
    }

    k_work_cancel_delayable(&usb_reconnect_work);
    g_usb_connected = false;

#ifdef CONFIG_USB_DEVICE_STACK
        usb_dc_detach();
#endif

    if (pm.getAdcDev()) {
        int rc = pm_device_action_run(pm.getAdcDev(), PM_DEVICE_ACTION_SUSPEND);
        if (rc != 0 && rc != -EALREADY) {
            LOG_ERR("ADC suspend failed: %d", rc);
            resume_background_threads();
            pm_policy_state_lock_put(PM_STATE_SUSPEND_TO_IDLE, PM_ALL_SUBSTATES);
            return false;
        }
    }
    if (pm.getI2cDev()) {
        int rc = i2c_recover_bus(pm.getI2cDev());
        if (rc != 0 && rc != -ENOSYS) {
            LOG_WRN("I2C bus recovery before STOP returned %d", rc);
        }
        rc = pm_device_action_run(pm.getI2cDev(), PM_DEVICE_ACTION_SUSPEND);
        if (rc != 0 && rc != -EALREADY) {
            LOG_ERR("I2C suspend failed: %d", rc);
            if (pm.getAdcDev()) {
                pm_device_action_run(pm.getAdcDev(), PM_DEVICE_ACTION_RESUME);
            }
            resume_background_threads();
            pm_policy_state_lock_put(PM_STATE_SUSPEND_TO_IDLE, PM_ALL_SUBSTATES);
            return false;
        }
    }

    k_msleep(250);

    LOG_INF("STOP PREPARED: UART=OFF, custom WFI");

    if (pm.getUartDev()) {
        int rc = pm_device_action_run(pm.getUartDev(), PM_DEVICE_ACTION_SUSPEND);
        if (rc != 0 && rc != -EALREADY) {
            LOG_ERR("UART suspend failed: %d", rc);
            if (pm.getI2cDev()) {
                pm_device_action_run(pm.getI2cDev(), PM_DEVICE_ACTION_RESUME);
            }
            if (pm.getAdcDev()) {
                pm_device_action_run(pm.getAdcDev(), PM_DEVICE_ACTION_RESUME);
            }
            resume_background_threads();
            pm_policy_state_lock_put(PM_STATE_SUSPEND_TO_IDLE, PM_ALL_SUBSTATES);
            return false;
        }
    }

    usb_disable_hardware_and_release_pins();

    adc_save_and_disable(s_stop_prev_adc_cr2, s_stop_prev_adc_ccr);
    gpio_save_and_set_all_analog(s_stop_gpio_snap, s_stop_prev_ahb1enr);

#if STOP_APB_GATING_ENABLED
    periph_clocks_save_and_gate(s_stop_periph_snap, APB1_STOP_KEEP_MASK, APB2_STOP_KEEP_MASK);
#endif

    clock_save_state(s_stop_clock_snap);

    s_stop_prev_pwr_cr = PWR->CR;

#ifdef CONFIG_DEBUG
    HAL_DBGMCU_EnableDBGStopMode();
#else
    HAL_DBGMCU_DisableDBGStopMode();
#endif

    __HAL_RCC_PWR_CLK_ENABLE();

    CLEAR_BIT(PWR->CR, PWR_CR_PDDS);

    SET_BIT(PWR->CR, PWR_CR_LPDS);
    SET_BIT(PWR->CR, PWR_CR_FPDS);

    pm.recordSleepTime();
    pm.setExpectedWakeTime(
        pm.getSleepTime() + (STOP_TOTAL_TIME_US / 1000));

    sleep_prepared = true;
    s_real_hw_sleep_active = true;

    return true;
#endif
}

IPowerState& StopState::execute(PowerManager& pm) {
#ifdef CONFIG_BOARD_MPS2_AN386
    if (!sleep_prepared) {
        return *this;
    }

    int rc = k_sem_take(&stop_wake_sem, K_MSEC(100));

    if (rc == 0) {
        if (atomic_cas(&s_activity_pending, 1, 0)) {
            s_stop_wake_reason = StopWakeReason::WAKE_ACTIVITY;
            pm.clearWakePending();

            LOG_INF("MPS2/QEMU STOP wake: ACTIVITY");

            return ActiveState::getInstance();
        }

        if (pm.consumeWakePending()) {
            s_stop_wake_reason = StopWakeReason::WAKE_RTC;
            s_elapsed_sleep_time_us = STOP_TOTAL_TIME_US;

            LOG_INF("MPS2/QEMU STOP wake: elapsed=%u us", s_elapsed_sleep_time_us);

            return ActiveState::getInstance();
        }

        s_stop_wake_reason = StopWakeReason::WAKE_UNKNOWN;
        return *this;
    }

    /* No wake event yet within the timeout. Remain in STOP. */
    return *this;

#else
    if (!s_real_hw_sleep_active) {
        return *this;
    }

    for (;;) {
        uint32_t systick_ctrl = SysTick->CTRL;
        SysTick->CTRL &= ~SysTick_CTRL_TICKINT_Msk;

        SET_BIT(PWR->CR, PWR_CR_CWUF);

        s_pre_wfi_exti_pr = EXTI->PR;
        s_pre_wfi_nvic_pending0 = NVIC->ISPR[0];

        SET_BIT(SCB->SCR, SCB_SCR_SLEEPDEEP_Msk);

        __DSB();
        __ISB();

        __WFI();

        s_wfi_wake_count++;
        s_wfi_icsr = SCB->ICSR;
        s_wfi_pwr_csr = PWR->CSR;
        s_wfi_pwr_cr = PWR->CR;
        s_wfi_nvic_pending0 = NVIC->ISPR[0];
        s_wfi_nvic_active0  = NVIC->IABR[0];

        SysTick->CTRL = systick_ctrl;

        __NOP();
        __NOP();
        __NOP();
        __ISB();

        CLEAR_BIT(SCB->SCR, SCB_SCR_SLEEPDEEP_Msk);

        if (atomic_cas(&s_activity_pending, 1, 0)) {
            s_stop_wake_reason = StopWakeReason::WAKE_ACTIVITY;
            continue;
        }

        if (pm.consumeWakePending()) {
            s_stop_wake_reason = StopWakeReason::WAKE_RTC;

            s_elapsed_sleep_time_us += s_current_sleep_interval_us;

            if (s_elapsed_sleep_time_us >= STOP_TOTAL_TIME_US) {
                return ActiveState::getInstance();
            }
            
            __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);
            uint32_t vos_timeout = 100000;
            while (!(PWR->CSR & PWR_CSR_VOSRDY) && --vos_timeout) { } // Confirm regulator settled

            MODIFY_REG(RCC->CFGR, RCC_CFGR_HPRE, RCC_CFGR_HPRE_DIV16);
            __HAL_FLASH_SET_LATENCY(FLASH_LATENCY_0); // 0 WS is valid at 1 MHz, any scale
            
            CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
            DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
            uint32_t cyc_start = DWT->CYCCNT;
  
            sys_context.feedWatchdog();

            uint32_t remaining_us = STOP_TOTAL_TIME_US - s_elapsed_sleep_time_us;
            uint32_t next_interval_us = (remaining_us > STOP_WAKE_INTERVAL_US) ?
                                        STOP_WAKE_INTERVAL_US : remaining_us;
            if (next_interval_us < 1000000U) {
                next_interval_us = 1000000U;
            }

            int err = set_rtc_alarm(pm.getRtcDev(), next_interval_us, &pm);
            uint32_t cyc_end = DWT->CYCCNT;
            
            // Store silently instead of logging (UART is OFF)
            s_last_full_wake_cycles = (cyc_end - cyc_start);
            if (err) {
                pm.reportPmFailure();
                return ActiveState::getInstance();
            }

            continue;
        }

        s_stop_wake_reason = StopWakeReason::WAKE_UNKNOWN;
        if (s_wfi_wake_count > 5) {
            s_stop_wake_reason = StopWakeReason::WAKE_ABORTED;
            return ActiveState::getInstance();
        }
        continue;
    }

    return ActiveState::getInstance();
#endif
}

void StopState::exit(PowerManager& pm) {
#ifdef CONFIG_BOARD_MPS2_AN386
    if (!sleep_prepared) {
        resume_background_threads();
        return;
    }

    k_timer_stop(&mps2_stop_timer);

    sleep_prepared = false;
    s_real_hw_sleep_active = false;

    /* No stale wake flag should survive into the next STOP entry. */
    pm.clearWakePending();

    pm.notifyAfterWakeup();

    switch (s_stop_wake_reason) {
        case StopWakeReason::WAKE_RTC:
            LOG_INF("STOP WAKE: TIMER");
            break;
        case StopWakeReason::WAKE_ACTIVITY:
            LOG_INF("STOP WAKE: ACTIVITY");
            break;
        default:
            LOG_INF("STOP WAKE: UNKNOWN");
            break;
    }

    if (s_stop_wake_reason == StopWakeReason::WAKE_RTC ||
        s_stop_wake_reason == StopWakeReason::WAKE_ACTIVITY) {
        pm.resetPmFailures();
    }

    s_stop_wake_reason = StopWakeReason::WAKE_NONE;
    
    (void)pm_device_resume_tolerant(pm.getUartDev(), "UART");
    (void)pm_device_resume_tolerant(pm.getI2cDev(), "I2C");
    (void)pm_device_resume_tolerant(pm.getAdcDev(), "ADC");

    resume_background_threads();

    pm_policy_state_lock_put(PM_STATE_SUSPEND_TO_IDLE, PM_ALL_SUBSTATES);

    LOG_INF("MPS2/QEMU STOP exit");
    return;

#else
    if (!sleep_prepared) {
        resume_background_threads();
        return;
    }

    sleep_prepared = false;

    if (!s_real_hw_sleep_active) {
        k_timer_stop(&sim_rtc_timer);
        pm.notifyAfterWakeup();
        resume_background_threads();

        pm_policy_state_lock_put(PM_STATE_SUSPEND_TO_IDLE, PM_ALL_SUBSTATES);
        return;
    }

    clear_rtc_alarm(pm.getRtcDev());

    bool clock_ok = clock_restore_from_idle(s_stop_clock_snap);

#if STOP_APB_GATING_ENABLED
    periph_clocks_restore(s_stop_periph_snap);
#endif

    gpio_restore_after_wake(s_stop_gpio_snap, s_stop_prev_ahb1enr);
    adc_restore_after_wake(s_stop_prev_adc_cr2, s_stop_prev_adc_ccr);

    CLEAR_BIT(SCB->SCR, SCB_SCR_SLEEPDEEP_Msk);
    constexpr uint32_t stop_pwr_mask = PWR_CR_PDDS | PWR_CR_LPDS | PWR_CR_FPDS;
    PWR->CR = (PWR->CR & ~stop_pwr_mask) | (s_stop_prev_pwr_cr & stop_pwr_mask);

#ifdef CONFIG_DEBUG
    HAL_DBGMCU_EnableDBGStopMode();
#else
    HAL_DBGMCU_DisableDBGStopMode();
#endif

    if (pm.getUartDev()) {
        int rc = pm_device_action_run(pm.getUartDev(), PM_DEVICE_ACTION_RESUME);
        if (rc != 0 && rc != -EALREADY) {
            LOG_ERR("UART resume failed: %d", rc);
        }
    }
    if (pm.getI2cDev()) {
        int rc = pm_device_action_run(pm.getI2cDev(), PM_DEVICE_ACTION_RESUME);
        if (rc != 0 && rc != -EALREADY) {
            LOG_ERR("I2C resume failed: %d", rc);
        }
    }
    if (pm.getAdcDev()) {
        int rc = pm_device_action_run(pm.getAdcDev(), PM_DEVICE_ACTION_RESUME);
        if (rc != 0 && rc != -EALREADY) {
            LOG_ERR("ADC resume failed: %d", rc);
        }
    }

    SysTick->CTRL = s_stop_clock_snap.systick_ctrl;

    if (!clock_ok) {
        LOG_ERR("Clock tree did not relock cleanly after STOP; continuing on HSI.");
        pm.reportPmFailure();
    }

    switch (s_stop_wake_reason) {
        case StopWakeReason::WAKE_RTC:
            LOG_INF("STOP WAKE: RTC");
            break;
        case StopWakeReason::WAKE_ACTIVITY:
            LOG_INF("STOP WAKE: ACTIVITY");
            break;
        case StopWakeReason::WAKE_ABORTED:
            LOG_ERR("STOP WAKE: ABORTED (WFI loop failed %u times)", s_wfi_wake_count);
            break;
        default:
            LOG_INF("STOP WAKE: UNKNOWN (spurious wake count this cycle=%u)", s_wfi_wake_count);
            break;
    }

    LOG_INF("WFI wake: ICSR=0x%08x PWR_CSR=0x%08x PWR_CR=0x%08x "
            "NVIC_ISPR0=0x%08x NVIC_IABR0=0x%08x",
            s_wfi_icsr,
            s_wfi_pwr_csr,
            s_wfi_pwr_cr,
            s_wfi_nvic_pending0,
            s_wfi_nvic_active0);

    LOG_INF("Pre-WFI pending (last iter): EXTI_PR=0x%08x NVIC_ISPR0=0x%08x",
            s_pre_wfi_exti_pr,
            s_pre_wfi_nvic_pending0);
    
    LOG_INF("Last RTC ALRAF clear: %u/100000 rem, Cycles: %u", 
            s_last_alraf_timeout, s_last_alraf_cycles);
    
    LOG_INF("Full mid-STOP service: %u cycles (~%u us @1MHz)",
            s_last_full_wake_cycles, s_last_full_wake_cycles);

    s_wfi_wake_count = 0;

    pm.notifyAfterWakeup();

    if (s_stop_wake_reason == StopWakeReason::WAKE_RTC ||
        s_stop_wake_reason == StopWakeReason::WAKE_ACTIVITY) {
        pm.resetPmFailures();
    }

    resume_background_threads();

#ifdef CONFIG_USB_DEVICE_STACK
    k_work_reschedule(&usb_reconnect_work, K_MSEC(USB_RECONNECT_DELAY_MS));
#endif

    s_real_hw_sleep_active = false;

    pm_policy_state_lock_put(PM_STATE_SUSPEND_TO_IDLE, PM_ALL_SUBSTATES);
#endif
}

void power_monitor_thread() {
    auto& pwr_manager = PowerManager::getInstance();

    if (!pwr_manager.init(rtc_hardware, i2c_hardware, uart_hardware, usb_hardware, adc_hardware, &sys_context)) {
        LOG_ERR("Power Manager Init Failed. Thread halting.");
        return;
    }

    do {
        pwr_manager.processFSM();

        k_timeout_t wait_timeout = K_MSEC(10);
        uint32_t current_time = k_uptime_get_32();
        
        if (s_in_idle_state) {
            uint32_t idle_elapsed = current_time - s_idle_entry_time_ms;
            if (idle_elapsed < IDLE_TIMEOUT_MS) {
                wait_timeout = K_MSEC(IDLE_TIMEOUT_MS - idle_elapsed);
            } else {
                wait_timeout = K_MSEC(10);
            }
        } else {
            uint32_t active_elapsed = current_time - pwr_manager.getLastActivityTime();
            if (active_elapsed < ACTIVE_TIMEOUT_MS) {
                wait_timeout = K_MSEC(ACTIVE_TIMEOUT_MS - active_elapsed);
            } else {
                wait_timeout = K_MSEC(10);
            }
        }

        (void)k_sem_take(&pm_wake_sem, wait_timeout);

    } while (THREAD_LOOP_CONDITION);
}

extern const k_tid_t pr_tid;
K_THREAD_DEFINE(pr_tid, 1024, power_monitor_thread, NULL, NULL, NULL, 14, 0, 0);
