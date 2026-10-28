#include "Power_Internal.h"

LOG_MODULE_DECLARE(PWR_SYS, LOG_LEVEL_INF);

using namespace pm_internal;

#ifdef CONFIG_BOARD_MPS2_AN386

static void simulated_stop_timer_handler(struct k_timer *timer_id){
    ARG_UNUSED(timer_id);

    auto& pm = PowerManager::getInstance();

    atomic_set(&pm.wake_pending, 1);

    k_sem_give(&pm_wake_sem);
    k_sem_give(&stop_wake_sem);
}
#else

static void simulated_rtc_timer_handler(struct k_timer *timer_id){
    ARG_UNUSED(timer_id);
    auto& pm = PowerManager::getInstance();
    atomic_set(&pm.wake_pending, 1);
    k_sem_give(&pm_wake_sem);
    k_sem_give(&stop_wake_sem);
}
#endif

namespace {
#ifndef CONFIG_BOARD_MPS2_AN386
uint32_t s_last_full_wake_cycles = 0;

uint32_t s_wfi_icsr = 0;
uint32_t s_wfi_pwr_csr = 0;
uint32_t s_wfi_pwr_cr = 0;
uint32_t s_wfi_nvic_pending0 = 0;
uint32_t s_wfi_nvic_active0 = 0;
uint32_t s_probe_scr = 0;          // SCB->SCR as seen right before the WFI
uint32_t s_probe_dhcsr = 0;        // CoreDebug->DHCSR right before the WFI (bit0 = C_DEBUGEN: a debugger is attached)
uint32_t s_probe_dbgmcu_cr = 0;    // DBGMCU->CR right before the WFI
#endif
uint32_t s_wfi_wake_count = 0;

#ifndef CONFIG_BOARD_MPS2_AN386
uint32_t s_pre_wfi_exti_pr = 0;
uint32_t s_pre_wfi_nvic_pending0 = 0;
uint32_t s_pre_wfi_pwr_cr = 0;     // PWR state captured right before the WFI
uint32_t s_pre_wfi_pwr_csr = 0;

GpioSnapshot s_stop_gpio_snap{};
uint32_t s_stop_prev_ahb1enr = 0;
AdcSnapshot s_stop_adc_snap{};
ClockSnapshot s_stop_clock_snap{};
uint32_t s_stop_prev_pwr_cr = 0;
uint32_t s_stop_prev_dac_cr = 0;
uint32_t s_stop_prev_demcr = 0;
uint32_t s_stop_prev_dwt_ctrl = 0;
static uint32_t s_stop_prev_exti_imr = 0;
static uint32_t s_stop_prev_exti_emr = 0;
static AhbClockSnapshot s_stop_ahb_snap{};
static uint32_t s_pre_wfi_rcc_cr = 0;
static uint32_t s_pre_wfi_rcc_cfgr = 0;

#endif
[[maybe_unused]] PeripheralClockSnapshot s_stop_periph_snap{};

const char* stop_wake_reason_str(StopWakeReason r) {
    switch (r) {
        case StopWakeReason::WAKE_RTC:      return "RTC/timer";
        case StopWakeReason::WAKE_ACTIVITY: return "activity";
        case StopWakeReason::WAKE_ABORTED:  return "aborted";
        case StopWakeReason::WAKE_UNKNOWN:  return "unknown";
        default:                            return "none";
    }
}
}

#ifdef CONFIG_BOARD_MPS2_AN386
K_TIMER_DEFINE(mps2_stop_timer, simulated_stop_timer_handler, nullptr);
#else
K_TIMER_DEFINE(sim_rtc_timer, simulated_rtc_timer_handler, nullptr);
#endif

#ifndef CONFIG_BOARD_MPS2_AN386
static void log_gpio_plan()
{
    GPIO_TypeDef* const ports[8] = {GPIOA, GPIOB, GPIOC, GPIOD, GPIOE, GPIOF, GPIOG, GPIOH};

    const uint32_t prev_ahb1enr = RCC->AHB1ENR;
    RCC->AHB1ENR = prev_ahb1enr | 0xFFU;
    __DSB();

    for (int i = 0; i < 8; ++i) {
        uint32_t out = 0U;
        uint32_t af  = 0U;
        for (int pin = 0; pin < 16; ++pin) {
            const uint32_t mode = (ports[i]->MODER >> (pin * 2)) & 0x3U;
            if (mode == 0x1U) {
                out |= (1U << pin);
            } else if (mode == 0x2U) {
                af |= (1U << pin);
            }
        }
        LOG_INF("STOP GPIO%c: out=0x%04x lvl=0x%04x | af=0x%04x lvl=0x%04x | od=0x%04x",'A' + i,(unsigned)out, (unsigned)(ports[i]->ODR & out),(unsigned)af,  (unsigned)(ports[i]->IDR & af),(unsigned)(ports[i]->OTYPER & (out | af)));
    }

    RCC->AHB1ENR = prev_ahb1enr;
    __DSB();
}
#endif

StopState& StopState::getInstance() {
    static constinit StopState instance;
    return instance;
}

bool StopState::enter(PowerManager& pm) {
    LOG_WRN("Preparing for Deep Sleep (PM_STATE_SUSPEND_TO_RAM): %u s total, wake every %u s",(unsigned)(STOP_TOTAL_TIME_US / 1000000U),(unsigned)(STOP_WAKE_INTERVAL_US / 1000000U));

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

#ifndef CONFIG_BOARD_MPS2_AN386
    log_gpio_plan();
#endif

#ifndef CONFIG_BOARD_MPS2_AN386

    if ((CoreDebug->DHCSR & CoreDebug_DHCSR_C_DEBUGEN_Msk) != 0U) {
        LOG_WRN("DEBUGGER ATTACHED (DHCSR=0x%08x, DBGMCU_CR=0x%08x): STOP will run as SLEEP, ~0.9 mA. ""End the debug session and power-cycle the board with SWD disconnected to measure STOP.",(unsigned)CoreDebug->DHCSR, (unsigned)DBGMCU->CR);
    }
#endif

    pm_log_flush();

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

    // 2. Save the full AHB clock state BEFORE any helper modifies AHB1ENR/AHB2ENR
    s_stop_ahb_snap.ahb1enr   = RCC->AHB1ENR;
    s_stop_ahb_snap.ahb2enr   = RCC->AHB2ENR;
    s_stop_ahb_snap.ahb3enr   = RCC->AHB3ENR;
    s_stop_ahb_snap.ahb1lpenr = RCC->AHB1LPENR;
    s_stop_ahb_snap.ahb2lpenr = RCC->AHB2LPENR;
    s_stop_ahb_snap.ahb3lpenr = RCC->AHB3LPENR;

    // 3. Downscale system clocks while PWR / Flash clocks are still active
    clock_downscale_for_idle(s_stop_clock_snap);

    // 4. Power down USB, ADC, DAC before the bus clocks are gated
    usb_disable_hardware_and_release_pins();
    adc_save_and_disable(s_stop_adc_snap);
    dac_save_and_disable(s_stop_prev_dac_cr);

    s_stop_prev_demcr    = CoreDebug->DEMCR;
    s_stop_prev_dwt_ctrl = DWT->CTRL;

    // 5. GPIO policy LAST among the pin-touching steps, then the PWR registers
    gpio_save_and_set_all_analog(s_stop_gpio_snap, s_stop_prev_ahb1enr);
    stop_configure_power(s_stop_prev_pwr_cr);

    // 6. Gate AHB1/2/3 Run and Low-Power clocks, keeping FLITFLPEN so Flash can power down
    RCC->AHB1ENR   = 0U;
    RCC->AHB2ENR   = 0U;
    RCC->AHB3ENR   = 0U;
    RCC->AHB1LPENR = RCC_AHB1LPENR_FLITFLPEN;
    RCC->AHB2LPENR = 0U;
    RCC->AHB3LPENR = 0U;
    __DSB();

#if STOP_APB_GATING_ENABLED
    periph_clocks_save_and_gate(s_stop_periph_snap, APB1_STOP_KEEP_MASK, APB2_STOP_KEEP_MASK);
#endif

    s_stop_prev_exti_imr = EXTI->IMR;
    s_stop_prev_exti_emr = EXTI->EMR;

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

    return *this;

#else
    if (!s_real_hw_sleep_active) {
        return *this;
    }

    for (;;) {
        uint32_t systick_ctrl = SysTick->CTRL;
        SysTick->CTRL &= ~(SysTick_CTRL_TICKINT_Msk | SysTick_CTRL_ENABLE_Msk);
        SCB->ICSR = SCB_ICSR_PENDSTCLR_Msk;

        DWT->CTRL = 0U;
        CoreDebug->DEMCR = 0U;

        sys_context.feedWatchdog();

        stop_reassert_power();
        RCC->APB1ENR   = RCC_APB1ENR_PWREN;
        RCC->APB2ENR   = 0U;
        RCC->APB1LPENR = RCC_APB1LPENR_PWRLPEN;
        RCC->APB2LPENR = 0U;
        RCC->AHB1ENR   = 0U;
        RCC->AHB2ENR   = 0U;
        RCC->AHB3ENR   = 0U;
        RCC->AHB1LPENR = RCC_AHB1LPENR_FLITFLPEN;
        RCC->AHB2LPENR = 0U;
        RCC->AHB3LPENR = 0U;
        __DSB();

        s_pre_wfi_exti_pr = EXTI->PR;
        s_pre_wfi_nvic_pending0 = NVIC->ISPR[0];
        s_pre_wfi_rcc_cr = RCC->CR;
        s_pre_wfi_rcc_cfgr = RCC->CFGR;
        s_pre_wfi_pwr_cr  = PWR->CR;
        s_pre_wfi_pwr_csr = PWR->CSR;

        SET_BIT(SCB->SCR, SCB_SCR_SLEEPDEEP_Msk);

        EXTI->IMR = (1U << 17);
        EXTI->EMR = 0U;
        __HAL_GPIO_EXTI_CLEAR_FLAG(0xFFFFFFFFU);   // EXTI->PR, write-1-to-clear

        for (int i = 0; i < 8; ++i) {
            NVIC->ICPR[i] = 0xFFFFFFFFU;
        }

        __DSB();
        __ISB();

        s_probe_scr       = SCB->SCR;
        s_probe_dhcsr     = CoreDebug->DHCSR;
        s_probe_dbgmcu_cr = DBGMCU->CR;

        (void)stop_enter_wfi(s_wfi_pwr_csr, s_wfi_icsr);

        __HAL_RCC_PWR_CLK_ENABLE();
        __DSB();

        s_wfi_wake_count++;
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
            return ActiveState::getInstance();
        }

        if (pm.consumeWakePending()) {
            s_stop_wake_reason = StopWakeReason::WAKE_RTC;

            s_elapsed_sleep_time_us += s_current_sleep_interval_us;

            if (s_elapsed_sleep_time_us >= STOP_TOTAL_TIME_US) {
                return ActiveState::getInstance();
            }

            __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);
            {
                uint32_t vos_timeout = 100000;
                while (!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY) && --vos_timeout) { }
            }
            MODIFY_REG(RCC->CFGR,
                       RCC_CFGR_HPRE | RCC_CFGR_PPRE1 | RCC_CFGR_PPRE2,
                       RCC_CFGR_HPRE_DIV16 | RCC_CFGR_PPRE1_DIV1 | RCC_CFGR_PPRE2_DIV1);
            __HAL_FLASH_SET_LATENCY(FLASH_LATENCY_0);
#ifdef CONFIG_SYS_CLOCK_TICKS_PER_SEC

            {
                const uint32_t svc_load = (1000000U / CONFIG_SYS_CLOCK_TICKS_PER_SEC);
                if (svc_load > 1U) {
                    SysTick->LOAD = svc_load - 1U;
                    SysTick->VAL  = 0U;
                }
            }
#endif

            CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
            DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
            const uint32_t svc_start_cycles = DWT->CYCCNT;

            sys_context.feedWatchdog();

            uint32_t remaining_us = STOP_TOTAL_TIME_US - s_elapsed_sleep_time_us;
            uint32_t next_interval_us = (remaining_us > STOP_WAKE_INTERVAL_US) ?
                                        STOP_WAKE_INTERVAL_US : remaining_us;
            if (next_interval_us < 1000000U) {
                next_interval_us = 1000000U;
            }

            int err = set_rtc_alarm(pm.getRtcDev(), next_interval_us, &pm);
            s_current_sleep_interval_us = next_interval_us;

            s_last_full_wake_cycles = DWT->CYCCNT - svc_start_cycles;

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

    if (s_stop_wake_reason == StopWakeReason::WAKE_RTC || s_stop_wake_reason == StopWakeReason::WAKE_ACTIVITY) {
        pm.resetPmFailures();
    }

    LOG_INF("Exiting PM_STATE_SUSPEND_TO_RAM -> %s (wake: %s, ~%u ms in STOP)",s_transition_target_name,stop_wake_reason_str(s_stop_wake_reason),(unsigned)(s_elapsed_sleep_time_us / 1000U));

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

        LOG_INF("Exiting PM_STATE_SUSPEND_TO_RAM -> %s (simulated STOP, no RTC)",s_transition_target_name);

        pm_policy_state_lock_put(PM_STATE_SUSPEND_TO_IDLE, PM_ALL_SUBSTATES);
        return;
    }

    RCC->AHB1LPENR = s_stop_ahb_snap.ahb1lpenr;
    RCC->AHB2LPENR = s_stop_ahb_snap.ahb2lpenr;
    RCC->AHB3LPENR = s_stop_ahb_snap.ahb3lpenr;
    RCC->AHB1ENR   = s_stop_ahb_snap.ahb1enr;
    RCC->AHB2ENR   = s_stop_ahb_snap.ahb2enr;
    RCC->AHB3ENR   = s_stop_ahb_snap.ahb3enr;
    __DSB();

#if STOP_APB_GATING_ENABLED
    periph_clocks_restore(s_stop_periph_snap);
#endif

    CLEAR_BIT(SCB->SCR, SCB_SCR_SLEEPDEEP_Msk);
    stop_restore_power(s_stop_prev_pwr_cr);

    bool clock_ok = clock_restore_from_idle(s_stop_clock_snap);

    clear_rtc_alarm(pm.getRtcDev());

    gpio_restore_after_wake(s_stop_gpio_snap, s_stop_prev_ahb1enr);
    adc_restore_after_wake(s_stop_adc_snap);
    dac_restore_after_wake(s_stop_prev_dac_cr);
    CoreDebug->DEMCR = s_stop_prev_demcr;
    DWT->CTRL        = s_stop_prev_dwt_ctrl;

    EXTI->IMR = s_stop_prev_exti_imr;
    EXTI->EMR = s_stop_prev_exti_emr;

    HAL_DBGMCU_DisableDBGSleepMode();
    HAL_DBGMCU_DisableDBGStopMode();
    HAL_DBGMCU_DisableDBGStandbyMode();
    DBGMCU->CR = 0U;

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

    LOG_INF("WFI wake: ICSR=0x%08x PWR_CSR=0x%08x PWR_CR=0x%08x ""NVIC_ISPR0=0x%08x NVIC_IABR0=0x%08x",s_wfi_icsr,s_wfi_pwr_csr,s_wfi_pwr_cr,s_wfi_nvic_pending0,s_wfi_nvic_active0);

    LOG_INF("Pre-WFI pending (last iter): EXTI_PR=0x%08x NVIC_ISPR0=0x%08x",s_pre_wfi_exti_pr,s_pre_wfi_nvic_pending0);
    LOG_INF("Pre-WFI RCC_CR=0x%08x RCC_CFGR=0x%08x", s_pre_wfi_rcc_cr, s_pre_wfi_rcc_cfgr);
    LOG_INF("Pre-WFI PWR_CR=0x%08x PWR_CSR=0x%08x (VOS3 expected; PDDS/LPDS/FPDS/UDEN all 0)", s_pre_wfi_pwr_cr, s_pre_wfi_pwr_csr);
    LOG_INF("DBGMCU_CR: right after the firmware cleared it = 0x%08x, earlier probe before WFI = 0x%08x%s",(unsigned)s_stop_dbgmcu_after_write, (unsigned)s_probe_dbgmcu_cr,(s_stop_dbgmcu_after_write == 0U && s_probe_dbgmcu_cr != 0U) ? "  <-- a debug probe is re-asserting it" : (s_stop_dbgmcu_after_write != 0U) ? "  <-- the write is NOT taking effect" : "");

    LOG_INF("Last RTC ALRAF clear: %u/100000 rem, Cycles: %u",s_last_alraf_timeout, s_last_alraf_cycles);

    for (int i = 0; i < 8; ++i) {
        if (s_stop_od_af_mask[i]) {
            LOG_INF("STOP OD lines GPIO%c: od_af=0x%04x low=0x%04x%s", 'A' + i,(unsigned)s_stop_od_af_mask[i], (unsigned)s_stop_od_low_mask[i],s_stop_od_low_mask[i] ? "  <-- STUCK LOW" : "");
        }
    }

    for (int i = 0; i < 8; ++i) {
        const uint32_t m = s_stop_applied_moder[i];
        const uint32_t p = s_stop_applied_pupdr[i];
        if (m == 0xFFFFFFFFU) {
            continue;
        }
        uint32_t out = 0U, in = 0U, pu = 0U, pd = 0U;
        for (int pin = 0; pin < 16; ++pin) {
            const uint32_t mode = (m >> (pin * 2)) & 0x3U;
            const uint32_t pull = (p >> (pin * 2)) & 0x3U;
            if (mode == 0x1U) { out |= (1U << pin); }
            if (mode == 0x0U) { in  |= (1U << pin); }
            if (mode != 0x3U && pull == 0x1U) { pu |= (1U << pin); }
            if (mode != 0x3U && pull == 0x2U) { pd |= (1U << pin); }
        }
        LOG_INF("STOP APPLIED GPIO%c: out=0x%04x in=0x%04x pu=0x%04x pd=0x%04x (pre-policy MODER=0x%08x)",'A' + i, (unsigned)out, (unsigned)in, (unsigned)pu, (unsigned)pd,(unsigned)s_stop_gpio_snap.moder[i]);
    }

    LOG_INF("STOP diag: SCR=0x%08x (SLEEPDEEP=%u) DHCSR=0x%08x (C_DEBUGEN=%u) DBGMCU_CR=0x%08x",(unsigned)s_probe_scr, (unsigned)((s_probe_scr >> 2) & 1U),(unsigned)s_probe_dhcsr, (unsigned)(s_probe_dhcsr & 1U),(unsigned)s_probe_dbgmcu_cr);
    if ((s_probe_dhcsr & 1U) != 0U) {
        LOG_WRN("STOP was NOT a real STOP: a debugger was attached (C_DEBUGEN=1). ""Power-cycle with SWD disconnected, then re-measure.");
    }

    LOG_INF("Full mid-STOP service: %u cycles (~%u us @1MHz)",s_last_full_wake_cycles, s_last_full_wake_cycles / 4U);
    
    LOG_INF("STOP total WFI wake count: %u", (unsigned)s_wfi_wake_count);

    s_wfi_wake_count = 0;

    pm.notifyAfterWakeup();

    if (s_stop_wake_reason == StopWakeReason::WAKE_RTC || s_stop_wake_reason == StopWakeReason::WAKE_ACTIVITY) {
        pm.resetPmFailures();
    }

    resume_background_threads();

#ifdef CONFIG_USB_DEVICE_STACK
    k_work_reschedule(&usb_reconnect_work, K_MSEC(USB_RECONNECT_DELAY_MS));
#endif

    s_real_hw_sleep_active = false;

    LOG_INF("Exiting PM_STATE_SUSPEND_TO_RAM -> %s (wake: %s, ~%u ms in STOP)",s_transition_target_name,stop_wake_reason_str(s_stop_wake_reason),(unsigned)(s_elapsed_sleep_time_us / 1000U));

    pm_policy_state_lock_put(PM_STATE_SUSPEND_TO_IDLE, PM_ALL_SUBSTATES);
#endif
}
