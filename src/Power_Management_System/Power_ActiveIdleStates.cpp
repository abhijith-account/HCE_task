// Power_ActiveIdleStates.cpp
// Concrete IPowerState implementations for the two light states: ActiveState and IdleState.
// The deep-sleep state (StopState) is in Power_StopState.cpp.

#include "Power_Internal.h"

LOG_MODULE_DECLARE(PWR_SYS, LOG_LEVEL_INF);

namespace pm_internal {
uint32_t s_idle_entry_time_ms = 0;
bool s_in_idle_state = false;
// Written by PowerManager::transitionTo() before it calls exit() on the old state.
const char* s_transition_target_name = "UNKNOWN";
bool s_idle_exit_to_stop = false;
}
using namespace pm_internal;

namespace {
[[maybe_unused]] PeripheralClockSnapshot s_idle_periph_snap{};
ClockSnapshot s_idle_clock_snap{};
}

#ifndef CONFIG_BOARD_MPS2_AN386
// ---------------------------------------------------------------------------
// Active-state load (see ACTIVE_* in Power_Internal.h): extra bus clocks + a duty-cycled CPU burner.
// Everything is undone in ActiveState::exit(), so Idle and STOP are not affected.
// ---------------------------------------------------------------------------
namespace {
struct ActiveClockAdds {            // bits THIS code switched on (were off before), so exit() only clears those
    uint32_t ahb1, ahb2, ahb3, apb1, apb2;
    uint32_t ahb1lp, ahb2lp, ahb3lp, apb1lp, apb2lp;
};
ActiveClockAdds s_active_adds{};
bool s_active_burn_started = false;

void active_clocks_on() {
    s_active_adds = {
        ACTIVE_AHB1_EXTRA_CLOCKS & ~RCC->AHB1ENR, ACTIVE_AHB2_EXTRA_CLOCKS & ~RCC->AHB2ENR,
        ACTIVE_AHB3_EXTRA_CLOCKS & ~RCC->AHB3ENR, ACTIVE_APB1_EXTRA_CLOCKS & ~RCC->APB1ENR,
        ACTIVE_APB2_EXTRA_CLOCKS & ~RCC->APB2ENR,
        ACTIVE_AHB1_EXTRA_CLOCKS & ~RCC->AHB1LPENR, ACTIVE_AHB2_EXTRA_CLOCKS & ~RCC->AHB2LPENR,
        ACTIVE_AHB3_EXTRA_CLOCKS & ~RCC->AHB3LPENR, ACTIVE_APB1_EXTRA_CLOCKS & ~RCC->APB1LPENR,
        ACTIVE_APB2_EXTRA_CLOCKS & ~RCC->APB2LPENR,
    };
    RCC->AHB1ENR |= ACTIVE_AHB1_EXTRA_CLOCKS;  RCC->AHB1LPENR |= ACTIVE_AHB1_EXTRA_CLOCKS;
    RCC->AHB2ENR |= ACTIVE_AHB2_EXTRA_CLOCKS;  RCC->AHB2LPENR |= ACTIVE_AHB2_EXTRA_CLOCKS;
    RCC->AHB3ENR |= ACTIVE_AHB3_EXTRA_CLOCKS;  RCC->AHB3LPENR |= ACTIVE_AHB3_EXTRA_CLOCKS;
    RCC->APB1ENR |= ACTIVE_APB1_EXTRA_CLOCKS;  RCC->APB1LPENR |= ACTIVE_APB1_EXTRA_CLOCKS;
    RCC->APB2ENR |= ACTIVE_APB2_EXTRA_CLOCKS;  RCC->APB2LPENR |= ACTIVE_APB2_EXTRA_CLOCKS;
    __DSB();
}

void active_clocks_off() {
    RCC->APB1LPENR &= ~s_active_adds.apb1lp;  RCC->APB1ENR &= ~s_active_adds.apb1;
    RCC->APB2LPENR &= ~s_active_adds.apb2lp;  RCC->APB2ENR &= ~s_active_adds.apb2;
    RCC->AHB1LPENR &= ~s_active_adds.ahb1lp;  RCC->AHB1ENR &= ~s_active_adds.ahb1;
    RCC->AHB2LPENR &= ~s_active_adds.ahb2lp;  RCC->AHB2ENR &= ~s_active_adds.ahb2;
    RCC->AHB3LPENR &= ~s_active_adds.ahb3lp;  RCC->AHB3ENR &= ~s_active_adds.ahb3;
    __DSB();
    s_active_adds = {};
}

// Lowest application priority: it only runs when nothing else is ready, but it keeps the core out of WFI for
// its busy part. Sleeps for the rest of each period so the core still idles ACTIVE_BURN_PERCENT-complement.
void active_burn_fn(void*, void*, void*) {
    constexpr uint32_t period_us = ACTIVE_BURN_PERIOD_MS * 1000U;
    constexpr uint32_t burn_us   = period_us / 100U * ACTIVE_BURN_PERCENT;
    constexpr uint32_t sleep_us  = period_us - burn_us;
    static uint32_t buf[64];
    static volatile uint32_t sink;
    for (;;) {
        if constexpr (ACTIVE_BURN_HEAVY) {
            // Multiply + SRAM load/store loop: much higher core switching activity than a spin.
            const uint32_t end = k_cycle_get_32() + k_us_to_cyc_ceil32(burn_us);
            uint32_t acc = 1U;
            while ((int32_t)(k_cycle_get_32() - end) < 0) {
                for (uint32_t i = 0; i < 64U; ++i) {
                    acc = acc * 1664525U + 1013904223U + buf[i];
                    buf[i] = acc ^ (acc >> 7);
                }
            }
            sink = acc;
        } else {
            k_busy_wait(burn_us);
        }
        if (sleep_us != 0U) {
            k_usleep(sleep_us);
        }
    }
}
}  // namespace

// Created un-started (delay = forever); ActiveState::enter() starts / resumes it, exit() suspends it.
K_THREAD_DEFINE(active_burn_tid, 768, active_burn_fn, NULL, NULL, NULL,
                K_LOWEST_APPLICATION_THREAD_PRIO, 0, SYS_FOREVER_MS);
#endif

// ============================================================================
// ActiveState
// ============================================================================
ActiveState& ActiveState::getInstance() {
    static constinit ActiveState instance;
    return instance;
}

bool ActiveState::enter(PowerManager& pm) {
    LOG_INF("Entering PM_STATE_ACTIVE");
    pm_policy_state_lock_get(PM_STATE_SUSPEND_TO_IDLE, PM_ALL_SUBSTATES);
#ifndef CONFIG_BOARD_MPS2_AN386
    active_clocks_on();
    if (ACTIVE_BURN_PERCENT > 0U) {
        if (!s_active_burn_started) {
            s_active_burn_started = true;
            k_thread_start(active_burn_tid);
        } else {
            k_thread_resume(active_burn_tid);
        }
    }
#endif
    return true;
}

IPowerState& ActiveState::execute(PowerManager& pm) {
    uint32_t elapsed = k_uptime_get_32() - pm.getLastActivityTime();
    if (elapsed >= ACTIVE_TIMEOUT_MS) {
        return IdleState::getInstance();
    }
    return *this;
}

void ActiveState::exit(PowerManager& pm) {
    // UART and the log thread are still fully alive here, so this line prints.
    // transitionTo() flushes the log queue before IdleState::enter() suspends anything.
    LOG_INF("Exiting PM_STATE_ACTIVE -> %s (no activity for %u ms)",
            s_transition_target_name,
            (unsigned)(k_uptime_get_32() - pm.getLastActivityTime()));

    pm_policy_state_lock_put(PM_STATE_SUSPEND_TO_IDLE, PM_ALL_SUBSTATES);
#ifndef CONFIG_BOARD_MPS2_AN386
    if (ACTIVE_BURN_PERCENT > 0U) {
        k_thread_suspend(active_burn_tid);   // must be parked before Idle/STOP, or the core never sleeps
    }
    active_clocks_off();
#endif
}

// ============================================================================
// IdleState (suspend-to-idle): downscales clocks, gates peripherals, suspends devices
// ============================================================================
IdleState& IdleState::getInstance() {
    static constinit IdleState instance;
    return instance;
}

bool IdleState::enter(PowerManager& pm) {
#ifdef IS_TEST_ENVIRONMENT
    if (g_force_idle_enter_fail) return false;
#endif
    LOG_INF("Entering PM_STATE_SUSPEND_TO_IDLE");
    // Let this line out before the UART is suspended and the log/background threads are parked.
    pm_log_flush();

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

    HAL_DBGMCU_DisableDBGSleepMode();
    HAL_DBGMCU_DisableDBGStopMode();
    HAL_DBGMCU_DisableDBGStandbyMode();
    DBGMCU->CR = 0U;
#endif

    clock_downscale_for_idle(s_idle_clock_snap, IDLE_HCLK_DIV);

#if STOP_APB_GATING_ENABLED
    periph_clocks_save_and_gate(s_idle_periph_snap, APB1_STOP_KEEP_MASK, APB2_STOP_KEEP_MASK);
#ifndef CONFIG_BOARD_MPS2_AN386
    // Deliberately clock a set of idle peripherals (ENR + LPENR) to raise Idle current to ~5 mA.
    // The snapshot above was taken BEFORE this, so periph_clocks_restore() in exit() undoes it.
    RCC->APB1ENR   |= IDLE_APB1_EXTRA_CLOCKS;
    RCC->APB2ENR   |= IDLE_APB2_EXTRA_CLOCKS;
    RCC->APB1LPENR |= IDLE_APB1_EXTRA_CLOCKS;
    RCC->APB2LPENR |= IDLE_APB2_EXTRA_CLOCKS;
    __DSB();
#endif
#endif

    return true;
}

IPowerState& IdleState::execute(PowerManager& pm) {
    (void)pm;
    uint32_t idle_elapsed = k_uptime_get_32() - s_idle_entry_time_ms;
    if (idle_elapsed >= IDLE_TIMEOUT_MS) {
        return StopState::getInstance();
    }
    return *this;
}

void IdleState::exit(PowerManager& pm) {
    // transitionTo() sets this only for Idle -> STOP. In that handoff StopState::enter()
    // follows immediately, so there is no point waking the ADC and the background threads
    // just to park them again a few ms later.
    const bool to_stop = s_idle_exit_to_stop;

    s_in_idle_state = false;
#ifndef CONFIG_BOARD_MPS2_AN386
    HAL_DBGMCU_DisableDBGSleepMode();
    HAL_DBGMCU_DisableDBGStopMode();
    HAL_DBGMCU_DisableDBGStandbyMode();
    DBGMCU->CR = 0U;
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
    if (!to_stop) {
        (void)pm_device_resume_tolerant(pm.getAdcDev(), "ADC");
    }
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
    if (pm.getAdcDev() && !to_stop) {
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

    if (!to_stop) {
        resume_background_threads();
    }

    // UART is back up at this point, so this prints. transitionTo() flushes the log queue
    // before StopState::enter() suspends the UART again.
    LOG_INF("Exiting PM_STATE_SUSPEND_TO_IDLE -> %s (idle for %u ms)%s",
            s_transition_target_name,
            (unsigned)(k_uptime_get_32() - s_idle_entry_time_ms),
            to_stop ? " [handoff: ADC + background threads stay parked]" : "");
}
