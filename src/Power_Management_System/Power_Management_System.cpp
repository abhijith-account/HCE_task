#include "Power_Internal.h"

LOG_MODULE_REGISTER(PWR_SYS, LOG_LEVEL_INF);

// ============================================================================
// Globals declared in Power_Management_System.h
// ============================================================================
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

using namespace pm_internal;

// ============================================================================
// Device handles used by the monitor thread (the test binary may override the weak ones)
// ============================================================================
extern const struct device* i2c_hardware;
extern const struct device* uart_hardware;
extern const struct device* usb_hardware;
extern const struct device* adc_hardware;

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

// ============================================================================
// PowerManager (singleton): FSM driver, observer registry, wake handling
// ============================================================================
K_MUTEX_DEFINE(pm_state_mutex);
K_MUTEX_DEFINE(pm_observer_mutex);

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

    HAL_DBGMCU_DisableDBGStopMode();
    DBGMCU->CR = 0U;
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

    // Tell the state hooks where we are heading: their exit logs name the destination, and
    // IdleState::exit() needs to know when this is the Idle -> STOP handoff.
    s_transition_target_name = next_state.getName();
    s_idle_exit_to_stop = (old_state == &IdleState::getInstance()) &&
                          (&next_state == &StopState::getInstance());

    if (old_state) {
        old_state->exit(*this);
    }

    s_idle_exit_to_stop = false;

    LOG_INF("Transition: %s -> %s",
            old_state ? old_state->getName() : "NONE",
            next_state.getName());

    // By now the old state's exit has run (UART is up for Active->Idle, Idle->STOP and
    // STOP->Active) and the next state has not yet suspended anything. Give the log thread
    // time to print the exit + transition lines before the next enter() parks the system.
    if (old_state) {
        pm_log_flush();
    }

    if (&next_state == &ActiveState::getInstance()) {
        last_activity_time.store(k_uptime_get_32());
    }

    if (next_state.enter(*this)) {
        current_state = &next_state;
        return;
    }

    LOG_WRN("State %s aborted entry. Evaluating cascaded fallback.", next_state.getName());

    s_transition_target_name = IdleState::getInstance().getName();
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

// ============================================================================
// Monitor thread: initialises PowerManager, then runs processFSM() and waits for the next timeout or wake event
// ============================================================================
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

        if (s_real_hw_sleep_active) {
            // Immediately execute StopState::execute() (__WFI()) on the next iteration
            // without waiting on a SysTick semaphore timeout!
            wait_timeout = K_NO_WAIT;
        } else if (s_in_idle_state) {
            uint32_t idle_elapsed = current_time - s_idle_entry_time_ms;
            if (idle_elapsed < IDLE_TIMEOUT_MS) {
                wait_timeout = K_MSEC(IDLE_TIMEOUT_MS - idle_elapsed);
            } else {
                wait_timeout = K_NO_WAIT;
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
