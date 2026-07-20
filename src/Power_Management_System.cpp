#include "Power_Management_System.h"
#include "Device_State_Machine+Watchdog.h"   // DeviceContext::triggerFault()
#include <zephyr/logging/log.h>
#include <zephyr/pm/pm.h>
#include <zephyr/pm/policy.h>
#include <zephyr/pm/device.h>
#include <zephyr/drivers/counter.h>

LOG_MODULE_REGISTER(PWR_SYS, LOG_LEVEL_INF);

#ifdef IS_TEST_ENVIRONMENT
    extern bool run_thread_once;
    #define THREAD_LOOP_CONDITION (run_thread_once ? (run_thread_once = false, true) : false)
#else
    #define THREAD_LOOP_CONDITION true
#endif

constexpr uint32_t ACTIVE_TIMEOUT_MS = 30000;
constexpr uint32_t IDLE_TIMEOUT_MS   = 5000;
constexpr uint32_t STOP_WAKEUP_US    = 60000000;
constexpr uint32_t THREAD_PERIOD_MS  = 1000;

extern const struct device* i2c_hardware;
extern const struct device* uart_hardware;
extern const struct device* usb_hardware;
extern DeviceContext sys_context;

// Mock the RTC hardware pointer injection for tests to avoid DEVICE_DT_GET null resolution
#ifdef IS_TEST_ENVIRONMENT
    // Using a weak symbol prevents undefined reference errors in other test suites 
    // (like test_usb_shell) that link this file but don't define their own mock.
    __attribute__((weak)) const struct device* rtc_hardware = nullptr;
    __attribute__((weak)) const struct device* uart_hardware = nullptr;
    __attribute__((weak)) const struct device* usb_hardware = nullptr;
#else
    const struct device* const rtc_hardware = DEVICE_DT_GET(DT_NODELABEL(rtc));

    // NOTE: bound directly to the usart2/usbotg_fs devicetree nodes (the
    // physical BMS UART and USB CDC controller from the board overlay) --
    // this is a deliberately separate device handle from whatever
    // Smart_Battery_System.cpp's file-local `uart_hardware` currently
    // resolves to for UARTManager's traffic (at time of writing, that one
    // is bound to DT_CHOSEN(zephyr_console) instead of usart2). Suspending
    // this handle during STOP suspends the usart2 peripheral itself
    // regardless of what any other subsystem thinks it's talking to; if
    // BMS traffic is not actually flowing over usart2, this suspend/resume
    // does not correspond to where that traffic lives. Reconcile which
    // physical UART the BMS is meant to use before relying on this to
    // gate BMS communication across sleep.
    const struct device* const uart_hardware = DEVICE_DT_GET(DT_NODELABEL(usart2));
    const struct device* const usb_hardware = DEVICE_DT_GET(DT_NODELABEL(usbotg_fs));
#endif

// ---------------------------------------------------------
// PowerManager Implementation
// ---------------------------------------------------------
PowerManager::PowerManager()
    : current_state(nullptr),
      last_sleep_time(0),
      expected_wake_time(0),
      rtc_dev(nullptr),
      i2c_dev(nullptr),
      uart_dev(nullptr),
      usb_dev(nullptr),
      observer_count(0),
      fault_context(nullptr),
      consecutive_pm_failures(0)
{
    k_mutex_init(&state_mutex);
    k_mutex_init(&observer_mutex);
    atomic_set(&wake_pending, 0);
}

PowerManager& PowerManager::getInstance() {
    static PowerManager instance;
    return instance;
}

bool PowerManager::init(const struct device* rtc, const struct device* i2c,
                         const struct device* uart, const struct device* usb,
                         DeviceContext* fault_ctx) {
    rtc_dev = rtc;
    i2c_dev = i2c;
    uart_dev = uart;
    usb_dev = usb;
    fault_context = fault_ctx;
    last_activity_time.store(k_uptime_get_32());

    if (!rtc_dev || !device_is_ready(rtc_dev)) {
        LOG_ERR("RTC device not ready");
        return false;
    }
    if (!i2c_dev || !device_is_ready(i2c_dev)) {
        LOG_ERR("I2C device not ready");
        return false;
    }
    if (!uart_dev || !device_is_ready(uart_dev)) {
        LOG_ERR("UART device not ready");
        return false;
    }
    if (!usb_dev || !device_is_ready(usb_dev)) {
        LOG_ERR("USB device not ready");
        return false;
    }

    int err = counter_start(rtc_dev);
    if (err) {
        LOG_ERR("Failed to start RTC counter (err: %d)", err);
        return false;
    }

    pm_policy_state_lock_get(PM_STATE_SUSPEND_TO_RAM, PM_ALL_SUBSTATES);

    LOG_INF("Power Manager initialized. Deep Sleep Locked.");

    k_mutex_lock(&state_mutex, K_FOREVER);
    transitionTo(ActiveState::getInstance());
    k_mutex_unlock(&state_mutex);

    return true;
}

bool PowerManager::registerObserver(IPowerObserver* obs) {
    k_mutex_lock(&observer_mutex, K_FOREVER);

    for (size_t i = 0; i < observer_count; ++i) {
        if (observers[i] == obs) {
            k_mutex_unlock(&observer_mutex);
            return true;
        }
    }

    if (observer_count < MAX_OBSERVERS) {
        observers[observer_count++] = obs;
        k_mutex_unlock(&observer_mutex);
        return true;
    }

    k_mutex_unlock(&observer_mutex);
    LOG_ERR("Observer limit reached");
    return false;
}

size_t PowerManager::captureObservers(std::array<IPowerObserver*, MAX_OBSERVERS>& out) {
    k_mutex_lock(&observer_mutex, K_FOREVER);
    size_t count = (observer_count > MAX_OBSERVERS) ? MAX_OBSERVERS : observer_count;
    for (size_t i = 0; i < count; ++i) out[i] = observers[i];
    k_mutex_unlock(&observer_mutex);
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

    k_mutex_lock(&state_mutex, K_FOREVER);
    if (current_state != &ActiveState::getInstance()) {
        LOG_INF("Hardware Activity Detected! Waking system...");
        transitionTo(ActiveState::getInstance());
    }
    k_mutex_unlock(&state_mutex);
}

void PowerManager::reportPmFailure() {
    ++consecutive_pm_failures;
    if (consecutive_pm_failures == PM_FAILURE_FAULT_THRESHOLD) {
        LOG_ERR("Power management failure threshold reached (%u consecutive). Escalating.",
                consecutive_pm_failures);
        if (fault_context) {
            fault_context->triggerFault("Power Management Failure");
        }
    }
}

void PowerManager::transitionTo(IPowerState& next_state) {
    if (current_state == &next_state) {
        return;
    }

    LOG_INF("Transition: %s -> %s",
            current_state ? current_state->getName() : "NONE",
            next_state.getName());

    if (current_state) {
        current_state->exit(*this);
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

    // Fire the fault handler first. It may re-enter the power manager
    // (e.g. via reportActivity()) to force the system awake for safe-halt
    // handling — that reentrant transition must not be allowed to
    // resurrect the FSM out of the halted state, so current_state is
    // pinned to nullptr *after* the callback returns, not before.
    if (fault_context) {
        fault_context->triggerFault("Power Manager FSM Halted");
    }
    current_state = nullptr;
}

void PowerManager::processFSM() {
    if (atomic_cas(&wake_pending, 1, 0)) {
        LOG_WRN("=== RTC Wakeup Triggered ===");
        reportActivity();
    }

    IPowerState* local_state = nullptr;

    k_mutex_lock(&state_mutex, K_FOREVER);
    local_state = current_state;
    k_mutex_unlock(&state_mutex);

    if (local_state) {
        IPowerState& next_state = local_state->execute(*this);

        if (&next_state != local_state) {
            k_mutex_lock(&state_mutex, K_FOREVER);
            if (current_state == local_state) {
                transitionTo(next_state);
            }
            k_mutex_unlock(&state_mutex);
        }
    }
}

void PowerManager::rtc_alarm_handler(const struct device* /* dev */, uint8_t /* chan_id */,
                                      uint32_t /* ticks */, void* user_data) {
    auto* self = static_cast<PowerManager*>(user_data);
    atomic_set(&self->wake_pending, 1);
}

#ifdef IS_TEST_ENVIRONMENT
void PowerManager::resetForTest() {
    k_mutex_lock(&state_mutex, K_FOREVER);
    current_state = nullptr;
    k_mutex_unlock(&state_mutex);

    last_activity_time.store(0);
    last_sleep_time = 0;
    expected_wake_time = 0;
    consecutive_pm_failures = 0;
    atomic_set(&wake_pending, 0);

    k_mutex_lock(&observer_mutex, K_FOREVER);
    observer_count = 0;
    observers.fill(nullptr);
    k_mutex_unlock(&observer_mutex);

    StopState::getInstance().resetForTest();
}
#endif

// ---------------------------------------------------------
// State Pattern Concrete Types
// ---------------------------------------------------------

// --- ACTIVE STATE ---
ActiveState& ActiveState::getInstance() {
    static constinit ActiveState instance; 
    return instance;
}
bool ActiveState::enter(PowerManager& /*pm*/) { return true; }
IPowerState& ActiveState::execute(PowerManager& pm) {
    uint32_t elapsed = k_uptime_get_32() - pm.getLastActivityTime();
    if (elapsed >= ACTIVE_TIMEOUT_MS) {
        return IdleState::getInstance();
    }
    return *this;
}
void ActiveState::exit(PowerManager& /*pm*/) {}

// --- IDLE STATE ---
IdleState& IdleState::getInstance() {
    static constinit IdleState instance;
    return instance;
}
#ifdef IS_TEST_ENVIRONMENT
// 1. Weak definition moved to the global scope
__attribute__((weak)) bool g_force_idle_enter_fail = false;
#endif

bool IdleState::enter(PowerManager& /*pm*/) { 
#ifdef IS_TEST_ENVIRONMENT
    // 2. Only the evaluation remains inside the function
    if (g_force_idle_enter_fail) return false;
#endif
    return true; 
}
IPowerState& IdleState::execute(PowerManager& pm) {
    uint32_t elapsed = k_uptime_get_32() - pm.getLastActivityTime();
    if (elapsed >= (ACTIVE_TIMEOUT_MS + IDLE_TIMEOUT_MS)) {
        return StopState::getInstance();
    }
    return *this;
}
void IdleState::exit(PowerManager& /*pm*/) {}

// --- STOP STATE ---
StopState& StopState::getInstance() {
    static constinit StopState instance;
    return instance;
}

bool StopState::enter(PowerManager& pm) {
    LOG_WRN("Preparing for Deep Sleep (STOP Mode)");
    sleep_prepared = false;

    int err = counter_cancel_channel_alarm(pm.getRtcDev(), 0);
    if (err && err != -ETIME && err != -ENOTSUP) {
        LOG_WRN("Failed to cancel RTC alarm (err: %d)", err);
    }

    struct counter_alarm_cfg alarm_cfg = {};
    alarm_cfg.flags = 0;

    uint32_t ticks = counter_us_to_ticks(pm.getRtcDev(), STOP_WAKEUP_US);
    if (ticks == 0) {
        LOG_ERR("Alarm tick conversion failed or overflowed. Aborting STOP entry.");
        pm.reportPmFailure();
        return false;
    }

    alarm_cfg.ticks = ticks;
    alarm_cfg.callback = PowerManager::rtc_alarm_handler;
    alarm_cfg.user_data = &pm;

    err = counter_set_channel_alarm(pm.getRtcDev(), 0, &alarm_cfg);
    if (err) {
        LOG_ERR("Failed to set RTC alarm (err: %d). Aborting STOP entry.", err);
        pm.reportPmFailure();
        return false;
    }

    pm.recordSleepTime();
    pm.setExpectedWakeTime(pm.getSleepTime() + (STOP_WAKEUP_US / 1000));

    pm.notifyBeforeSleep();

    err = pm_device_action_run(pm.getI2cDev(), PM_DEVICE_ACTION_SUSPEND);
    if (err && err != -EALREADY) {
        LOG_ERR("Failed to suspend I2C peripheral (err: %d). Rolling back.", err);
        (void)counter_cancel_channel_alarm(pm.getRtcDev(), 0);
        pm.notifySleepAborted();
        pm.reportPmFailure();
        return false;
    }

    // Suspended in I2C -> UART -> USB order; on failure, roll back only the
    // peripherals that were already suspended by this same call (in reverse)
    // before aborting, so a partial STOP entry never leaves hardware in a
    // half-suspended state.
    err = pm_device_action_run(pm.getUartDev(), PM_DEVICE_ACTION_SUSPEND);
    if (err && err != -EALREADY) {
        LOG_ERR("Failed to suspend UART peripheral (err: %d). Rolling back.", err);
        (void)pm_device_action_run(pm.getI2cDev(), PM_DEVICE_ACTION_RESUME);
        (void)counter_cancel_channel_alarm(pm.getRtcDev(), 0);
        pm.notifySleepAborted();
        pm.reportPmFailure();
        return false;
    }

    err = pm_device_action_run(pm.getUsbDev(), PM_DEVICE_ACTION_SUSPEND);
    if (err && err != -EALREADY) {
        LOG_ERR("Failed to suspend USB peripheral (err: %d). Rolling back.", err);
        (void)pm_device_action_run(pm.getUartDev(), PM_DEVICE_ACTION_RESUME);
        (void)pm_device_action_run(pm.getI2cDev(), PM_DEVICE_ACTION_RESUME);
        (void)counter_cancel_channel_alarm(pm.getRtcDev(), 0);
        pm.notifySleepAborted();
        pm.reportPmFailure();
        return false;
    }

    sleep_prepared = true;
    pm_policy_state_lock_put(PM_STATE_SUSPEND_TO_RAM, PM_ALL_SUBSTATES);
    return true;
}

IPowerState& StopState::execute(PowerManager& /*pm*/) {
    return *this;
}

void StopState::exit(PowerManager& pm) {
    if (!sleep_prepared) {
        return;
    }

    int err = counter_cancel_channel_alarm(pm.getRtcDev(), 0);
    if (err && err != -ETIME && err != -ENOTSUP) {
        LOG_WRN("Failed to cancel RTC alarm on STOP exit (err: %d)", err);
    }

    pm_policy_state_lock_get(PM_STATE_SUSPEND_TO_RAM, PM_ALL_SUBSTATES);

    uint32_t actual_wake = k_uptime_get_32();
    uint32_t expected_wake = pm.getExpectedWakeTime();
    int32_t delay = static_cast<int32_t>(actual_wake - expected_wake);

    if (delay >= 0) {
        LOG_INF("System Awoken via RTC. RTC wake delay: %d ms", delay);
    } else {
        LOG_INF("System Awoken Early (External Preemption).");
    }

    // Resumed in the reverse of suspend order (USB -> UART -> I2C). Each
    // resume is attempted regardless of whether an earlier one failed --
    // partial resume-on-error would leave working peripherals stuck
    // suspended for no reason -- and the overall success/failure across all
    // three is what drives reportPmFailure()/resetPmFailures(), matching the
    // single-failure-counter contract documented on those methods.
    bool resume_ok = true;

    err = pm_device_action_run(pm.getUsbDev(), PM_DEVICE_ACTION_RESUME);
    if (err && err != -EALREADY) {
        LOG_ERR("Failed to resume USB (err: %d)", err);
        resume_ok = false;
    }

    err = pm_device_action_run(pm.getUartDev(), PM_DEVICE_ACTION_RESUME);
    if (err && err != -EALREADY) {
        LOG_ERR("Failed to resume UART (err: %d)", err);
        resume_ok = false;
    }

    err = pm_device_action_run(pm.getI2cDev(), PM_DEVICE_ACTION_RESUME);
    if (err && err != -EALREADY) {
        LOG_ERR("Failed to resume I2C (err: %d)", err);
        resume_ok = false;
    }

    if (resume_ok) {
        pm.resetPmFailures();
    } else {
        pm.reportPmFailure();
    }

    pm.notifyAfterWakeup();
    sleep_prepared = false;
}

// ---------------------------------------------------------
// RTOS Thread
// ---------------------------------------------------------
void power_monitor_thread() {
    auto& pwr_manager = PowerManager::getInstance();
    if (!pwr_manager.init(rtc_hardware, i2c_hardware, uart_hardware, usb_hardware, &sys_context)) {
        LOG_ERR("Power Manager Init Failed. Thread halting.");
        return;
    }

    do {
        pwr_manager.processFSM();

        size_t unused;
        if (k_thread_stack_space_get(k_current_get(), &unused) == 0) {
            LOG_DBG("Power Thread Stack Remaining: %zu bytes", unused);
        }

        k_msleep(THREAD_PERIOD_MS);
    } while (THREAD_LOOP_CONDITION);
}

K_THREAD_DEFINE(pr_tid, 1024, power_monitor_thread, NULL, NULL, NULL, 14, 0, 0);
