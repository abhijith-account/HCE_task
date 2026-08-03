#include "Power_Management_System.h"
#include "Device_State_Machine+Watchdog.h"
#include <zephyr/logging/log.h>
#include <zephyr/pm/pm.h>
#include <zephyr/pm/policy.h>
#include <zephyr/pm/device.h>
#include <zephyr/drivers/rtc.h>
#include <cstring>

LOG_MODULE_REGISTER(PWR_SYS, LOG_LEVEL_INF);

#ifdef IS_TEST_ENVIRONMENT
    extern bool run_thread_once;
    __attribute__((weak)) bool g_force_init_fail = false;
    #define THREAD_LOOP_CONDITION (run_thread_once ? (run_thread_once = false, true) : false)
#else
    #define THREAD_LOOP_CONDITION true
#endif

static struct k_timer sim_rtc_timer;
static void sim_rtc_timer_handler(struct k_timer *timer_id) {
    PowerManager::rtc_alarm_handler(nullptr, 0, &PowerManager::getInstance());
}

constexpr uint32_t ACTIVE_TIMEOUT_MS = 30000;
constexpr uint32_t IDLE_TIMEOUT_MS   = 5000;
constexpr uint32_t STOP_WAKEUP_US    = 60000000;
constexpr uint32_t THREAD_PERIOD_MS  = 1000;

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
    const struct device* rtc_hardware = DEVICE_DT_GET_OR_NULL(DT_ALIAS(rtc0));
    #ifndef (CONFIG_BOARD_MPS2_AN386)
    const struct device* adc_hardware = DEVICE_DT_GET_OR_NULL(DT_NODELABEL(adc1));
    #else
    const struct device* adc_hardware = DEVICE_DT_GET_OR_NULL(DT_NODELABEL(adc_emul0));
    #endif
    const struct device* usb_hardware = DEVICE_DT_GET_OR_NULL(DT_ALIAS(cdc_acm_uart0));

    #if DT_NODE_EXISTS(DT_CHOSEN(zephyr_console)) && DT_NODE_HAS_COMPAT(DT_CHOSEN(zephyr_console), st_stm32_usart)
        const struct device* uart_hardware = DEVICE_DT_GET(DT_CHOSEN(zephyr_console));
    #else
        const struct device* uart_hardware = DEVICE_DT_GET_OR_NULL(DT_ALIAS(uart_hardware));
    #endif
#endif

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
    k_mutex_init(&state_mutex);
    k_mutex_init(&observer_mutex);
    atomic_set(&wake_pending, 0);
}

PowerManager& PowerManager::getInstance() {
    static PowerManager instance;
    return instance;
}

bool PowerManager::init(const struct device* rtc, const struct device* i2c,const struct device* uart, const struct device* usb,const struct device* adc, DeviceContext* fault_ctx) {
    #ifdef IS_TEST_ENVIRONMENT
    // Allows the test framework to evaluate the false branch of pwr_manager.init()
    if (g_force_init_fail) return false;
    #endif
    rtc_dev = rtc;
    i2c_dev = i2c;
    uart_dev = uart;
    usb_dev = usb;
    adc_dev = adc;
    fault_context = fault_ctx;
    last_activity_time.store(k_uptime_get_32());

    if (rtc_dev == nullptr || !device_is_ready(rtc_dev)) {
        LOG_WRN("RTC device not ready. Bypassing for simulation.");
        rtc_dev = nullptr;
        k_timer_init(&sim_rtc_timer, sim_rtc_timer_handler, NULL);
    }
    if (i2c_dev == nullptr || !device_is_ready(i2c_dev)) {
        LOG_WRN("I2C device not ready. Bypassing for simulation.");
        i2c_dev = nullptr;
    }
    if (uart_dev == nullptr || !device_is_ready(uart_dev)) {
        LOG_WRN("UART device not ready. Bypassing for simulation.");
        uart_dev = nullptr;
    }
    if (usb_dev == nullptr || !device_is_ready(usb_dev)) {
        LOG_WRN("USB device not ready. Bypassing for simulation.");
        usb_dev = nullptr;
    }
    
    if (adc_dev == nullptr || !device_is_ready(adc_dev)) {
        LOG_WRN("ADC device not ready. Bypassing for simulation.");
        adc_dev = nullptr;
    }

    if (rtc_dev) {
        /* Seed the RTC if it is uninitialized (important for QEMU emulator) */
        struct rtc_time time;
        if (rtc_get_time(rtc_dev, &time) != 0) {
            memset(&time, 0, sizeof(time));
            time.tm_mday = 1;
            time.tm_year = 126; // 2026
            rtc_set_time(rtc_dev, &time);
        }
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

void PowerManager::rtc_alarm_handler(const struct device* , uint16_t , void* user_data) {
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

ActiveState& ActiveState::getInstance() {
    static constinit ActiveState instance;
    return instance;
}
bool ActiveState::enter(PowerManager& ) { return true; }
IPowerState& ActiveState::execute(PowerManager& pm) {
    uint32_t elapsed = k_uptime_get_32() - pm.getLastActivityTime();
    if (elapsed >= ACTIVE_TIMEOUT_MS) {
        return IdleState::getInstance();
    }
    return *this;
}
void ActiveState::exit(PowerManager& ) {}

IdleState& IdleState::getInstance() {
    static constinit IdleState instance;
    return instance;
}
#ifdef IS_TEST_ENVIRONMENT
__attribute__((weak)) bool g_force_idle_enter_fail = false;
#endif

bool IdleState::enter(PowerManager& ) {
#ifdef IS_TEST_ENVIRONMENT
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
void IdleState::exit(PowerManager& ) {}

StopState& StopState::getInstance() {
    static constinit StopState instance;
    return instance;
}

bool StopState::enter(PowerManager& pm) {
    LOG_WRN("Preparing for Deep Sleep (STOP Mode)");
    sleep_prepared = false;
    int err = 0;
    if (pm.getRtcDev() != nullptr) {
        /* 1. Disable existing alarm by passing a null mask/time */
        rtc_alarm_set_time(pm.getRtcDev(), 0, 0, nullptr);

        /* 2. Get current time */
        struct rtc_time time;
        rtc_get_time(pm.getRtcDev(), &time);

        /* 3. Add STOP_WAKEUP_US (60 seconds) */
        time.tm_sec += (STOP_WAKEUP_US / 1000000);

        /* Normalize seconds and minutes for the 60s jump */
        if (time.tm_sec >= 60) {
            time.tm_sec -= 60;
            time.tm_min += 1;
            if (time.tm_min >= 60) {
                time.tm_min -= 60;
                time.tm_hour += 1;
                if (time.tm_hour >= 24) time.tm_hour = 0;
            }
        }

        /* 4. Set Callback and Alarm */
        rtc_alarm_set_callback(pm.getRtcDev(), 0, PowerManager::rtc_alarm_handler, &pm);

        uint16_t mask = RTC_ALARM_TIME_MASK_SECOND | RTC_ALARM_TIME_MASK_MINUTE | RTC_ALARM_TIME_MASK_HOUR;
        err = rtc_alarm_set_time(pm.getRtcDev(), 0, mask, &time);

        if (err) {
            LOG_ERR("Failed to set RTC alarm (err: %d). Aborting STOP entry.", err);
            pm.reportPmFailure();
            return false;
        }
    } else {
        LOG_WRN("RTC disabled in simulation. Bypassing hardware wake alarm setup.");
        k_timer_start(&sim_rtc_timer, K_USEC(STOP_WAKEUP_US), K_NO_WAIT);
    }

    pm.recordSleepTime();
    pm.setExpectedWakeTime(pm.getSleepTime() + (STOP_WAKEUP_US / 1000));

    pm.notifyBeforeSleep();
    
    err = pm_device_action_run(pm.getAdcDev(), PM_DEVICE_ACTION_SUSPEND);
    if (err && err != -EALREADY && err != -ENOSYS && err != -ENOTSUP) {
        LOG_ERR("Failed to suspend ADC peripheral (err: %d). Rolling back.", err);
        if (pm.getRtcDev()) {
            rtc_alarm_set_time(pm.getRtcDev(), 0, 0, nullptr);
        }
        pm.notifySleepAborted();
        pm.reportPmFailure();
        return false;
    }

    err = pm_device_action_run(pm.getI2cDev(), PM_DEVICE_ACTION_SUSPEND);
    if (err && err != -EALREADY && err != -ENOSYS && err != -ENOTSUP) {
        LOG_ERR("Failed to suspend I2C peripheral (err: %d). Rolling back.", err);
        if (pm.getRtcDev()) {
            rtc_alarm_set_time(pm.getRtcDev(), 0, 0, nullptr);
        }
        pm.notifySleepAborted();
        pm.reportPmFailure();
        return false;
    }

   #ifndef IS_TEST_ENVIRONMENT
     #if DT_NODE_EXISTS(DT_CHOSEN(zephyr_console)) && DT_NODE_HAS_COMPAT(DT_CHOSEN(zephyr_console), st_stm32_usart)
      err = pm_device_action_run(pm.getUartDev(), PM_DEVICE_ACTION_SUSPEND);
      if (err && err != -EALREADY && err != -ENOSYS && err != -ENOTSUP) {
          LOG_ERR("Failed to suspend UART peripheral (err: %d). Rolling back.", err);
          (void)pm_device_action_run(pm.getI2cDev(), PM_DEVICE_ACTION_RESUME);
          if (pm.getRtcDev()) {
              rtc_alarm_set_time(pm.getRtcDev(), 0, 0, nullptr);
          }
          pm.notifySleepAborted();
          pm.reportPmFailure();
          return false;
      }
      #endif
    #endif

    sleep_prepared = true;
    pm_policy_state_lock_put(PM_STATE_SUSPEND_TO_RAM, PM_ALL_SUBSTATES);
    return true;
}

IPowerState& StopState::execute(PowerManager& ) {
    return *this;
}

void StopState::exit(PowerManager& pm) {
    if (!sleep_prepared) {
        return;
    }
    int err = 0;
    if (pm.getRtcDev() != nullptr) {
        int err = rtc_alarm_set_time(pm.getRtcDev(), 0, 0, nullptr);
        if (err && err != -ENOTSUP) {
            LOG_WRN("Failed to cancel RTC alarm on STOP exit (err: %d)", err);
        }
    } else {
        k_timer_stop(&sim_rtc_timer);
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

    bool resume_ok = true;

    #ifndef IS_TEST_ENVIRONMENT
      #if DT_NODE_EXISTS(DT_CHOSEN(zephyr_console)) && DT_NODE_HAS_COMPAT(DT_CHOSEN(zephyr_console), st_stm32_usart)
      err = pm_device_action_run(pm.getUartDev(), PM_DEVICE_ACTION_RESUME);
      if (err && err != -EALREADY && err != -ENOSYS && err != -ENOTSUP) {
          LOG_ERR("Failed to resume UART (err: %d)", err);
          resume_ok = false;
      }
      #endif
    #endif

    err = pm_device_action_run(pm.getI2cDev(), PM_DEVICE_ACTION_RESUME);
    if (err && err != -EALREADY && err != -ENOSYS && err != -ENOTSUP) {
        LOG_ERR("Failed to resume I2C (err: %d)", err);
        resume_ok = false;
    }
    
    err = pm_device_action_run(pm.getAdcDev(), PM_DEVICE_ACTION_RESUME);
    if (err && err != -EALREADY && err != -ENOSYS && err != -ENOTSUP) {
        LOG_ERR("Failed to resume ADC (err: %d)", err);
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

void power_monitor_thread() {
    auto& pwr_manager = PowerManager::getInstance();
    if (!pwr_manager.init(rtc_hardware, i2c_hardware, uart_hardware, usb_hardware, adc_hardware, &sys_context)) {
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
