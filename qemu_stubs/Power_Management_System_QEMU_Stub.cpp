#include "Power_Management_System.h"
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(PWR_SYS_QEMU, LOG_LEVEL_INF);

/*
 * QEMU-safe PowerManager implementation.
 * Updated to reflect the State Pattern and Meyers Singleton architecture.
 * This stub replaces hardware-specific power states with a dummy
 * implementation that keeps the system permanently ACTIVE.
 */

// ---------------------------------------------------------
// PowerManager Implementation
// ---------------------------------------------------------

PowerManager::PowerManager()
    : current_state(nullptr),
      last_sleep_time(0),
      expected_wake_time(0),
      rtc_dev(nullptr),
      i2c_dev(nullptr),
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

bool PowerManager::init(const struct device* rtc, const struct device* i2c, DeviceContext* fault_ctx)
{
    rtc_dev = rtc;
    i2c_dev = i2c;
    fault_context = fault_ctx;
    last_activity_time.store(k_uptime_get_32());
    current_state = &ActiveState::getInstance();
    
    LOG_INF("QEMU PowerManager stub initialized.");
    return true;
}

void PowerManager::reportActivity()
{
    last_activity_time.store(k_uptime_get_32());
    current_state = &ActiveState::getInstance();
}

void PowerManager::processFSM()
{
    // Do nothing in QEMU; keep system actively spinning
}

void PowerManager::rtc_alarm_handler(const struct device* /* dev */, uint8_t /* chan_id */,
                                     uint32_t /* ticks */, void* user_data)
{
    auto* self = static_cast<PowerManager*>(user_data);
    if (self) {
        atomic_set(&self->wake_pending, 1);
    }
}

void PowerManager::transitionTo(IPowerState& next_state) { current_state = &next_state; }
bool PowerManager::registerObserver(IPowerObserver* obs) { return true; }
size_t PowerManager::captureObservers(std::array<IPowerObserver*, MAX_OBSERVERS>& out) { return 0; }
void PowerManager::notifyBeforeSleep() {}
void PowerManager::notifyAfterWakeup() {}
void PowerManager::notifySleepAborted() {}
void PowerManager::reportPmFailure() {}

#ifdef IS_TEST_ENVIRONMENT
void PowerManager::resetForTest() {
    current_state = nullptr;
    observer_count = 0;
    consecutive_pm_failures = 0;
}
#endif

// ---------------------------------------------------------
// State Pattern Concrete Types (Stubs)
// ---------------------------------------------------------

// --- ACTIVE STATE ---
ActiveState& ActiveState::getInstance() {
    static ActiveState instance;
    return instance;
}
bool ActiveState::enter(PowerManager& /*pm*/) { return true; }
IPowerState& ActiveState::execute(PowerManager& pm) { return *this; }
void ActiveState::exit(PowerManager& /*pm*/) {}

// --- IDLE STATE ---
IdleState& IdleState::getInstance() {
    static IdleState instance;
    return instance;
}
bool IdleState::enter(PowerManager& /*pm*/) { return true; }
IPowerState& IdleState::execute(PowerManager& pm) { return *this; }
void IdleState::exit(PowerManager& /*pm*/) {}

// --- STOP STATE ---
StopState& StopState::getInstance() {
    static StopState instance;
    return instance;
}
bool StopState::enter(PowerManager& pm) { return true; }
IPowerState& StopState::execute(PowerManager& pm) { return *this; }
void StopState::exit(PowerManager& pm) {}

#ifdef IS_TEST_ENVIRONMENT
void StopState::resetForTest() {}
#endif

// ---------------------------------------------------------
// RTOS Thread (Stubbed for QEMU)
// ---------------------------------------------------------
void power_monitor_thread() {
    auto& pwr_manager = PowerManager::getInstance();
    pwr_manager.init(nullptr, nullptr, nullptr);
    
    while (true) {
        k_msleep(1000);
    }
}
K_THREAD_DEFINE(pr_tid, 1024, power_monitor_thread, NULL, NULL, NULL, 14, 0, 0);
