#include "Device_State_Machine+Watchdog.h"
#include "Power_Management_System.h"
#include <zephyr/logging/log.h>
#include <zephyr/device.h>
#include <zephyr/drivers/watchdog.h>
#include <zephyr/spinlock.h>
#include <zephyr/sys/atomic.h>

LOG_MODULE_REGISTER(DEVICE_STATE, LOG_LEVEL_INF);

static struct k_spinlock device_state_lock;
static struct k_spinlock wdt_access_lock;

WatchdogTimer::WatchdogTimer() : wdt_dev(DEVICE_DT_GET_OR_NULL(DT_ALIAS(watchdog0))), channel_id(-1) {}

bool WatchdogTimer::init(uint32_t timeout_ms){
    if (!device_is_ready(wdt_dev)){
        LOG_ERR("Watchdog device not ready");
        return false;
    }

    const struct wdt_timeout_cfg wdt_config = {
        .window = {
            .min = 0U,
            .max = timeout_ms
        },
        .callback = nullptr,
        .flags = WDT_FLAG_RESET_SOC
    };

    const int installed_ch = wdt_install_timeout(wdt_dev, &wdt_config);
    if (installed_ch < 0){
        LOG_ERR("Watchdog timeout install failed: %d", installed_ch);
        return false;
    }

    const int setup_rc = wdt_setup(wdt_dev, WDT_OPT_PAUSE_HALTED_BY_DBG | WDT_OPT_PAUSE_IN_SLEEP);
    if (setup_rc != 0){
        LOG_ERR("Watchdog setup failed: %d", setup_rc);
        return false;
    }

    const k_spinlock_key_t key = k_spin_lock(&wdt_access_lock);
    channel_id = installed_ch;
    k_spin_unlock(&wdt_access_lock, key);

    LOG_INF("Watchdog initialized with timeout: %u ms", timeout_ms);
    return true;
}

void WatchdogTimer::feed(){
    const k_spinlock_key_t key = k_spin_lock(&wdt_access_lock);
    const int ch = channel_id;
    if (ch >= 0)
    {
        const int rc = wdt_feed(wdt_dev, ch);
        k_spin_unlock(&wdt_access_lock, key);

        if (rc != 0)
        {
            LOG_ERR("Watchdog feed failed: %d", rc);
        }
    }
    else
    {
        k_spin_unlock(&wdt_access_lock, key);
    }
}

bool WatchdogTimer::isInitialized() const{
    const k_spinlock_key_t key = k_spin_lock(&wdt_access_lock);
    const bool ready = (channel_id >= 0);
    k_spin_unlock(&wdt_access_lock, key);
    return ready;
}

DeviceContext::DeviceContext() noexcept : current_state(SystemState::INIT), wdt{} {}

bool DeviceContext::isLegalTransition(SystemState from, SystemState to){
    if (from == to){
        return true;
    }
    
    switch (from){
        case SystemState::INIT:
            return (to == SystemState::RUNNING) || (to == SystemState::FAULT) || (to == SystemState::SAFE_HALT);
        case SystemState::RUNNING:
            return (to == SystemState::INIT) || (to == SystemState::FAULT) || (to == SystemState::SAFE_HALT);
        case SystemState::FAULT:
            return (to == SystemState::SAFE_HALT);
        case SystemState::SAFE_HALT:
            return (to == SystemState::INIT);
        default:
            return false;
    }
}

SystemState DeviceContext::getState() const{
    const k_spinlock_key_t key = k_spin_lock(&device_state_lock);
    const SystemState snapshot = current_state;
    k_spin_unlock(&device_state_lock, key);
    return snapshot;
}

bool DeviceContext::requestTransition(SystemState next_state){
    const k_spinlock_key_t key = k_spin_lock(&device_state_lock);
    const SystemState previous_state = current_state;
    
    if (!isLegalTransition(previous_state, next_state))
    {
        current_state = SystemState::SAFE_HALT;
        k_spin_unlock(&device_state_lock, key);
        LOG_ERR("Illegal state transition rejected: %d->%d", static_cast<int>(previous_state), static_cast<int>(next_state));
        PowerManager::getInstance().reportActivity();
        return false;
    }

    current_state = next_state;
    k_spin_unlock(&device_state_lock, key);
    LOG_INF("System transitioned :%d->%d", static_cast<int>(previous_state), static_cast<int>(next_state));

    PowerManager::getInstance().reportActivity();
    return true;
}

void DeviceContext::triggerFault(const char* reason){
    const char* const fault_reason = (reason != nullptr) ? reason : "unspecified fault";
    const k_spinlock_key_t key = k_spin_lock(&device_state_lock);
    current_state = SystemState::FAULT;
    k_spin_unlock(&device_state_lock, key);
    LOG_ERR("CRITICAL FAULT: %s. Forcing FAULT.", fault_reason);
}

bool DeviceContext::initWatchdog(uint32_t timeout_ms){
    const bool success = wdt.init(timeout_ms);
    if (success){
        PowerManager::getInstance().registerObserver(this);
    }
    return success;
}

void DeviceContext::feedWatchdog(){
    wdt.feed();
}

void DeviceContext::beforeSleep(){
    wdt.feed();
}

void DeviceContext::afterWakeup(){
    wdt.feed();
}

void DeviceContext::sleepAborted(){
    wdt.feed();
}

DeviceContext sys_context;

extern "C" void daly_watchdog_feed_hook(void){
    const SystemState state = sys_context.getState();

    if (state == SystemState::FAULT){
        return;
    }
    if (state == SystemState::SAFE_HALT){
        sys_context.feedWatchdog();
        return;
    }
    
    static uint32_t last_health_check = 0;
    const uint32_t now = k_uptime_get_32();
    
    if (now - last_health_check < 1000){
        sys_context.feedWatchdog();
        return;
    }
    
    last_health_check = now;
    
    const bool prod_ok = atomic_cas(&g_producer_alive, 1, 0);
    const bool proc_ok = atomic_cas(&g_processor_alive, 1, 0);
    const bool log_ok = atomic_cas(&g_logger_alive, 1, 0);
    const bool hr_ok = atomic_cas(&g_hr_prod_alive, 1, 0);
    const bool disp_ok = atomic_cas(&g_disp_cons_alive, 1, 0);
    const bool bms_ok = atomic_cas(&g_bms_comm_alive, 1, 0);
    const bool batt_ok = atomic_cas(&g_batt_mon_alive, 1, 0);
    const bool mem_ok = atomic_cas(&g_mem_mon_alive, 1, 0);
    const bool shell_ok = atomic_cas(&g_shell_alive, 1, 0); // NEW
    
    if (prod_ok && proc_ok && log_ok && hr_ok && disp_ok && bms_ok && batt_ok && mem_ok && shell_ok){
        sys_context.feedWatchdog();
    }
    else{
        LOG_DBG("WDT Skipped - Prod:%d Proc:%d Log:%d HR:%d Disp:%d BMS:%d Batt:%d Mem:%d Shell:%d",
                prod_ok, proc_ok, log_ok, hr_ok, disp_ok, bms_ok, batt_ok, mem_ok, shell_ok);
    }
}
