#include "Device_State_Machine+Watchdog.h"
#include <zephyr/logging/log.h>
#include <zephyr/device.h>
#include <zephyr/drivers/watchdog.h>

LOG_MODULE_REGISTER(DEVICE_STATE, LOG_LEVEL_WRN);
  
WatchdogTimer::WatchdogTimer():wdt_dev(DEVICE_DT_GET(DT_NODELABEL(iwdg))),channel_id(-1){}

bool WatchdogTimer::init(uint32_t timeout_ms){
    if (!device_is_ready(wdt_dev)){
      LOG_ERR("Watchdog device not ready");
      return false;
    }
    
    struct wdt_timeout_cfg wdt_config={
        .window={ .min=0U, .max=timeout_ms},
        .callback=nullptr,
        .flags=WDT_FLAG_RESET_SOC
    };
    
    channel_id=wdt_install_timeout(wdt_dev,&wdt_config);
    
    if (channel_id<0){
      LOG_ERR("Watchdog timeout install failed: %d",channel_id);
      return false;
    }
    
    const int setup_rc=wdt_setup(wdt_dev, WDT_OPT_PAUSE_HALTED_BY_DBG);
    
    if (setup_rc!=0){
      LOG_ERR("Watchdog setup failed: %d",setup_rc);
      channel_id=-1;
      return false;
    }
    
    LOG_INF("Watchdog initialized with timeout: %u ms", timeout_ms);
    return true;
}

void WatchdogTimer::feed(){
    if (channel_id>=0){
    
      const int rc= wdt_feed(wdt_dev,channel_id);
      if (rc!=0){
          LOG_ERR("Watchdog feed failed: %d",rc);
      }
    }
}

bool WatchdogTimer::isInitialized() const{
    return channel_id>=0;
}

DeviceContext::DeviceContext():state_mutex{},current_state(SystemState::INIT),wdt{}{
    k_mutex_init(&state_mutex);
}

bool DeviceContext::isLegalTransition(SystemState from, SystemState to){
    if (from==to){
        return true;
    }
    switch (from){
        case SystemState::INIT:
            return (to==SystemState::RUNNING)||(to==SystemState::FAULT)||(to==SystemState::SAFE_HALT);
        case SystemState::RUNNING:
            return (to==SystemState::INIT)||(to==SystemState::FAULT)||(to==SystemState::SAFE_HALT);
        case SystemState::FAULT:
            return (to==SystemState::SAFE_HALT);
        case SystemState::SAFE_HALT:
            return (to==SystemState::INIT);
        default:
            return false;
    }
}

SystemState DeviceContext::getState() const{
  k_mutex_lock(&state_mutex,K_FOREVER);
  const SystemState snapshot = current_state;
  k_mutex_unlock(&state_mutex);
  return snapshot;
}

bool DeviceContext::requestTransition(SystemState next_state){
    k_mutex_lock(&state_mutex,K_FOREVER);
    const SystemState previous_state=current_state;
    if (!isLegalTransition(previous_state,next_state)){
        LOG_ERR("Illegal state transition rejected: %d->%d",static_cast<int>(previous_state),static_cast<int>(next_state));
        current_state=SystemState::SAFE_HALT;
        k_mutex_unlock(&state_mutex);
        return false;
    }
    
    current_state=next_state;
    LOG_INF("System transitioned :%d->%d",static_cast<int>(previous_state),static_cast<int>(current_state));
    
    k_mutex_unlock(&state_mutex);
    return true;
}

void DeviceContext::triggerFault(const char* reason){
    k_mutex_lock(&state_mutex,K_FOREVER);
    const char* const fault_reason = (reason != nullptr) ? reason : "unspecified fault";
    
    LOG_ERR("CRITICAL FAULT: %s. Forcing SAFE_HALT.", fault_reason); 
    
    current_state=SystemState::SAFE_HALT;
    k_mutex_unlock(&state_mutex);
}

bool DeviceContext::initWatchdog(uint32_t timeout_ms){
      return wdt.init(timeout_ms);
}

void DeviceContext::feedWatchdog(){
     wdt.feed();
}
DeviceContext sys_context;
extern "C" void daly_watchdog_feed_hook(void)
{
    sys_context.feedWatchdog();
}
