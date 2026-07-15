#include "Power_Management_System.h"
#include <zephyr/logging/log.h>

#ifdef IS_TEST_ENVIRONMENT
    extern bool run_thread_once;
    #define THREAD_LOOP_CONDITION (run_thread_once ? (run_thread_once = false, true) : false)
#else 
    #define THREAD_LOOP_CONDITION true
#endif

LOG_MODULE_REGISTER(PWR_SYS, LOG_LEVEL_INF);

extern const device* i2c_hardware;
const device* rtc_hardware=DEVICE_DT_GET(DT_NODELABEL(rtc));
PowerManager pwr_manager(rtc_hardware,i2c_hardware);

PowerManager::PowerManager(const device* rtc, const device* i2c)
    : current_mode(PowerMode::ACTIVE),
      last_activity_time(0),
      rtc_dev(rtc),
      i2c_dev(i2c)
{
}

bool PowerManager::init()
{
    last_activity_time = k_uptime_get_32();

    if (!device_is_ready(rtc_dev)) {
        LOG_ERR("RTC device not ready");
        return false;
    }

    pm_policy_state_lock_get(PM_STATE_SUSPEND_TO_RAM, PM_ALL_SUBSTATES);
    LOG_INF("Power Manager initialized. Deep Sleep Locked.");

    return true;
}

void PowerManager::reportActivity(){
    last_activity_time=k_uptime_get_32();
    
    if (current_mode==PowerMode::STOP){
        LOG_INF("Hardware Activity Detected! Waking up I2C Bus...");
        
        pm_device_action_run(i2c_dev, PM_DEVICE_ACTION_RESUME);
        
        pm_policy_state_lock_get(PM_STATE_SUSPEND_TO_RAM,PM_ALL_SUBSTATES);
    }
    
    current_mode=PowerMode::ACTIVE;
}

void PowerManager::rtc_alarm_handler(const device* /* dev */,uint8_t /* chan_id */,uint32_t /* ticks */,void *user_data){
    auto* self=static_cast<PowerManager*>(user_data);
    LOG_WRN("=== 60s RTC Wakeup Triggered! Restoring System State ===");
    self->reportActivity();
}

void PowerManager::processFSM(){
    uint32_t elapsed=k_uptime_get_32()-last_activity_time;
    
    if (current_mode==PowerMode::ACTIVE && elapsed >=30000){
       LOG_INF("30s Inactivity: Transitioning to IDLE mode");
       current_mode=PowerMode::IDLE;
    }
    else if(current_mode==PowerMode::IDLE && elapsed>=35000){
        LOG_WRN("35s Inactivity: Transitioning to STOP mode (Deep Sleep).");
        current_mode=PowerMode::STOP;
        
        pm_device_action_run(i2c_dev,PM_DEVICE_ACTION_SUSPEND);
        
        struct counter_alarm_cfg alarm_cfg={};
        alarm_cfg.flags=0;
        alarm_cfg.ticks=counter_us_to_ticks(rtc_dev,60000000);
        alarm_cfg.callback=rtc_alarm_handler;
        alarm_cfg.user_data=this;
        counter_set_channel_alarm(rtc_dev,0,&alarm_cfg);
    
        pm_policy_state_lock_put(PM_STATE_SUSPEND_TO_RAM,PM_ALL_SUBSTATES);
    }
}

void power_monitor_thread(){
    pwr_manager.init();
    do{
        pwr_manager.processFSM();
        k_msleep(1000);
    }while(THREAD_LOOP_CONDITION);
}

K_THREAD_DEFINE(pr_tid,1024,power_monitor_thread,NULL,NULL,NULL,14,0,0);

PowerMode PowerManager::getMode() const {
    return current_mode;
}
