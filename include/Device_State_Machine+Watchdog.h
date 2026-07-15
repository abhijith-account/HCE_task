#include <zephyr/kernel.h>
#include <zephyr/drivers/watchdog.h>
#pragma once

enum class SystemState{
  INIT,
  RUNNING,
  FAULT,
  SAFE_HALT
};

class WatchdogTimer{
    private:
        const device* wdt_dev;
        int channel_id;
    public:
        WatchdogTimer();
        bool init(uint32_t timeout_ms);
        void feed();
        bool isInitialized() const;
};

class DeviceContext {
    private:
        mutable struct k_mutex state_mutex;
        SystemState current_state;
        WatchdogTimer wdt;
        static bool isLegalTransition(SystemState from,SystemState to);
    public:
        DeviceContext();
        
        SystemState getState() const;
        bool requestTransition(SystemState next_state);
        
        bool initWatchdog(uint32_t timeout_ms);
        void feedWatchdog();
        void triggerFault(const char* reason);
};      
