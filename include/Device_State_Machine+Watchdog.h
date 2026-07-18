#pragma once
#include <zephyr/kernel.h>
#include <zephyr/drivers/watchdog.h>
#include "Power_Management_System.h" // Added for IPowerObserver and PowerManager

enum class SystemState {
  INIT,
  RUNNING,
  FAULT,
  SAFE_HALT
};

class WatchdogTimer {
    private:
        const device* wdt_dev;
        int channel_id;
    public:
        WatchdogTimer();
        bool init(uint32_t timeout_ms);
        void feed();
        bool isInitialized() const;
};

// Inherit from IPowerObserver so we can safely wrap sleep operations 
// with watchdog feeds, ensuring we never sleep on a nearly-expired timer.
class DeviceContext : public IPowerObserver {
    private:
        mutable struct k_mutex state_mutex;
        SystemState current_state;
        WatchdogTimer wdt;
        static bool isLegalTransition(SystemState from, SystemState to);
        
    public:
        DeviceContext();
        
        SystemState getState() const;
        bool requestTransition(SystemState next_state);
        
        bool initWatchdog(uint32_t timeout_ms);
        void feedWatchdog();
        void triggerFault(const char* reason);
        
        // IPowerObserver implementations
        void beforeSleep() override;
        void afterWakeup() override;
        void sleepAborted() override;
};
