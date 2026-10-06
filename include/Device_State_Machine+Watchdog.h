#pragma once
#include <zephyr/kernel.h>
#include <zephyr/drivers/watchdog.h>
#include "Power_Management_System.h"
/* Global Thread Health Flags (Updated by ALL RTOS Threads) */
extern atomic_t g_producer_alive;
extern atomic_t g_processor_alive;
extern atomic_t g_logger_alive;
extern atomic_t g_hr_prod_alive;
extern atomic_t g_disp_cons_alive;
extern atomic_t g_bms_comm_alive;
extern atomic_t g_batt_mon_alive;
extern atomic_t g_mem_mon_alive;
extern atomic_t g_shell_alive;
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

class DeviceContext : public IPowerObserver {
    private:
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
        void beforeSleep() override;
        void afterWakeup() override;
        void sleepAborted() override;
};
