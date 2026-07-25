#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/printk.h>

#include "RTOS_Command_based_thread_system.h"
#include "RTOS_Synchronization_Layer.h"
#include "Device_State_Machine+Watchdog.h"
#include "Smart_Battery_System.h"
#include "Persistent_Configuration_System.h"
#include "Static_Memory+MISRA_Compliance_Layer.h"
#include "Power_Management_System.h"

#include <zephyr/debug/thread_analyzer.h>
#include <new>

LOG_MODULE_REGISTER(MAIN_OS, LOG_LEVEL_INF);

extern DeviceContext sys_context;
extern ZephyrWorkQueue status_work;

int main(void)
{
    LOG_INF("Command-Based RTOS Booting");
    
    ConfigStore& config = ConfigStore::getInstance();

    if (config.init()) {
        config.validateEndurance(ConfigKey::ALARM_THRESHOLD_BASE);

        uint16_t infusion_rate = 0;

        if (!config.get(ConfigKey::INFUSION_RATE_BASE, infusion_rate)) {
            LOG_WRN("First boot detected. Setting default infusion rate.");
            config.set(ConfigKey::INFUSION_RATE_BASE, static_cast<uint16_t>(50));
        } else {
            LOG_INF("Loaded Infusion Rate from NVS: %u mL/hr", infusion_rate);
        }
    }

    sys_context.requestTransition(SystemState::RUNNING);

    status_work.schedule(K_SECONDS(1));

    return 0;
}
