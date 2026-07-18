#include "Power_Management_System.h"
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(PWR_SYS_QEMU, LOG_LEVEL_INF);

/*
 * QEMU-safe PowerManager implementation.
 * This keeps your real firmware linkage intact:
 * - RTOS_Command_based_thread_system.cpp still gets pwr_manager
 * - producer_thread() can still call pwr_manager.reportActivity()
 *
 * But it avoids Zephyr PM, PM device suspend/resume, and RTC alarm setup,
 * because those paths are hardware/runtime specific and are currently
 * blocking the olimex_stm32_h405 QEMU startup.
 */

PowerManager pwr_manager(nullptr, nullptr);

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
    current_mode = PowerMode::ACTIVE;

    LOG_INF("QEMU PowerManager stub initialized.");
    return true;
}

void PowerManager::reportActivity()
{
    last_activity_time = k_uptime_get_32();
    current_mode = PowerMode::ACTIVE;
}

void PowerManager::rtc_alarm_handler(const device* /* dev */,
                                     uint8_t /* chan_id */,
                                     uint32_t /* ticks */,
                                     void* user_data)
{
    auto* self = static_cast<PowerManager*>(user_data);

    if (self != nullptr) {
        self->reportActivity();
    }
}

void PowerManager::processFSM()
{
    /*
     * No PM state transition in QEMU.
     * Real hardware file handles ACTIVE / IDLE / STOP.
     */
    current_mode = PowerMode::ACTIVE;
}

PowerMode PowerManager::getMode() const
{
    return current_mode;
}
