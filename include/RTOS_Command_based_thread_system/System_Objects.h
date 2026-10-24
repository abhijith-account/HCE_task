#pragma once

#include <zephyr/kernel.h>
#include <zephyr/sys/atomic.h>
#include "Fault_Tolerant_I2C_Communication_Layer.h"
#include "Device_State_Machine+Watchdog.h"
#include "Power_Management_System.h"

/* Access to the system-wide services (state machine, I2C, power manager). */
namespace SystemObjects {
    DeviceContext& context();
    I2CManager& i2c();
    PowerManager& power();
}

/* Tracks whether the system is asleep so the worker threads can pause. */
class ThreadSystemPowerObserver final : public IPowerObserver {
private:
    atomic_t is_sleeping{};
public:
    ThreadSystemPowerObserver() { atomic_set(&is_sleeping, 0); }
    void beforeSleep() override { atomic_set(&is_sleeping, 1); }
    void afterWakeup() override { atomic_set(&is_sleeping, 0); }
    void sleepAborted() override { atomic_set(&is_sleeping, 0); }
    bool isSleeping() const noexcept { return atomic_get(&is_sleeping) != 0; }
    void resetForTest() noexcept { atomic_set(&is_sleeping, 0); }
};


extern ThreadSystemPowerObserver g_powerObserver;

/* Heartbeat flags, set by each thread and checked by the watchdog. */
extern atomic_t g_producer_alive;
extern atomic_t g_processor_alive;
extern atomic_t g_logger_alive;
