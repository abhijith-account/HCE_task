#include "Device_State_Machine+Watchdog.h"
#include <zephyr/kernel.h>

WatchdogTimer::WatchdogTimer()
    : wdt_dev(nullptr),
      channel_id(-1)
{
}

bool WatchdogTimer::init(uint32_t /* timeout_ms */)
{
    return true;
}

void WatchdogTimer::feed()
{
}

DeviceContext::DeviceContext()
    : current_state(SystemState::INIT)
{
}

SystemState DeviceContext::getState() const
{
    return current_state;
}

bool DeviceContext::requestTransition(SystemState next_state)
{
    if (current_state == SystemState::FAULT || current_state == SystemState::SAFE_HALT) {
        return false;
    }

    current_state = next_state;
    return true;
}

void DeviceContext::triggerFault(const char* /* reason */)
{
    current_state = SystemState::SAFE_HALT;
}

bool WatchdogTimer::isInitialized() const
{
    return true;
}

bool DeviceContext::initWatchdog(uint32_t timeout_ms)
{
    return wdt.init(timeout_ms);
}

void DeviceContext::feedWatchdog()
{
    wdt.feed();
}

void DeviceContext::beforeSleep()
{
    wdt.feed();
}

void DeviceContext::afterWakeup()
{
    wdt.feed();
}

void DeviceContext::sleepAborted()
{
    wdt.feed();
}
