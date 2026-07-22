#include "Smart_Battery_System.h"
#include "Device_State_Machine+Watchdog.h"
#include "Fault_Tolerant_I2C_Communication_Layer.h"

#include <zephyr/kernel.h>
#include <array>

/*----------------------------------------------------------------------------
 * Global objects
 *---------------------------------------------------------------------------*/

DeviceContext sys_context;
DeviceContext& device_context = sys_context;

// Mock I2C Manager for the test stub
I2CManager i2c_bus_manager;

SbsBattery smart_battery(
    &i2c_bus_manager,
    &device_context
);

/*----------------------------------------------------------------------------
 * Global accessors
 *---------------------------------------------------------------------------*/

SbsBattery* getSmartBatteryInstance()
{
    return &smart_battery;
}

I2CManager* getI2cBusManagerInstance()
{
    return &i2c_bus_manager;
}

/*----------------------------------------------------------------------------
 * SbsBattery (Stub Implementation)
 *---------------------------------------------------------------------------*/

SbsBattery::SbsBattery(
    I2CManager* i2c_bus,
    DeviceContext* context,
    WatchdogFeedHook hook)
    : ina226(i2c_bus),
      sys_context(context),
      current_state(BatteryFSM::IDLE),
      full_charge_logged(false),
      watchdog_feed_hook(hook),
      cache_mutex{},
      last_valid_comm_time(0U),
      consecutive_comm_failures(0U),
      cache{},
      stats{},
      soc_initialized(false),
      accumulated_uAh(0),
      last_poll_time_ms(0U),
      consecutive_jump_rejects(0U)
{
    k_mutex_init(&cache_mutex);
}

bool SbsBattery::init()
{
    // Stub always succeeds
    return true; 
}

void SbsBattery::setWatchdogFeedHook(WatchdogFeedHook hook)
{
    watchdog_feed_hook.store(hook);
}

void SbsBattery::notifySystemWakeup()
{
    k_mutex_lock(&cache_mutex, K_FOREVER);
    last_valid_comm_time = k_uptime_get_32();
    last_poll_time_ms = last_valid_comm_time;
    if (cache.valid) cache.timestamp_ms = last_valid_comm_time;
    k_mutex_unlock(&cache_mutex);
}

void SbsBattery::pollHardwareAndUpdateCache()
{
    k_mutex_lock(&cache_mutex, K_FOREVER);

    cache.valid = true;
    cache.last_error = CommFault::NONE;

    // Hardcoded stub values
    cache.voltage = Millivolts{12000};
    cache.current = Milliamps{0};
    cache.soc = Percent{95};
    cache.temperature = Kelvin{298};
    cache.capacity = MilliAmpHours{3000};

    cache.timestamp_ms = k_uptime_get_32();

    k_mutex_unlock(&cache_mutex);
}

result<Millivolts> SbsBattery::getVoltage() const
{
    return result<Millivolts>::Ok(Millivolts{12000});
}

result<Milliamps> SbsBattery::getCurrent() const
{
    return result<Milliamps>::Ok(Milliamps{0});
}

result<Percent> SbsBattery::getStateOfCharge() const
{
    return result<Percent>::Ok(Percent{95});
}

result<Kelvin> SbsBattery::getTemperature() const
{
    return result<Kelvin>::Ok(Kelvin{298});
}

result<MilliAmpHours> SbsBattery::getCapacity() const
{
    return result<MilliAmpHours>::Ok(MilliAmpHours{3000});
}

void SbsBattery::processFSM()
{
    current_state.store(BatteryFSM::IDLE);
}

BatteryFSM SbsBattery::getState() const
{
    return current_state.load();
}

CommStatistics SbsBattery::getStats() const
{
    CommStatistics snapshot{};
    snapshot.reads = static_cast<uint32_t>(atomic_get(const_cast<atomic_t*>(&stats.reads)));
    snapshot.i2c_faults = static_cast<uint32_t>(atomic_get(const_cast<atomic_t*>(&stats.i2c_faults)));
    snapshot.thermistor_faults = static_cast<uint32_t>(atomic_get(const_cast<atomic_t*>(&stats.thermistor_faults)));
    snapshot.retries = static_cast<uint32_t>(atomic_get(const_cast<atomic_t*>(&stats.retries)));
    snapshot.validation_errors = static_cast<uint32_t>(atomic_get(const_cast<atomic_t*>(&stats.validation_errors)));
    snapshot.successful_publishes = static_cast<uint32_t>(atomic_get(const_cast<atomic_t*>(&stats.successful_publishes)));
    return snapshot;
}

/*----------------------------------------------------------------------------
 * Private helpers
 *---------------------------------------------------------------------------*/

void SbsBattery::feedWatchdog() const
{
    WatchdogFeedHook hook = watchdog_feed_hook.load();
    if (hook != nullptr)
    {
        hook();
    }
}

void SbsBattery::publishError(CommFault fault)
{
    k_mutex_lock(&cache_mutex, K_FOREVER);

    cache.last_error = fault;
    cache.valid = false;

    k_mutex_unlock(&cache_mutex);
}

BmsCache SbsBattery::getCacheSnapshot() const
{
    k_mutex_lock(&cache_mutex, K_FOREVER);
    BmsCache snapshot = cache;
    k_mutex_unlock(&cache_mutex);

    return snapshot;
}
