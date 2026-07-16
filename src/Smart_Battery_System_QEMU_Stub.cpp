#include "Smart_Battery_System.h"
#include "Device_State_Machine+Watchdog.h"

#include <zephyr/kernel.h>
#include <array>

/*----------------------------------------------------------------------------
 * Global objects
 *---------------------------------------------------------------------------*/

DeviceContext sys_context;
DeviceContext& device_context = sys_context;

static const struct device* uart_hw = nullptr;

UARTManager uart_bus_manager(uart_hw);

SbsBattery smart_battery(
    &uart_bus_manager,
    &device_context
);

/*----------------------------------------------------------------------------
 * Global accessors
 *---------------------------------------------------------------------------*/

SbsBattery* getSmartBatteryInstance()
{
    return &smart_battery;
}

UARTManager* getUartBusManagerInstance()
{
    return &uart_bus_manager;
}

/*----------------------------------------------------------------------------
 * UARTManager
 *---------------------------------------------------------------------------*/

UARTManager::UARTManager(const struct device* dev)
    : uart_dev(dev),
      uart_mutex{},
      rx_msgq{},
      rx_msgq_buffer{},
      tx_buf{},
      tx_idx(0),
      hw_fault(0),
      stats{},
      initialized(false)
{
    k_mutex_init(&uart_mutex);

    k_msgq_init(&rx_msgq,
                rx_msgq_buffer.data(),
                sizeof(char),
                rx_msgq_buffer.size());
}

bool UARTManager::init()
{
    initialized = true;
    return true;
}

result<bool> UARTManager::executeTransaction(
    DalyCommand,
    DalyProtocol::Payload& payload_out)
{
    payload_out.fill(0U);

    return result<bool>::Ok(true);
}

void UARTManager::recordRetry()
{
    atomic_inc(&stats.retries);
}

CommStatistics UARTManager::getStatsSnapshot() const
{
    CommStatistics snapshot;

    snapshot.tx_frames =
        atomic_get(const_cast<atomic_t*>(&stats.tx_frames));

    snapshot.rx_frames =
        atomic_get(const_cast<atomic_t*>(&stats.rx_frames));

    snapshot.crc_errors =
        atomic_get(const_cast<atomic_t*>(&stats.crc_errors));

    snapshot.frame_errors =
        atomic_get(const_cast<atomic_t*>(&stats.frame_errors));

    snapshot.retries =
        atomic_get(const_cast<atomic_t*>(&stats.retries));

    snapshot.overflows =
        atomic_get(const_cast<atomic_t*>(&stats.overflows));

    snapshot.timeouts =
        atomic_get(const_cast<atomic_t*>(&stats.timeouts));

    return snapshot;
}

void UARTManager::uart_isr(const struct device*, void*)
{
}

atomic_val_t UARTManager::faultToAtomic(CommFault fault)
{
    return static_cast<atomic_val_t>(fault);
}

CommFault UARTManager::atomicToFault(atomic_val_t fault)
{
    return static_cast<CommFault>(fault);
}

result<bool> UARTManager::receiveFrame(
    DalyCommand,
    DalyProtocol::FrameBuffer& rx_frame)
{
    rx_frame.fill(0U);

    return result<bool>::Ok(true);
}

/*----------------------------------------------------------------------------
 * SbsBattery
 *---------------------------------------------------------------------------*/

SbsBattery::SbsBattery(
    UARTManager* uart_bus,
    DeviceContext* context,
    WatchdogFeedHook hook)
    : uart_bus(uart_bus),
      sys_context(context),
      current_state(BatteryFSM::IDLE),
      cache_mutex{},
      full_charge_logged(false),
      last_valid_comm_time(0U),
      consecutive_comm_failures(0U),
      cache{},
      watchdog_feed_hook(hook)
{
    k_mutex_init(&cache_mutex);
}

void SbsBattery::setWatchdogFeedHook(WatchdogFeedHook hook)
{
    watchdog_feed_hook = hook;
}

void SbsBattery::pollHardwareAndUpdateCache()
{
    k_mutex_lock(&cache_mutex, K_FOREVER);

    cache.valid = true;
    cache.last_error = CommFault::NONE;

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
    current_state = BatteryFSM::IDLE;
}

BatteryFSM SbsBattery::getState() const
{
    return current_state;
}

CommStatistics SbsBattery::getStats() const
{
    return uart_bus->getStatsSnapshot();
}

/*----------------------------------------------------------------------------
 * Private helpers
 *---------------------------------------------------------------------------*/

result<bool> SbsBattery::fetchWithRetry(
    DalyCommand,
    DalyProtocol::Payload& payload)
{
    payload.fill(0U);

    return result<bool>::Ok(true);
}

void SbsBattery::feedWatchdog() const
{
    if (watchdog_feed_hook != nullptr)
    {
        watchdog_feed_hook();
    }
}

void SbsBattery::publishCache(const BmsCache& next_cache)
{
    k_mutex_lock(&cache_mutex, K_FOREVER);
    cache = next_cache;
    k_mutex_unlock(&cache_mutex);
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
