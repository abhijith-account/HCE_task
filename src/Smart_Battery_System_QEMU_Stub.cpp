#include "Smart_Battery_System.h"
#include "Device_State_Machine+Watchdog.h"

extern DeviceContext sys_context;

const device* i2c_hardware = nullptr;
I2CManager i2c_bus_manager(i2c_hardware);
SbsBattery smart_battery(&i2c_bus_manager, &sys_context);

SbsBattery::SbsBattery(I2CManager* bus, DeviceContext* context)
    : i2c_bus(bus),
      sys_context(context),
      current_state(BatteryFSM::IDLE),
      full_charge_logged(false)
{
}

Result<uint16_t> SbsBattery::getVoltage()
{
    return Result<uint16_t>::Ok(12000);
}

Result<int16_t> SbsBattery::getCurrent()
{
    return Result<int16_t>::Ok(0);
}

Result<uint8_t> SbsBattery::getStateOfCharge()
{
    return Result<uint8_t>::Ok(95);
}

Result<uint16_t> SbsBattery::getTemperature()
{
    return Result<uint16_t>::Ok(250);
}

Result<uint16_t> SbsBattery::getCapacity()
{
    return Result<uint16_t>::Ok(3000);
}

void SbsBattery::processFSM()
{
    current_state = BatteryFSM::IDLE;
}

BatteryFSM SbsBattery::getState() const
{
    return current_state;
}
