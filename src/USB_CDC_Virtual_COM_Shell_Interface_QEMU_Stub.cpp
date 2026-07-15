#include "USB_CDC_Virtual_COM_Shell_Interface.h"
#include "Device_State_Machine+Watchdog.h"
#include "Smart_Battery_System.h"
#include <array>
#include <string_view>

extern DeviceContext sys_context;
extern SbsBattery smart_battery;

UsbShell diag_shell(&sys_context, &smart_battery);

UsbCdcFacade::UsbCdcFacade()
    : dtr_ready(false),
      rx_head(0),
      rx_tail(0)
{
    dev = nullptr;
}

bool UsbCdcFacade::init()
{
    return true;
}

bool UsbCdcFacade::isConnected()
{
    return false;
}

void UsbCdcFacade::transmit(std::string_view /* data */)
{
}

void UsbCdcFacade::uartInterruptHandler(const device* /* dev */, void* /* user_data */)
{
}

bool UsbCdcFacade::readLine(std::array<char, MAX_CMD_LEN>& /* out_line */)
{
    return false;
}

UsbShell::UsbShell(DeviceContext* ctx, SbsBattery* bat)
    : sys_ctx(ctx),
      battery(bat)
{
}

void UsbShell::process()
{
}

void UsbShell::dispatchCommand(std::string_view /* cmd */)
{
}

void UsbShell::cmdStatus()
{
}

void UsbShell::cmdSetRate(std::string_view /* args */)
{
}

void UsbShell::cmdLogDump()
{
}

void UsbShell::cmdReboot()
{
}
