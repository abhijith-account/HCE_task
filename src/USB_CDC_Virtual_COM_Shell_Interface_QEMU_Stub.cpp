#include "USB_CDC_Virtual_COM_Shell_Interface.h"
#include "Device_State_Machine+Watchdog.h"
#include "Smart_Battery_System.h"

#include <array>
#include <string_view>

/* -------------------------------------------------------------------------- */
/* Global Objects                                                             */
/* -------------------------------------------------------------------------- */

extern DeviceContext device_context;

extern UARTManager uart_bus_manager;

extern SbsBattery smart_battery;

UsbShell diag_shell(
    &device_context,
    &smart_battery
);

/* -------------------------------------------------------------------------- */
/* UsbCdcFacade                                                               */
/* -------------------------------------------------------------------------- */

UsbCdcFacade::UsbCdcFacade() noexcept
    : dev(nullptr),
      dtr_ready(false),
      initialized(false),
      line_ctrl_get_failed_logged(false),
      rx_head(0),
      rx_tail(0),
      overflow_logged(false),
      dropped_bytes(0),
      overflow_count(0)
{
}

bool UsbCdcFacade::init()
{
    initialized = true;
    return true;
}

bool UsbCdcFacade::isConnected()
{
    return false;
}

void UsbCdcFacade::transmit(std::string_view /*data*/) noexcept
{
}

void UsbCdcFacade::uartInterruptHandler(const device*,
                                       void*)
{
}

bool UsbCdcFacade::readLine(CommandBuffer&) noexcept
{
    return false;
}

/* -------------------------------------------------------------------------- */
/* UsbShell                                                                    */
/* -------------------------------------------------------------------------- */

const std::array<UsbShell::Command, UsbShell::CommandCount>
UsbShell::kCommandTable{{
    {"status",  false, &UsbShell::cmdStatus},
    {"setrate", true,  &UsbShell::cmdSetRate},
    {"logdump", false, &UsbShell::cmdLogDump},
    {"reboot",  false, &UsbShell::cmdReboot},
}};

UsbShell::UsbShell(DeviceContext* ctx,
                   SbsBattery* bat) noexcept
    : sys_ctx(ctx),
      battery(bat)
{
}

void UsbShell::process()
{
}

void UsbShell::dispatchCommand(std::string_view)
{
}

void UsbShell::cmdSetRate(std::string_view) noexcept
{
}

void UsbShell::cmdStatus(std::string_view) noexcept
{
}

void UsbShell::cmdLogDump(std::string_view) noexcept
{
}

void UsbShell::cmdReboot(std::string_view) noexcept
{
}

UsbShell::StatusSnapshot UsbShell::collectStatus() const
{
    return {};
}

std::size_t UsbShell::formatStatus(const StatusSnapshot&,
                                   StatusBuffer&)
{
    return 0U;
}

void shell_thread(void)
{
}
