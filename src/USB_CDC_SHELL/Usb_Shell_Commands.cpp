// Usb_Shell_Commands.cpp
// UsbShell command layer: the command table, dispatchCommand() (trim, prefix/argument matching), and the handlers
// cmdStatus / cmdSetRate / cmdLogDump / cmdReboot with collectStatus() and the JSON status formatter (formatStatus()).
// The main loop, power gating and the thread are in Usb_Shell_Runtime.cpp; the transport is in Usb_Cdc_Facade.cpp.

#include "USB_CDC_Virtual_COM_Shell_Interface.h"
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/sys/__assert.h>
#include "Persistent_Configuration_System.h"
#include <zephyr/usb/usb_device.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/reboot.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/atomic.h>
#include <charconv>
#include <cstring>
#include <cstdarg>
#include <cstdio>
#include <utility>
#include "Device_State_Machine+Watchdog.h"
#include "Power_Management_System.h" 

// snprintf indirection: the host tests inject formatting failures / truncation through testable_snprintf().
#ifdef IS_TEST_ENVIRONMENT
    extern int mock_snprintf_call_count;
    extern int mock_snprintf_fail_on_call;
    extern int mock_snprintf_truncate_on_call;
    extern int mock_snprintf_exact_return_on_call;
    extern int mock_snprintf_exact_return_value;
    static inline int testable_snprintf(char* str, size_t size, const char* format, ...) {
        mock_snprintf_call_count++;
        if (mock_snprintf_call_count == mock_snprintf_fail_on_call) return -1;
        if (mock_snprintf_call_count == mock_snprintf_truncate_on_call) return static_cast<int>(size + 10);
        if (mock_snprintf_call_count == mock_snprintf_exact_return_on_call) return mock_snprintf_exact_return_value;
        va_list args;
        va_start(args, format);
        int ret = vsnprintf(str, size, format, args);
        va_end(args);
        return ret;
    }
    #define SNPRINTF testable_snprintf
#else
    #define SNPRINTF snprintf
#endif

LOG_MODULE_DECLARE(USB_CLI, LOG_LEVEL_INF);

namespace {

    [[nodiscard]] bool parseIntToken(std::string_view token, int& out_value) noexcept {
        if (token.empty()) {
            return false;
        }
        const auto* first = token.data();
        const auto* last  = token.data() + token.size();
        auto [ptr, ec] = std::from_chars(first, last, out_value);
        return ec == std::errc{} && ptr == last;
    }

    [[nodiscard]] std::string_view trim(std::string_view s) noexcept {
        while (!s.empty() && (s.front() == ' ' || s.front() == '\t')) {
            s.remove_prefix(1);
        }
        while (!s.empty() && (s.back() == ' ' || s.back() == '\t')) {
            s.remove_suffix(1);
        }
        return s;
    }

    constexpr std::string_view kSetRatePrefix = "set_rate";
    constexpr std::string_view kStatusCmd     = "status";
    constexpr std::string_view kLogDumpCmd    = "log dump";
    constexpr std::string_view kRebootCmd     = "reboot";
    constexpr std::string_view kUnknownCmdMsg       = "Error: Unknown command.\r\n";
    constexpr std::string_view kSetRateUsageMsg     = "Error: Usage: set_rate <device_id> <rate 1-100>\r\n";
    constexpr std::string_view kRateRangeMsg        = "Error: Rate must be between 1 and 100.\r\n";
    constexpr std::string_view kPersistFailMsg      = "Error: Failed to persist infusion rate.\r\n";
    constexpr std::string_view kNotProvisionedMsg   = "Error: Device ID not provisioned.\r\n";
    constexpr std::string_view kThresholdFallbackMsg = "Error: Rate exceeds alarm threshold. Please reset the rate.\r\n";
    constexpr std::string_view kSetRateSuccessFallback = "Success: infusion rate updated.\r\n";

    constexpr std::string_view kSafeHaltRejectMsg   = "Error: Operational command rejected. System locked in SAFE_HALT/FAULT.\r\n";
    
    constexpr std::string_view kRebootingMsg        = "Rebooting system... \r\n";
    constexpr std::string_view kLogDumpHeader       = "--- NVS System Log Dump ---\r\n";
    constexpr std::string_view kLogDumpBooted       = "System Booted\r\n";
    constexpr std::string_view kLogDumpI2c          = "I2C Manager Initialized\r\n";
    constexpr std::string_view kLogDumpFooter       = "--- End of Logs ---\r\n";
}

#ifdef IS_TEST_ENVIRONMENT
bool test_parseIntToken(std::string_view token, int& out_value) noexcept {
    return parseIntToken(token, out_value);
}
std::string_view test_trim(std::string_view s) noexcept {
    return trim(s);
}
#endif

UsbShell::UsbShell(DeviceContext* ctx, SbsBattery* bat) noexcept : sys_ctx(ctx), battery(bat) {}

const std::array<UsbShell::Command, UsbShell::CommandCount> UsbShell::kCommandTable{{
    {kStatusCmd,     false, &UsbShell::cmdStatus},
    {kSetRatePrefix, true,  &UsbShell::cmdSetRate},
    {kLogDumpCmd,    false, &UsbShell::cmdLogDump},
    {kRebootCmd,     false, &UsbShell::cmdReboot},
}};

void UsbShell::dispatchCommand(std::string_view cmd) {
    cmd = trim(cmd);
    LOG_INF("CLI command: %.*s", static_cast<int>(cmd.size()), cmd.data());

    for (const auto& entry : kCommandTable) {
        if (entry.takes_args) {
            if (cmd.starts_with(entry.name)) {
                if (cmd.size() == entry.name.size()) {
                    cmdSetRate(std::string_view{});
                    return;
                }
                const char separator = cmd[entry.name.size()];
                if (separator == ' ') {
                    cmdSetRate(trim(cmd.substr(entry.name.size() + 1)));
                    return;
                }
            }
        } else if (cmd == entry.name) {
            if (entry.name == kStatusCmd) {
                cmdStatus(std::string_view{});
            } else if (entry.name == kLogDumpCmd) {
                cmdLogDump(std::string_view{});
            } else {
                cmdReboot(std::string_view{});
            }
            return;
        }
    }
    usb.transmit(kUnknownCmdMsg);
}

UsbShell::StatusSnapshot UsbShell::collectStatus() const {
    static_assert(InfusionDeviceConfig::MaxSlot - InfusionDeviceConfig::MinSlot + 1<= StatusSnapshot::MaxTrackedSlots,"device slot range exceeds StatusSnapshot::MaxTrackedSlots");
    StatusSnapshot snap{};
    snap.state_val = static_cast<int>(sys_ctx->getState());
    const auto soc_res = battery->getStateOfCharge();
    snap.battery_soc = soc_res.success ? soc_res.value.value : 0;
    auto& cfg = ConfigStore::getInstance();
    uint8_t idx = 0;
    for (uint8_t slot = InfusionDeviceConfig::MinSlot; slot <= InfusionDeviceConfig::MaxSlot; ++slot) {
        uint32_t deviceId = InfusionDeviceConfig::UnprovisionedId;
        uint8_t rate = 0;
        uint8_t threshold = 0;
        (void)cfg.getDeviceId(slot, deviceId);
        const bool haveRate = cfg.getInfusionRate(slot, rate);
        (void)cfg.getAlarmThreshold(slot, threshold);
        snap.slots[idx] = {deviceId, haveRate ? rate : static_cast<uint8_t>(0), threshold};
        ++idx;
    }
    snap.slot_count = idx;
    return snap;
}

std::size_t UsbShell::formatStatus(const StatusSnapshot& snap, StatusBuffer& out_buf) {
    int offset = SNPRINTF(out_buf.data(), out_buf.size(),"{\"sys_state\":%d,\"battery_soc\":%u,\"devices\":[", snap.state_val, snap.battery_soc);
    if (offset < 0) {
        return 0;
    }
    if (static_cast<std::size_t>(offset) >= out_buf.size()) {
        LOG_WRN("Status JSON truncated at header");
        return out_buf.size() - 1;
    }
    for (uint8_t i = 0; i < snap.slot_count; ++i) {
        if (static_cast<std::size_t>(offset) >= out_buf.size() - JsonSafetyMargin) {
            LOG_WRN("Status JSON truncated after %u of %u device entries", i, snap.slot_count);
            break;
        }
        const auto& slot = snap.slots[i];
        
        const int written = SNPRINTF(out_buf.data() + offset, out_buf.size() - offset,"%s{\"device_id\":%u,\"rate\":%u,\"alarm_threshold\":%u}",(i == 0) ? "" : ",", slot.device_id, slot.rate, slot.alarm_threshold);
            
        if (written <= 0 || static_cast<std::size_t>(written) >= out_buf.size() - offset) {
            LOG_WRN("Status JSON truncated after %u of %u device entries", i, snap.slot_count);
            break;
        }
        offset += written;
    }

    const int written = SNPRINTF(out_buf.data() + offset, out_buf.size() - offset, "]}\r\n");
    if (written > 0) {
        const std::size_t remaining = out_buf.size() - static_cast<std::size_t>(offset);
        if (remaining > 1) {
            offset += std::min<std::size_t>(static_cast<std::size_t>(written), remaining - 1);
        }
    }
    return static_cast<std::size_t>(offset);
}

void UsbShell::cmdStatus([[maybe_unused]] std::string_view args) noexcept {
    const StatusSnapshot snap = collectStatus();
    static StatusBuffer json_stack_buf;
    [[maybe_unused]] const std::size_t json_len = formatStatus(snap, json_stack_buf);
    LOG_INF("status json: %u bytes, %u slots", (unsigned)json_len, (unsigned)snap.slot_count);
    usb.transmit(json_stack_buf.data());
}

void UsbShell::cmdSetRate(std::string_view args) noexcept {
    const SystemState current_state = sys_ctx->getState();
    if (current_state == SystemState::SAFE_HALT || current_state == SystemState::FAULT) {
        usb.transmit(kSafeHaltRejectMsg);
        return;
    }

    args = trim(args);

    const std::size_t separator_pos = args.find_first_of(" \t");

    if (separator_pos == std::string_view::npos) {
        usb.transmit(kSetRateUsageMsg);
        return;
    }

    std::string_view device_str = trim(args.substr(0, separator_pos));
    std::string_view rate_str = trim(args.substr(separator_pos + 1));
    
    int device_id_in = 0;
    int rate_in        = 0;
    if (!parseIntToken(device_str, device_id_in) ||
        !parseIntToken(rate_str, rate_in)) {
        usb.transmit(kSetRateUsageMsg);
        return;
    }
    if (rate_in < 1 || rate_in > 100) {
        usb.transmit(kRateRangeMsg);
        return;
    }
    const auto rate = static_cast<uint8_t>(rate_in);
    auto& cfg = ConfigStore::getInstance();
    uint8_t slot = 0;
    if (!cfg.findSlotByDeviceId(static_cast<uint32_t>(device_id_in), slot)) {
        std::array<char, SetRateErrMsgSize> msg;
        transmitFormatted(msg, kNotProvisionedMsg, "Error: Device ID %d not provisioned.\r\n", device_id_in);
        return;
    }
    uint8_t threshold = 0;
    const bool haveThreshold = cfg.getAlarmThreshold(slot, threshold);
    if (haveThreshold && rate > threshold) {
        std::array<char, SetRateErrMsgSize> msg;
        transmitFormatted(msg, kThresholdFallbackMsg,"Error: Rate %u exceeds alarm threshold (%u) for device %d. Please reset the rate.\r\n",rate, threshold, device_id_in);
        return;
    }
    if (cfg.setInfusionRate(slot, rate)) {
        std::array<char, SetRateMsgSize> msg;
        transmitFormatted(msg, kSetRateSuccessFallback,"Success: Device %d infusion rate set to %u%%\r\n", device_id_in, rate);
    } else {
        usb.transmit(kPersistFailMsg);
    }
}

void UsbShell::cmdLogDump([[maybe_unused]] std::string_view args) noexcept {
    usb.transmit(kLogDumpHeader);

    auto& cfg = ConfigStore::getInstance();
    const uint32_t logCount = cfg.getStoredLogCount();

    if (logCount == 0) {
        usb.transmit("No logs found.\r\n");
    } else {
        std::array<char, 128> logLineBuffer{}; 

        for (uint32_t i = 0; i < logCount; ++i) {
            ConfigStore::LogEntry entry{};
            if (cfg.getLogEntry(i, entry)) {
                
                const int written = SNPRINTF(logLineBuffer.data(), logLineBuffer.size(),
                    "[%08lu] Event: %u, Data: %d\r\n", 
                    static_cast<unsigned long>(entry.timestamp_ms),
                    static_cast<unsigned>(entry.event_id), 
                    static_cast<int>(entry.event_data));

                if (written > 0 && static_cast<std::size_t>(written) < logLineBuffer.size()) {
                    usb.transmit(std::string_view(logLineBuffer.data(), static_cast<std::size_t>(written)));
                } else {
                    LOG_WRN("Log entry %u formatting truncated or failed", i);
                }
            } else {
                usb.transmit("Error: Failed to read log entry.\r\n");
                break;
            }
            
            if ((i % 10) == 0) {
                k_yield(); 
            }
        }
    }

    usb.transmit(kLogDumpFooter);
}

void UsbShell::cmdReboot([[maybe_unused]] std::string_view args) noexcept {
    usb.transmit(kRebootingMsg);
    k_msleep(100);
    sys_reboot(SYS_REBOOT_COLD);
}
