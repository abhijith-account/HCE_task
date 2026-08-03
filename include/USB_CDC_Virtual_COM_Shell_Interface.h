#pragma once

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/usb/usb_device.h>
#include <array>
#include <atomic>
#include <cstddef>
#include <string_view>
#include <stdio.h>
#include <cstdlib>
#include "Device_State_Machine+Watchdog.h"
#include "Smart_Battery_System.h"
#include "Power_Management_System.h"
#include <cstdio>
constexpr std::size_t RX_RING_BUF_SIZE  = 256;
constexpr std::size_t MAX_CMD_LEN       = 128;
constexpr std::size_t StatusBufferSize  = 512;
constexpr std::size_t SetRateMsgSize    = 80;
constexpr std::size_t SetRateErrMsgSize = 128;
constexpr std::size_t JsonSafetyMargin  = 64;
constexpr std::string_view PromptStr = "med-device:~$ ";

constexpr std::size_t ShellStackSize   = 2048;
constexpr int ShellPriority            = 13;
constexpr std::uint32_t ShellPollMs    = 50;

constexpr char CR = '\r';
constexpr char LF = '\n';

static_assert(MAX_CMD_LEN < RX_RING_BUF_SIZE,
              "command buffer must fit within the RX ring buffer");
static_assert(StatusBufferSize >= 256,
              "status JSON buffer needs headroom for the sys_state/battery_soc header");
static_assert(ShellStackSize >= 1024,
              "shell thread stack must be at least 1024 bytes for this command set");

using CommandBuffer = std::array<char, MAX_CMD_LEN>;
using StatusBuffer  = std::array<char, StatusBufferSize>;

class UsbCdcFacade {
private:
    const device* dev{nullptr};
    bool dtr_ready{false};
    bool initialized{false};

    bool line_ctrl_get_failed_logged{false};

    std::array<char, RX_RING_BUF_SIZE> rx_buffer{};
    std::atomic<std::size_t> rx_head{0};
    std::atomic<std::size_t> rx_tail{0};

    std::atomic<bool> overflow_logged{false};
    std::atomic<std::size_t> dropped_bytes{0};
    std::atomic<std::size_t> overflow_count{0};

    static void uartInterruptHandler(const device* dev, void* user_data);

public:
    UsbCdcFacade() noexcept;

    [[nodiscard]] bool init();

    [[nodiscard]] bool isConnected();

    void transmit(std::string_view data) noexcept;

    [[nodiscard]] bool readLine(CommandBuffer& out_line) noexcept;
};

class UsbShell {
private:
    UsbCdcFacade usb;

    DeviceContext* const sys_ctx;
    SbsBattery* const battery;

    struct Command {
        std::string_view name;
        bool takes_args;
        void (UsbShell::*handler)(std::string_view args);
    };
    static constexpr std::size_t CommandCount = 4;
    static const std::array<Command, CommandCount> kCommandTable;

    void cmdStatus(std::string_view args) noexcept;
    void cmdSetRate(std::string_view args) noexcept;
    void cmdLogDump(std::string_view args) noexcept;
    void cmdReboot(std::string_view args) noexcept;

    template <std::size_t N, typename... Args>
    void transmitFormatted(std::array<char, N>& buf, std::string_view fallback,
                           const char* fmt, Args... args) noexcept {
        const int written = snprintf(buf.data(), buf.size(), fmt, args...);
        if (written > 0 && static_cast<std::size_t>(written) < buf.size()) {
            usb.transmit(buf.data());
        } else {
            usb.transmit(fallback);
        }
    }

    struct StatusSnapshot {

        static constexpr std::size_t MaxTrackedSlots = 16;

        int state_val{0};
        uint8_t battery_soc{0};
        struct SlotInfo {
            uint32_t device_id{0};
            uint8_t rate{0};
            uint8_t alarm_threshold{0};
        };
        std::array<SlotInfo, MaxTrackedSlots> slots{};
        uint8_t slot_count{0};
    };
    [[nodiscard]] StatusSnapshot collectStatus() const;
    [[nodiscard]] static std::size_t formatStatus(const StatusSnapshot& snap, StatusBuffer& out_buf);

public:
    explicit UsbShell(DeviceContext* ctx, SbsBattery* bat) noexcept;

    void process();
    void dispatchCommand(std::string_view cmd);
};

void shell_thread(void);

