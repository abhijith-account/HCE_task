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
constexpr std::size_t JsonSafetyMargin  = 64; // headroom reserved before appending another status entry

// Trailing space matches conventional shell prompt style ("$ " not "$").
constexpr std::string_view PromptStr = "med-device:~$ ";

// Thread configuration for the shell's dedicated kernel thread.
constexpr std::size_t ShellStackSize   = 1024;
constexpr int ShellPriority            = 13;
constexpr std::uint32_t ShellPollMs    = 50; // process() polling interval

// CR/LF as named constants for readability at each comparison site.
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

/**
 * Thin wrapper around a USB CDC-ACM UART: owns the RX interrupt handler,
 * an internal ring buffer, and line framing.
 *
 * Threading model: uartInterruptHandler() runs in ISR context and is the
 * sole writer of rx_head. The owning shell thread is the sole writer of
 * rx_tail via readLine(). This is a classic single-producer/single-consumer
 * ring buffer; rx_head/rx_tail are std::atomic<size_t> with acquire/release
 * ordering so each side observes the other's progress correctly without a
 * lock: the producer release-stores rx_head after writing a byte, the
 * consumer acquire-loads rx_head before reading it (and symmetrically for
 * rx_tail). Do not add a second writer to either index without revisiting
 * this ordering.
 *
 * init(), isConnected(), transmit(), and readLine() are all meant to be
 * called only from the owning shell thread (never from ISR context, and
 * never concurrently from two threads). uartInterruptHandler() is the only
 * member meant to run in ISR context.
 */
class UsbCdcFacade {
private:
    const device* dev{nullptr};
    bool dtr_ready{false};
    bool initialized{false};

    // Set once uart_line_ctrl_get() has already failed, so a persistently
    // disconnected/misbehaving terminal logs one error rather than one
    // per poll.
    bool line_ctrl_get_failed_logged{false};

    std::array<char, RX_RING_BUF_SIZE> rx_buffer{};
    std::atomic<std::size_t> rx_head{0};
    std::atomic<std::size_t> rx_tail{0};

    // Set from ISR context, cleared from the shell thread once room frees
    // up again -- also part of the SPSC handoff above, hence atomic<bool>
    // rather than a plain bool.
    std::atomic<bool> overflow_logged{false};
    std::atomic<std::size_t> dropped_bytes{0};
    std::atomic<std::size_t> overflow_count{0};

    static void uartInterruptHandler(const device* dev, void* user_data);

public:
    UsbCdcFacade() noexcept;

    /** Registers the RX IRQ callback and enables RX. Safe to call more than once. */
    [[nodiscard]] bool init();

    /** Polls DTR and returns the current terminal connection state. */
    [[nodiscard]] bool isConnected();

    /** Blocking, best-effort transmit; no-ops if no terminal is attached. */
    void transmit(std::string_view data) noexcept;

    /**
     * Copies one complete CR/LF-terminated line out of the RX ring buffer
     * into out_line, consuming (and skipping) any leading line breaks.
     * If a command is longer than out_line can hold, the excess is
     * discarded up to the next line break rather than left in the buffer
     * to be misread as a second command. Returns false if no complete
     * line is available yet.
     */
    [[nodiscard]] bool readLine(CommandBuffer& out_line) noexcept;
};

/**
 * Parses and dispatches CLI commands received over the USB shell, and
 * owns the read-modify-report logic for each supported command.
 *
 * process() and dispatchCommand() (and everything they call) are intended
 * to run on the dedicated shell thread; none of this class's members are
 * safe to call concurrently from another thread or from ISR context.
 */
class UsbShell {
private:
    UsbCdcFacade usb;
    // Never reassigned after construction -- pointers are const to
    // document that ownership/binding is fixed for the object's lifetime
    // (the pointed-to DeviceContext/SbsBattery are still mutable, since
    // command handlers do read their live state).
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

    // snprintf()s into buf using fmt/args; transmits buf on success, or
    // `fallback` if the format failed or would have truncated. Removes
    // the repeated snprintf-then-check-then-transmit-or-fallback pattern
    // that cmdSetRate's error/success paths all shared.
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
        // Sized to comfortably exceed the configured device-slot range;
        // see the static_assert next to collectStatus()'s definition,
        // which fails to compile if that range ever grows past this.
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

// shell_thread() must keep external linkage: K_THREAD_DEFINE references it
// in the .cpp, and the host-side gtest suite also calls it directly via its
// own `extern void shell_thread(void);` declaration to drive coverage of
// the thread loop. Do not mark it static.
void shell_thread(void);
