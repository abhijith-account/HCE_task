#pragma once

#include "Device_State_Machine+Watchdog.h"
#include "Power_Management_System.h"
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>
#include <cstdint>
#include "Static_Memory+MISRA_Compliance_Layer.h"
#include <array>
#include <span>
#include <cstddef>
#include <zephyr/sys/atomic.h>

enum class CommFault : uint8_t
{
    NONE,
    RX_TIMEOUT,
    TX_TIMEOUT,
    FRAME_ERROR,
    CHECKSUM_ERROR,
    UART_OVERRUN,
    UART_NOISE,
    UART_PARITY,
    RX_OVERFLOW,
    CACHE_INVALID,
    DEVICE_NOT_READY,
    VALIDATION_ERROR
};
enum class BatteryFSM : uint8_t
{
    IDLE,
    CHARGING,
    DISCHARGING,
    CUTOFF
};

template<typename T>
struct result
{
    T value{};
    CommFault error{CommFault::NONE};
    bool success{false};

    static constexpr result Ok(const T& value)
    {
        return {value, CommFault::NONE, true};
    }

    static constexpr result Err(CommFault error)
    {
        return {T{}, error, false};
    }
};

struct Millivolts { uint16_t value = 0U; };
struct Milliamps { int32_t value = 0; };
struct Percent { uint8_t value = 0U; };
struct Kelvin { uint16_t value = 0U; };
struct MilliAmpHours { uint32_t value = 0U; };

struct CommStatistics {
    uint32_t tx_frames = 0U;
    uint32_t rx_frames = 0U;
    uint32_t crc_errors = 0U;
    uint32_t frame_errors = 0U;
    uint32_t retries = 0U;
    uint32_t overflows = 0U;
    uint32_t timeouts = 0U;
};

struct AtomicCommStatistics {
    atomic_t tx_frames{};
    atomic_t rx_frames{};
    atomic_t crc_errors{};
    atomic_t frame_errors{};
    atomic_t retries{};
    atomic_t overflows{};
    atomic_t timeouts{};
};

enum class DalyCommand : uint8_t {
    V_I_SOC = 0x90U,
    MIN_MAX_CELL = 0X91U,
    TEMP = 0X92U,
    STATUS_CAPACITY = 0x93U
};

namespace DalyProtocol {
    static constexpr uint8_t FRAME_SIZE = 13U;
    static constexpr uint8_t PAYLOAD_SIZE = 8U;
    static constexpr uint8_t START_BYTE = 0xA5U;
    static constexpr uint8_t HOST_ID = 0x40U;
    static constexpr uint8_t BMS_ID = 0x01U;
    static constexpr uint8_t HEADER_SIZE = 4U;
    static constexpr uint8_t CHECKSUM_INDEX = FRAME_SIZE - 1U;
    static constexpr uint32_t DEFAULT_BAUD_RATE = 9600U;
    static constexpr uint32_t BITS_PER_UART_8N1 = 10U;
    static constexpr uint32_t RX_TIMEOUT_MS = ((static_cast<uint32_t>(FRAME_SIZE) * BITS_PER_UART_8N1 * 1000U) / DEFAULT_BAUD_RATE) * 3U + 20U;

    using Payload = std::array<uint8_t, PAYLOAD_SIZE>;
    using FrameBuffer = std::array<uint8_t, FRAME_SIZE>;

    struct Frame {
        DalyCommand command = DalyCommand::V_I_SOC;
        Payload payload{};
    };

    uint8_t calculateChecksum(std::span<const uint8_t> bytes);
    FrameBuffer buildRequest(DalyCommand cmd);
    result<Frame> decodeResponse(std::span<const uint8_t, FRAME_SIZE> frame, DalyCommand expected_cmd);

    class FrameParser {
    private:
        enum class RxState : uint8_t { WAIT_START, READ_HEADER, READ_PAYLOAD };
        
        RxState state;
        FrameBuffer frame;
        size_t bytes_read;
        DalyCommand expected_command;
        
        bool validateHeader() const;
        void restartFromStartByte();
        void reset();
    
    public:
        explicit FrameParser(DalyCommand expected_cmd);
        result<bool> pushByte(uint8_t byte, FrameBuffer& out_frame);
    };
}

class UARTManager {
private:
    const struct device* uart_dev;
    struct k_mutex uart_mutex;
    struct k_msgq rx_msgq;
    std::array<char, 64U> rx_msgq_buffer; 
    DalyProtocol::FrameBuffer tx_buf;
    int tx_idx=0;
    atomic_t hw_fault; 
    AtomicCommStatistics stats;
    bool initialized;

    static void uart_isr(const struct device* dev, void* user_data);
    static atomic_val_t faultToAtomic(CommFault fault);
    static CommFault atomicToFault(atomic_val_t fault);
    result<bool> receiveFrame(DalyCommand cmd, DalyProtocol::FrameBuffer& rx_frame);

public:
    explicit UARTManager(const struct device* dev);
    bool init();
    result<bool> executeTransaction(DalyCommand cmd, DalyProtocol::Payload& payload_out);
    void recordRetry(); 
    CommStatistics getStatsSnapshot() const;
};

struct BatteryLimits {
    static constexpr uint8_t CUTOFF_SOC_PCT = 10U;
    static constexpr uint8_t REENABLE_SOC_PCT = 15U;
    static constexpr uint32_t CACHE_STALE_MS = 3000U;
};

struct BmsCache {
    Millivolts voltage{};
    Milliamps current{};
    Percent soc{};
    Kelvin temperature{};
    MilliAmpHours capacity{};
    uint32_t timestamp_ms = 0U;
    CommFault last_error = CommFault::CACHE_INVALID;
    bool valid = false;
};

using WatchdogFeedHook = void (*)(); 

class SbsBattery {
public:
    SbsBattery(UARTManager* uart_bus, DeviceContext* context, WatchdogFeedHook hook = nullptr);
    void setWatchdogFeedHook(WatchdogFeedHook hook);
    void pollHardwareAndUpdateCache();
    
    result<Millivolts> getVoltage() const;
    result<Milliamps> getCurrent() const;
    result<Percent> getStateOfCharge() const;
    result<Kelvin> getTemperature() const;
    result<MilliAmpHours> getCapacity() const;
    
    void processFSM();
    BatteryFSM getState() const;
    CommStatistics getStats() const;
    
    void notifySystemWakeup();

private:
    UARTManager* uart_bus;
    DeviceContext* sys_context;
    BatteryFSM current_state;
    mutable struct k_mutex cache_mutex;
    
    bool full_charge_logged;
    uint32_t last_valid_comm_time;
    uint32_t consecutive_comm_failures;
    
    static constexpr uint32_t COMM_TIMEOUT_MS = 5000U;
    static constexpr uint32_t MAX_RETRIES = 3U;
    static constexpr uint32_t INITIAL_BACKOFF_MS = 20U;
    static constexpr uint32_t MAX_BACKOFF_MS = 160U;
    static constexpr uint32_t WATCHDOG_FAILURE_THRESHOLD = 2U;
    
    BmsCache cache;
    WatchdogFeedHook watchdog_feed_hook;
    
    result<bool> fetchWithRetry(DalyCommand cmd, DalyProtocol::Payload& payload);
    void feedWatchdog() const;
    void publishCache(const BmsCache& next_cache);
    void publishError(CommFault fault);
    BmsCache getCacheSnapshot() const;
};

SbsBattery* getSmartBatteryInstance();
UARTManager* getUartBusManagerInstance();
