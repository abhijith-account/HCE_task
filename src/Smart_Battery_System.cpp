#include <zephyr/logging/log.h>
#include <zephyr/kernel.h>
#include "Smart_Battery_System.h"
#include <zephyr/device.h>
#include <array>
#include <new>
#include <cstdint>
#include <algorithm>

#ifdef IS_TEST_ENVIRONMENT
    // Safely define the test variable here as a weak symbol to prevent linker errors across executables
    __attribute__((weak)) int test_iterations_remaining = 0;
    #define THREAD_LOOP_CONDITION (test_iterations_remaining > 0 ? (test_iterations_remaining--, true) : false)
#else
    #define THREAD_LOOP_CONDITION true
#endif

extern DeviceContext sys_context;
LOG_MODULE_REGISTER(BATTERY_SYS, LOG_LEVEL_INF);

extern "C" void daly_watchdog_feed_hook(void) __attribute__((weak));

#ifndef IS_TEST_ENVIRONMENT
extern "C" void daly_watchdog_feed_hook(void) {
    // This weak fallback is excluded during tests because the test 
    // executable links against the strong symbol, making this dead code.
    return;
}
#endif

namespace DalyProtocol {

uint8_t calculateChecksum(std::span<const uint8_t> bytes) {
    uint8_t sum = 0U;
    for (uint8_t byte : bytes) {
        sum = static_cast<uint8_t>(sum + byte);
    }
    return sum;
}

FrameBuffer buildRequest(DalyCommand cmd) {
    FrameBuffer frame{};
    frame[0U] = START_BYTE;
    frame[1U] = HOST_ID;
    frame[2U] = static_cast<uint8_t>(cmd);
    frame[3U] = PAYLOAD_SIZE;
    
    for (size_t i = HEADER_SIZE; i < CHECKSUM_INDEX; ++i) {
        frame[i] = 0U;
    }
    
    frame[CHECKSUM_INDEX] = calculateChecksum(std::span<const uint8_t>(frame.data(), CHECKSUM_INDEX));
    return frame;
}

result<Frame> decodeResponse(std::span<const uint8_t, FRAME_SIZE> raw_frame, DalyCommand expected_cmd) {
    if (raw_frame[0U] != START_BYTE) {
        return result<Frame>::Err(CommFault::FRAME_ERROR);
    }
    
    if ((raw_frame[1U] != BMS_ID) || (raw_frame[2U] != static_cast<uint8_t>(expected_cmd)) || (raw_frame[3U] != PAYLOAD_SIZE)) {
        return result<Frame>::Err(CommFault::FRAME_ERROR);
    }
    
    const uint8_t expected_checksum = calculateChecksum(std::span<const uint8_t>(raw_frame.data(), CHECKSUM_INDEX));
    if (expected_checksum != raw_frame[CHECKSUM_INDEX]) {
        return result<Frame>::Err(CommFault::CHECKSUM_ERROR);
    }
    
    Frame decoded{};
    decoded.command = expected_cmd;
    
    for (size_t i = 0U; i < PAYLOAD_SIZE; ++i) {
        decoded.payload[i] = raw_frame[HEADER_SIZE + i];
    }
    
    return result<Frame>::Ok(decoded);
}

FrameParser::FrameParser(DalyCommand expected_cmd) : state(RxState::WAIT_START), frame{}, bytes_read(0U), expected_command(expected_cmd) {}

bool FrameParser::validateHeader() const {
    return (frame[1U] == BMS_ID) && 
           (frame[2U] == static_cast<uint8_t>(expected_command)) && 
           (frame[3U] == PAYLOAD_SIZE);
}

void FrameParser::restartFromStartByte() {
    frame.fill(0U);
    frame[0U] = START_BYTE;
    bytes_read = 1U;
    state = RxState::READ_HEADER;
}

void FrameParser::reset() {
    frame.fill(0U);
    bytes_read = 0U;
    state = RxState::WAIT_START;
}

result<bool> FrameParser::pushByte(uint8_t byte, FrameBuffer& out_frame) {
    switch (state) {
        case RxState::WAIT_START:
            if (byte == START_BYTE) {
                frame[0U] = byte;
                bytes_read = 1U;
                state = RxState::READ_HEADER;
            }
            break;
        
        case RxState::READ_HEADER:
            if (byte == START_BYTE) {
                restartFromStartByte();
                break;
            }
            
            frame[bytes_read] = byte;
            ++bytes_read;

            if (bytes_read == HEADER_SIZE) {
                if (validateHeader()) {
                    state = RxState::READ_PAYLOAD;
                } else {
                    reset();
                    return result<bool>::Err(CommFault::FRAME_ERROR);
                }
            }
            break;
    
        case RxState::READ_PAYLOAD:
            frame[bytes_read] = byte;
            ++bytes_read;
            
            if (bytes_read == FRAME_SIZE) {
                const auto decoded = decodeResponse(std::span<const uint8_t, FRAME_SIZE>(frame.data(), FRAME_SIZE), expected_command);
                if (decoded.success) {
                    out_frame = frame;
                    reset();
                    return result<bool>::Ok(true);
                }
                
                const CommFault fault = decoded.error;
                if (byte == START_BYTE) {
                    restartFromStartByte();
                } else {
                    reset();
                }
                return result<bool>::Err(fault);
            }
            break;
        
        default:
            reset();
            return result<bool>::Err(CommFault::FRAME_ERROR);
    }
    
    return result<bool>::Ok(false);
}

} // namespace DalyProtocol

UARTManager::UARTManager(const device* dev) : uart_dev(dev), uart_mutex{}, rx_msgq{}, rx_msgq_buffer{}, tx_buf{}, tx_idx(0), hw_fault(0), stats{}, initialized(false) {
    k_mutex_init(&uart_mutex);
    k_msgq_init(&rx_msgq, rx_msgq_buffer.data(), sizeof(uint8_t), rx_msgq_buffer.size());
    atomic_set(&hw_fault, faultToAtomic(CommFault::NONE));
}

bool UARTManager::init() {
    if (!device_is_ready(uart_dev)) {
        return false;
    }
    
    k_mutex_lock(&uart_mutex, K_FOREVER);
    
    if (!initialized) {
        uart_irq_callback_user_data_set(uart_dev, uart_isr, this);
        uart_irq_rx_enable(uart_dev);
        initialized = true;
    }
    
    k_mutex_unlock(&uart_mutex);
    return true;
}

atomic_val_t UARTManager::faultToAtomic(CommFault fault) {
    return static_cast<atomic_val_t>(fault);
}

CommFault UARTManager::atomicToFault(atomic_val_t fault) {
    return static_cast<CommFault>(fault);
}

void UARTManager::recordRetry() {
    atomic_inc(&stats.retries);
}

CommStatistics UARTManager::getStatsSnapshot() const {
    CommStatistics snapshot{};
    
    snapshot.tx_frames = static_cast<uint32_t>(atomic_get(&stats.tx_frames));
    snapshot.rx_frames = static_cast<uint32_t>(atomic_get(&stats.rx_frames));
    snapshot.crc_errors = static_cast<uint32_t>(atomic_get(&stats.crc_errors));
    snapshot.frame_errors = static_cast<uint32_t>(atomic_get(&stats.frame_errors));
    snapshot.overflows = static_cast<uint32_t>(atomic_get(&stats.overflows));
    snapshot.retries = static_cast<uint32_t>(atomic_get(&stats.retries));
    snapshot.timeouts = static_cast<uint32_t>(atomic_get(&stats.timeouts));
    
    return snapshot;
}    

void UARTManager::uart_isr(const struct device *dev, void *user_data) {
    UARTManager* const manager = static_cast<UARTManager*>(user_data);
    
    uart_irq_update(dev);
    
    const int err = uart_err_check(dev);
    if (err != 0) {
        if ((err & UART_ERROR_OVERRUN) != 0) {
            atomic_set(&manager->hw_fault, faultToAtomic(CommFault::UART_OVERRUN));
        } else if ((err & UART_ERROR_FRAMING) != 0) {
            atomic_set(&manager->hw_fault, faultToAtomic(CommFault::FRAME_ERROR));
        } else if ((err & UART_ERROR_NOISE) != 0) {
            atomic_set(&manager->hw_fault, faultToAtomic(CommFault::UART_NOISE));       
        } else if ((err & UART_ERROR_PARITY) != 0) {
            atomic_set(&manager->hw_fault, faultToAtomic(CommFault::UART_PARITY));                
        } else {
            atomic_set(&manager->hw_fault, faultToAtomic(CommFault::FRAME_ERROR));
        }
    }
    
    if (uart_irq_tx_ready(dev)) {
        const int remaining = static_cast<int>(DalyProtocol::FRAME_SIZE) - manager->tx_idx;
        if (remaining > 0) {
            const int written = uart_fifo_fill(dev, &manager->tx_buf[static_cast<size_t>(manager->tx_idx)], remaining);
            manager->tx_idx += written;
        }
        
        if (manager->tx_idx >= static_cast<int>(DalyProtocol::FRAME_SIZE)) {
            uart_irq_tx_disable(dev);
        }
    }
    
    if (uart_irq_rx_ready(dev)) {
        std::array<uint8_t, 16U> temp_buffer{};
        const int len = uart_fifo_read(dev, temp_buffer.data(), static_cast<int>(temp_buffer.size()));
        for (int i = 0; i < len; ++i) {
            const uint8_t byte = temp_buffer[static_cast<size_t>(i)];
            if (k_msgq_put(&manager->rx_msgq, &byte, K_NO_WAIT) != 0) {
                atomic_set(&manager->hw_fault, faultToAtomic(CommFault::RX_OVERFLOW));
                atomic_inc(&manager->stats.overflows);
            }
        }
    }
}

result<bool> UARTManager::receiveFrame(DalyCommand cmd, DalyProtocol::FrameBuffer& rx_frame) {
    DalyProtocol::FrameParser parser(cmd);
    uint8_t rx_byte = 0U;
      
    while (true) {
        if (k_msgq_get(&rx_msgq, &rx_byte, K_MSEC(DalyProtocol::RX_TIMEOUT_MS)) != 0) {
            atomic_inc(&stats.timeouts);
            const CommFault fault = atomicToFault(atomic_get(&hw_fault));
            return (fault != CommFault::NONE) ? result<bool>::Err(fault) : result<bool>::Err(CommFault::RX_TIMEOUT);
        }
          
        const result<bool> parser_result = parser.pushByte(rx_byte, rx_frame);
        if (!parser_result.success) {
            if (parser_result.error == CommFault::CHECKSUM_ERROR) {
                atomic_inc(&stats.crc_errors);
            } else {
                atomic_inc(&stats.frame_errors);
            }
            continue;
        }
          
        if (parser_result.value) {
            return result<bool>::Ok(true);
        }
    }
}

result<bool> UARTManager::executeTransaction(DalyCommand cmd, DalyProtocol::Payload& payload_out) {
    if (!initialized) {
       return result<bool>::Err(CommFault::DEVICE_NOT_READY);
    }

    k_mutex_lock(&uart_mutex, K_FOREVER);

    tx_buf = DalyProtocol::buildRequest(cmd);
    tx_idx = 0;

    uart_irq_rx_disable(uart_dev);
    k_msgq_purge(&rx_msgq);
    uart_irq_rx_enable(uart_dev);
    
    uart_irq_tx_enable(uart_dev);
    atomic_inc(&stats.tx_frames);

    DalyProtocol::FrameBuffer rx_frame{};
    const result<bool> rx_result = receiveFrame(cmd, rx_frame);

    uart_irq_tx_disable(uart_dev);

    if (!rx_result.success) {
        k_mutex_unlock(&uart_mutex);
        return rx_result;
    }
    
    // receiveFrame guarantees the frame is valid, correctly sized, and checksummed.
    for (size_t i = 0U; i < DalyProtocol::PAYLOAD_SIZE; ++i) {
        payload_out[i] = rx_frame[DalyProtocol::HEADER_SIZE + i];
    }
    atomic_inc(&stats.rx_frames);
    
    k_mutex_unlock(&uart_mutex);
    return result<bool>::Ok(true);
}

SbsBattery::SbsBattery(UARTManager* bus, DeviceContext* context, WatchdogFeedHook hook)
    : uart_bus(bus), sys_context(context), current_state(BatteryFSM::IDLE), cache_mutex{},
      full_charge_logged(false), last_valid_comm_time(k_uptime_get_32()), consecutive_comm_failures(0U),
      cache{}, watchdog_feed_hook(hook) {
    k_mutex_init(&cache_mutex);
    if (watchdog_feed_hook == nullptr) {
        watchdog_feed_hook = daly_watchdog_feed_hook;
    }
}

void SbsBattery::setWatchdogFeedHook(WatchdogFeedHook hook) {
    k_mutex_lock(&cache_mutex, K_FOREVER);
    watchdog_feed_hook = (hook != nullptr) ? hook : daly_watchdog_feed_hook;
    k_mutex_unlock(&cache_mutex);
}

void SbsBattery::feedWatchdog() const {
    watchdog_feed_hook();
}

result<bool> SbsBattery::fetchWithRetry(DalyCommand cmd, DalyProtocol::Payload& payload) {
    result<bool> response = result<bool>::Err(CommFault::RX_TIMEOUT);
    uint32_t backoff_ms = INITIAL_BACKOFF_MS;
    
    for (uint32_t attempt = 0U; attempt <= MAX_RETRIES; ++attempt) {
        feedWatchdog();
        
        response = uart_bus->executeTransaction(cmd, payload);
        if (response.success) {
            return response;
        }
        
        uart_bus->recordRetry();
        
        if (attempt < MAX_RETRIES) {
            k_msleep(backoff_ms);
            backoff_ms = (backoff_ms >= (MAX_BACKOFF_MS / 2U)) ? MAX_BACKOFF_MS : (backoff_ms * 2U);
        }
    }
    
    return response;
}

void SbsBattery::publishCache(const BmsCache& next_cache) {
    k_mutex_lock(&cache_mutex, K_FOREVER);
    cache = next_cache;
    cache.valid = true;
    cache.last_error = CommFault::NONE;
    cache.timestamp_ms = k_uptime_get_32();
    consecutive_comm_failures = 0U;
    last_valid_comm_time = cache.timestamp_ms;
    k_mutex_unlock(&cache_mutex);
}

void SbsBattery::publishError(CommFault fault) {
    k_mutex_lock(&cache_mutex, K_FOREVER);
    cache.valid = false;
    cache.last_error = fault;
    ++consecutive_comm_failures;
    const bool watchdog_threshold_reached = consecutive_comm_failures >= WATCHDOG_FAILURE_THRESHOLD;
    const bool timeout_reached = (k_uptime_get_32() - last_valid_comm_time) > COMM_TIMEOUT_MS;
    k_mutex_unlock(&cache_mutex);
    
    if ((watchdog_threshold_reached || timeout_reached) && (current_state != BatteryFSM::CUTOFF)) {
        LOG_ERR("CRITICAL: BMS communication watchdog triggered. Fault:%d", static_cast<int>(fault));
        sys_context->triggerFault("BMS Communication Watchdog");
        current_state = BatteryFSM::CUTOFF;
    }
}

BmsCache SbsBattery::getCacheSnapshot() const {
    k_mutex_lock(&cache_mutex, K_FOREVER);
    BmsCache snapshot = cache;
    k_mutex_unlock(&cache_mutex);
    return snapshot;
}

static bool isCacheFresh(const BmsCache& c) {
    return (c.timestamp_ms != 0U) && ((k_uptime_get_32() - c.timestamp_ms) <= BatteryLimits::CACHE_STALE_MS);
}

static CommFault cacheFailureReason(const BmsCache& c) {
    return (c.last_error != CommFault::NONE) ? c.last_error : CommFault::CACHE_INVALID;
}

static bool isCacheValid(const BmsCache& c) {
    return c.valid && (c.last_error == CommFault::NONE) && isCacheFresh(c);
}

void SbsBattery::pollHardwareAndUpdateCache() {
#ifdef CONFIG_BOARD_QEMU_CORTEX_M3
    BmsCache qemu_cache{};
    qemu_cache.voltage = Millivolts{11100U};
    qemu_cache.current = Milliamps{-150};
    qemu_cache.soc = Percent{95U};
    qemu_cache.temperature = Kelvin{2980U};
    qemu_cache.capacity = MilliAmpHours{2500U};
    publishCache(qemu_cache);
    feedWatchdog();
    return;
#else
    DalyProtocol::Payload payload{};
    BmsCache next_cache{};

    result<bool> result = fetchWithRetry(DalyCommand::V_I_SOC, payload);
    if (!result.success) {
        publishError(result.error);
        return;
    }

    const uint16_t pack_voltage_deci_volts = static_cast<uint16_t>((static_cast<uint16_t>(payload[0U]) << 8U) | static_cast<uint16_t>(payload[1U]));
    const int32_t raw_current = static_cast<int32_t>((static_cast<uint16_t>(payload[4U]) << 8U) | static_cast<uint16_t>(payload[5U]));
    const uint16_t soc_tenths = static_cast<uint16_t>((static_cast<uint16_t>(payload[6U]) << 8U) | static_cast<uint16_t>(payload[7U]));
    
    const uint32_t pack_voltage_mv = static_cast<uint32_t>(pack_voltage_deci_volts) * 100U;
    next_cache.voltage = Millivolts{static_cast<uint16_t>(pack_voltage_mv)};
    next_cache.current = Milliamps{static_cast<int32_t>((raw_current - 30000L) * 100L)};
    next_cache.soc = Percent{static_cast<uint8_t>(soc_tenths / 10U)};
    
    if ((next_cache.soc.value > 100U) || (pack_voltage_mv > 20000U)) {
        publishError(CommFault::VALIDATION_ERROR);
        return;
    }
    
    result = fetchWithRetry(DalyCommand::TEMP, payload);
    if (!result.success) {
        publishError(result.error);
        return;
    }
    
    const int16_t temp_celsius = static_cast<int16_t>(payload[1U]) - 40;
    if (temp_celsius > 125) {
        publishError(CommFault::VALIDATION_ERROR);
        return;
    }
    next_cache.temperature = Kelvin{static_cast<uint16_t>((temp_celsius * 10) + 2731)};
    
    result = fetchWithRetry(DalyCommand::STATUS_CAPACITY, payload);
    if (!result.success) {
        publishError(result.error);
        return;
    }
    
    next_cache.capacity = MilliAmpHours{(static_cast<uint32_t>(payload[4U]) << 24U) | (static_cast<uint32_t>(payload[5U]) << 16U) | (static_cast<uint32_t>(payload[6U]) << 8U) | static_cast<uint32_t>(payload[7U])};
    
    publishCache(next_cache);
    feedWatchdog();
#endif
}

result<Millivolts> SbsBattery::getVoltage() const {
    const BmsCache snapshot = getCacheSnapshot();
    return isCacheValid(snapshot) ? result<Millivolts>::Ok(snapshot.voltage) : result<Millivolts>::Err(cacheFailureReason(snapshot)); 
}

result<Milliamps> SbsBattery::getCurrent() const {
    const BmsCache snapshot = getCacheSnapshot();
    return isCacheValid(snapshot) ? result<Milliamps>::Ok(snapshot.current) : result<Milliamps>::Err(cacheFailureReason(snapshot));
}

result<Percent> SbsBattery::getStateOfCharge() const {
    const BmsCache snapshot = getCacheSnapshot();
    return isCacheValid(snapshot) ? result<Percent>::Ok(snapshot.soc) : result<Percent>::Err(cacheFailureReason(snapshot));
}

result<Kelvin> SbsBattery::getTemperature() const {
    const BmsCache snapshot = getCacheSnapshot();
    return isCacheValid(snapshot) ? result<Kelvin>::Ok(snapshot.temperature) : result<Kelvin>::Err(cacheFailureReason(snapshot));
}

result<MilliAmpHours> SbsBattery::getCapacity() const { 
    const BmsCache snapshot = getCacheSnapshot();
    return isCacheValid(snapshot) ? result<MilliAmpHours>::Ok(snapshot.capacity) : result<MilliAmpHours>::Err(cacheFailureReason(snapshot));
}

void SbsBattery::processFSM() {
    BmsCache snapshot = getCacheSnapshot();
    auto current_res = getCurrent();
    auto soc_res = getStateOfCharge();
    
    if (!current_res.success) {
        LOG_ERR("Battery cache unavailable. Error Code:%d", static_cast<int>(snapshot.last_error));
        return;
    }
    
    const int32_t current_ma = current_res.value.value;
    const uint8_t soc_pct = soc_res.value.value;
    
    if (current_state != BatteryFSM::CUTOFF) {
        if (current_ma > 0) {
            current_state = BatteryFSM::CHARGING;
        } else if (current_ma < 0) {
            current_state = BatteryFSM::DISCHARGING;
        } else {
            current_state = BatteryFSM::IDLE;
        }
    }
    
    if ((soc_pct >= 100U) && (!full_charge_logged)) {
        LOG_INF("BATTERY FULLY CHARGED. Triggering NVS Log entry.");
        full_charge_logged = true;
    } else if (soc_pct < 95U) {
        full_charge_logged = false;
    }
    
    if (soc_pct <= BatteryLimits::CUTOFF_SOC_PCT) {
        if (current_state != BatteryFSM::CUTOFF) {
            LOG_ERR("DISCHARGE GUARD TRIGGERED! SoC:%u%%. Halting System.", soc_pct);
            sys_context->triggerFault("Battery Critically Low");
            current_state = BatteryFSM::CUTOFF;
        }
    } else if (soc_pct >= BatteryLimits::REENABLE_SOC_PCT) {
        if (sys_context->getState() == SystemState::SAFE_HALT) {
            LOG_INF("Battery recovered to %u%%. System safe to restart.", soc_pct);
            sys_context->requestTransition(SystemState::INIT);
            current_state = (current_ma > 0) ? BatteryFSM::CHARGING : BatteryFSM::IDLE;
        }
    }
}

BatteryFSM SbsBattery::getState() const {
    return current_state;
}  

CommStatistics SbsBattery::getStats() const {
    return uart_bus->getStatsSnapshot();
}

UARTManager* uart_bus_manager = nullptr;
SbsBattery* smart_battery = nullptr;
namespace {
static const device* uart_hardware = DEVICE_DT_GET(DT_NODELABEL(usart1));
static UARTManager static_uart_manager(uart_hardware);
static SbsBattery static_smart_battery(&static_uart_manager, &sys_context, daly_watchdog_feed_hook);
static bool bms_objects_initialized = false;

K_SEM_DEFINE(bms_objects_ready_sem, 0, 1);

static void initializeBmsObjects() {
    if (!bms_objects_initialized) {
        uart_bus_manager = &static_uart_manager;
        smart_battery = &static_smart_battery;
        bms_objects_initialized = true;
    }
}
}
SbsBattery* getSmartBatteryInstance() {
    initializeBmsObjects();
    return smart_battery;
}
UARTManager* getUartBusManagerInstance() {
    initializeBmsObjects();
    return uart_bus_manager;
}

void bms_comm_thread(void) {
    initializeBmsObjects();
    k_sem_give(&bms_objects_ready_sem);

    if ((uart_bus_manager == nullptr) || (!uart_bus_manager->init())) {
        LOG_ERR("UART hardware not ready.");
        return;
    }

    do {
       if (smart_battery != nullptr) {
           smart_battery->pollHardwareAndUpdateCache();
       }
       k_msleep(1000);
    } while(THREAD_LOOP_CONDITION);
}

void battery_monitor_thread(void) {
    if (!device_is_ready(uart_hardware)) {
        LOG_ERR("UART hardware not ready. Battery monitor halting.");
        return;
    }
    
    do {
        if (smart_battery != nullptr) {
            smart_battery->processFSM();
        
            if (smart_battery->getState() == BatteryFSM::DISCHARGING) {
                const auto soc = smart_battery->getStateOfCharge();
                if (soc.success) {
                    LOG_INF("Battery Discharging: %u%% remaining", soc.value.value);
                }
            }
        }
        k_msleep(1000);
    } while(THREAD_LOOP_CONDITION);
}

K_THREAD_DEFINE(bms_comm_tid, 1536, bms_comm_thread, NULL, NULL, NULL, 10, 0, 0);
K_THREAD_DEFINE(battery_tid, 1024, battery_monitor_thread, NULL, NULL, NULL, 11, 0, 0);
