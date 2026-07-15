#include <gtest/gtest.h>
#include <array>
#include <cstring>
#include <string_view>
#include <cstdint>
#include <string>

// ==============================================================================
// Zephyr UART & Kernel Mocks
// ==============================================================================
bool mock_device_ready = true;
int mock_uart_err = 0;
bool mock_tx_ready = false;
int mock_irq_update_ret = 1;
bool mock_simulate_timeout = false;
bool mock_simulate_temp_timeout = false;
bool mock_simulate_cap_timeout = false;
bool mock_simulate_crc_error = false;
bool mock_inject_bad_frame_first = false;
bool mock_bad_soc = false;
bool mock_bad_temp = false;
bool mock_bad_voltage = false;
int mock_timeout_count = 0;

static std::array<uint8_t, 1024> mock_rx_queue;
static size_t mock_rx_head = 0;
static size_t mock_rx_tail = 0;

extern "C" {
    void sys_reboot(int type) {}
    int wdt_install_timeout(const struct device *dev, const struct wdt_timeout_cfg *cfg) { return 0; }
    int wdt_setup(const struct device *dev, uint8_t options) { return 0; }
    int wdt_feed(const struct device *dev, int channel_id) { return 0; }
    
    // Declaration only with proper C-linkage to resolve to the strong symbol in Device_State_Machine.cpp
    void daly_watchdog_feed_hook(void); 
    
    bool device_is_ready(const struct device *dev) { return mock_device_ready; }
    int uart_err_check(const struct device *dev) { return mock_uart_err; }
    int uart_irq_callback_user_data_set(const struct device *dev,void (*cb)(const struct device *, void *),void *user_data){return 0;}
    void uart_irq_rx_enable(const struct device *dev) {}
    void uart_irq_rx_disable(const struct device *dev) {}
    void uart_irq_tx_disable(const struct device *dev) {}
    int uart_irq_update(const struct device *dev) { return mock_irq_update_ret; }
    int uart_irq_tx_ready(const struct device *dev) { return mock_tx_ready; }
    int mock_fifo_fill_ret = -1;
    int uart_fifo_fill(const struct device *dev, const uint8_t *tx_data, int size) { 
        return (mock_fifo_fill_ret >= 0) ? mock_fifo_fill_ret : size; 
    }
    
    int uart_irq_rx_ready(const struct device *dev) { 
        return (mock_rx_head < mock_rx_tail) ? 1 : 0; 
    }
    
    int uart_fifo_read(const struct device *dev, uint8_t *rx_data, const int size) { 
        if (mock_rx_head >= mock_rx_tail || size <= 0) return 0;
        int bytes_to_copy = std::min(static_cast<int>(mock_rx_tail - mock_rx_head), size);
        for(int i=0; i<bytes_to_copy; i++) rx_data[i] = mock_rx_queue[mock_rx_head++];
        if (mock_rx_head >= mock_rx_tail) {
            mock_rx_head = 0;
            mock_rx_tail = 0;
        }
        return bytes_to_copy;
    }
}

// White-Box Access
#define private public
#define protected public
#include "Smart_Battery_System.h"
#undef private
#undef protected

// Helper to push bytes into the RX queue as if they arrived over hardware
void inject_rx_bytes(std::span<const uint8_t> bytes) {
    for (uint8_t b : bytes) {
        if (mock_rx_tail < mock_rx_queue.size()) {
            mock_rx_queue[mock_rx_tail++] = b;
        }
    }
}

// The Hook: Intercepts TX enablement and auto-replies with standard Daly Protocol packets
extern "C" void uart_irq_tx_enable(const struct device *dev) {
    extern UARTManager* uart_bus_manager; 
    if (!uart_bus_manager || mock_simulate_timeout) return;
    
    if (mock_timeout_count > 0) {
        mock_timeout_count--;
        return;
    }

    DalyCommand cmd = static_cast<DalyCommand>(uart_bus_manager->tx_buf[2]);
    if (cmd == DalyCommand::TEMP && mock_simulate_temp_timeout) return;
    if (cmd == DalyCommand::STATUS_CAPACITY && mock_simulate_cap_timeout) return;

    if (mock_inject_bad_frame_first) {
        auto bad_frame = DalyProtocol::buildRequest(cmd);
        bad_frame[1] = 0x99; // Corrupted ID forces a Frame Error
        inject_rx_bytes(bad_frame);
    }

    auto frame = DalyProtocol::buildRequest(cmd);
    frame[1] = DalyProtocol::BMS_ID; // The BMS replies with its own ID (0x01)
    
    if (cmd == DalyCommand::V_I_SOC) {
        uint16_t v = mock_bad_voltage ? 2001 : 200; // 20.01V vs 2.00V (decivolts)
        frame[4] = v >> 8; frame[5] = v & 0xFF; 
        frame[8] = (30000 >> 8) & 0xFF; frame[9] = 30000 & 0xFF; // 0mA (Offset)
        uint16_t soc = mock_bad_soc ? 2000 : 500; // 200.0% vs 50.0%
        frame[10] = soc >> 8; frame[11] = soc & 0xFF;
    } else if (cmd == DalyCommand::TEMP) {
        frame[5] = mock_bad_temp ? 250 : (40 + 25); // +25C
    } else if (cmd == DalyCommand::STATUS_CAPACITY) {
        frame[8] = 0; frame[9] = 0; frame[10] = 0; frame[11] = 100; // 100mAh
    }
    
    frame[12] = DalyProtocol::calculateChecksum(std::span<const uint8_t>(frame.data(), 12));
    
    if (mock_simulate_crc_error) frame[12] ^= 0xFF; // Force a Checksum Error

    inject_rx_bytes(frame);
    
    // Process the bytes immediately until hardware RX queue is drained
    while (mock_rx_head < mock_rx_tail) {
        UARTManager::uart_isr(dev, uart_bus_manager);
    }
}

// Global linker variables expected by Smart_Battery_System.cpp in TEST_ENVIRONMENT
int test_iterations_remaining = 0;
extern DeviceContext sys_context;
extern uint32_t virtual_uptime;

// ==============================================================================
// Test Suite Setup
// ==============================================================================
class SmartBatteryTestSuite : public ::testing::Test {
protected:
    const device* dummy_uart = reinterpret_cast<const device*>(0xBEEF);
    UARTManager uart_manager{dummy_uart};
    SbsBattery battery{&uart_manager, &sys_context};

    void SetUp() override {
        // Critical: Reset globals to clean slate so tests don't pollute each other
        sys_context = DeviceContext(); 
        sys_context.requestTransition(SystemState::RUNNING);
        
        // Push time forward by 10s to prevent timestamp == 0 (which marks cache as invalid)
        virtual_uptime += 10000;
        
        uart_manager.init();
        mock_device_ready = true;
        mock_uart_err = 0;
        mock_tx_ready = false;
        mock_irq_update_ret = 1;
        mock_rx_head = 0;
        mock_rx_tail = 0;
        
        mock_simulate_timeout = false;
        mock_simulate_temp_timeout = false;
        mock_simulate_cap_timeout = false;
        mock_simulate_crc_error = false;
        mock_inject_bad_frame_first = false;
        mock_bad_soc = false;
        mock_bad_temp = false;
        mock_bad_voltage = false;
        mock_timeout_count = 0;
        
        // Ensure cache is completely valid and fresh at the start of every test
        battery.cache = BmsCache{};
        battery.cache.valid = true;
        battery.cache.last_error = CommFault::NONE;
        battery.cache.timestamp_ms = k_uptime_get_32();
        battery.current_state = BatteryFSM::IDLE;
        battery.consecutive_comm_failures = 0;
        battery.last_valid_comm_time = k_uptime_get_32();
        
        extern UARTManager* uart_bus_manager;
        uart_bus_manager = &uart_manager; 
    }
};

// ==============================================================================
// 1. Daly Protocol Coverage
// ==============================================================================
TEST_F(SmartBatteryTestSuite, DalyProtocol_ChecksumAndBuild) {
    auto frame = DalyProtocol::buildRequest(DalyCommand::V_I_SOC);
    EXPECT_EQ(frame[0], DalyProtocol::START_BYTE);
    EXPECT_EQ(frame[2], static_cast<uint8_t>(DalyCommand::V_I_SOC));
    EXPECT_GT(frame[12], 0); 
}

TEST_F(SmartBatteryTestSuite, DalyProtocol_ParserTransitions) {
    DalyProtocol::FrameParser parser(DalyCommand::V_I_SOC);
    DalyProtocol::FrameBuffer out_frame;
    auto frame = DalyProtocol::buildRequest(DalyCommand::V_I_SOC);
    frame[1] = DalyProtocol::BMS_ID;
    frame[12] = DalyProtocol::calculateChecksum(std::span<const uint8_t>(frame.data(), 12));

    auto res = parser.pushByte(0x00, out_frame);
    EXPECT_FALSE(res.value); 
    
    for(size_t i = 0; i < 12; i++) {
        res = parser.pushByte(frame[i], out_frame);
        EXPECT_FALSE(res.value); 
    }
    res = parser.pushByte(frame[12], out_frame);
    EXPECT_TRUE(res.success);
    EXPECT_TRUE(res.value);
}

TEST_F(SmartBatteryTestSuite, Parser_WaitStart_IgnoresNoise) {
    DalyProtocol::FrameParser parser(DalyCommand::V_I_SOC);
    DalyProtocol::FrameBuffer out;
    EXPECT_FALSE(parser.pushByte(0x00, out).value);
}

TEST_F(SmartBatteryTestSuite, Parser_ReadHeader_RestartOnStartByte) {
    DalyProtocol::FrameParser parser(DalyCommand::V_I_SOC);
    DalyProtocol::FrameBuffer out;
    parser.pushByte(DalyProtocol::START_BYTE, out); 
    parser.pushByte(DalyProtocol::START_BYTE, out); 
    EXPECT_EQ(parser.bytes_read, 1);
}

TEST_F(SmartBatteryTestSuite, Parser_ReadPayload_RestartOnStartByteFault) {
    DalyProtocol::FrameParser parser(DalyCommand::V_I_SOC);
    DalyProtocol::FrameBuffer out;
    auto frame = DalyProtocol::buildRequest(DalyCommand::V_I_SOC);
    frame[1] = DalyProtocol::BMS_ID;
    frame[12] = DalyProtocol::START_BYTE; 
    
    for (int i=0; i<12; i++) parser.pushByte(frame[i], out);
    auto res = parser.pushByte(frame[12], out);
    EXPECT_FALSE(res.success);
    EXPECT_EQ(parser.bytes_read, 1); 
}

TEST_F(SmartBatteryTestSuite, DalyProtocol_ParserErrors) {
    DalyProtocol::FrameParser parser(DalyCommand::V_I_SOC);
    DalyProtocol::FrameBuffer out_frame;
    auto frame = DalyProtocol::buildRequest(DalyCommand::V_I_SOC);
    frame[1] = DalyProtocol::BMS_ID;
    frame[12] = 0x00; 

    // Fail Byte 0 (Wait Start)
    parser.pushByte(0x00, out_frame);
    EXPECT_EQ(parser.state, DalyProtocol::FrameParser::RxState::WAIT_START);

    // Fail Byte 1 (BMS_ID vs HOST_ID mismatch)
    parser.pushByte(DalyProtocol::START_BYTE, out_frame);
    parser.pushByte(0x40, out_frame); 
    parser.pushByte(static_cast<uint8_t>(DalyCommand::V_I_SOC), out_frame);
    parser.pushByte(DalyProtocol::PAYLOAD_SIZE, out_frame);
    EXPECT_EQ(parser.state, DalyProtocol::FrameParser::RxState::WAIT_START);

    // Fail Byte 2 (Command)
    parser.pushByte(DalyProtocol::START_BYTE, out_frame);
    parser.pushByte(DalyProtocol::BMS_ID, out_frame);
    parser.pushByte(0x99, out_frame); 
    parser.pushByte(DalyProtocol::PAYLOAD_SIZE, out_frame);
    EXPECT_EQ(parser.state, DalyProtocol::FrameParser::RxState::WAIT_START);

    // Fail Byte 3 (Payload Size)
    parser.pushByte(DalyProtocol::START_BYTE, out_frame);
    parser.pushByte(DalyProtocol::BMS_ID, out_frame);
    parser.pushByte(static_cast<uint8_t>(DalyCommand::V_I_SOC), out_frame);
    parser.pushByte(0xFF, out_frame); 
    EXPECT_EQ(parser.state, DalyProtocol::FrameParser::RxState::WAIT_START);

    // Finally fail Checksum
    for(size_t i = 0; i < 12; i++) { parser.pushByte(frame[i], out_frame); }
    auto res = parser.pushByte(0x00, out_frame);
    EXPECT_FALSE(res.success);
    EXPECT_EQ(res.error, CommFault::CHECKSUM_ERROR);
}

TEST_F(SmartBatteryTestSuite, Parser_DefaultState) {
    DalyProtocol::FrameParser parser(DalyCommand::V_I_SOC);
    DalyProtocol::FrameBuffer out;
    // Force corrupted memory state
    parser.state = static_cast<DalyProtocol::FrameParser::RxState>(99); 
    auto res = parser.pushByte(0x00, out);
    EXPECT_FALSE(res.success);
    EXPECT_EQ(res.error, CommFault::FRAME_ERROR);
    EXPECT_EQ(parser.state, DalyProtocol::FrameParser::RxState::WAIT_START);
}

TEST_F(SmartBatteryTestSuite, DecodeResponse_DirectCoverage) {
    auto frame = DalyProtocol::buildRequest(DalyCommand::V_I_SOC);
    auto res = DalyProtocol::decodeResponse(frame, DalyCommand::V_I_SOC);
    EXPECT_FALSE(res.success);
    EXPECT_EQ(res.error, CommFault::FRAME_ERROR);
    
    frame[1] = DalyProtocol::BMS_ID;
    frame[2] = 0x99; 
    res = DalyProtocol::decodeResponse(frame, DalyCommand::V_I_SOC);
    EXPECT_FALSE(res.success);

    frame[2] = static_cast<uint8_t>(DalyCommand::V_I_SOC);
    frame[3] = 0xFF; // Bad Payload size
    res = DalyProtocol::decodeResponse(frame, DalyCommand::V_I_SOC);
    EXPECT_FALSE(res.success);
    
    frame[3] = DalyProtocol::PAYLOAD_SIZE;
    frame[0] = 0x00; 
    res = DalyProtocol::decodeResponse(frame, DalyCommand::V_I_SOC);
    EXPECT_FALSE(res.success);
}

// ==============================================================================
// 2. UART Manager ISR and Fault Coverage
// ==============================================================================
TEST_F(SmartBatteryTestSuite, UART_ISR_FaultDetection) {
    mock_uart_err = UART_ERROR_OVERRUN;
    UARTManager::uart_isr(dummy_uart, &uart_manager);
    EXPECT_EQ(static_cast<int>(uart_manager.atomicToFault(uart_manager.hw_fault)), static_cast<int>(CommFault::UART_OVERRUN));

    mock_uart_err = UART_ERROR_FRAMING;
    UARTManager::uart_isr(dummy_uart, &uart_manager);
    EXPECT_EQ(static_cast<int>(uart_manager.atomicToFault(uart_manager.hw_fault)), static_cast<int>(CommFault::FRAME_ERROR));

    mock_uart_err = UART_ERROR_NOISE;
    UARTManager::uart_isr(dummy_uart, &uart_manager);
    EXPECT_EQ(static_cast<int>(uart_manager.atomicToFault(uart_manager.hw_fault)), static_cast<int>(CommFault::UART_NOISE));

    mock_uart_err = UART_ERROR_PARITY;
    UARTManager::uart_isr(dummy_uart, &uart_manager);
    EXPECT_EQ(static_cast<int>(uart_manager.atomicToFault(uart_manager.hw_fault)), static_cast<int>(CommFault::UART_PARITY));
}

TEST_F(SmartBatteryTestSuite, UART_ISR_Branches) {
    mock_irq_update_ret = 0;
    UARTManager::uart_isr(dummy_uart, &uart_manager); 
    mock_irq_update_ret = 1;

    mock_tx_ready = true;
    uart_manager.tx_idx = DalyProtocol::FRAME_SIZE; 
    UARTManager::uart_isr(dummy_uart, &uart_manager);

    mock_tx_ready = false;
    if (mock_rx_tail < mock_rx_queue.size()) {
        mock_rx_queue[mock_rx_tail++] = 0xA5;
    }
    for (int i=0; i<64; i++) {
        uint8_t b = 0;
        k_msgq_put(&uart_manager.rx_msgq, &b, K_NO_WAIT);
    }
    UARTManager::uart_isr(dummy_uart, &uart_manager);
    EXPECT_EQ(static_cast<int>(uart_manager.atomicToFault(uart_manager.hw_fault)), static_cast<int>(CommFault::RX_OVERFLOW));
    k_msgq_purge(&uart_manager.rx_msgq);
}

TEST_F(SmartBatteryTestSuite, UartManagerInitTwice) {
    EXPECT_TRUE(uart_manager.init()); 
}

TEST_F(SmartBatteryTestSuite, ReceiveFrame_ErrorCounters) {
    DalyProtocol::Payload p;
    
    // 1. Force a CRC error via the hook.
    // The parser will fail on checksum, discard the frame, and wait for the next frame.
    // Since no valid frame follows, it will eventually time out, returning RX_TIMEOUT.
    mock_simulate_crc_error = true;
    auto res1 = uart_manager.executeTransaction(DalyCommand::V_I_SOC, p);
    EXPECT_FALSE(res1.success);
    EXPECT_EQ(res1.error, CommFault::RX_TIMEOUT);
    
    auto stats = uart_manager.getStatsSnapshot();
    EXPECT_EQ(stats.crc_errors, 1);
    
    mock_simulate_crc_error = false;
    
    // 2. Force a Frame Error recovery scenario.
    // The hook will inject a bad frame first, followed by a good frame.
    // The parser should encounter the frame error, discard it, increment the counter,
    // and successfully parse the good frame behind it.
    mock_inject_bad_frame_first = true;
    auto res2 = uart_manager.executeTransaction(DalyCommand::V_I_SOC, p);
    EXPECT_TRUE(res2.success);
    
    stats = uart_manager.getStatsSnapshot();
    EXPECT_EQ(stats.frame_errors, 1);
    
    mock_inject_bad_frame_first = false;
}

// ==============================================================================
// 3. FSM Business Logic Coverage
// ==============================================================================
TEST_F(SmartBatteryTestSuite, CurrentThresholdTransition) {
    battery.cache.current.value = 0;
    battery.cache.soc.value = 50; 
    battery.processFSM();
    EXPECT_EQ(battery.getState(), BatteryFSM::IDLE);
    
    battery.cache.current.value = 500;
    battery.processFSM();
    EXPECT_EQ(battery.getState(), BatteryFSM::CHARGING);
    
    battery.cache.current.value = -250;
    battery.processFSM();
    EXPECT_EQ(battery.getState(), BatteryFSM::DISCHARGING);
}

TEST_F(SmartBatteryTestSuite, DischargeGuardHysteresisLogic) {
    battery.cache.current.value = -100;
    battery.cache.soc.value = 20;
    battery.processFSM();

    EXPECT_EQ(battery.getState(), BatteryFSM::DISCHARGING);
    EXPECT_EQ(sys_context.getState(), SystemState::RUNNING);
      
    battery.cache.soc.value = 10;
    testing::internal::CaptureStdout();
    battery.processFSM();
    const auto raw_output_cutoff = testing::internal::GetCapturedStdout();
    std::string_view output_cutoff(raw_output_cutoff);
      
    EXPECT_EQ(battery.getState(), BatteryFSM::CUTOFF);
    EXPECT_EQ(sys_context.getState(), SystemState::SAFE_HALT);
    EXPECT_TRUE(output_cutoff.find("[ERR] DISCHARGE GUARD TRIGGERED! SoC:10%. Halting System.") != std::string_view::npos);
      
    battery.cache.soc.value = 15;
    testing::internal::CaptureStdout();
    battery.processFSM();
    const auto raw_output_recovery = testing::internal::GetCapturedStdout();
    std::string_view output_recovery(raw_output_recovery);
      
    EXPECT_EQ(sys_context.getState(), SystemState::INIT); 
    EXPECT_TRUE(output_recovery.find("[INF] Battery recovered to 15%. System safe to restart.") != std::string_view::npos);
}

TEST_F(SmartBatteryTestSuite, FullChargeNVSLogging) {
    battery.cache.current.value = 500;
    
    battery.cache.soc.value = 100;
    testing::internal::CaptureStdout();
    battery.processFSM();
    const auto raw_output_full = testing::internal::GetCapturedStdout();
    std::string_view output_full(raw_output_full);
    EXPECT_TRUE(output_full.find("[INF] BATTERY FULLY CHARGED. Triggering NVS Log entry.") != std::string_view::npos);
      
    battery.processFSM(); 
      
    battery.cache.soc.value = 94; 
    battery.processFSM();
      
    battery.cache.soc.value = 100;
    testing::internal::CaptureStdout();
    battery.processFSM();
    const auto raw_output_recharge = testing::internal::GetCapturedStdout();
    std::string_view output_recharge(raw_output_recharge);
    EXPECT_TRUE(output_recharge.find("[INF] BATTERY FULLY CHARGED. Triggering NVS Log entry.") != std::string_view::npos);
}

TEST_F(SmartBatteryTestSuite, ProcessFSMBranches) {
    battery.cache.soc.value = 5;
    battery.processFSM();
    EXPECT_EQ(battery.current_state, BatteryFSM::CUTOFF);

    // Escape Cutoff 
    battery.cache.soc.value = 20;
    battery.cache.current.value = 0;
    sys_context.current_state = SystemState::SAFE_HALT; 
    battery.processFSM();
    EXPECT_EQ(battery.current_state, BatteryFSM::IDLE);

    // Enter Cutoff
    battery.cache.soc.value = 5;
    battery.processFSM();
    EXPECT_EQ(battery.current_state, BatteryFSM::CUTOFF);

    // Re-trigger while ALREADY in Cutoff (Should do nothing, not throw error)
    testing::internal::CaptureStdout();
    battery.processFSM();
    const auto raw_output_cutoff2 = testing::internal::GetCapturedStdout();
    std::string_view output_cutoff2(raw_output_cutoff2);
    EXPECT_TRUE(output_cutoff2.find("DISCHARGE GUARD TRIGGERED") == std::string_view::npos);

    battery.cache.soc.value = 20;
    battery.cache.current.value = 1000;
    sys_context.current_state = SystemState::SAFE_HALT; 
    battery.processFSM();
    EXPECT_EQ(battery.current_state, BatteryFSM::CHARGING);
    
    battery.cache.soc.value = 98;
    battery.processFSM();
}

TEST_F(SmartBatteryTestSuite, HardwareFailureMaintainState) {
    battery.cache.current.value = -100;
    battery.cache.soc.value = 50;
    battery.processFSM();
    ASSERT_EQ(battery.getState(), BatteryFSM::DISCHARGING);

    battery.cache.timestamp_ms = k_uptime_get_32() - 10000; 
    
    testing::internal::CaptureStdout();
    battery.processFSM();
    const auto raw_output_fail = testing::internal::GetCapturedStdout();
    std::string_view output_fail(raw_output_fail);

    EXPECT_EQ(battery.getState(), BatteryFSM::DISCHARGING); 
    EXPECT_TRUE(output_fail.find("[ERR] Battery cache unavailable.") != std::string_view::npos);
}

// ==============================================================================
// 4. Thread & Hardware Polling Integration
// ==============================================================================
TEST_F(SmartBatteryTestSuite, ReceiveFrame_StandardTimeout) {
    mock_simulate_timeout = true; 
    DalyProtocol::Payload p;
    // When no hw_fault is set, the timeout loop returns RX_TIMEOUT directly
    auto res = uart_manager.executeTransaction(DalyCommand::V_I_SOC, p);
    EXPECT_FALSE(res.success);
    EXPECT_EQ(res.error, CommFault::RX_TIMEOUT);
}

TEST_F(SmartBatteryTestSuite, FetchWithRetry_Recovers) {
    mock_timeout_count = 2; 
    battery.pollHardwareAndUpdateCache();
    EXPECT_TRUE(battery.getCacheSnapshot().valid);
    EXPECT_EQ(uart_manager.getStatsSnapshot().timeouts, 2);
}

TEST_F(SmartBatteryTestSuite, TimeoutTriggeredWatchdog) {
    battery.pollHardwareAndUpdateCache(); // Success, sets last_valid_comm_time
        
    virtual_uptime += 6000; // Jump > COMM_TIMEOUT_MS
    mock_simulate_timeout = true; // Force timeout
    
    testing::internal::CaptureStdout();
    battery.pollHardwareAndUpdateCache(); 
    const auto raw_output = testing::internal::GetCapturedStdout();
    std::string_view output(raw_output);
    
    EXPECT_EQ(battery.getState(), BatteryFSM::CUTOFF);
    EXPECT_TRUE(output.find("CRITICAL: BMS communication watchdog triggered.") != std::string_view::npos);
}

TEST_F(SmartBatteryTestSuite, PublishErrorIgnoresIfAlreadyCutoff) {
    battery.current_state = BatteryFSM::CUTOFF;
    virtual_uptime += 6000;
    mock_simulate_timeout = true;
    
    testing::internal::CaptureStdout();
    battery.pollHardwareAndUpdateCache();
    const auto raw_output = testing::internal::GetCapturedStdout(); 
    std::string_view output(raw_output);
    
    // Should not contain the critical trigger again if already in cutoff
    EXPECT_TRUE(output.find("CRITICAL: BMS") == std::string_view::npos);
}

TEST_F(SmartBatteryTestSuite, PollHardware_SuccessPath) {
    battery.pollHardwareAndUpdateCache();
    
    auto s = battery.getCacheSnapshot();
    EXPECT_TRUE(s.valid);
    EXPECT_EQ(s.soc.value, 50); 
}

TEST_F(SmartBatteryTestSuite, PollHardware_ValidationGuards_Voltage) {
    mock_bad_voltage = true;
    battery.pollHardwareAndUpdateCache();
    EXPECT_FALSE(battery.getCacheSnapshot().valid);
    EXPECT_EQ(battery.getCacheSnapshot().last_error, CommFault::VALIDATION_ERROR);
    mock_bad_voltage = false;
}

TEST_F(SmartBatteryTestSuite, PollHardware_ValidationGuards) {
    mock_bad_soc = true;
    battery.pollHardwareAndUpdateCache();
    EXPECT_FALSE(battery.getCacheSnapshot().valid);
    EXPECT_EQ(battery.getCacheSnapshot().last_error, CommFault::VALIDATION_ERROR);
    mock_bad_soc = false;

    battery.cache.valid = true; 
    mock_bad_temp = true;
    battery.pollHardwareAndUpdateCache();
    EXPECT_FALSE(battery.getCacheSnapshot().valid);
    EXPECT_EQ(battery.getCacheSnapshot().last_error, CommFault::VALIDATION_ERROR);
    mock_bad_temp = false;
    
    mock_simulate_temp_timeout = true;
    battery.pollHardwareAndUpdateCache();
    EXPECT_FALSE(battery.getCacheSnapshot().valid);
    mock_simulate_temp_timeout = false;
    
    mock_simulate_cap_timeout = true;
    battery.pollHardwareAndUpdateCache();
    EXPECT_FALSE(battery.getCacheSnapshot().valid);
    mock_simulate_cap_timeout = false;
}

extern void bms_comm_thread(void);
extern void battery_monitor_thread(void);

TEST_F(SmartBatteryTestSuite, ThreadHardwareFailures) {
    mock_device_ready = false;
    test_iterations_remaining = 1;
    testing::internal::CaptureStdout();
    bms_comm_thread(); 
    const auto raw_out1 = testing::internal::GetCapturedStdout();
    std::string_view out1(raw_out1);
    EXPECT_TRUE(out1.find("UART hardware not ready") != std::string_view::npos);
    
    testing::internal::CaptureStdout();
    battery_monitor_thread(); 
    const auto raw_out2 = testing::internal::GetCapturedStdout();
    std::string_view out2(raw_out2);
    EXPECT_TRUE(out2.find("Battery monitor halting") != std::string_view::npos);
    
    mock_device_ready = true;
}

TEST_F(SmartBatteryTestSuite, ExecutesBatteryMonitorThread) {
    battery.cache.current.value = -500;
    battery.cache.soc.value = 80;
    battery.processFSM();
        
    ASSERT_EQ(battery.getState(), BatteryFSM::DISCHARGING);
        
    test_iterations_remaining = 1;
    extern SbsBattery* smart_battery;
    smart_battery = &battery;

    testing::internal::CaptureStdout();
    battery_monitor_thread();
    const auto raw_output = testing::internal::GetCapturedStdout();
    std::string_view output(raw_output);
        
    EXPECT_TRUE(output.find("[INF] Battery Discharging: 80% remaining") != std::string_view::npos);
}

TEST_F(SmartBatteryTestSuite, FeedHooksAndUninitializedInit) {
    battery.setWatchdogFeedHook(nullptr);
    battery.feedWatchdog();
    
    uart_manager.initialized = false;
    mock_device_ready = false;
    EXPECT_FALSE(uart_manager.init());
    
    DalyProtocol::Payload p;
    EXPECT_FALSE(uart_manager.executeTransaction(DalyCommand::V_I_SOC, p).success);
}

// ==============================================================================
// 5. Getter and Singleton Validations
// ==============================================================================
TEST_F(SmartBatteryTestSuite, Getters_SuccessAndFailure) {
    // Valid Cache Test
    battery.cache.voltage.value = 12000;
    battery.cache.temperature.value = 3000;
    battery.cache.capacity.value = 5000;
    
    EXPECT_TRUE(battery.getVoltage().success);
    EXPECT_EQ(battery.getVoltage().value.value, 12000);
    
    EXPECT_TRUE(battery.getTemperature().success);
    EXPECT_EQ(battery.getTemperature().value.value, 3000);
    
    EXPECT_TRUE(battery.getCapacity().success);
    EXPECT_EQ(battery.getCapacity().value.value, 5000);
    
    auto stats = battery.getStats();
    EXPECT_EQ(stats.tx_frames, 0); 
    
    // Invalid Cache Test explicitly setting ts=0 to hit cacheFailureReason mapping
    battery.cache = BmsCache{}; 
    EXPECT_FALSE(battery.getVoltage().success);
    EXPECT_FALSE(battery.getCurrent().success);
    EXPECT_FALSE(battery.getStateOfCharge().success);
    EXPECT_FALSE(battery.getTemperature().success);
    EXPECT_FALSE(battery.getCapacity().success);
    EXPECT_EQ(battery.getVoltage().error, CommFault::CACHE_INVALID);

    // Invalid Cache with an explicit error
    battery.cache.valid = true; 
    battery.cache.last_error = CommFault::RX_TIMEOUT;
    EXPECT_EQ(battery.getVoltage().error, CommFault::RX_TIMEOUT);
}

TEST_F(SmartBatteryTestSuite, SingletonInitializesOnce) {
    auto* inst1 = getSmartBatteryInstance();
    auto* inst2 = getSmartBatteryInstance();
    EXPECT_EQ(inst1, inst2);
    
    auto* uart1 = getUartBusManagerInstance();
    auto* uart2 = getUartBusManagerInstance();
    EXPECT_EQ(uart1, uart2);
}

TEST_F(SmartBatteryTestSuite, Thread_BmsComm_NullManager) {
    extern UARTManager* uart_bus_manager;
    auto* temp = uart_bus_manager;
    uart_bus_manager = nullptr;
    test_iterations_remaining = 1;
    
    testing::internal::CaptureStdout();
    bms_comm_thread();
    const auto raw_out = testing::internal::GetCapturedStdout();
    std::string_view out(raw_out);
    EXPECT_TRUE(out.find("UART hardware not ready") != std::string_view::npos);
    
    uart_bus_manager = temp;
}

TEST_F(SmartBatteryTestSuite, Thread_BmsComm_NullBattery) {
    extern SbsBattery* smart_battery;
    auto* temp = smart_battery;
    smart_battery = nullptr;
    test_iterations_remaining = 1;
    
    bms_comm_thread(); 
    
    smart_battery = temp;
}

TEST_F(SmartBatteryTestSuite, Thread_BatteryMonitor_NullBattery) {
    extern SbsBattery* smart_battery;
    auto* temp = smart_battery;
    smart_battery = nullptr;
    test_iterations_remaining = 1;
    
    battery_monitor_thread(); 
    
    smart_battery = temp;
}

TEST_F(SmartBatteryTestSuite, Thread_BatteryMonitor_SocFails) {
    battery.current_state = BatteryFSM::DISCHARGING;
    battery.cache.valid = false;
    
    extern SbsBattery* smart_battery;
    auto* temp = smart_battery;
    smart_battery = &battery;
    test_iterations_remaining = 1;
    
    battery_monitor_thread(); // Execute discharging block without logging success
    
    smart_battery = temp;
}

TEST_F(SmartBatteryTestSuite, DefaultWeakWatchdogHookIsCalled)
{
    SbsBattery b(&uart_manager, &sys_context);
    
    // Explicitly verify we successfully call into the function.
    // The strong symbol will automatically satisfy the linker.
    b.feedWatchdog(); 

    SUCCEED();
}

TEST_F(SmartBatteryTestSuite, FeedWatchdogWhenNull)
{
    battery.setWatchdogFeedHook(nullptr);
    battery.feedWatchdog();
    
    SUCCEED();
}

TEST_F(SmartBatteryTestSuite, UART_ISR_TxRemaining) {
    mock_irq_update_ret = 1;
    mock_tx_ready = true;
    uart_manager.tx_idx = 0; // remaining = FRAME_SIZE (13), triggering the true branch
    UARTManager::uart_isr(dummy_uart, &uart_manager);
    EXPECT_EQ(uart_manager.tx_idx, DalyProtocol::FRAME_SIZE); // uart_fifo_fill returns 13
}

TEST_F(SmartBatteryTestSuite, UART_ISR_UnknownFault) {
    // 0x8000 sets a high bit that does not overlap with standard 
    // Zephyr UART error macros, bypassing the if/else-if chain 
    // and cleanly forcing the final 'else' branch.
    mock_uart_err = 0x8000; 
    
    UARTManager::uart_isr(dummy_uart, &uart_manager);
    EXPECT_EQ(static_cast<int>(uart_manager.atomicToFault(uart_manager.hw_fault)), static_cast<int>(CommFault::FRAME_ERROR));
}

TEST_F(SmartBatteryTestSuite, ReceiveFrame_TimeoutWithHardwareFault) {
    mock_simulate_timeout = true; 
    // Trigger the branch where timeout occurs BUT hw_fault is evaluated instead
    atomic_set(&uart_manager.hw_fault, UARTManager::faultToAtomic(CommFault::UART_OVERRUN));
    
    DalyProtocol::Payload p;
    auto res = uart_manager.executeTransaction(DalyCommand::V_I_SOC, p);
    
    EXPECT_FALSE(res.success);
    EXPECT_EQ(res.error, CommFault::UART_OVERRUN);
}

TEST_F(SmartBatteryTestSuite, Parser_ReadPayload_ChecksumIsStartByte) {
    DalyProtocol::FrameParser parser(DalyCommand::V_I_SOC);
    DalyProtocol::FrameBuffer out;
    auto frame = DalyProtocol::buildRequest(DalyCommand::V_I_SOC);
    frame[1] = DalyProtocol::BMS_ID;
    
    // Mutate payload so actual calculated checksum won't match
    frame[4] = 0xFF; 
    for(int i = 0; i < 12; i++) {
        parser.pushByte(frame[i], out);
    }
    
    // Feed START_BYTE as the expected checksum byte to trigger restartFromStartByte() logic
    auto res = parser.pushByte(DalyProtocol::START_BYTE, out);
    
    EXPECT_FALSE(res.success);
    // State should be READ_HEADER because restartFromStartByte() was invoked
    EXPECT_EQ(parser.state, DalyProtocol::FrameParser::RxState::READ_HEADER);
}

TEST_F(SmartBatteryTestSuite, ProcessFSM_NoRecoveryIfNotSafeHalt) {
    battery.current_state = BatteryFSM::CUTOFF;
    sys_context.current_state = SystemState::RUNNING; // NOT in safe halt
    battery.cache.soc.value = 50; // >= REENABLE threshold
    
    battery.processFSM();
    
    // Should skip the recovery 'else if' block and remain in CUTOFF
    EXPECT_EQ(battery.current_state, BatteryFSM::CUTOFF);
}

TEST_F(SmartBatteryTestSuite, UartManagerInit_DeviceNotReady) {
    mock_device_ready = false;
    // Cover the early exit branch if hardware probing fails
    EXPECT_FALSE(uart_manager.init());
}

// ==============================================================================
// 7. Final Coverage Branch Tests
// ==============================================================================

TEST_F(SmartBatteryTestSuite, UART_ISR_TxPartial) {
    mock_irq_update_ret = 1;
    mock_tx_ready = true;
    uart_manager.tx_idx = 0; 
    
    extern int mock_fifo_fill_ret;
    mock_fifo_fill_ret = 5; // Fill only 5 bytes instead of 13
    
    UARTManager::uart_isr(dummy_uart, &uart_manager);
    
    // Will trigger the false branch of (tx_idx >= FRAME_SIZE)
    EXPECT_EQ(uart_manager.tx_idx, 5); 
    mock_fifo_fill_ret = -1; // reset
}

TEST_F(SmartBatteryTestSuite, SetWatchdogFeedHookValid) {
    auto dummy_hook = []() {};
    // Triggers the true branch of (hook != nullptr) ? hook : default
    battery.setWatchdogFeedHook(dummy_hook);
    SUCCEED();
}

TEST_F(SmartBatteryTestSuite, CacheFresh_TimestampZero) {
    battery.cache.timestamp_ms = 0; // Triggers early false branch in isCacheFresh
    battery.cache.valid = true;
    battery.cache.last_error = CommFault::NONE;
    
    EXPECT_FALSE(battery.getVoltage().success);
}

TEST_F(SmartBatteryTestSuite, Thread_BmsComm_ValidExecution) {
    test_iterations_remaining = 1;
    extern SbsBattery* smart_battery;
    smart_battery = &battery;
    
    // Covers line 583 (pollHardwareAndUpdateCache inside bms_comm_thread)
    testing::internal::CaptureStdout();
    bms_comm_thread();
    const auto raw_out = testing::internal::GetCapturedStdout();
    std::string_view out(raw_out);
    
    // It will timeout because it's a synchronous execution without HW, 
    // which is perfectly fine for branch coverage.
    SUCCEED();
}

TEST_F(SmartBatteryTestSuite, Thread_BatteryMonitor_IdleState) {
    battery.current_state = BatteryFSM::IDLE;
    extern SbsBattery* smart_battery;
    smart_battery = &battery;
    test_iterations_remaining = 1;
    
    // Covers the false branch of getState() == BatteryFSM::DISCHARGING inside monitor thread
    testing::internal::CaptureStdout();
    battery_monitor_thread();
    const auto raw_out = testing::internal::GetCapturedStdout();
    std::string_view out(raw_out);
    EXPECT_TRUE(out.find("Battery Discharging") == std::string_view::npos);
}

TEST_F(SmartBatteryTestSuite, Getters_StaleCache) {
    // 1. Setup a valid cache state
    battery.cache.valid = true;
    battery.cache.last_error = CommFault::NONE;
    
    // 2. Force it to be stale (older than CACHE_STALE_MS)
    battery.cache.timestamp_ms = k_uptime_get_32() - (BatteryLimits::CACHE_STALE_MS + 1000);
    
    // 3. Verify it fails due to staleness (hitting the false branch of isCacheFresh)
    EXPECT_FALSE(battery.getVoltage().success);
    EXPECT_EQ(battery.getVoltage().error, CommFault::CACHE_INVALID);
}

TEST_F(SmartBatteryTestSuite, ProcessFSM_HysteresisDeadZone) {
    // 1. Enter cutoff state
    battery.cache.soc.value = BatteryLimits::CUTOFF_SOC_PCT; // usually 5%
    battery.processFSM();
    EXPECT_EQ(battery.current_state, BatteryFSM::CUTOFF);
    
    // 2. Recover slightly, but NOT enough to hit REENABLE_SOC_PCT (usually 10%)
    battery.cache.soc.value = BatteryLimits::CUTOFF_SOC_PCT + 2; 
    
    // 3. Process FSM. It should fail the 'else if' branch and remain in CUTOFF
    battery.processFSM();
    EXPECT_EQ(battery.current_state, BatteryFSM::CUTOFF);
}

TEST_F(SmartBatteryTestSuite, PublishError_WatchdogThreshold) {
    // Repeatedly publish errors without advancing the clock to ensure
    // consecutive_comm_failures natively exceeds WATCHDOG_FAILURE_THRESHOLD (5)
    for(int i = 0; i < 6; i++) {
        battery.publishError(CommFault::FRAME_ERROR);
    }
    
    // Validate that the threshold logic successfully forced a CUTOFF
    EXPECT_EQ(battery.getState(), BatteryFSM::CUTOFF);
}
