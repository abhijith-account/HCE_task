// test_battery_fsm.cpp
#include <gtest/gtest.h>
#include <array>
#include <cstring>
#include <string_view>
#include <cstdint>

#include "Fault_Tolerant_I2C_Communication_Layer.h"
#include "Power_Management_System.h"

// White-box access
#define private public
#define protected public
#include "Smart_Battery_System.h"
#undef private
#undef protected

// Global Test Hooks
extern int g_i2c_force_errno;
extern int g_i2c_call_counter;
extern int g_i2c_fail_on_call_n;
extern int g_i2c_fail_on_call_errno;
extern uint32_t virtual_uptime;
extern DeviceContext sys_context;
extern int g_adc_sequence_init_dt_errno;
extern int test_iterations_remaining;
extern int g_mutex_lock_force_errno;
extern bool g_adc_ready_mock;
extern int g_adc_read_force_errno;

uint16_t g_i2c_mock_read_val = 0;
uint16_t g_i2c_mock_v_val = 10000;
uint16_t g_i2c_mock_i_val = 0;
int32_t g_adc_mock_mv_val = 2500;
int g_adc_raw_to_mv_errno = 0;
int g_i2c_consecutive_failures = 0;
int g_i2c_fail_after_reads = -1; 

extern "C" {
    // Strongly override Zephyr I2C and ADC mocks to inject dynamic payloads/faults
    int i2c_burst_read(const struct device *dev, uint16_t dev_addr, uint8_t start_addr, uint8_t *buf, uint32_t num_bytes) {
        ++g_i2c_call_counter;
        if (g_i2c_force_errno != 0) return g_i2c_force_errno;
        if (g_i2c_fail_on_call_n != 0 && g_i2c_call_counter == g_i2c_fail_on_call_n) return g_i2c_fail_on_call_errno;
        if (g_i2c_consecutive_failures > 0) {
            g_i2c_consecutive_failures--;
            return -EIO;
        }
        if (g_i2c_fail_after_reads >= 0 && g_i2c_call_counter > g_i2c_fail_after_reads) return -EIO;
        
        if (buf && num_bytes >= 2) {
            uint16_t val = g_i2c_mock_read_val;
            
            if (start_addr == 0x02) {
                val = g_i2c_mock_v_val; 
            } else if (start_addr == 0x04) {
                val = g_i2c_mock_i_val; 
            }
            
            // Revert back to proper I2C Big-Endian alignment now that the driver fixes it
            buf[0] = (val >> 8) & 0xFF;
            buf[1] = val & 0xFF;
        } else if (buf && num_bytes > 0) {
            memset(buf, 0, num_bytes);
        }
        return 0;
    }

    int i2c_write_read(const struct device *dev, uint16_t addr, const void *write_buf, size_t num_write, void *read_buf, size_t num_read) {
        ++g_i2c_call_counter;
        if (g_i2c_force_errno != 0) return g_i2c_force_errno;
        if (g_i2c_fail_on_call_n != 0 && g_i2c_call_counter == g_i2c_fail_on_call_n) return g_i2c_fail_on_call_errno;
        if (g_i2c_consecutive_failures > 0) {
            g_i2c_consecutive_failures--;
            return -EIO;
        }
        if (g_i2c_fail_after_reads >= 0 && g_i2c_call_counter > g_i2c_fail_after_reads) return -EIO;
        
        if (read_buf && num_read >= 2) {
            uint8_t* b = static_cast<uint8_t*>(read_buf);
            uint16_t val = g_i2c_mock_read_val;
            
            if (write_buf && num_write >= 1) {
                uint8_t reg = static_cast<const uint8_t*>(write_buf)[0];
                if (reg == 0x02) val = g_i2c_mock_v_val; // REG_BUS_VOLT
                else if (reg == 0x04) val = g_i2c_mock_i_val; // REG_CURRENT
            }
            
            // Revert back to proper I2C Big-Endian alignment
            b[0] = (val >> 8) & 0xFF;
            b[1] = val & 0xFF;
        } else if (read_buf && num_read > 0) {
            memset(read_buf, 0, num_read);
        }
        return 0;
    }

    int i2c_write(const struct device *dev, const uint8_t *buf, uint32_t num_bytes, uint16_t addr) {
        ++g_i2c_call_counter;
        if (g_i2c_force_errno != 0) return g_i2c_force_errno;
        if (g_i2c_fail_on_call_n != 0 && g_i2c_call_counter == g_i2c_fail_on_call_n) return g_i2c_fail_on_call_errno;
        if (g_i2c_fail_after_reads >= 0 && g_i2c_call_counter > g_i2c_fail_after_reads) return -EIO;
        return 0;
    }

    int adc_raw_to_millivolts_dt(const struct adc_dt_spec *spec, int32_t *val) {
        if (g_adc_raw_to_mv_errno != 0) return g_adc_raw_to_mv_errno;
        if (val) *val = g_adc_mock_mv_val;
        return 0;
    }
}

static const device dummy_i2c_dev;
I2CManager i2c_manager(&dummy_i2c_dev);

static bool custom_hook_called = false;
static void custom_watchdog_hook(void) { custom_hook_called = true; }

class SmartBatteryTestSuite : public ::testing::Test {
protected:
    SbsBattery battery{&i2c_manager, &sys_context};

    void SetUp() override {
        sys_context = DeviceContext(); 
        sys_context.requestTransition(SystemState::RUNNING);
        
        virtual_uptime = 10000;
        g_i2c_force_errno = 0;
        g_i2c_call_counter = 0;
        g_i2c_fail_on_call_n = 0;
        g_i2c_fail_after_reads = -1;
        g_i2c_mock_read_val = 0;
        g_i2c_mock_v_val = 10000;
        g_i2c_mock_i_val = 0;
        g_i2c_consecutive_failures = 0;
        g_adc_ready_mock = true;
        g_adc_mock_mv_val = 2500;
        g_adc_raw_to_mv_errno = 0;
        g_mutex_lock_force_errno = 0;
        custom_hook_called = false;
        g_adc_sequence_init_dt_errno = 0; 
        
        battery.cache = BmsCache{};
        battery.cache.valid = true;
        battery.cache.last_error = CommFault::NONE;
        battery.cache.timestamp_ms = k_uptime_get_32();
        battery.cache.temperature.value = Thermistor::KELVIN_OFFSET_TENTHS + 13; 
        
        battery.current_state = BatteryFSM::IDLE;
        battery.consecutive_comm_failures = 0;
        battery.consecutive_mutex_failures = 0;
        battery.last_valid_comm_time = k_uptime_get_32();
        battery.soc_initialized = false;
        battery.accumulated_uAh = 0; 
        battery.consecutive_jump_rejects = 0;
        battery.rest_period_start_ms = 0;
    }
};

// ==============================================================================
// 1. Core Drivers & Utility Coverages
// ==============================================================================

TEST_F(SmartBatteryTestSuite, CurveFitting_LUT_Edges) {
    EXPECT_EQ(battery.estimateSocFromVoltage(8700), 0);
    EXPECT_EQ(battery.estimateSocFromVoltage(8600), 0);
    EXPECT_EQ(battery.estimateSocFromVoltage(12600), 100);
    EXPECT_EQ(battery.estimateSocFromVoltage(13000), 100);
    EXPECT_EQ(battery.estimateSocFromVoltage(9000), 1); 
    
    g_adc_mock_mv_val = 3300; 
    EXPECT_EQ(Thermistor::readCelsius().error, Thermistor::Fault::OUT_OF_RANGE);
    g_adc_mock_mv_val = 3220; 
    EXPECT_EQ(Thermistor::readCelsius().value, -400);
    g_adc_mock_mv_val = 3250; 
    EXPECT_EQ(Thermistor::readCelsius().value, -400);
    g_adc_mock_mv_val = 114;  
    EXPECT_EQ(Thermistor::readCelsius().value, 1250);
    g_adc_mock_mv_val = 100;  
    EXPECT_EQ(Thermistor::readCelsius().value, 1250);
}

TEST_F(SmartBatteryTestSuite, Thermistor_InitAndReadFailures) {
    g_adc_ready_mock = false;
    EXPECT_FALSE(Thermistor::init());
    EXPECT_EQ(Thermistor::readCelsius().error, Thermistor::Fault::ADC_NOT_READY);
    
    g_adc_ready_mock = true;
    g_adc_raw_to_mv_errno = -EIO;
    EXPECT_EQ(Thermistor::readCelsius().error, Thermistor::Fault::ADC_READ_ERROR);
    
    g_adc_raw_to_mv_errno = 0;
    g_adc_mock_mv_val = 0;
    EXPECT_EQ(Thermistor::readCelsius().error, Thermistor::Fault::OUT_OF_RANGE);
}

TEST_F(SmartBatteryTestSuite, INA226_InitFailures) {
    INA226::Driver drv(nullptr);
    EXPECT_FALSE(drv.init());

    INA226::Driver drv2(&i2c_manager);
    g_i2c_call_counter = 0;
    g_i2c_fail_on_call_n = 2;
    g_i2c_fail_on_call_errno = -EIO;
    EXPECT_FALSE(drv2.init());
}

// ==============================================================================
// 2. Hardware Polling & Retries
// ==============================================================================
TEST_F(SmartBatteryTestSuite, FetchWithRetry_ExactRetryCount) {
    g_i2c_consecutive_failures = 3; 
    auto res = battery.fetchBusVoltageRawWithRetry();
    EXPECT_TRUE(res.success);
    EXPECT_EQ(battery.getStats().retries, 3);
}

TEST_F(SmartBatteryTestSuite, FetchCurrent_RetryAndFail) {
    g_i2c_force_errno = -EIO;
    auto res = battery.fetchCurrentRawWithRetry();
    EXPECT_FALSE(res.success);
    EXPECT_EQ(res.error, CommFault::I2C_NACK);
    g_i2c_force_errno = 0;
}

TEST_F(SmartBatteryTestSuite, PollHardware_ValidationRejects) {
    // Voltage Reject
    g_i2c_mock_v_val = 30000; 
    battery.pollHardwareAndUpdateCache();
    EXPECT_EQ(battery.cache.last_error, CommFault::VALIDATION_ERROR);
    
    // Current Reject
    battery.cache.last_error = CommFault::NONE;
    battery.cache.valid = true; // <-- Add this to cleanly isolate the fault
    g_i2c_mock_v_val = 10000; 
    g_i2c_mock_i_val = 32000; 
    battery.pollHardwareAndUpdateCache(); 
    EXPECT_EQ(battery.cache.last_error, CommFault::VALIDATION_ERROR);
}

TEST_F(SmartBatteryTestSuite, JumpReject_ThresholdReached) {
    // Independent branch (jump logic triggered but not escalating)
    battery.cache.voltage.value = 0;
    battery.cache.current.value = 0;
    g_i2c_mock_v_val = 5000;
    g_i2c_mock_i_val = 0;
    battery.pollHardwareAndUpdateCache();
    EXPECT_EQ(battery.consecutive_jump_rejects, 1);
    
    // Consecutive limits trigger fault
    battery.consecutive_jump_rejects = 0;
    battery.cache.valid = true;
    battery.cache.last_error = CommFault::NONE;
    battery.cache.voltage.value = 5000;
    g_i2c_mock_v_val = 0; 
    
    for(int i = 0; i < 10; ++i) { // Exceed MAX_CONSECUTIVE_JUMP_REJECTS limit
        battery.pollHardwareAndUpdateCache();
    }
    EXPECT_EQ(battery.cache.last_error, CommFault::VALIDATION_ERROR);
}

TEST_F(SmartBatteryTestSuite, JumpDetection_NoJump_CoversRemainingLines)
{
    battery.cache.valid = true;
    battery.cache.last_error = CommFault::NONE;

    battery.cache.voltage.value = 10000;
    battery.cache.current.value = 0;
    battery.cache.temperature.value = Thermistor::KELVIN_OFFSET_TENTHS + 250;

    // Small changes (below thresholds)
    g_i2c_mock_v_val = 8000;          // -> pack_mv = 10000
    g_i2c_mock_i_val = 0;
    g_adc_mock_mv_val = 1650;         // ~25°C

    battery.pollHardwareAndUpdateCache();

    EXPECT_EQ(battery.consecutive_jump_rejects, 0);
}

TEST_F(SmartBatteryTestSuite, JumpDetection_CurrentOnly)
{
    battery.cache.valid = true;
    battery.cache.last_error = CommFault::NONE;

    battery.cache.voltage.value = 10000;
    battery.cache.current.value = -8001;
    battery.cache.temperature.value = Thermistor::KELVIN_OFFSET_TENTHS + 250;

    g_i2c_mock_v_val = 8000;
    g_i2c_mock_i_val = 0;     // huge current jump
    g_adc_mock_mv_val = 1650;

    battery.pollHardwareAndUpdateCache();

    EXPECT_GT(battery.consecutive_jump_rejects,0);
}

TEST_F(SmartBatteryTestSuite, JumpDetection_TemperatureOnly)
{
    battery.cache.valid = true;
    battery.cache.last_error = CommFault::NONE;

    battery.cache.voltage.value = 10000;
    battery.cache.current.value = 0;
    battery.cache.temperature.value = Thermistor::KELVIN_OFFSET_TENTHS + 250;

    g_i2c_mock_v_val = 8000;
    g_i2c_mock_i_val = 0;
    g_adc_mock_mv_val = 114;      // 125°C

    battery.pollHardwareAndUpdateCache();

    EXPECT_GT(battery.consecutive_jump_rejects,0);
}
// ==============================================================================
// 3. Coulomb Counter Dynamics
// ==============================================================================
TEST_F(SmartBatteryTestSuite, CoulombCounter_AtRestDuration) {
    battery.seedOrResyncCoulombCounter(11000, 10, false);
    EXPECT_EQ(battery.rest_period_start_ms, virtual_uptime);
    
    battery.seedOrResyncCoulombCounter(11000, 500, false);
    EXPECT_EQ(battery.rest_period_start_ms, 0);

    battery.soc_initialized = false;
    battery.seedOrResyncCoulombCounter(BatteryLimits::PACK_MAX_VOLTAGE_MV, 10, false);
    EXPECT_TRUE(battery.soc_initialized);

    battery.soc_initialized = false;
    battery.seedOrResyncCoulombCounter(BatteryLimits::PACK_MIN_VOLTAGE_MV, 10, false);
    EXPECT_TRUE(battery.soc_initialized);
}

TEST_F(SmartBatteryTestSuite, UpdateStateAndPublish_ClampLimits) {
    battery.soc_initialized = true;
    
    battery.accumulated_uAh = 50000 * 1000LL; 
    battery.updateStateAndPublish(11000, 100, 250);
    EXPECT_EQ(battery.cache.soc.value, 100);

    battery.accumulated_uAh = -50000;
    battery.updateStateAndPublish(11000, 100, 250);
    EXPECT_EQ(battery.cache.soc.value, 0);
}

// ==============================================================================
// 4. Faults & Notifications
// ==============================================================================
TEST_F(SmartBatteryTestSuite, NotifySystemWakeup_Edges) {
    battery.cache.timestamp_ms = 0;
    battery.notifySystemWakeup();
    EXPECT_EQ(battery.cache.timestamp_ms, virtual_uptime);
    
    battery.cache.valid = false;
    battery.cache.timestamp_ms = 0;
    battery.notifySystemWakeup();
    EXPECT_EQ(battery.cache.timestamp_ms, 0);
    
    g_mutex_lock_force_errno = -EAGAIN;
    testing::internal::CaptureStdout();
    battery.notifySystemWakeup();
    const auto out = testing::internal::GetCapturedStdout();
    EXPECT_NE(out.find("Could not lock cache_mutex"), std::string::npos);
}

TEST_F(SmartBatteryTestSuite, PublishError_NullContext) {
    DeviceContext* backup = battery.sys_context;
    battery.sys_context = nullptr;
    battery.current_state.store(BatteryFSM::IDLE);
    battery.consecutive_comm_failures = 0; 
    battery.last_valid_comm_time = 0;
    virtual_uptime = 999999; 
    
    battery.publishError(CommFault::I2C_TIMEOUT);
    EXPECT_EQ(battery.getState(), BatteryFSM::CUTOFF);
    battery.sys_context = backup;
}

TEST_F(SmartBatteryTestSuite, ProcessFSM_AllBranches) {
    battery.cache.soc.value = 50;

    battery.cache.current.value = 0;
    battery.current_state.store(BatteryFSM::CHARGING);
    battery.processFSM();
    EXPECT_EQ(battery.getState(), BatteryFSM::IDLE);
    
    battery.full_charge_logged = true;
    battery.cache.soc.value = 90;
    battery.processFSM();
    EXPECT_FALSE(battery.full_charge_logged);
    
    DeviceContext* backup = battery.sys_context;
    battery.sys_context = nullptr;
    battery.current_state.store(BatteryFSM::CUTOFF);
    battery.cache.soc.value = 90;
    battery.processFSM(); 
    battery.sys_context = backup;
}

// ==============================================================================
// 5. Threads and Singletons
// ==============================================================================
extern void bms_comm_thread(void);
extern void battery_monitor_thread(void);

TEST_F(SmartBatteryTestSuite, Threads_NullGuardsAndSkips) {
    extern SbsBattery* smart_battery;
    auto backup = smart_battery;
    
    smart_battery = nullptr;
    test_iterations_remaining = 1;
    battery_monitor_thread(); 
    
    smart_battery = backup;
    
    test_iterations_remaining = 1;
    PowerManager::getInstance().notifyBeforeSleep();
    bms_comm_thread(); 
    PowerManager::getInstance().notifyAfterWakeup();

    // Trigger Sleep Aborted
#if defined(IPOWER_OBSERVER_HAS_SLEEP_ABORTED) || 1
    // Simulate observer sleep aborted via standard interfaces
    PowerManager::getInstance().notifySleepAborted();
#endif
}

TEST_F(SmartBatteryTestSuite, I2CFaultMapping_AllBranches) {
    auto test_map = [this](int errno_val) {
        g_i2c_force_errno = errno_val;
        auto res = battery.fetchBusVoltageRawWithRetry();
        g_i2c_force_errno = 0;
        return res.error;
    };
    
    EXPECT_EQ(test_map(0), CommFault::NONE);
    EXPECT_EQ(test_map(-ENODEV), CommFault::DEVICE_NOT_READY);
    EXPECT_EQ(test_map(-ETIMEDOUT), CommFault::I2C_TIMEOUT);
    EXPECT_EQ(test_map(-EBUSY), CommFault::I2C_BUS_BUSY);
    EXPECT_EQ(test_map(-EAGAIN), CommFault::I2C_ARBITRATION_LOST);
    EXPECT_EQ(test_map(-EIO), CommFault::I2C_NACK); 
    EXPECT_EQ(test_map(-EPERM), CommFault::I2C_NACK);
}

// ==============================================================================
// 6. PHASE 2 - DEEP COVERAGE (NEW TESTS TO REACH 100%)
// ==============================================================================

TEST_F(SmartBatteryTestSuite, Getters_ValidAndInvalidCache) {
    battery.cache.valid = true;
    battery.cache.last_error = CommFault::NONE;
    battery.cache.timestamp_ms = virtual_uptime; 
    
    EXPECT_TRUE(battery.getVoltage().success);
    EXPECT_TRUE(battery.getCurrent().success);
    EXPECT_TRUE(battery.getStateOfCharge().success);
    EXPECT_TRUE(battery.getTemperature().success);
    EXPECT_TRUE(battery.getCapacity().success);
    
    virtual_uptime += 5000;
    EXPECT_FALSE(battery.getVoltage().success);
    EXPECT_EQ(battery.getVoltage().error, CommFault::CACHE_INVALID);
    
    virtual_uptime = battery.cache.timestamp_ms; 
    battery.cache.last_error = CommFault::I2C_NACK;
    EXPECT_FALSE(battery.getCurrent().success);
    EXPECT_FALSE(battery.getStateOfCharge().success);
    EXPECT_FALSE(battery.getTemperature().success);
    EXPECT_FALSE(battery.getCapacity().success);
}

TEST_F(SmartBatteryTestSuite, HardwarePoll_SequentialFaults) {
    g_i2c_consecutive_failures = 5;
    battery.pollHardwareAndUpdateCache(); 

    g_i2c_consecutive_failures = 0;
    g_i2c_call_counter = 0;
    g_i2c_fail_after_reads = 1; 
    battery.pollHardwareAndUpdateCache(); 
    g_i2c_fail_after_reads = -1; 

    g_adc_raw_to_mv_errno = -EIO;
    battery.pollHardwareAndUpdateCache(); 
    g_adc_raw_to_mv_errno = 0;
}

TEST_F(SmartBatteryTestSuite, MutexContention_AllPaths) {
    g_mutex_lock_force_errno = -EAGAIN;
    
    battery.updateStateAndPublish(10000, 100, 250);
    
    BmsCache c = battery.getCacheSnapshot();
    EXPECT_FALSE(c.valid);
    EXPECT_EQ(c.last_error, CommFault::MUTEX_TIMEOUT);
    
    battery.publishError(CommFault::I2C_NACK);
    
    for (int i = 0; i < 5; ++i) battery.publishError(CommFault::MUTEX_TIMEOUT);
    EXPECT_EQ(battery.getState(), BatteryFSM::CUTOFF);
    
    g_mutex_lock_force_errno = 0;
}

TEST_F(SmartBatteryTestSuite, CoulombCounter_LongRestResync) {
    const uint32_t now = virtual_uptime;
    battery.seedOrResyncCoulombCounter(11000, 0, false);
    EXPECT_EQ(battery.rest_period_start_ms, now);
    
    virtual_uptime += (31 * 60 * 1000);
    
    battery.soc_initialized = false;
    battery.seedOrResyncCoulombCounter(11000, 0, false);
    EXPECT_TRUE(battery.soc_initialized); 
}

TEST_F(SmartBatteryTestSuite, FSM_DeepCoverage) {
    battery.cache.valid = false;
    battery.processFSM(); 
    
    // Complete charge logged event (transitions from false -> true)
    battery.cache.valid = true;
    battery.cache.soc.value = 100;
    battery.full_charge_logged.store(false);
    battery.processFSM();
    EXPECT_TRUE(battery.full_charge_logged);
    
    // Complete charge logged event (transitions from true -> true, compare_exchange fails)
    battery.processFSM(); 
    EXPECT_TRUE(battery.full_charge_logged);

    // State changes logic checks
    battery.cache.soc.value = 50;
    
    // Check CHARGING positive branch
    battery.cache.current.value = 100;
    battery.current_state.store(BatteryFSM::IDLE);
    battery.processFSM();
    EXPECT_EQ(battery.getState(), BatteryFSM::CHARGING);
    
    // Check IDLE branch
    battery.cache.current.value = 0;
    battery.current_state.store(BatteryFSM::CHARGING);
    battery.processFSM();
    EXPECT_EQ(battery.getState(), BatteryFSM::IDLE);

    // Check DISCHARGING negative branch
    battery.cache.current.value = -100;
    battery.current_state.store(BatteryFSM::IDLE);
    battery.processFSM();
    EXPECT_EQ(battery.getState(), BatteryFSM::DISCHARGING);

    // Critical low discharge triggers fault & CUTOFF transition
    battery.cache.soc.value = BatteryLimits::CUTOFF_SOC_PCT;
    battery.current_state.store(BatteryFSM::DISCHARGING);
    battery.processFSM();
    EXPECT_EQ(battery.getState(), BatteryFSM::CUTOFF);
    
    // Critical low discharge WHEN ALREADY IN CUTOFF (should skip fault trigger block)
    battery.cache.soc.value = BatteryLimits::CUTOFF_SOC_PCT - 1;
    battery.processFSM(); 
    
    // Recovery transition requested safely
    battery.cache.soc.value = BatteryLimits::REENABLE_SOC_PCT;
    sys_context.requestTransition(SystemState::SAFE_HALT); 
    battery.processFSM(); 
    
    // Recovery threshold reached but NOT in SAFE_HALT (should skip transition block)
    battery.current_state.store(BatteryFSM::CUTOFF);
    battery.cache.soc.value = BatteryLimits::REENABLE_SOC_PCT;
    sys_context.requestTransition(SystemState::RUNNING); 
    battery.processFSM(); 
    
    battery.cache.current.value = 0;
    battery.cache.soc.value = BatteryLimits::REENABLE_SOC_PCT;
    sys_context.requestTransition(SystemState::SAFE_HALT);
    battery.current_state.store(BatteryFSM::CUTOFF);
    battery.processFSM();
    EXPECT_EQ(battery.getState(), BatteryFSM::IDLE);
}

TEST_F(SmartBatteryTestSuite, BmsCommThread_InitRetryLog) {
    extern SbsBattery* smart_battery;
    auto backup = smart_battery;
    smart_battery = &battery;

    // Force init to fail continuously until it hits MAX_INIT_RETRIES to log the escalation
    g_i2c_force_errno = -EIO; 
    test_iterations_remaining = 0; 
    bms_comm_thread(); 
    g_i2c_force_errno = 0; 

    smart_battery = backup;
}

TEST_F(SmartBatteryTestSuite, ThreadExecution_And_Singletons) {
    extern SbsBattery* smart_battery;
    auto backup = smart_battery;
    smart_battery = &battery;
    
    EXPECT_EQ(getSmartBatteryInstance(), &battery);
    
    // Normal execution path
    test_iterations_remaining = 1;
    bms_comm_thread(); 
    
    PowerManager::getInstance().notifyBeforeSleep();
    PowerManager::getInstance().notifyAfterWakeup();
    
    // Test battery monitor thread executing DISCHARGING output log
    battery.cache.current.value = -500; // Must be discharging for processFSM to shift state
    battery.cache.soc.value = 60;
    battery.cache.timestamp_ms = virtual_uptime; 
    battery.cache.valid = true;
    test_iterations_remaining = 1;
    battery_monitor_thread(); 
    
    // Test monitor thread when cache goes invalid mid-discharge 
    battery.current_state.store(BatteryFSM::DISCHARGING);
    battery.cache.valid = false;
    test_iterations_remaining = 1;
    battery_monitor_thread(); 
    
    smart_battery = backup;
}

TEST_F(SmartBatteryTestSuite, Init_WatchdogHooks) {
    battery.setWatchdogFeedHook([]() { custom_hook_called = true; });
    battery.feedWatchdog();
    EXPECT_TRUE(custom_hook_called);
    
    battery.setWatchdogFeedHook(nullptr);
}

TEST_F(SmartBatteryTestSuite, Init_INA_Fail) {
    g_i2c_fail_on_call_n = 1;
    g_i2c_fail_on_call_errno = -EIO;
    EXPECT_FALSE(battery.init());
    g_i2c_fail_on_call_n = 0;
}

TEST_F(SmartBatteryTestSuite, Init_Thermistor_Fail) {
    g_i2c_fail_on_call_n = 0;
    g_i2c_force_errno = 0;
    g_adc_ready_mock = false;
    EXPECT_FALSE(battery.init());
}

// ==============================================================================
// 7. PHASE 3 - FINAL COVERAGE (100% LINE/BRANCH)
// ==============================================================================

TEST_F(SmartBatteryTestSuite, CoulombCounter_AtRestBranchCombos) {
    // at_rest = (current_ma > -THRESH) && (current_ma < THRESH). No existing test hits
    // the "first term false" short-circuit (large negative current).
    battery.rest_period_start_ms = 12345;
    battery.seedOrResyncCoulombCounter(11000, -500, false);
    EXPECT_EQ(battery.rest_period_start_ms, 0U);

    // full_charge/full_discharge: voltage condition true, at_rest false (never combined before)
    battery.soc_initialized = false;
    battery.seedOrResyncCoulombCounter(BatteryLimits::PACK_MAX_VOLTAGE_MV, 500, false);
    EXPECT_FALSE(battery.soc_initialized);

    battery.soc_initialized = false;
    battery.seedOrResyncCoulombCounter(BatteryLimits::PACK_MIN_VOLTAGE_MV, -500, false);
    EXPECT_FALSE(battery.soc_initialized);
}

TEST_F(SmartBatteryTestSuite, UpdateStateAndPublish_FirstSeedBranch) {
    // pollHardwareAndUpdateCache() never reaches updateStateAndPublish() with
    // soc_initialized==false, because SetUp's zeroed baseline cache always trips the
    // jump-reject guard first. Drive it directly instead.
    battery.soc_initialized = false;
    battery.accumulated_uAh = 0;
    battery.last_poll_time_ms = 0;

    battery.updateStateAndPublish(11000, 10, 250);

    EXPECT_TRUE(battery.soc_initialized);
    EXPECT_EQ(battery.last_poll_time_ms, virtual_uptime);
}

TEST_F(SmartBatteryTestSuite, UpdateStateAndPublish_NoClampNeeded) {
    // std::clamp's "already in range" branch -- existing test only hits the two clamped ends.
    battery.soc_initialized = true;
    battery.accumulated_uAh = 1000 * 1000LL; // 1000mAh of 2000mAh, well inside range
    battery.updateStateAndPublish(11000, 0, 250);
    EXPECT_GT(battery.cache.soc.value, 0);
    EXPECT_LT(battery.cache.soc.value, 100);
}

TEST_F(SmartBatteryTestSuite, WatchdogHook_ConstructorAndNullHook) {
    // Constructor's (hook != nullptr) ? hook : daly_watchdog_feed_hook -- non-null path.
    custom_hook_called = false;
    SbsBattery hooked_battery(&i2c_manager, &sys_context, &custom_watchdog_hook);
    hooked_battery.feedWatchdog();
    EXPECT_TRUE(custom_hook_called);

    // feedWatchdog()'s "hook is null" branch. The public setter can never store a literal
    // nullptr (it always falls back to daly_watchdog_feed_hook), so reach in directly.
    battery.watchdog_feed_hook.store(nullptr);
    custom_hook_called = false;
    battery.feedWatchdog();
    EXPECT_FALSE(custom_hook_called);
}

TEST_F(SmartBatteryTestSuite, PollHardware_TotalI2CFailure_NoCacheFallback) {
    // I2CManager's own last-known-good register cache can mask a total bus failure from
    // SbsBattery's retry logic. Clear it first so the failure genuinely reaches publishError.
    extern void resetI2CCacheForTests();
    resetI2CCacheForTests();

    g_i2c_force_errno = -EIO;
    battery.pollHardwareAndUpdateCache();
    EXPECT_EQ(battery.cache.last_error, CommFault::I2C_NACK);
    g_i2c_force_errno = 0;
}

TEST_F(SmartBatteryTestSuite, PollHardware_CurrentFetchTotalFailure) {
    extern void resetI2CCacheForTests();
    resetI2CCacheForTests();

    // Let the voltage read through once, then fail everything after -- isolates the
    // current-fetch failure path inside pollHardwareAndUpdateCache().
    g_i2c_fail_after_reads = 1;
    battery.pollHardwareAndUpdateCache();
    g_i2c_fail_after_reads = -1;
    EXPECT_EQ(battery.cache.last_error, CommFault::I2C_NACK);
}

TEST_F(SmartBatteryTestSuite, CacheFreshness_ZeroTimestampShortCircuit) {
    // isCacheFresh()'s (timestamp_ms != 0) short-circuit -- SetUp always seeds a nonzero one.
    battery.cache.valid = true;
    battery.cache.last_error = CommFault::NONE;
    battery.cache.timestamp_ms = 0;
    EXPECT_FALSE(battery.getVoltage().success);
    EXPECT_EQ(battery.getVoltage().error, CommFault::CACHE_INVALID);
}

TEST_F(SmartBatteryTestSuite, PublishError_MutexFaultWithSuccessfulLock) {
    // is_mutex_fault==true with a *successful* internal lock. Every existing
    // MUTEX_TIMEOUT test also forces the lock itself to fail, masking this combo.
    g_mutex_lock_force_errno = 0;
    battery.consecutive_mutex_failures = 0;
    battery.current_state.store(BatteryFSM::IDLE);
    battery.publishError(CommFault::MUTEX_TIMEOUT);
    EXPECT_EQ(battery.consecutive_mutex_failures, 1U);
}

TEST_F(SmartBatteryTestSuite, PowerObserver_SleepAborted) {
    extern SbsBattery* smart_battery;
    auto backup = smart_battery;
    smart_battery = &battery;

    // Registers BmsPowerObserver as a side effect of a successful init.
    test_iterations_remaining = 1;
    bms_comm_thread();

    // Unreachable elsewhere: Threads_NullGuardsAndSkips triggers this with
    // smart_battery==nullptr, before the observer is ever registered.
    PowerManager::getInstance().notifySleepAborted();

    smart_battery = backup;
}

TEST_F(SmartBatteryTestSuite, CurveFitting_InteriorSegments) {
    // Walk every LUT segment so the linear-scan loops take both the
    // "keep scanning" and "match found" branches repeatedly, not just once.
    static constexpr uint16_t ocv_probe_mv[] = {
        9100, 9900, 10500, 10950, 11250, 11550, 11850, 12150, 12450
    };
    for (uint16_t mv : ocv_probe_mv) {
        (void)battery.estimateSocFromVoltage(mv);
    }

    static constexpr int32_t ntc_probe_mv[] = {
        3200, 3160, 3110, 3050, 2970, 2870, 2750, 2610, 2460, 2290,
        2110, 1930, 1740, 1560, 1385, 1220, 1070,  935,  810,  700,
         605,  520,  450,  390,  340,  300,  260,  230,  200,  175,
         155,  135,  120
    };
    for (int32_t mv : ntc_probe_mv) {
        g_adc_mock_mv_val = mv;
        EXPECT_TRUE(Thermistor::readCelsius().success);
    }
}

// ==============================================================================
// 8. PHASE 4 - THREAD SLEEP/WAKE BRANCHES & FSM BAND EDGES
// ==============================================================================

TEST_F(SmartBatteryTestSuite, BmsCommThread_SkipsPollWhileSleeping) {
    // Line 638 (`if (!g_bmsPowerObserver.isSleeping())`) only ever sees the
    // "awake" branch elsewhere. do-while runs the body once regardless of
    // test_iterations_remaining, so 0 is enough for a single pass each call.
    extern SbsBattery* smart_battery;
    auto backup = smart_battery;
    smart_battery = &battery;

    test_iterations_remaining = 1;
    bms_comm_thread();  // registers the observer, exercises "awake"

    PowerManager::getInstance().notifyBeforeSleep();
    test_iterations_remaining = 1;
    bms_comm_thread();  // now "asleep" -> covers the skipped-poll branch
    PowerManager::getInstance().notifyAfterWakeup();

    smart_battery = backup;
}

TEST_F(SmartBatteryTestSuite, BatteryMonitorThread_SkipsProcessWhileSleeping) {
    // Same gap as above but in battery_monitor_thread's guard (line 650).
    extern SbsBattery* smart_battery;
    auto backup = smart_battery;
    smart_battery = &battery;

    PowerManager::getInstance().notifyBeforeSleep();
    test_iterations_remaining = 1;
    battery_monitor_thread();
    PowerManager::getInstance().notifyAfterWakeup();

    smart_battery = backup;
}

TEST_F(SmartBatteryTestSuite, BatteryMonitorThread_NonDischargingSkipsLog) {
    // Line 653 (`if (getState() == DISCHARGING)`) only ever sees "true" --
    // every existing thread test leaves the FSM in DISCHARGING from a prior
    // call. Drive it to IDLE first so processFSM() lands somewhere else.
    extern SbsBattery* smart_battery;
    auto backup = smart_battery;
    smart_battery = &battery;

    battery.current_state.store(BatteryFSM::IDLE);
    battery.cache.valid = true;
    battery.cache.last_error = CommFault::NONE;
    battery.cache.timestamp_ms = virtual_uptime;
    battery.cache.current.value = 0;   // keeps processFSM's result at IDLE
    battery.cache.soc.value = 50;

    test_iterations_remaining = 0;
    battery_monitor_thread();
    EXPECT_EQ(battery.getState(), BatteryFSM::IDLE);

    smart_battery = backup;
}

TEST_F(SmartBatteryTestSuite, PowerObserver_AfterWakeup_NullSmartBattery) {
    // Line 578 (`if (smart_battery != nullptr)` inside afterWakeup()) only
    // ever sees the non-null case. Register the observer via one successful
    // init, then null the global out before broadcasting wakeup.
    extern SbsBattery* smart_battery;
    auto backup = smart_battery;

    smart_battery = &battery;
    test_iterations_remaining = 0;
    bms_comm_thread();  // registers g_bmsPowerObserver

    smart_battery = nullptr;
    PowerManager::getInstance().notifyBeforeSleep();
    PowerManager::getInstance().notifyAfterWakeup();  // hits the null branch

    smart_battery = backup;
}

TEST_F(SmartBatteryTestSuite, ProcessFSM_MidBandsAndChargingRecovery) {
    battery.cache.valid = true;
    battery.cache.last_error = CommFault::NONE;
    battery.cache.timestamp_ms = virtual_uptime;

    // soc strictly between CUTOFF_SOC_PCT and REENABLE_SOC_PCT: neither the
    // cutoff guard (line 534) nor the reenable check (line 540) fires.
    battery.cache.current.value = 0;
    battery.cache.soc.value = (BatteryLimits::CUTOFF_SOC_PCT + BatteryLimits::REENABLE_SOC_PCT) / 2;
    battery.current_state.store(BatteryFSM::IDLE);
    battery.processFSM();
    EXPECT_EQ(battery.getState(), BatteryFSM::IDLE);

    // soc in [95,100): neither the full-charge-logged set (line 525) nor the
    // clear (line 530) fires -- flag must be left exactly as it was.
    battery.cache.soc.value = 97;
    battery.full_charge_logged.store(true);
    battery.processFSM();
    EXPECT_TRUE(battery.full_charge_logged);

    // Recovery ternary (line 544): every existing SAFE_HALT-recovery test
    // leaves current negative, so only the IDLE arm is exercised. Do it again
    // with a positive current to hit the CHARGING arm.
    battery.cache.current.value = 100;
    battery.cache.soc.value = BatteryLimits::REENABLE_SOC_PCT;
    battery.current_state.store(BatteryFSM::CUTOFF);
    sys_context.requestTransition(SystemState::SAFE_HALT);
    battery.processFSM();
    EXPECT_EQ(battery.getState(), BatteryFSM::CHARGING);
}

TEST_F(SmartBatteryTestSuite, JumpDetection_AllConditions)
{
    battery.cache.valid=true;
    battery.cache.last_error=CommFault::NONE;

    battery.cache.voltage.value=10000;
    battery.cache.current.value=0;
    battery.cache.temperature.value=
        Thermistor::KELVIN_OFFSET_TENTHS+250;

    // Current jump only
    g_i2c_mock_v_val=8000;
    g_i2c_mock_i_val=3000;
    g_adc_mock_mv_val=2023;

    battery.cache.voltage.value=10000;
    battery.cache.current.value=0;
    battery.cache.temperature.value=
        Thermistor::KELVIN_OFFSET_TENTHS+150;

    battery.pollHardwareAndUpdateCache();

    // Temperature jump only
    battery.consecutive_jump_rejects=0;

    g_i2c_mock_v_val=8000;
    g_i2c_mock_i_val=0;
    g_adc_mock_mv_val=114;

    battery.pollHardwareAndUpdateCache();

    // No jump
    battery.consecutive_jump_rejects=5;

    g_i2c_mock_v_val=8000;
    g_i2c_mock_i_val=0;
    g_adc_mock_mv_val=2023;

    battery.cache.voltage.value=10000;
    battery.cache.current.value=0;
    battery.cache.temperature.value=
        Thermistor::KELVIN_OFFSET_TENTHS+150;

    battery.pollHardwareAndUpdateCache();

    EXPECT_EQ(battery.consecutive_jump_rejects,0);
}

// Clears Line 128: Exercises the ADC sequence init and adc_read failure branches
TEST_F(SmartBatteryTestSuite, Thermistor_AdcReadFailures) {
    g_adc_ready_mock = true;
    
    // Test adc_sequence_init_dt failure
    g_adc_sequence_init_dt_errno = -EIO;
    EXPECT_EQ(Thermistor::readCelsius().error, Thermistor::Fault::ADC_READ_ERROR);
    g_adc_sequence_init_dt_errno = 0;

    // Test adc_read failure (using the global hook)
    g_adc_read_force_errno = -EIO;
    EXPECT_EQ(Thermistor::readCelsius().error, Thermistor::Fault::ADC_READ_ERROR);
    g_adc_read_force_errno = 0;
}

// Clears Line 396: Exercises (valid == false && last_error != VALIDATION_ERROR)
TEST_F(SmartBatteryTestSuite, PollHardware_InvalidCache_NotValidationError) {
    battery.cache.valid = false;
    battery.cache.last_error = CommFault::I2C_NACK; 
    
    g_i2c_mock_v_val = 10000;
    g_i2c_mock_i_val = 0;
    
    battery.pollHardwareAndUpdateCache();
    
    // Jump check is bypassed, and normal polling successfully recovers the cache
    EXPECT_EQ(battery.cache.last_error, CommFault::NONE);
    EXPECT_TRUE(battery.cache.valid);
}

// Clears Lines 225 & 246: Hits the inline branch inside feedWatchdog() when hook is null
TEST_F(SmartBatteryTestSuite, Fetch_NullWatchdogHook_DuringRetry) {
    battery.setWatchdogFeedHook(nullptr);
    
    // Force I2C failures so the loop repeats, hitting the null hook branch repeatedly
    g_i2c_force_errno = -EIO; 
    battery.fetchBusVoltageRawWithRetry();
    battery.fetchCurrentRawWithRetry();
    g_i2c_force_errno = 0;
}

// ==============================================================================
// 9. PHASE 5 - REMAINING BRANCH COVERAGE (thread guards / FSM null-context / cache reason)
// ==============================================================================

TEST_F(SmartBatteryTestSuite, ThreadLoops_SleepAwakeBranch_Robust) {
    // Lines 638/650 (`if (!g_bmsPowerObserver.isSleeping())`) each need both
    // arms hit in a single, order-independent test rather than relying on
    // global observer state left over from earlier tests.
    extern SbsBattery* smart_battery;
    auto backup = smart_battery;
    smart_battery = &battery;

    // Force a known "awake" starting state regardless of prior test order.
    PowerManager::getInstance().notifyAfterWakeup();

    battery.current_state.store(BatteryFSM::IDLE);
    battery.cache.valid = true;
    battery.cache.last_error = CommFault::NONE;
    battery.cache.timestamp_ms = virtual_uptime;
    battery.cache.current.value = 0;
    battery.cache.soc.value = 50;

    // Awake pass: hits the "true" arm of both thread guards.
    test_iterations_remaining = 0;
    bms_comm_thread();
    test_iterations_remaining = 0;
    battery_monitor_thread();

    // Asleep pass: hits the "false" arm of both thread guards.
    PowerManager::getInstance().notifyBeforeSleep();
    test_iterations_remaining = 0;
    bms_comm_thread();
    test_iterations_remaining = 0;
    battery_monitor_thread();
    PowerManager::getInstance().notifyAfterWakeup();

    smart_battery = backup;
}

TEST_F(SmartBatteryTestSuite, BatteryMonitorThread_DischargingSocFailureSkipsLog) {
    // Line 654 (`if (soc.success)`) only ever sees the success arm elsewhere.
    // Let processFSM run against a fresh cache (state -> DISCHARGING), then
    // age the cache past CACHE_STALE_MS before getStateOfCharge() is called,
    // so soc.success is false while getState() is still DISCHARGING.
    extern SbsBattery* smart_battery;
    auto backup = smart_battery;
    smart_battery = &battery;

    battery.current_state.store(BatteryFSM::DISCHARGING);
    battery.cache.valid = true;
    battery.cache.last_error = CommFault::NONE;
    battery.cache.current.value = -100;
    battery.cache.soc.value = 50;
    battery.cache.timestamp_ms = virtual_uptime;

    virtual_uptime += (BatteryLimits::CACHE_STALE_MS + 1000);

    test_iterations_remaining = 0;
    battery_monitor_thread();

    smart_battery = backup;
}

TEST_F(SmartBatteryTestSuite, ProcessFSM_CutoffTrigger_NullContextSkipsFaultCall) {
    // Line 537 (`if (sys_context != nullptr) sys_context->triggerFault(...)`)
    // inside the discharge-guard block only ever sees the non-null arm
    // elsewhere (PublishError_NullContext exercises a different call site).
    DeviceContext* backup_ctx = battery.sys_context;
    battery.sys_context = nullptr;

    battery.cache.valid = true;
    battery.cache.last_error = CommFault::NONE;
    battery.cache.timestamp_ms = virtual_uptime;
    battery.cache.current.value = -100;
    battery.cache.soc.value = BatteryLimits::CUTOFF_SOC_PCT;
    battery.current_state.store(BatteryFSM::DISCHARGING);

    battery.processFSM();
    EXPECT_EQ(battery.getState(), BatteryFSM::CUTOFF);

    battery.sys_context = backup_ctx;
}

TEST_F(SmartBatteryTestSuite, ProcessFSM_ReenableCheck_NullContextSkipsRecovery) {
    // Line 541 (`if (sys_context != nullptr && sys_context->getState() == SAFE_HALT)`)
    // never sees the short-circuited sys_context == nullptr arm elsewhere.
    DeviceContext* backup_ctx = battery.sys_context;
    battery.sys_context = nullptr;

    battery.cache.valid = true;
    battery.cache.last_error = CommFault::NONE;
    battery.cache.timestamp_ms = virtual_uptime;
    battery.cache.current.value = 0;
    battery.cache.soc.value = BatteryLimits::REENABLE_SOC_PCT;
    battery.current_state.store(BatteryFSM::CUTOFF);

    battery.processFSM();
    // No sys_context to consult -> must remain in CUTOFF, not fall through.
    EXPECT_EQ(battery.getState(), BatteryFSM::CUTOFF);

    battery.sys_context = backup_ctx;
}

TEST_F(SmartBatteryTestSuite, ProcessFSM_CacheFailureReason_BothBranches) {
    // Line 511's cacheFailureReason(snapshot) ternary: cover both the
    // "fault already recorded" and "last_error == NONE -> CACHE_INVALID"
    // arms specifically from processFSM's own LOG_ERR call site (getVoltage
    // etc. exercise the same helper, but from a different call site).
    testing::internal::CaptureStdout();

    battery.cache.valid = false;
    battery.cache.last_error = CommFault::I2C_NACK;
    battery.processFSM();

    battery.cache.valid = false;
    battery.cache.last_error = CommFault::NONE;
    battery.processFSM();

    testing::internal::GetCapturedStdout();
}

TEST_F(SmartBatteryTestSuite, BatteryMonitorThread_DischargingSocFailure_MutexPath) {
    // Alternate path to line 654's soc.success==false arm: force it via a
    // mutex failure on getStateOfCharge() rather than a stale cache, in case
    // the stale-cache route collapses to the same branch slot as another test.
    extern SbsBattery* smart_battery;
    auto backup = smart_battery;
    smart_battery = &battery;

    battery.current_state.store(BatteryFSM::DISCHARGING);
    battery.cache.valid = true;
    battery.cache.last_error = CommFault::NONE;
    battery.cache.current.value = -50;
    battery.cache.soc.value = 40;
    battery.cache.timestamp_ms = virtual_uptime;

    g_mutex_lock_force_errno = -EAGAIN;  // getStateOfCharge()'s internal snapshot lock fails
    test_iterations_remaining = 0;
    battery_monitor_thread();
    g_mutex_lock_force_errno = 0;

    smart_battery = backup;
}

TEST_F(SmartBatteryTestSuite, ProcessFSM_Reenable_SysContextNonNull_WrongState_Direct) {
    // Isolate the "sys_context != nullptr TRUE, getState()==SAFE_HALT FALSE"
    // sub-branch on its own, independent of any prior CUTOFF/recovery test.
    battery.cache.valid = true;
    battery.cache.last_error = CommFault::NONE;
    battery.cache.timestamp_ms = virtual_uptime;
    battery.cache.current.value = 0;
    battery.cache.soc.value = BatteryLimits::REENABLE_SOC_PCT;
    battery.current_state.store(BatteryFSM::CUTOFF);

    sys_context.requestTransition(SystemState::INIT);  // anything but SAFE_HALT
    battery.processFSM();
    EXPECT_EQ(battery.getState(), BatteryFSM::CUTOFF);  // must NOT recover
}
