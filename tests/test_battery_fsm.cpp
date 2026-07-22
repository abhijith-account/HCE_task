// test_battery_fsm.cpp (revised)
#include <gtest/gtest.h>
#include <array>
#include <cstring>
#include <string_view>
#include <cstdint>

#include "Fault_Tolerant_I2C_Communication_Layer.h"

// White-box access
#define private public
#define protected public
#include "Smart_Battery_System.h"
#undef private
#undef protected

// Use globals provided by mock_globals.cpp (do NOT redefine ADC/kernel functions)
extern int g_i2c_force_errno;
extern int g_i2c_call_counter;
extern int g_i2c_fail_on_call_n;
extern int g_i2c_fail_on_call_errno;
extern uint32_t virtual_uptime;
extern DeviceContext sys_context;
int test_iterations_remaining = 0;

// Dummy device for I2CManager (required constructor arg)
static const device dummy_i2c_dev;
I2CManager i2c_manager(&dummy_i2c_dev);

static bool custom_hook_called = false;
static void custom_watchdog_hook(void) { custom_hook_called = true; }

// ==============================================================================
// Test Suite Setup
// ==============================================================================
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
        
        custom_hook_called = false;
        
        battery.cache = BmsCache{};
        battery.cache.valid = true;
        battery.cache.last_error = CommFault::NONE;
        battery.cache.timestamp_ms = k_uptime_get_32();
        battery.current_state = BatteryFSM::IDLE;
        battery.consecutive_comm_failures = 0;
        battery.last_valid_comm_time = k_uptime_get_32();
        battery.soc_initialized = false;
        battery.accumulated_uAh = 0;        // fixed name
    }
};

// ==============================================================================
// 1. Thermistor & INA226 Driver Coverage
// ==============================================================================
TEST_F(SmartBatteryTestSuite, Thermistor_InitBranches) {
    // The mocks always return success, so we can only test the happy path.
    EXPECT_TRUE(Thermistor::init());
}

TEST_F(SmartBatteryTestSuite, Thermistor_ReadBranches) {
    auto res = Thermistor::readCelsius();
    EXPECT_TRUE(res.success);
    // mock_adc_raw_to_millivolts_dt returns 2500 mV -> ~25 °C (250 tenths)
    EXPECT_NEAR(res.value, 250, 5);
}

TEST_F(SmartBatteryTestSuite, INA226_DriverInitBranches) {
    INA226::Driver driver(nullptr);
    EXPECT_FALSE(driver.init());

    INA226::Driver valid_driver(&i2c_manager);
    g_i2c_force_errno = -EIO;
    EXPECT_FALSE(valid_driver.init());
    g_i2c_force_errno = 0;
    EXPECT_TRUE(valid_driver.init());
}

TEST_F(SmartBatteryTestSuite, INA226_ReadRegisters) {
    INA226::Driver driver(&i2c_manager);
    
    g_i2c_force_errno = -ENODEV;
    EXPECT_FALSE(driver.readBusVoltageRaw().isOk());
    EXPECT_FALSE(driver.readCurrentRaw().isOk());

    g_i2c_force_errno = 0;
    // Mock returns 0, byte swap still 0
    EXPECT_EQ(driver.readBusVoltageRaw().unwrap(), 0);
    EXPECT_EQ(driver.readCurrentRaw().unwrap(), 0);
}

// ==============================================================================
// 2. Hardware Polling & Cache Validation
// ==============================================================================
TEST_F(SmartBatteryTestSuite, HardwarePoll_VoltageFailures) {
    g_i2c_force_errno = -ETIMEDOUT;
    battery.pollHardwareAndUpdateCache();
    EXPECT_FALSE(battery.getCacheSnapshot().valid);
    EXPECT_EQ(battery.getCacheSnapshot().last_error, CommFault::I2C_TIMEOUT);
    g_i2c_force_errno = 0;
}

TEST_F(SmartBatteryTestSuite, HardwarePoll_CoulombCounterInitialization) {
    EXPECT_FALSE(battery.soc_initialized);
    battery.pollHardwareAndUpdateCache();
    EXPECT_TRUE(battery.soc_initialized);
    EXPECT_GT(battery.accumulated_uAh, 0);   // fixed name

    virtual_uptime += 3600000;
    battery.pollHardwareAndUpdateCache();
    EXPECT_TRUE(battery.soc_initialized);
}

TEST_F(SmartBatteryTestSuite, FetchWithRetry_RecoversLogic) {
    g_i2c_call_counter = 0;
    g_i2c_fail_on_call_n = 3;   // first two calls fail, third succeeds
    g_i2c_fail_on_call_errno = -EIO;
    g_i2c_force_errno = 0;

    auto res = battery.fetchBusVoltageRawWithRetry();
    EXPECT_TRUE(res.success);
    EXPECT_EQ(battery.getStats().retries, 2);
    g_i2c_fail_on_call_n = 0;
}

// ==============================================================================
// 3. System FSM & Fault Recovery Logic
// ==============================================================================
TEST_F(SmartBatteryTestSuite, ProcessFSM_Transitions) {
    battery.cache.current.value = 500;
    battery.cache.soc.value = 50;
    battery.processFSM();
    EXPECT_EQ(battery.getState(), BatteryFSM::CHARGING);
    
    battery.cache.current.value = -250;
    battery.processFSM();
    EXPECT_EQ(battery.getState(), BatteryFSM::DISCHARGING);

    battery.cache.current.value = 0;
    battery.processFSM();
    EXPECT_EQ(battery.getState(), BatteryFSM::IDLE);
}

TEST_F(SmartBatteryTestSuite, ProcessFSM_FullChargeLogging) {
    battery.cache.current.value = 500;
    battery.cache.soc.value = 100;
    
    testing::internal::CaptureStdout();
    battery.processFSM();
    const auto out = testing::internal::GetCapturedStdout();
    const std::string_view view{out};
    EXPECT_NE(view.find("BATTERY FULLY CHARGED"), std::string_view::npos);
    EXPECT_TRUE(battery.full_charge_logged);

    testing::internal::CaptureStdout();
    battery.processFSM();
    const auto out2 = testing::internal::GetCapturedStdout();
    const std::string_view view2{out2};
    EXPECT_TRUE(view2.empty());

    battery.cache.soc.value = 94;
    battery.processFSM();
    EXPECT_FALSE(battery.full_charge_logged);
}

TEST_F(SmartBatteryTestSuite, ProcessFSM_DischargeGuard_And_Recovery) {
    battery.cache.current.value = -100;
    battery.cache.soc.value = 20;
    battery.processFSM();
    EXPECT_EQ(battery.getState(), BatteryFSM::DISCHARGING);
        
    battery.cache.soc.value = 10;
    testing::internal::CaptureStdout();
    battery.processFSM();
    const auto out_cutoff = testing::internal::GetCapturedStdout();
    const std::string_view view_cutoff{out_cutoff};
    
    EXPECT_EQ(battery.getState(), BatteryFSM::CUTOFF);
    EXPECT_EQ(sys_context.getState(), SystemState::SAFE_HALT);
    EXPECT_NE(view_cutoff.find("DISCHARGE GUARD TRIGGERED"), std::string_view::npos);
        
    battery.cache.soc.value = 12;
    battery.processFSM();
    EXPECT_EQ(battery.getState(), BatteryFSM::CUTOFF);
    
    battery.cache.soc.value = 15;
    testing::internal::CaptureStdout();
    battery.processFSM();
    const auto out_recovery = testing::internal::GetCapturedStdout();
    const std::string_view view_recovery{out_recovery};
        
    EXPECT_EQ(sys_context.getState(), SystemState::INIT); 
    EXPECT_NE(view_recovery.find("Battery recovered to 15%"), std::string_view::npos);
}

TEST_F(SmartBatteryTestSuite, HardwareFailure_TriggersWatchdogFSM) {
    battery.last_valid_comm_time = 0; 
    virtual_uptime = 10000;
    
    testing::internal::CaptureStdout();
    battery.publishError(CommFault::I2C_TIMEOUT);
    const auto out = testing::internal::GetCapturedStdout();
    const std::string_view view{out};
    
    EXPECT_EQ(battery.getState(), BatteryFSM::CUTOFF);
    EXPECT_NE(view.find("CRITICAL: BMS communication watchdog triggered"), std::string_view::npos);
}

// ==============================================================================
// 4. State Management, Caching, and Fault Mapping
// ==============================================================================
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
    EXPECT_EQ(test_map(-EIO), CommFault::I2C_NACK); // default
}

TEST_F(SmartBatteryTestSuite, SOC_Estimation_Limits) {
    EXPECT_EQ(battery.estimateSocFromVoltage(8000), 0U);
    EXPECT_EQ(battery.estimateSocFromVoltage(13000), 100U);
    EXPECT_EQ(battery.estimateSocFromVoltage(10650), 50U);
}

TEST_F(SmartBatteryTestSuite, Getters_StaleCache_And_Errors) {
    battery.cache.timestamp_ms = k_uptime_get_32() - (BatteryLimits::CACHE_STALE_MS + 1000);
    EXPECT_FALSE(battery.getVoltage().success);
    EXPECT_EQ(battery.getVoltage().error, CommFault::CACHE_INVALID);

    battery.cache.timestamp_ms = k_uptime_get_32();
    battery.cache.valid = false;
    battery.cache.last_error = CommFault::I2C_TIMEOUT;
    EXPECT_EQ(battery.getCurrent().error, CommFault::I2C_TIMEOUT);
}

// ==============================================================================
// 5. Hooks, ISRs, and Singleton Logic
// ==============================================================================
TEST_F(SmartBatteryTestSuite, SetWatchdogFeedHookValid) {
    battery.setWatchdogFeedHook(custom_watchdog_hook);
    battery.feedWatchdog();
    EXPECT_TRUE(custom_hook_called);

    battery.setWatchdogFeedHook(nullptr);
    battery.feedWatchdog();
    SUCCEED();
}

TEST_F(SmartBatteryTestSuite, NotifySystemWakeup_ISR_Context) {
    battery.notifySystemWakeup();
    EXPECT_EQ(battery.last_valid_comm_time, virtual_uptime);
}

TEST_F(SmartBatteryTestSuite, SingletonInitializesOnce) {
    auto* inst1 = getSmartBatteryInstance();
    auto* inst2 = getSmartBatteryInstance();
    EXPECT_EQ(inst1, inst2);
    // getI2cBusManagerInstance is unavailable in test environment, omitted
}

// ==============================================================================
// 6. Thread Routines
// ==============================================================================
extern void bms_comm_thread(void);
extern void battery_monitor_thread(void);

TEST_F(SmartBatteryTestSuite, Thread_BmsComm_NullInstance) {
    extern SbsBattery* smart_battery;
    auto* temp = smart_battery;
    smart_battery = nullptr;
    
    testing::internal::CaptureStdout();
    bms_comm_thread(); 
    const auto out = testing::internal::GetCapturedStdout();
    const std::string_view view{out};
    EXPECT_NE(view.find("Smart battery instance is null"), std::string_view::npos);
    
    smart_battery = temp;
}

TEST_F(SmartBatteryTestSuite, Thread_BmsComm_InitRetryLoop) {
    extern SbsBattery* smart_battery;
    smart_battery = &battery;
    
    g_i2c_force_errno = -EIO;   // force init failure
    test_iterations_remaining = 0;
    bms_comm_thread(); 
    g_i2c_force_errno = 0;
    SUCCEED();
}

TEST_F(SmartBatteryTestSuite, Thread_BatteryMonitor_Executes) {
    extern SbsBattery* smart_battery;
    smart_battery = &battery;
    
    battery.cache.current.value = -500;
    battery.cache.soc.value = 80;
    battery.processFSM();
    
    test_iterations_remaining = 1;
    testing::internal::CaptureStdout();
    battery_monitor_thread();
    const auto out = testing::internal::GetCapturedStdout();
    const std::string_view view{out};
    EXPECT_NE(view.find("Battery Discharging: 80% remaining"), std::string_view::npos);
}

TEST_F(SmartBatteryTestSuite, DeepSleepGatingAndPowerObserver) {
    extern SbsBattery* smart_battery;
    smart_battery = &battery;
    
    test_iterations_remaining = 1;
    bms_comm_thread();

    PowerManager::getInstance().notifyBeforeSleep();
    
    test_iterations_remaining = 1;
    testing::internal::CaptureStdout();
    bms_comm_thread();
    battery_monitor_thread();
    const auto out = testing::internal::GetCapturedStdout();
    const std::string_view view{out};
    EXPECT_TRUE(view.empty()); 
    
    PowerManager::getInstance().notifyAfterWakeup();
    PowerManager::getInstance().notifyBeforeSleep();
    PowerManager::getInstance().notifySleepAborted();
    SUCCEED();
}
