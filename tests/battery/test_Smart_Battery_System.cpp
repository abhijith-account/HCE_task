// test_Smart_Battery_System.cpp
// Tests for Smart_Battery_System.cpp (core SbsBattery): init, watchdog hook, cache getters/freshness, wake-up
// notification and publishError()/mutex handling. Fetch-with-retry is in test_Smart_Battery_Fetch.cpp, polling in
// test_Smart_Battery_Poll.cpp, SoC/Kalman in test_Smart_Battery_Soc.cpp, processFSM in test_Smart_Battery_FSM.cpp.

#include "test_battery_common.h"

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

TEST_F(SmartBatteryTestSuite, Getters_ValidAndInvalidCache) {
    battery.cache.valid = true;
    battery.cache.last_error = CommFault::NONE;
    battery.cache.timestamp_ms = virtual_uptime;

    EXPECT_TRUE(battery.getVoltage().success);
    EXPECT_TRUE(battery.getCurrent().success);
    EXPECT_TRUE(battery.getStateOfCharge().success);
    EXPECT_TRUE(battery.getTemperature().success);
    EXPECT_TRUE(battery.getCapacity().success);
    EXPECT_TRUE(battery.getShuntVoltage().success);

    virtual_uptime += 5000;
    EXPECT_FALSE(battery.getVoltage().success);
    EXPECT_EQ(battery.getVoltage().error, CommFault::CACHE_INVALID);

    virtual_uptime = battery.cache.timestamp_ms;
    battery.cache.last_error = CommFault::I2C_NACK;
    EXPECT_FALSE(battery.getCurrent().success);
    EXPECT_FALSE(battery.getStateOfCharge().success);
    EXPECT_FALSE(battery.getTemperature().success);
    EXPECT_FALSE(battery.getCapacity().success);
    EXPECT_FALSE(battery.getShuntVoltage().success);
}

TEST_F(SmartBatteryTestSuite, MutexContention_AllPaths) {
    g_mutex_lock_force_errno = -EAGAIN;

    battery.updateStateAndPublish(10000, 100, 250, 0);

    BmsCache c = battery.getCacheSnapshot();
    EXPECT_FALSE(c.valid);
    EXPECT_EQ(c.last_error, CommFault::MUTEX_TIMEOUT);

    battery.publishError(CommFault::I2C_NACK);

    for (int i = 0; i < 5; ++i) battery.publishError(CommFault::MUTEX_TIMEOUT);
    EXPECT_EQ(battery.getState(), BatteryFSM::CUTOFF);

    g_mutex_lock_force_errno = 0;
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

TEST_F(SmartBatteryTestSuite, WatchdogHook_ConstructorAndNullHook) {
    custom_hook_called = false;
    SbsBattery hooked_battery(&i2c_manager, &sys_context, &custom_watchdog_hook);
    hooked_battery.feedWatchdog();
    EXPECT_TRUE(custom_hook_called);

    battery.watchdog_feed_hook.store(nullptr);
    custom_hook_called = false;
    battery.feedWatchdog();
    EXPECT_FALSE(custom_hook_called);
}

TEST_F(SmartBatteryTestSuite, CacheFreshness_ZeroTimestampShortCircuit) {
    battery.cache.valid = true;
    battery.cache.last_error = CommFault::NONE;
    battery.cache.timestamp_ms = 0;
    EXPECT_FALSE(battery.getVoltage().success);
    EXPECT_EQ(battery.getVoltage().error, CommFault::CACHE_INVALID);
}

TEST_F(SmartBatteryTestSuite, PublishError_MutexFaultWithSuccessfulLock) {
    g_mutex_lock_force_errno = 0;
    battery.consecutive_mutex_failures = 0;
    battery.current_state.store(BatteryFSM::IDLE);
    battery.publishError(CommFault::MUTEX_TIMEOUT);
    EXPECT_EQ(battery.consecutive_mutex_failures, 1U);
}

TEST_F(SmartBatteryTestSuite, Init_Success) {
    g_i2c_force_errno = 0;
    g_adc_ready_mock = true;
    EXPECT_TRUE(battery.init());
}

TEST_F(SmartBatteryTestSuite, PublishError_MutexLockFailure_Threshold) {
    battery.current_state.store(BatteryFSM::IDLE);
    battery.consecutive_mutex_failures = battery.WATCHDOG_MUTEX_FAILURE_THRESHOLD - 1;

    g_mutex_lock_force_errno = -EAGAIN;
    battery.publishError(CommFault::MUTEX_TIMEOUT);
    g_mutex_lock_force_errno = 0;

    EXPECT_EQ(battery.getState(), BatteryFSM::CUTOFF);
}
