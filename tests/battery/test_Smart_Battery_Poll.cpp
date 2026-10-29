// test_Smart_Battery_Poll.cpp
// Tests for SbsBattery::pollHardwareAndUpdateCache() (Smart_Battery_System.cpp): validation rejects,
// jump detection, battery-disconnect handling and I2C/ADC failure paths during a poll.

#include "test_battery_common.h"

TEST_F(SmartBatteryTestSuite, PollHardware_ValidationRejects) {
    g_i2c_mock_v_val = 0;
    battery.pollHardwareAndUpdateCache();
    EXPECT_EQ(battery.cache.last_error, CommFault::NONE);

    battery.cache.last_error = CommFault::NONE;
    battery.cache.valid = true;
    g_i2c_mock_v_val = 0xFFFF;
    battery.pollHardwareAndUpdateCache();
    EXPECT_EQ(battery.cache.last_error, CommFault::VALIDATION_ERROR);
}

TEST_F(SmartBatteryTestSuite, JumpReject_ThresholdReached) {
    battery.cache.voltage.value = 10000; 
    battery.cache.current.value = 0;
    battery.cache.temperature.value = Thermistor::KELVIN_OFFSET_TENTHS + 250;
    battery.cache.valid = true;
    battery.cache.last_error = CommFault::NONE;
    battery.cache.timestamp_ms = virtual_uptime;
    
    g_i2c_mock_v_val = 8000;
    g_i2c_mock_i_val = 0; 
    g_adc_mock_mv_val = 1650; 

    battery.cache.current.value = -8001; 

    battery.pollHardwareAndUpdateCache();
    EXPECT_EQ(battery.consecutive_jump_rejects, 1);

    battery.consecutive_jump_rejects = 0;
    
    for(int i = 0; i < 10; ++i) { 
        battery.pollHardwareAndUpdateCache();
        if (battery.cache.last_error == CommFault::VALIDATION_ERROR) {
            break;
        }
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

    g_i2c_mock_v_val = 8000;
    g_i2c_mock_i_val = 0;
    g_adc_mock_mv_val = 1650;

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
    g_i2c_mock_i_val = 0;
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
    g_adc_mock_mv_val = 114;

    battery.pollHardwareAndUpdateCache();

    EXPECT_GT(battery.consecutive_jump_rejects,0);
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

TEST_F(SmartBatteryTestSuite, PollHardware_TotalI2CFailure_NoCacheFallback) {
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

    g_i2c_fail_after_reads = 2;
    battery.pollHardwareAndUpdateCache();
    g_i2c_fail_after_reads = -1;
    EXPECT_EQ(battery.cache.last_error, CommFault::I2C_NACK);
}

TEST_F(SmartBatteryTestSuite, JumpDetection_AllConditions) {
    battery.cache.valid=true;
    battery.cache.last_error=CommFault::NONE;

    battery.cache.voltage.value=10000;
    battery.cache.current.value=0;
    battery.cache.temperature.value=Thermistor::KELVIN_OFFSET_TENTHS+250;

    g_i2c_mock_v_val=8000;
    g_i2c_mock_i_val=3000;
    g_adc_mock_mv_val=2023;

    battery.cache.voltage.value=10000;
    battery.cache.current.value=0;
    battery.cache.temperature.value=Thermistor::KELVIN_OFFSET_TENTHS+150;

    battery.pollHardwareAndUpdateCache();

    battery.consecutive_jump_rejects=0;

    g_i2c_mock_v_val=8000;
    g_i2c_mock_i_val=0;
    g_adc_mock_mv_val=114;

    battery.pollHardwareAndUpdateCache();

    battery.consecutive_jump_rejects=5;

    g_i2c_mock_v_val=8000;
    g_i2c_mock_i_val=0;
    g_adc_mock_mv_val=2023;

    battery.cache.voltage.value=10000;
    battery.cache.current.value=0;
    battery.cache.temperature.value=Thermistor::KELVIN_OFFSET_TENTHS+150;

    battery.pollHardwareAndUpdateCache();

    EXPECT_EQ(battery.consecutive_jump_rejects,0);
}

TEST_F(SmartBatteryTestSuite, PollHardware_InvalidCache_NotValidationError) {
    battery.cache.valid = false;
    battery.cache.last_error = CommFault::I2C_NACK;

    g_i2c_mock_v_val = 10000;
    g_i2c_mock_i_val = 0;

    battery.pollHardwareAndUpdateCache();

    EXPECT_EQ(battery.cache.last_error, CommFault::NONE);
    EXPECT_TRUE(battery.cache.valid);
}

TEST_F(SmartBatteryTestSuite, PollHardware_JumpDetection_CacheCombinations) {
    battery.cache.timestamp_ms = 0;
    g_i2c_mock_v_val = 10000;
    battery.pollHardwareAndUpdateCache();

    battery.cache.timestamp_ms = virtual_uptime;
    battery.cache.valid = false;
    battery.cache.last_error = CommFault::VALIDATION_ERROR;
    battery.pollHardwareAndUpdateCache();
}

TEST_F(SmartBatteryTestSuite, PollHardware_DisconnectAfterLargeVoltageDrop)
{
    battery.cache.valid = true;
    battery.cache.last_error = CommFault::NONE;
    battery.cache.timestamp_ms = virtual_uptime;
    battery.cache.voltage.value = 12000;

    g_i2c_mock_v_val = 6000;   // pack = 7500 mV

    battery.pollHardwareAndUpdateCache();

    EXPECT_EQ(battery.getState(), BatteryFSM::IDLE);
}

TEST_F(SmartBatteryTestSuite, JumpDetection_VoltageOnly)
{
    battery.cache.valid = true;
    battery.cache.last_error = CommFault::NONE;
    battery.cache.timestamp_ms = virtual_uptime;
    
    // Set a baseline cached voltage
    battery.cache.voltage.value = 10000;
    battery.cache.current.value = 0;
    battery.cache.temperature.value = Thermistor::KELVIN_OFFSET_TENTHS + 25;

    // Provide a new mock voltage that creates a delta larger than MAX_VOLTAGE_DELTA_MV
    g_i2c_mock_v_val = 15000; // Large jump from 10000
    g_i2c_mock_i_val = 0;
    g_adc_mock_mv_val = 2500;

    battery.pollHardwareAndUpdateCache();
    
    // Verify that the jump was rejected and the counter incremented
    EXPECT_GT(battery.consecutive_jump_rejects, 0);
}

TEST_F(SmartBatteryTestSuite, Disconnect_BranchCoverage) {
    // 1. was_connected = true, pack_mv < MIN_VALID_VOLTAGE_MV, v_drop <= MAX_VOLTAGE_DELTA_MV
    battery.cache.valid = true;
    battery.cache.last_error = CommFault::NONE;
    battery.cache.timestamp_ms = virtual_uptime;
    battery.cache.voltage.value = 3000;

    g_i2c_mock_v_val = 2000; // Pack voltage = 2500 mV (v_drop = 500 mV)
    battery.pollHardwareAndUpdateCache();
    EXPECT_EQ(battery.getState(), BatteryFSM::IDLE);

    // 2. was_connected = false, pack_mv >= PACK_MIN_VOLTAGE_MV
    battery.cache.valid = false;
    battery.cache.voltage.value = 0;
    g_i2c_mock_v_val = 8000; // Pack voltage = 10000 mV
    battery.pollHardwareAndUpdateCache();

    // 3. Disconnect with mutex lock failure
    battery.cache.valid = true;
    battery.cache.voltage.value = 10000;
    g_i2c_mock_v_val = 0; // Pack voltage = 0 mV (disconnected)
    g_mutex_lock_force_errno = -EAGAIN;

    battery.pollHardwareAndUpdateCache();
    g_mutex_lock_force_errno = 0;
}

TEST_F(SmartBatteryTestSuite, JumpDetection_AllBranchCombinations) {
    // 1. snapshot.timestamp_ms == 0 (skips jump calculation)
    battery.cache.timestamp_ms = 0;
    battery.cache.valid = true;
    g_i2c_mock_v_val = 10000;
    battery.pollHardwareAndUpdateCache();

    // 2. snapshot.valid == false BUT snapshot.last_error == CommFault::VALIDATION_ERROR
    battery.cache.timestamp_ms = virtual_uptime;
    battery.cache.valid = false;
    battery.cache.last_error = CommFault::VALIDATION_ERROR;
    battery.cache.voltage.value = 10000;
    battery.cache.current.value = 0;
    battery.cache.temperature.value = Thermistor::KELVIN_OFFSET_TENTHS + 250;

    g_i2c_mock_v_val = 15000; // Voltage jump
    battery.pollHardwareAndUpdateCache();

    // 3. Jump rejection via Temperature Delta alone
    battery.cache.valid = true;
    battery.cache.last_error = CommFault::NONE;
    battery.cache.timestamp_ms = virtual_uptime;
    battery.cache.voltage.value = 10000;
    battery.cache.current.value = 0;
    battery.cache.temperature.value = Thermistor::KELVIN_OFFSET_TENTHS + 250;

    g_i2c_mock_v_val = 8000;
    g_i2c_mock_i_val = 0;
    g_adc_mock_mv_val = 100; // Temperature jump

    battery.pollHardwareAndUpdateCache();
    EXPECT_GT(battery.consecutive_jump_rejects, 0);
}

// Line 400: big voltage drop while connected, but new pack voltage is still >= PACK_MIN
TEST_F(SmartBatteryTestSuite, PollHardware_LargeDropButPackAboveMin_NotDisconnect) {
    battery.cache.valid = true;
    battery.cache.last_error = CommFault::NONE;
    battery.cache.timestamp_ms = virtual_uptime;
    battery.cache.voltage.value = 30000;
    battery.cache.current.value = 0;
    battery.cache.temperature.value = Thermistor::KELVIN_OFFSET_TENTHS + 250;

    g_i2c_mock_v_val = 8000;     // pack = 10000 mV
    g_i2c_mock_i_val = 0;
    g_adc_mock_mv_val = 1650;

    battery.pollHardwareAndUpdateCache();
    EXPECT_GT(battery.consecutive_jump_rejects, 0U);
}

// Lines 440-442: shunt register read fails after the bus-voltage read succeeded
TEST_F(SmartBatteryTestSuite, PollHardware_ShuntFetchTotalFailure) {
    extern void resetI2CCacheForTests();
    resetI2CCacheForTests();

    g_i2c_fail_after_reads = 1;
    battery.pollHardwareAndUpdateCache();
    g_i2c_fail_after_reads = -1;
    EXPECT_EQ(battery.cache.last_error, CommFault::I2C_NACK);
}

// Lines 453-456: |current| above MAX_VALID_CURRENT_MA.
// The largest value the INA226 current register can produce is 32768 * CURRENT_LSB_UA / 1000 mA.
// If MAX_VALID_CURRENT_MA is not below that, the check can never fire from any test, so the
// test reports that (SKIPPED, with the numbers) instead of failing. To cover lines 454-456,
// lower BatteryLimits::MAX_VALID_CURRENT_MA (or raise CURRENT_LSB_UA) so the limit is reachable.
TEST_F(SmartBatteryTestSuite, PollHardware_CurrentOutOfRange_ValidationError) {
    const int32_t lsb_ua = static_cast<int32_t>(INA226::CURRENT_LSB_UA);
    const int32_t max_reachable_ma = (32767 * lsb_ua) / 1000;
    const int32_t limit_ma = static_cast<int32_t>(BatteryLimits::MAX_VALID_CURRENT_MA);

    if (max_reachable_ma <= limit_ma) {
        GTEST_SKIP() << "MAX_VALID_CURRENT_MA (" << limit_ma << " mA) is not below the largest current the INA226 "
                     << "register can report (" << max_reachable_ma << " mA at " << lsb_ua
                     << " uA/LSB): the check at Smart_Battery_System.cpp:453 is unreachable.";
    }

    g_i2c_mock_v_val = 8000;
    g_adc_mock_mv_val = 1650;

    const uint16_t raw_extremes[] = {0x7FFF, 0x8000};
    for (uint16_t raw : raw_extremes) {
        battery.cache.valid = true;
        battery.cache.last_error = CommFault::NONE;
        battery.cache.timestamp_ms = virtual_uptime;
        g_i2c_mock_i_val = raw;

        battery.pollHardwareAndUpdateCache();
        EXPECT_EQ(battery.cache.last_error, CommFault::VALIDATION_ERROR) << "raw current 0x" << std::hex << raw;
    }
}

TEST_F(SmartBatteryTestSuite, PollHardware_ThermistorFailure_PublishesThermistorFault) {
    extern void resetI2CCacheForTests();
    resetI2CCacheForTests();             // no stale last-known-good fallback: the I2C reads succeed on their own

    g_i2c_mock_v_val = 10000;            // healthy 12500 mV pack
    g_i2c_mock_i_val = 0;
    g_adc_ready_mock = false;            // Thermistor::readCelsius() fails on every attempt

    battery.pollHardwareAndUpdateCache();

    EXPECT_FALSE(battery.cache.valid);
    EXPECT_EQ(battery.cache.last_error, CommFault::THERMISTOR_FAULT);
    EXPECT_EQ(battery.getStats().thermistor_faults, 1U);
    EXPECT_EQ(battery.getStats().retries, 4U);            // MAX_RETRIES (3) + 1 attempts, one on_retry per failed attempt
    EXPECT_EQ(battery.getStats().successful_publishes, 0U);
}
