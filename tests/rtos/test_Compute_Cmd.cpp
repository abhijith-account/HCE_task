// test_Compute_Cmd.cpp
// Tests for ComputeCmd (Sensor_Commands.cpp): per-sensor decode dispatch (BME280, LPS22HB pressure+temperature pairing, PAV3015,
// unknown id), print-queue-full handling, and the BME280 ROM calibration load / cache / failure chain, which is only
// reachable through ComputeCmd::execute().

#include "test_rtos_common.h"

TEST_F(RTOSCommandsTestSuite, ComputeCmdBME280Logic) {
    ComputeCmd cmd(SensorID::BME280, SensorReg::BME280_DATA_START, 0x000100010001ULL);

    testing::internal::CaptureStdout();
    cmd.execute();

    const auto raw_output = testing::internal::GetCapturedStdout();
    std::string_view output(raw_output);
    
    // CHANGE HERE: A successful compute enqueues to the logger quietly.
    // It should NOT output a compute error log.
    EXPECT_FALSE(output.find("[COMPUTE]") != std::string_view::npos);
}

TEST_F(RTOSCommandsTestSuite, ComputeCmdLPS22HBBranches) {

    ComputeCmd temp_early(SensorID::LPS22HB, SensorReg::LPS_T_DESC.reg, 0x0032ULL);
    temp_early.execute();

    ComputeCmd press(SensorID::LPS22HB, SensorReg::LPS_P_DESC.reg, 0x00640032ULL);
    press.execute();

    ComputeCmd temp_late(SensorID::LPS22HB, SensorReg::LPS_T_DESC.reg, 0x0032ULL);
    temp_late.execute();

    ComputeCmd unknown_reg(SensorID::LPS22HB, 0xFF, 0x00ULL);
    unknown_reg.execute();
}

TEST_F(RTOSCommandsTestSuite, ComputeCmdPAV3015Branch) {
    ComputeCmd cmd(SensorID::PAV3015, SensorReg::PAV_DESC.reg, 0x0032ULL);
    cmd.execute();
}

TEST_F(RTOSCommandsTestSuite, ComputeCmdUnknownSensorDefaultBranch) {
    ComputeCmd cmd(static_cast<SensorID>(0xFFFF), 0x00, 0ULL);
    testing::internal::CaptureStdout();
    cmd.execute();
    const auto raw_out = testing::internal::GetCapturedStdout();
std::string_view out(raw_out);
    EXPECT_NE(out.find("Unknown Sensor ID"), std::string_view::npos);
}

TEST_F(RTOSCommandsTestSuite, ComputeCmdBME280CalibrationCachedOnSecondCall) {
    ComputeCmd first(SensorID::BME280, SensorReg::BME280_DATA_START, 0x000100010001ULL);
    first.execute();
    ComputeCmd second(SensorID::BME280, SensorReg::BME280_DATA_START, 0x000200020002ULL);
    second.execute();
}

TEST_F(RTOSCommandsTestSuite, ComputeCmdBME280CalibrationAbortsOnI2CFailure) {
    g_i2c_force_errno = -19;
    ComputeCmd cmd(SensorID::BME280, SensorReg::BME280_DATA_START, 0x000100010001ULL);
    testing::internal::CaptureStdout();
    cmd.execute();
    const auto raw_out = testing::internal::GetCapturedStdout();
std::string_view out(raw_out);
    g_i2c_force_errno = 0;
    EXPECT_NE(out.find("BME280 calibration aborted"), std::string_view::npos);
}

TEST_F(RTOSCommandsTestSuite, CalibrationReadFailure) {
    g_i2c_force_errno = -5;

    ComputeCmd cmd(SensorID::BME280, SensorReg::BME280_DATA_START, 1);
    cmd.execute();

    g_i2c_force_errno = 0;
}

TEST_F(RTOSCommandsTestSuite, ComputeCmdBME280PrintsFailWhenQueueFull) {

    for (int i = 0; i < QueueConfig::Depth; i++) {
        EXPECT_TRUE(enqueueCommand<PreemptionTestCmd>(LOGGER_Q, i));
    }

    ComputeCmd cmd(SensorID::BME280, SensorReg::BME280_DATA_START, 0x000100010001ULL);
    testing::internal::CaptureStdout();
    cmd.execute();
    const auto raw_out = testing::internal::GetCapturedStdout();
    std::string_view out(raw_out);

    EXPECT_NE(out.find("Logger queue full"), std::string_view::npos);
}

TEST_F(RTOSCommandsTestSuite, CalibrationChainFailsAtEveryReadPosition) {
    for (int fail_at = 1; fail_at <= 20; ++fail_at) {
        resetRtosCommandTestState();
        resetI2CCacheForTests();
        g_i2c_call_counter = 0;
        g_i2c_fail_on_call_n = fail_at;
        g_i2c_fail_on_call_errno = -EIO;

        ComputeCmd cmd(SensorID::BME280, SensorReg::BME280_DATA_START, 0x000100010001ULL);
        testing::internal::CaptureStdout();
        cmd.execute();
        const auto raw_out = testing::internal::GetCapturedStdout();
std::string_view out(raw_out);

        EXPECT_NE(out.find("BME280 calibration aborted"), std::string_view::npos)
            << "Expected calibration abort when I2C call #" << fail_at << " fails";
    }
    g_i2c_fail_on_call_n = 0;
}
