// test_Sensor_Read_Cmd.cpp
// Tests for SensorReadCmd (Sensor_Commands.cpp): readHardwareData() per ReadLength, I2C failure logging, the 3-failure fault
// escalation and the activity-reporting / compute-queue-full paths of execute().

#include "test_rtos_common.h"

TEST_F(RTOSCommandsTestSuite, SensorReadCmdLogsCorrectly) {
    SensorReadCmd cmd(SensorID::BME280, SensorReg::BME280_DATA_START, ReadLength::Block);

    testing::internal::CaptureStdout();
    cmd.execute();
    const auto raw_output = testing::internal::GetCapturedStdout();
    std::string_view output(raw_output);

    // CHANGE HERE: A successful read should be silent. It should NOT output an error log.
    EXPECT_FALSE(output.find("[READ]") != std::string_view::npos);
}

TEST_F(RTOSCommandsTestSuite, SensorReadCmdBlockNonBMEReturnsNack) {
    SensorReadCmd cmd(SensorID::LPS22HB, SensorReg::LPS_P_DESC.reg, ReadLength::Block);
    testing::internal::CaptureStdout();
    cmd.execute();
    const auto raw_out = testing::internal::GetCapturedStdout();
std::string_view out(raw_out);
    EXPECT_NE(out.find("I2C Transaction Failed"), std::string_view::npos);
}

TEST_F(RTOSCommandsTestSuite, SensorReadCmdInvalidLengthDefaultBranch) {
    SensorReadCmd cmd(SensorID::BME280, SensorReg::BME280_DATA_START, static_cast<ReadLength>(0xFF));
    testing::internal::CaptureStdout();
    cmd.execute();
    const auto raw_out = testing::internal::GetCapturedStdout();
std::string_view out(raw_out);
    EXPECT_NE(out.find("I2C Transaction Failed"), std::string_view::npos);
}

TEST_F(RTOSCommandsTestSuite, SensorReadCmdTripleAndWordSuccess) {
    SensorReadCmd triple(SensorID::LPS22HB, SensorReg::LPS_P_DESC.reg, ReadLength::Triple);
    triple.execute();

    SensorReadCmd word(SensorID::LPS22HB, SensorReg::LPS_T_DESC.reg, ReadLength::Word);
    word.execute();

    EXPECT_GT(g_queueStats.commandsCreated, 0u);
}

TEST_F(RTOSCommandsTestSuite, HardwareDataTripleAndWordFailures) {
    g_i2c_force_errno = -EIO;

    SensorReadCmd triple(SensorID::LPS22HB, SensorReg::LPS_P_DESC.reg, ReadLength::Triple);
    auto res_triple = triple.readHardwareData();
    EXPECT_FALSE(res_triple.isOk());

    SensorReadCmd word(SensorID::PAV3015, SensorReg::PAV_DESC.reg, ReadLength::Word);
    auto res_word = word.readHardwareData();
    EXPECT_FALSE(res_word.isOk());

    g_i2c_force_errno = 0;
}

TEST_F(RTOSCommandsTestSuite, SensorReadCmdComputeQueueFullLogsError) {
    for (int i = 0; i < QueueConfig::Depth; i++) {
        ASSERT_TRUE(enqueueCommand<PreemptionTestCmd>(PROCESSOR_Q, i));
    }
    SensorReadCmd cmd(SensorID::BME280, SensorReg::BME280_DATA_START, ReadLength::Block);
    testing::internal::CaptureStdout();

    cmd.execute();

    const auto raw_out = testing::internal::GetCapturedStdout();
    std::string_view out(raw_out);
    EXPECT_NE(out.find("Compute queue full"), std::string_view::npos);
}

TEST_F(RTOSCommandsTestSuite, SensorReadCmdI2CFailureLogsError) {
    g_i2c_force_errno = -110;
    SensorReadCmd cmd(SensorID::BME280, SensorReg::BME280_DATA_START, ReadLength::Block);
    testing::internal::CaptureStdout();
    cmd.execute();
    const auto raw_out = testing::internal::GetCapturedStdout();
std::string_view out(raw_out);
    g_i2c_force_errno = 0;
    EXPECT_NE(out.find("I2C Transaction Failed"), std::string_view::npos);
}

TEST_F(RTOSCommandsTestSuite, SensorReadCmdExecuteBranchCoverage) {
    SensorReadCmd pav_cmd(SensorID::PAV3015, SensorReg::PAV_DESC.reg, ReadLength::Word);
    pav_cmd.execute();

    SensorReadCmd unknown_reg_cmd(SensorID::LPS22HB, 0xFF, ReadLength::Word);
    unknown_reg_cmd.execute();

    SensorReadCmd lps_t_cmd(SensorID::LPS22HB, SensorReg::LPS_T_DESC.reg, ReadLength::Word);

    g_i2c_mock_word_val = 0x0010;
    lps_t_cmd.execute();               // last_raw_lps_t becomes 0x0010

    g_i2c_mock_word_val = 0x0100;
    lps_t_cmd.execute();               // last_raw_lps_t becomes 0x0100

    g_i2c_mock_word_val = 0x00F8;
    lps_t_cmd.execute();

    SensorReadCmd unknown_id_err(static_cast<SensorID>(0xFFFF), 0x00, ReadLength::Block);
    unknown_id_err.execute();

    SensorReadCmd unknown_id_ok(static_cast<SensorID>(0xFFFF), 0x00, ReadLength::Word);
    unknown_id_ok.execute();
}

TEST_F(RTOSCommandsTestSuite, SensorReadCmdTriggersFaultOnThreeFailures) {
    g_i2c_force_errno = -EIO;
    SensorReadCmd bme_cmd(SensorID::BME280, SensorReg::BME280_DATA_START, ReadLength::Block);
    SensorReadCmd lps_cmd(SensorID::LPS22HB, SensorReg::LPS_P_DESC.reg, ReadLength::Triple);
    SensorReadCmd pav_cmd(SensorID::PAV3015, SensorReg::PAV_DESC.reg, ReadLength::Word);

    bme_cmd.execute();
    bme_cmd.execute();
    bme_cmd.execute(); // 3rd failure triggers fault
    EXPECT_EQ(SystemObjects::context().getState(), SystemState::FAULT);
    SystemObjects::context().requestTransition(SystemState::INIT);

    lps_cmd.execute();
    lps_cmd.execute();
    lps_cmd.execute();
    EXPECT_EQ(SystemObjects::context().getState(), SystemState::FAULT);
    SystemObjects::context().requestTransition(SystemState::INIT);

    pav_cmd.execute();
    pav_cmd.execute();
    pav_cmd.execute();
    EXPECT_EQ(SystemObjects::context().getState(), SystemState::FAULT);
    SystemObjects::context().requestTransition(SystemState::INIT);

    g_i2c_force_errno = 0;
}
