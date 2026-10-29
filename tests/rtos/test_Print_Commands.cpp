// test_Print_Commands.cpp
// Tests for Print_Commands.cpp: PrintCmd / PrintBME280Cmd / PrintLPS22HBCmd and the print*Measurement() helpers.

#include "test_rtos_common.h"

TEST_F(RTOSCommandsTestSuite, PrintMeasurementLoggerQueueFull) {
    for (int i = 0; i < QueueConfig::Depth; i++) {
        ASSERT_TRUE(enqueueCommand<PreemptionTestCmd>(LOGGER_Q, i));
    }
    EXPECT_FALSE(printMeasurement(SensorID::PAV3015, 1.0f));
    EXPECT_GT(g_queueStats.loggerQueueFull, 0u);
}

TEST_F(RTOSCommandsTestSuite, PrintMeasurementSuccess) {
    EXPECT_TRUE(printMeasurement(SensorID::PAV3015,5.0f));

    ICommand *cmd = nullptr;
    ASSERT_EQ(k_msgq_get(LOGGER_Q, &cmd, K_NO_WAIT), 0);
    cmd->destroy();
}

TEST_F(RTOSCommandsTestSuite, PrintCmdAllSensorNames) {
    std::array<SensorID,7> ids = {
        SensorID::BME280, SensorID::BME280_PRESS, SensorID::BME280_HUM,
        SensorID::LPS22HB, SensorID::LPS22HB_TEMP, SensorID::PAV3015,
        static_cast<SensorID>(0xFFFF)
    };
    for (auto id : ids) {
        PrintCmd cmd(id, 1.0f);
        testing::internal::CaptureStdout();
        cmd.execute();
        const auto raw_out = testing::internal::GetCapturedStdout();
std::string_view out(raw_out);
        EXPECT_NE(out.find("Metric:"), std::string_view::npos);
    }
}

TEST_F(RTOSCommandsTestSuite, PrintCmdUnknownSensorDirectCall) {
    PrintCmd cmd(static_cast<SensorID>(0xFFFF), 1.0f);
    EXPECT_STREQ(cmd.getSensorName(), "Unknown Sensor");
}

TEST_F(RTOSCommandsTestSuite, PrintBME280AndLPS22HBExecuteDirectly) {
    BME280Data bme_data{25.0f, 1013.25f, 50.0f};
    PrintBME280Cmd bme_cmd(bme_data);

    testing::internal::CaptureStdout();
    bme_cmd.execute();
    std::string_view out_bme(testing::internal::GetCapturedStdout());
    EXPECT_NE(out_bme.find("BME280 Temp ="), std::string_view::npos);

    LPS22HBData lps_data{25.0f, 1013.25f};
    PrintLPS22HBCmd lps_cmd(lps_data);

    testing::internal::CaptureStdout();
    lps_cmd.execute();
    std::string_view out_lps(testing::internal::GetCapturedStdout());
    EXPECT_NE(out_lps.find("LPS22HB Temp ="), std::string_view::npos);
}

TEST_F(RTOSCommandsTestSuite, PrintLPS22HBMeasurementLoggerQueueFull) {

    for (int i = 0; i < QueueConfig::Depth; i++) {
        ASSERT_TRUE(enqueueCommand<PreemptionTestCmd>(LOGGER_Q, i));
    }

    LPS22HBData mock_data{25.0f, 1013.0f};
    testing::internal::CaptureStdout();

    EXPECT_FALSE(printLPS22HBMeasurement(mock_data));

    const auto raw_out = testing::internal::GetCapturedStdout();
    std::string_view out(raw_out);
    EXPECT_NE(out.find("Logger queue full"), std::string_view::npos);
    EXPECT_GT(g_queueStats.loggerQueueFull, 0u);
}
