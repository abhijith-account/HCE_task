// test_Shell_LogDump_Reboot.cpp
// Tests for the no-argument UsbShell commands and the dispatcher's fall-back paths (Usb_Shell_Commands.cpp): the log dump
// (empty, with entries, flash read failure, snprintf failure / truncation), reboot, blank and unknown commands, and the
// transmitFormatted<> instantiations and snprintf-failure fall-backs reached through the command handlers.

#include "test_usb_shell_common.h"

namespace {
// Runs one CLI command with the snprintf mock armed to fail / truncate on the given call
// (-1 = leave that failure mode off) and returns everything transmitted.
std::string run_cli(UsbShell& shell, const char* cmd, int fail_call, int trunc_call) {
    mock_tx_index = 0; mock_tx_buffer.fill(0);
    enable_snprintf_mock = (fail_call > 0 || trunc_call > 0);
    mock_snprintf_call_count = 0;
    mock_snprintf_fail_on_call = fail_call;
    mock_snprintf_truncate_on_call = trunc_call;
    shell.dispatchCommand(cmd);
    enable_snprintf_mock = false;
    mock_snprintf_fail_on_call = -1;
    mock_snprintf_truncate_on_call = -1;
    return std::string(mock_tx_buffer.data(), mock_tx_index);
}
}

TEST_F(UsbShellTestSuite, DispatchCommands) {
    DeviceContext ctx;
    I2CManager i2c(dummy_uart_dev);
    SbsBattery battery(&i2c, &ctx);
    UsbShell shell(&ctx, &battery);
    mock_dtr_state = 1; mock_tx_index = 0; mock_tx_buffer.fill(0);

    shell.dispatchCommand("status");
    EXPECT_NE(std::string_view(mock_tx_buffer.data(), mock_tx_index).find("sys_state"), std::string_view::npos);

    mock_tx_index = 0; mock_tx_buffer.fill(0);
    shell.dispatchCommand("log dump");
    EXPECT_NE(std::string_view(mock_tx_buffer.data(), mock_tx_index).find("NVS System Log Dump"), std::string_view::npos);

    mock_tx_index = 0; mock_tx_buffer.fill(0);
    shell.dispatchCommand("reboot");
    EXPECT_NE(std::string_view(mock_tx_buffer.data(), mock_tx_index).find("Rebooting system..."), std::string_view::npos);

    mock_tx_index = 0; mock_tx_buffer.fill(0);
    shell.dispatchCommand("bogus");
    EXPECT_NE(std::string_view(mock_tx_buffer.data(), mock_tx_index).find("Unknown command"), std::string_view::npos);

    mock_tx_index = 0; mock_tx_buffer.fill(0);
    shell.dispatchCommand("STATUS");
    EXPECT_NE(std::string_view(mock_tx_buffer.data(), mock_tx_index).find("Unknown"), std::string_view::npos);

    mock_tx_index = 0; mock_tx_buffer.fill(0);
    shell.dispatchCommand("status ");
    EXPECT_NE(std::string_view(mock_tx_buffer.data(), mock_tx_index).find("sys_state"), std::string_view::npos);

    mock_tx_index = 0; mock_tx_buffer.fill(0);
    shell.dispatchCommand("status\t");
    EXPECT_NE(std::string_view(mock_tx_buffer.data(), mock_tx_index).find("sys_state"), std::string_view::npos);

    mock_tx_index = 0; mock_tx_buffer.fill(0);
    shell.dispatchCommand("status x");
    EXPECT_NE(std::string_view(mock_tx_buffer.data(), mock_tx_index).find("Unknown command"), std::string_view::npos);
}

TEST_F(UsbShellTestSuite, DispatchBlankCommand) {
    DeviceContext ctx;
    I2CManager i2c(dummy_uart_dev);
    SbsBattery battery(&i2c, &ctx);
    UsbShell shell(&ctx, &battery);
    mock_dtr_state = 1; mock_tx_index = 0; mock_tx_buffer.fill(0);
    shell.dispatchCommand("  \t  ");
    std::string_view out(mock_tx_buffer.data(), mock_tx_index);
    EXPECT_NE(out.find("Unknown"), std::string_view::npos);
}

TEST_F(UsbShellTestSuite, DispatchNoArgumentHandlersDirectly)
{
    DeviceContext ctx;
    I2CManager i2c(dummy_uart_dev);
    SbsBattery battery(&i2c, &ctx);
    UsbShell shell(&ctx, &battery);
    mock_dtr_state = 1;

    mock_tx_index = 0;
    mock_tx_buffer.fill(0);
    shell.dispatchCommand("log dump");
    EXPECT_NE(std::string_view(mock_tx_buffer.data(), mock_tx_index)
                  .find("NVS System Log Dump"),
              std::string_view::npos);

    mock_tx_index = 0;
    mock_tx_buffer.fill(0);
    shell.dispatchCommand("reboot");
    EXPECT_NE(std::string_view(mock_tx_buffer.data(), mock_tx_index)
                  .find("Rebooting system"),
              std::string_view::npos);
}

// transmitFormatted<> is instantiated three times (not-provisioned / threshold / success);
// every instantiation needs: formatted OK, snprintf failure, snprintf truncation.
TEST_F(UsbShellTestSuite, TransmitFormattedAllInstantiations) {
    DeviceContext ctx;
    I2CManager i2c(dummy_uart_dev);
    SbsBattery battery(&i2c, &ctx);
    UsbShell shell(&ctx, &battery);
    mock_dtr_state = 1;
    auto& cfg = ConfigStore::getInstance();
    uint8_t slot = 0;
    ASSERT_TRUE(cfg.findSlotByDeviceId(1001, slot));
    ASSERT_TRUE(cfg.setAlarmThreshold(slot, 80));

    // --- not provisioned
    EXPECT_NE(run_cli(shell, "set_rate 9999 50", -1, -1).find("Device ID 9999 not provisioned"), std::string::npos);
    EXPECT_NE(run_cli(shell, "set_rate 9999 50",  1, -1).find("Device ID not provisioned"),      std::string::npos);
    EXPECT_NE(run_cli(shell, "set_rate 9999 50", -1,  1).find("Device ID not provisioned"),      std::string::npos);

    // --- exceeds alarm threshold
    EXPECT_NE(run_cli(shell, "set_rate 1001 99", -1, -1).find("Rate 99 exceeds alarm threshold (80) for device 1001"), std::string::npos);
    EXPECT_NE(run_cli(shell, "set_rate 1001 99",  1, -1).find("Rate exceeds alarm threshold. Please reset"),            std::string::npos);
    EXPECT_NE(run_cli(shell, "set_rate 1001 99", -1,  1).find("Rate exceeds alarm threshold. Please reset"),            std::string::npos);

    // --- success
    EXPECT_NE(run_cli(shell, "set_rate 1001 50", -1, -1).find("Device 1001 infusion rate set to 50%"), std::string::npos);
    EXPECT_NE(run_cli(shell, "set_rate 1001 50",  1, -1).find("Success: infusion rate updated"),       std::string::npos);
    EXPECT_NE(run_cli(shell, "set_rate 1001 50", -1,  1).find("Success: infusion rate updated"),       std::string::npos);
}

TEST_F(UsbShellTestSuite, LogDumpWithNoLogs) {
    DeviceContext ctx;
    I2CManager i2c(dummy_uart_dev);
    SbsBattery battery(&i2c, &ctx);
    UsbShell shell(&ctx, &battery);

    mock_fcb_entries.clear();
    mock_dtr_state = 1;
    mock_tx_index = 0; mock_tx_buffer.fill(0);
    shell.dispatchCommand("log dump");
    std::string_view out(mock_tx_buffer.data(), mock_tx_index);
    EXPECT_NE(out.find("No logs found"), std::string_view::npos);
    EXPECT_NE(out.find("End of Logs"), std::string_view::npos);
}

TEST_F(UsbShellTestSuite, TransmitFormattedFallbackOnSnprintfFailure) {
    DeviceContext ctx;
    I2CManager i2c(dummy_uart_dev);
    SbsBattery battery(&i2c, &ctx);
    UsbShell shell(&ctx, &battery);
    mock_dtr_state = 1;
    mock_tx_index = 0;
    mock_tx_buffer.fill(0);

    enable_snprintf_mock = true;
    mock_snprintf_fail_on_call = 1;
    shell.dispatchCommand("set_rate 1001 50");
    std::string_view out(mock_tx_buffer.data(), mock_tx_index);
    EXPECT_NE(out.find("Success: infusion rate updated"), std::string_view::npos);

    enable_snprintf_mock = false;
}

TEST_F(UsbShellTestSuite, LogDumpWithEntries) {
    DeviceContext ctx;
    I2CManager i2c(dummy_uart_dev);
    SbsBattery battery(&i2c, &ctx);
    UsbShell shell(&ctx, &battery);
    
    mock_fcb_entries.clear();
    
    auto& cfg = ConfigStore::getInstance();
    
    // Add multiple entries to fully cover the (i % 10) == 0 alternating condition
    ConfigStore::LogEntry log_entry1{12345, 0x01, 100};
    ConfigStore::LogEntry log_entry2{12346, 0x02, 200};
    ASSERT_TRUE(cfg.set(ConfigKey::FULL_CHARGE_LOG, log_entry1));
    ASSERT_TRUE(cfg.set(ConfigKey::FULL_CHARGE_LOG, log_entry2));
    
    mock_dtr_state = 1;
    mock_tx_index = 0;
    mock_tx_buffer.fill(0);
    shell.dispatchCommand("log dump");
    std::string_view out(mock_tx_buffer.data(), mock_tx_index);
    EXPECT_NE(out.find("Event: 1"), std::string_view::npos);
    EXPECT_NE(out.find("Event: 2"), std::string_view::npos);
}

TEST_F(UsbShellTestSuite, LogDumpReadFailure) {
    DeviceContext ctx;
    I2CManager i2c(dummy_uart_dev);
    SbsBattery battery(&i2c, &ctx);
    UsbShell shell(&ctx, &battery);
    
    mock_fcb_entries.clear();
    
    auto& cfg = ConfigStore::getInstance();
    ConfigStore::LogEntry log_entry{12345, 0x01, 100};
    ASSERT_TRUE(cfg.set(ConfigKey::FULL_CHARGE_LOG, log_entry));
    
    mock_dtr_state = 1;
    mock_tx_index = 0;
    mock_tx_buffer.fill(0);
    
    mock_flash_read_call_count = 0;
    // Call 1: getStoredLogCount reads header (returns 0)
    // Call 2: getLogEntry reads header (returns 0)
    // Call 3: getLogEntry reads payload (returns -1 due to mock instruction)
    mock_flash_read_fail_on_call = 3; 
    
    shell.dispatchCommand("log dump");
    std::string_view out(mock_tx_buffer.data(), mock_tx_index);
    EXPECT_NE(out.find("Failed to read log entry"), std::string_view::npos);
}

TEST_F(UsbShellTestSuite, LogDumpSnprintfFailure) {
    DeviceContext ctx;
    I2CManager i2c(dummy_uart_dev);
    SbsBattery battery(&i2c, &ctx);
    UsbShell shell(&ctx, &battery);
    
    mock_fcb_entries.clear();
    
    auto& cfg = ConfigStore::getInstance();
    ConfigStore::LogEntry log_entry{12345, 0x01, 100};
    ASSERT_TRUE(cfg.set(ConfigKey::FULL_CHARGE_LOG, log_entry));
    
    mock_dtr_state = 1;
    mock_tx_index = 0;
    mock_tx_buffer.fill(0);
    
    enable_snprintf_mock = true;
    mock_snprintf_call_count = 0;
    mock_snprintf_fail_on_call = 1; 
    
    testing::internal::CaptureStdout();
    shell.dispatchCommand("log dump");
    std::string stdout_out = testing::internal::GetCapturedStdout();
    
    enable_snprintf_mock = false;
    mock_snprintf_fail_on_call = -1;
    
    EXPECT_NE(stdout_out.find("formatting truncated or failed"), std::string_view::npos);
}

TEST_F(UsbShellTestSuite, LogDumpSnprintfTruncation) {
    DeviceContext ctx;
    I2CManager i2c(dummy_uart_dev);
    SbsBattery battery(&i2c, &ctx);
    UsbShell shell(&ctx, &battery);
    
    mock_fcb_entries.clear();
    
    auto& cfg = ConfigStore::getInstance();
    ConfigStore::LogEntry log_entry{12345, 0x01, 100};
    ASSERT_TRUE(cfg.set(ConfigKey::FULL_CHARGE_LOG, log_entry));
    
    mock_dtr_state = 1;
    mock_tx_index = 0;
    mock_tx_buffer.fill(0);
    
    enable_snprintf_mock = true;
    mock_snprintf_call_count = 0;
    mock_snprintf_truncate_on_call = 1; 
    
    testing::internal::CaptureStdout();
    shell.dispatchCommand("log dump");
    std::string stdout_out = testing::internal::GetCapturedStdout();
    
    enable_snprintf_mock = false;
    mock_snprintf_truncate_on_call = -1;
    
    EXPECT_NE(stdout_out.find("formatting truncated or failed"), std::string_view::npos);
}
