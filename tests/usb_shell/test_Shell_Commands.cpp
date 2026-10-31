// test_Shell_Commands.cpp
// Tests for the UsbShell command layer (USB_CDC_Virtual_COM_Shell_Interface.cpp): dispatchCommand(), set_rate,
// status / formatStatus / collectStatus, log dump, transmitFormatted<> snprintf failure paths, and the parse/trim helpers.
// Note: BatteryCacheAndStateOfCharge exercises SbsBattery::getStateOfCharge() (used by the status command).

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

TEST_F(UsbShellTestSuite, ParseIntTokenAndTrimDirectEdgeCases) {
    int val = 0;
    EXPECT_FALSE(test_parseIntToken("", val));
    EXPECT_TRUE(test_parseIntToken("42", val));
    EXPECT_EQ(val, 42);
    EXPECT_FALSE(test_parseIntToken("12x", val));

    EXPECT_EQ(test_trim(""), "");
    EXPECT_EQ(test_trim(" "), "");
    EXPECT_EQ(test_trim("\t"), "");
    EXPECT_EQ(test_trim(" a "), "a");
    EXPECT_EQ(test_trim("\ta\t"), "a");
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

TEST_F(UsbShellTestSuite,SetRateVariants){
    DeviceContext ctx;
    I2CManager i2c(dummy_uart_dev);
    SbsBattery battery(&i2c,&ctx);
    UsbShell shell(&ctx,&battery);
    mock_dtr_state=1; mock_tx_index=0; mock_tx_buffer.fill(0);

    shell.dispatchCommand("set_rate 1001 50");
    EXPECT_NE(std::string_view(mock_tx_buffer.data(),mock_tx_index).find("Success"),std::string_view::npos);
    uint8_t rate=0; EXPECT_TRUE(ConfigStore::getInstance().getInfusionRate(1,rate)); EXPECT_EQ(rate,50);

    mock_tx_index=0; shell.dispatchCommand("set_rate 1001");
    EXPECT_NE(std::string(mock_tx_buffer.data(),mock_tx_index).find("Usage"),std::string_view::npos);

    mock_tx_index=0; shell.dispatchCommand("set_rate ABC 50");
    EXPECT_NE(std::string(mock_tx_buffer.data(),mock_tx_index).find("Usage"),std::string_view::npos);

    mock_tx_index=0; shell.dispatchCommand("set_rate 1001 XYZ");
    EXPECT_NE(std::string(mock_tx_buffer.data(),mock_tx_index).find("Usage"),std::string_view::npos);

    mock_tx_index=0; shell.dispatchCommand("set_rate 1001 150");
    EXPECT_NE(std::string(mock_tx_buffer.data(),mock_tx_index).find("Rate must be between 1 and 100"),std::string_view::npos);

    mock_tx_index=0; shell.dispatchCommand("set_rate 1001 -1");
    EXPECT_NE(std::string(mock_tx_buffer.data(),mock_tx_index).find("Rate must be between 1 and 100"),std::string_view::npos);

    mock_tx_index=0; shell.dispatchCommand("set_rate 1001 50abc");
    EXPECT_NE(std::string(mock_tx_buffer.data(),mock_tx_index).find("Usage"),std::string_view::npos);

    mock_tx_index=0; shell.dispatchCommand("set_rate 1001 +50");
    EXPECT_NE(std::string(mock_tx_buffer.data(),mock_tx_index).find("Usage"),std::string_view::npos);

    mock_tx_index=0; shell.dispatchCommand("set_rate 9999 50");
    EXPECT_NE(std::string(mock_tx_buffer.data(),mock_tx_index).find("not provisioned"),std::string_view::npos);

    mock_tx_index=0; shell.dispatchCommand("set_rate 1001 85");
    EXPECT_NE(std::string(mock_tx_buffer.data(),mock_tx_index).find("exceeds alarm threshold"),std::string_view::npos);

    mock_tx_index=0; mock_nvs_write_fail=true; shell.dispatchCommand("set_rate 1001 50");
    EXPECT_NE(std::string(mock_tx_buffer.data(),mock_tx_index).find("Failed to persist infusion rate"),std::string_view::npos);
    mock_nvs_write_fail=false;

    mock_tx_index=0;
    for (auto it = mock_fcb_entries.begin(); it != mock_fcb_entries.end(); ) {
        FcbRecordHeader header;
        memcpy(&header, &mock_flash_area[it->offset], sizeof(header));
        if (static_cast<uint16_t>(header.key) == static_cast<uint16_t>(ConfigKey::ALARM_THRESHOLD_BASE)) {
            it = mock_fcb_entries.erase(it);
        } else {
            ++it;
        }
    }
    
    ConfigStore::getInstance().init(); shell.dispatchCommand("set_rate 1001 75");
    EXPECT_NE(std::string_view(mock_tx_buffer.data(),mock_tx_index).find("Success"),std::string_view::npos);

    mock_tx_index=0; shell.dispatchCommand("set_rate      1001     50");
    EXPECT_NE(std::string_view(mock_tx_buffer.data(),mock_tx_index).find("Success"),std::string_view::npos);

    mock_tx_index=0; shell.dispatchCommand("set_rate 1001 0");
    EXPECT_NE(std::string(mock_tx_buffer.data(),mock_tx_index).find("Rate must be between 1 and 100"),std::string_view::npos);

    mock_tx_index=0; shell.dispatchCommand("set_rate 2147483648 50");
    EXPECT_NE(std::string(mock_tx_buffer.data(),mock_tx_index).find("Usage"),std::string_view::npos);
    mock_tx_index=0; shell.dispatchCommand("set_rate 1001 2147483648");
    EXPECT_NE(std::string(mock_tx_buffer.data(),mock_tx_index).find("Usage"),std::string_view::npos);

    ASSERT_TRUE(ConfigStore::getInstance().init());    

    auto &cfg = ConfigStore::getInstance();
    uint8_t slot=0;

    ASSERT_TRUE(cfg.findSlotByDeviceId(1001,slot));
    ASSERT_TRUE(cfg.setAlarmThreshold(slot,80));
    ASSERT_TRUE(cfg.setInfusionRate(slot,10));

    mock_tx_index=0;
    mock_tx_buffer.fill(0);

    shell.dispatchCommand("set_rate 1001 80");

    EXPECT_NE(std::string_view(mock_tx_buffer.data(),mock_tx_index).find("Success"),std::string_view::npos);

    enable_snprintf_mock=true; mock_snprintf_call_count=0; mock_snprintf_fail_on_call=1;
    mock_tx_index=0; shell.dispatchCommand("set_rate 9999 50");
    EXPECT_NE(std::string(mock_tx_buffer.data(),mock_tx_index).find("Device ID not provisioned"),std::string_view::npos);
    mock_snprintf_fail_on_call=-1; mock_snprintf_call_count=0; mock_snprintf_truncate_on_call=1;
    mock_tx_index=0; shell.dispatchCommand("set_rate 1001 50");
    EXPECT_NE(std::string_view(mock_tx_buffer.data(),mock_tx_index).find("Success"),std::string_view::npos);
    enable_snprintf_mock=false;

    mock_tx_index=0; shell.dispatchCommand("set_rate 1001 ");
    EXPECT_NE(std::string(mock_tx_buffer.data(),mock_tx_index).find("Usage"),std::string_view::npos);
}

TEST_F(UsbShellTestSuite,StatusFormatting){
    DeviceContext ctx;
    I2CManager i2c(dummy_uart_dev);
    SbsBattery battery(&i2c,&ctx);
    UsbShell shell(&ctx,&battery);

    mock_dtr_state=1; mock_fcb_entries.clear(); mock_tx_index=0;
    shell.dispatchCommand("status");
    EXPECT_NE(std::string_view(mock_tx_buffer.data(),mock_tx_index).find("\"device_id\":0"),std::string_view::npos);

    UsbShell::StatusSnapshot snap{}; StatusBuffer buf{};
    size_t written=UsbShell::formatStatus(snap,buf);
    EXPECT_NE(std::string_view(buf.data(),written).find("\"devices\":[]"),std::string_view::npos);
    EXPECT_TRUE(std::string_view(buf.data(),written).ends_with("]}\r\n"));

    enable_snprintf_mock=true; mock_snprintf_call_count=0; mock_snprintf_truncate_on_call=1;
    mock_dtr_state=1; mock_tx_index=0; testing::internal::CaptureStdout();
    shell.dispatchCommand("status");
    EXPECT_NE(testing::internal::GetCapturedStdout().find("Status JSON truncated at header"),std::string_view::npos);

    mock_snprintf_call_count=0; mock_snprintf_truncate_on_call=3;
    mock_tx_index=0; testing::internal::CaptureStdout(); shell.dispatchCommand("status");
    EXPECT_NE(testing::internal::GetCapturedStdout().find("Status JSON truncated after"),std::string_view::npos);

    snap.slot_count=static_cast<uint8_t>(snap.slots.size());
    for(uint8_t i=0;i<snap.slot_count;++i) snap.slots[i]={static_cast<uint32_t>(1000+i),50,80};
    written=UsbShell::formatStatus(snap,buf);
    EXPECT_LT(written,buf.size());

    mock_snprintf_call_count=0; mock_snprintf_fail_on_call=2; mock_snprintf_truncate_on_call=-1;
    written=UsbShell::formatStatus(UsbShell::StatusSnapshot{},buf); EXPECT_LT(written,buf.size());
    mock_snprintf_call_count=0; mock_snprintf_truncate_on_call=2; mock_snprintf_fail_on_call=-1;
    written=UsbShell::formatStatus(UsbShell::StatusSnapshot{},buf); EXPECT_LT(written,buf.size());
    enable_snprintf_mock=false;

    mock_dtr_state=1; mock_tx_index=0; mock_tx_buffer.fill(0);
    shell.dispatchCommand("status");
    EXPECT_NE(std::string(mock_tx_buffer.data(),mock_tx_index).find("\"battery_soc\":0"),std::string_view::npos);
    virtual_uptime=5000; BmsCache mc{}; mc.valid=true; mc.last_error=CommFault::NONE;
    mc.timestamp_ms=5000; mc.soc.value=85; battery.cache=mc;
    mock_tx_index=0; mock_tx_buffer.fill(0); shell.dispatchCommand("status");
    EXPECT_NE(std::string(mock_tx_buffer.data(),mock_tx_index).find("\"battery_soc\":85"),std::string_view::npos);

    mock_fcb_entries.clear();
    ConfigStore::getInstance().init();
    auto snap2 = shell.collectStatus();
    EXPECT_EQ(snap2.slots[0].rate,0);

    enable_snprintf_mock=true;
    mock_snprintf_call_count=0;
    mock_snprintf_fail_on_call=1;
    EXPECT_EQ(UsbShell::formatStatus(UsbShell::StatusSnapshot{},buf),0u);
    EXPECT_EQ(mock_snprintf_call_count,1);
    enable_snprintf_mock=false;
    mock_snprintf_fail_on_call=-1;

    UsbShell::StatusSnapshot snap3{};
    snap3.slot_count=2;
    snap3.slots[0]={1001,50,80};
    snap3.slots[1]={1002,60,90};
    enable_snprintf_mock=true;
    mock_snprintf_call_count=0;
    mock_snprintf_fail_on_call=2;
    const auto written2 = UsbShell::formatStatus(snap3, buf);
    EXPECT_GT(written2, 0u);
    enable_snprintf_mock=false;
    mock_snprintf_fail_on_call=-1;
}

TEST_F(UsbShellTestSuite, FormatStatusFooterExactlyOneByteRemaining) {
    UsbShell::StatusSnapshot snap{};
    snap.slot_count = 0;
    StatusBuffer buf{};

    enable_snprintf_mock = true;
    mock_snprintf_call_count = 0;
    mock_snprintf_fail_on_call = -1;
    mock_snprintf_truncate_on_call = -1;
    mock_snprintf_exact_return_on_call = 1;
    mock_snprintf_exact_return_value = static_cast<int>(buf.size() - 1);

    const size_t written = UsbShell::formatStatus(snap, buf);
    EXPECT_EQ(written, buf.size() - 1);

    enable_snprintf_mock = false;
    mock_snprintf_exact_return_on_call = -1;
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

TEST_F(UsbShellTestSuite, SetRateWithTabsInArgs) {
    DeviceContext ctx;
    I2CManager i2c(dummy_uart_dev);
    SbsBattery battery(&i2c, &ctx);
    UsbShell shell(&ctx, &battery);
    mock_dtr_state = 1; mock_tx_index = 0; mock_tx_buffer.fill(0);
    shell.dispatchCommand("set_rate \t1001\t 50");
    std::string_view out(mock_tx_buffer.data(), mock_tx_index);
    EXPECT_NE(out.find("Success"), std::string_view::npos);
}

TEST_F(UsbShellTestSuite, DispatchSetRateWithOnlySpacesAfterCommand) {
    DeviceContext ctx;
    I2CManager i2c(dummy_uart_dev);
    SbsBattery battery(&i2c, &ctx);
    UsbShell shell(&ctx, &battery);
    mock_dtr_state = 1; mock_tx_index = 0; mock_tx_buffer.fill(0);
    shell.dispatchCommand("set_rate ");
    std::string_view out(mock_tx_buffer.data(), mock_tx_index);
    EXPECT_NE(out.find("Usage"), std::string_view::npos);
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

TEST_F(UsbShellTestSuite, CollectStatusAllSlotsHaveRates)
{
    DeviceContext ctx;
    I2CManager i2c(dummy_uart_dev);
    SbsBattery battery(&i2c, &ctx);
    UsbShell shell(&ctx, &battery);
    auto &cfg = ConfigStore::getInstance();

    for (uint8_t slot = InfusionDeviceConfig::MinSlot;
         slot <= InfusionDeviceConfig::MaxSlot;
         ++slot)
    {
        ASSERT_TRUE(cfg.setInfusionRate(slot, 42));
    }

    auto snap = shell.collectStatus();
    for (uint8_t i = 0; i < snap.slot_count; ++i)
    {
        EXPECT_EQ(snap.slots[i].rate, 42);
    }
}

TEST_F(UsbShellTestSuite, SetRateEqualAndLessThanThreshold)
{
    DeviceContext ctx;
    I2CManager i2c(dummy_uart_dev);
    SbsBattery battery(&i2c, &ctx);
    UsbShell shell(&ctx, &battery);
    mock_dtr_state = 1;

    auto &cfg = ConfigStore::getInstance();
    uint8_t slot = 0;

    ASSERT_TRUE(cfg.findSlotByDeviceId(1001, slot));
    ASSERT_TRUE(cfg.setAlarmThreshold(slot, 80));

    mock_tx_index = 0; mock_tx_buffer.fill(0);
    shell.dispatchCommand("set_rate 1001 79");
    EXPECT_NE(std::string_view(mock_tx_buffer.data(), mock_tx_index).find("Success"), std::string_view::npos);

    mock_tx_index = 0; mock_tx_buffer.fill(0);
    shell.dispatchCommand("set_rate 1001 80");
    EXPECT_NE(std::string_view(mock_tx_buffer.data(), mock_tx_index).find("Success"), std::string_view::npos);
}

TEST_F(UsbShellTestSuite, DispatchSetRatePrefixWithoutSpace)
{
    DeviceContext ctx;
    I2CManager i2c(dummy_uart_dev);
    SbsBattery battery(&i2c, &ctx);
    UsbShell shell(&ctx,&battery);
    mock_dtr_state = 1;
    mock_tx_index = 0; mock_tx_buffer.fill(0);
    shell.dispatchCommand("set_rateXYZ");
    EXPECT_NE(std::string_view(mock_tx_buffer.data(),mock_tx_index).find("Unknown"), std::string_view::npos);
}

TEST_F(UsbShellTestSuite, DispatchSetRateFollowedByTab)
{
    DeviceContext ctx;
    I2CManager i2c(dummy_uart_dev);
    SbsBattery battery(&i2c, &ctx);
    UsbShell shell(&ctx, &battery);
    mock_dtr_state = 1;
    mock_tx_index = 0; mock_tx_buffer.fill(0);
    shell.dispatchCommand("set_rate\t1001 50");
    EXPECT_NE(std::string_view(mock_tx_buffer.data(), mock_tx_index).find("Unknown"), std::string_view::npos);
}

TEST_F(UsbShellTestSuite, DispatchExactSetRateCommand)
{
    DeviceContext ctx;
    I2CManager i2c(dummy_uart_dev);
    SbsBattery battery(&i2c, &ctx);
    UsbShell shell(&ctx,&battery);
    mock_dtr_state = 1;
    mock_tx_index = 0; mock_tx_buffer.fill(0);
    shell.dispatchCommand("set_rate");
    std::string_view out(mock_tx_buffer.data(), mock_tx_index);
    EXPECT_NE(out.find("Usage"), std::string_view::npos);
}

TEST_F(UsbShellTestSuite, DispatchSetRatePrefixButInvalidSeparator)
{
    DeviceContext ctx;
    I2CManager i2c(dummy_uart_dev);
    SbsBattery battery(&i2c, &ctx);
    UsbShell shell(&ctx, &battery);
    mock_dtr_state = 1;
    mock_tx_index = 0; mock_tx_buffer.fill(0);
    shell.dispatchCommand("set_rate:");
    std::string_view out(mock_tx_buffer.data(), mock_tx_index);
    EXPECT_NE(out.find("Unknown"), std::string_view::npos);
}

TEST_F(UsbShellTestSuite, DispatchSetRateWhitespaceOnlyArgument)
{
    DeviceContext ctx;
    I2CManager i2c(dummy_uart_dev);
    SbsBattery battery(&i2c, &ctx);
    UsbShell shell(&ctx, &battery);
    mock_dtr_state = 1;
    mock_tx_index = 0; mock_tx_buffer.fill(0);
    shell.dispatchCommand("set_rate          ");
    std::string_view out(mock_tx_buffer.data(), mock_tx_index);
    EXPECT_NE(out.find("Usage"), std::string_view::npos);
}

TEST_F(UsbShellTestSuite, SetRateWithoutAlarmThreshold)
{
    DeviceContext ctx;
    I2CManager i2c(dummy_uart_dev);
    SbsBattery battery(&i2c, &ctx);
    UsbShell shell(&ctx, &battery);
    auto &cfg = ConfigStore::getInstance();
    mock_dtr_state = 1;

    for (uint16_t slot = InfusionDeviceConfig::MinSlot;
         slot <= InfusionDeviceConfig::MaxSlot;
         ++slot)
    {
        for (auto it = mock_fcb_entries.begin(); it != mock_fcb_entries.end(); ) {
            FcbRecordHeader header;
            memcpy(&header, &mock_flash_area[it->offset], sizeof(header));
            if (static_cast<uint16_t>(header.key) == static_cast<uint16_t>(ConfigKey::ALARM_THRESHOLD_BASE) + slot) {
                it = mock_fcb_entries.erase(it);
            } else {
                ++it;
            }
        }
    }
    // NOTE: no cfg.init() here - init() re-seeds the default thresholds, which would
    // make haveThreshold true and leave the "no threshold" branch uncovered.
    uint8_t probe = 0;
    ASSERT_FALSE(cfg.getAlarmThreshold(1, probe)) << "threshold entries should be gone";

    mock_tx_index = 0; mock_tx_buffer.fill(0);
    shell.dispatchCommand("set_rate 1001 50");
    std::string_view out(mock_tx_buffer.data(), mock_tx_index);
    EXPECT_NE(out.find("Success"), std::string_view::npos);
}

TEST_F(UsbShellTestSuite, SetRateThresholdExhaustiveBranches) {
    DeviceContext ctx;
    I2CManager i2c(dummy_uart_dev);
    SbsBattery battery(&i2c, &ctx);
    UsbShell shell(&ctx, &battery);
    mock_dtr_state = 1; mock_tx_index = 0; mock_tx_buffer.fill(0);
    auto &cfg = ConfigStore::getInstance();
    uint8_t slot = 0;
    ASSERT_TRUE(cfg.findSlotByDeviceId(1001, slot));

    for (auto it = mock_fcb_entries.begin(); it != mock_fcb_entries.end(); ) {
        FcbRecordHeader header;
        memcpy(&header, &mock_flash_area[it->offset], sizeof(header));
        if (static_cast<uint16_t>(header.key) == static_cast<uint16_t>(ConfigKey::ALARM_THRESHOLD_BASE) + slot) {
            it = mock_fcb_entries.erase(it);
        } else {
            ++it;
        }
    }

    mock_tx_index = 0; mock_tx_buffer.fill(0);
    shell.dispatchCommand("set_rate 1001 50");
    EXPECT_NE(std::string_view(mock_tx_buffer.data(), mock_tx_index).find("Success"), std::string_view::npos);

    mock_tx_index = 0; mock_tx_buffer.fill(0);
    ASSERT_TRUE(cfg.setAlarmThreshold(slot, 80));
    shell.dispatchCommand("set_rate 1001 80");
    EXPECT_NE(std::string_view(mock_tx_buffer.data(), mock_tx_index).find("Success"), std::string_view::npos);

    mock_tx_index = 0; mock_tx_buffer.fill(0);
    shell.dispatchCommand("set_rate 1001 81");
    EXPECT_NE(std::string_view(mock_tx_buffer.data(), mock_tx_index).find("exceeds alarm threshold"), std::string_view::npos);
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

TEST_F(UsbShellTestSuite, BatteryCacheAndStateOfCharge) {
    DeviceContext ctx;
    I2CManager i2c(dummy_uart_dev);
    SbsBattery battery(&i2c, &ctx);

    virtual_uptime = 10000;

    BmsCache valid_cache{};
    valid_cache.valid = true;
    valid_cache.last_error = CommFault::NONE;
    valid_cache.soc.value = 75;
    valid_cache.timestamp_ms = k_uptime_get_32();
    battery.cache = valid_cache;

    auto soc = battery.getStateOfCharge();
    ASSERT_TRUE(soc.success);
    EXPECT_EQ(soc.value.value, 75);

    valid_cache.timestamp_ms = 0;
    battery.cache = valid_cache;
    soc = battery.getStateOfCharge();
    EXPECT_FALSE(soc.success);
    EXPECT_EQ(soc.error, CommFault::CACHE_INVALID);
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

TEST_F(UsbShellTestSuite, SetRateEmptyArgs) {
    DeviceContext ctx;
    I2CManager i2c(dummy_uart_dev);
    SbsBattery battery(&i2c, &ctx);
    UsbShell shell(&ctx, &battery);
    
    mock_dtr_state = 1;
    mock_tx_index = 0;
    mock_tx_buffer.fill(0);
    shell.dispatchCommand("set_rate  \t  ");
    std::string_view out(mock_tx_buffer.data(), mock_tx_index);
    EXPECT_NE(out.find("Usage"), std::string_view::npos);
}
