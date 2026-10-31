// test_Config_Slots_Endurance.cpp
// Tests for the ConfigStore slot configuration and flash endurance validation (Persistent_Configuration_System.cpp):
// default seeding (device IDs, alarm thresholds), the slot accessors and their bounds checks, findSlotByDeviceId, generic get() for unknown keys,
// and validateEndurance() (1,000-cycle write/read/verify, skip-if-done, write/read/mismatch/FCB errors, FAULT abort).

#include "test_config_common.h"

TEST_F(ConfigStoreTestSuite, SeedsDefaultsOnFirstBoot) {
    ConfigStore& config = ConfigStore::getInstance();
    testing::internal::CaptureStdout();
    ASSERT_TRUE(config.init());
    const auto raw_out = testing::internal::GetCapturedStdout();
    std::string_view output(raw_out);

    for (uint8_t i = 1; i <= 5; ++i) {
        uint32_t devId = 0;
        uint8_t alarm = 0;
        EXPECT_TRUE(config.getDeviceId(i, devId));
        EXPECT_EQ(devId, InfusionDeviceConfig::DefaultDeviceIds[i-1]);
        EXPECT_TRUE(config.getAlarmThreshold(i, alarm));
        EXPECT_EQ(alarm, InfusionDeviceConfig::DefaultAlarmThresholds[i-1]);
        
        char expected_msg[32];
        snprintf(expected_msg, sizeof(expected_msg), "Provisioned slot %u", i);
        EXPECT_TRUE(output.find(expected_msg) != std::string_view::npos);
    }
}

TEST_F(ConfigStoreTestSuite, SeedingLogsWarningOnWriteFail) {
    ConfigStore& config = ConfigStore::getInstance();
    mock_flash_write_fail = true;

    testing::internal::CaptureStdout();
    ASSERT_TRUE(config.init());
    const auto raw_out = testing::internal::GetCapturedStdout();
    std::string_view output(raw_out);

    EXPECT_TRUE(output.find("[WRN] Failed to provision default Device ID") != std::string_view::npos);
    EXPECT_TRUE(output.find("[WRN] Failed to seed default alarm threshold") != std::string_view::npos);
}

TEST_F(ConfigStoreTestSuite, SeedingSkipsExistingValues) {
    ConfigStore& config = ConfigStore::getInstance();
    config.initialized = true;
    EXPECT_TRUE(config.setDeviceId(1, 9999));
    EXPECT_TRUE(config.setAlarmThreshold(1, 88));
    config.initialized = false;

    testing::internal::CaptureStdout();
    ASSERT_TRUE(config.init());
    const auto raw_out = testing::internal::GetCapturedStdout();
    std::string_view output(raw_out);

    uint32_t devId;
    EXPECT_TRUE(config.getDeviceId(1, devId));
    EXPECT_EQ(devId, 9999);

    uint8_t alarm;
    EXPECT_TRUE(config.getAlarmThreshold(1, alarm));
    EXPECT_EQ(alarm, 88);

    EXPECT_TRUE(output.find("Provisioned slot 1") == std::string_view::npos);
}

TEST_F(ConfigStoreTestSuite, SeedingOverwritesUnprovisionedIdSentinel) {
    ConfigStore& config = ConfigStore::getInstance();
    config.initialized = true;
    EXPECT_TRUE(config.setDeviceId(2, InfusionDeviceConfig::UnprovisionedId));
    config.initialized = false;
    config.init();

    uint32_t devId = 0;
    EXPECT_TRUE(config.getDeviceId(2, devId));
    EXPECT_EQ(devId, InfusionDeviceConfig::DefaultDeviceIds[1]);
}

TEST_F(ConfigStoreTestSuite, FindSlotByDeviceId) {
    ConfigStore& config = ConfigStore::getInstance();
    config.init();
    uint8_t slot;
    EXPECT_TRUE(config.findSlotByDeviceId(1003, slot));
    EXPECT_EQ(slot, 3);
    EXPECT_FALSE(config.findSlotByDeviceId(9999, slot));
}

TEST_F(ConfigStoreTestSuite, InfusionRateBoundsCheck) {
    ConfigStore& config = ConfigStore::getInstance();
    config.init();
    EXPECT_FALSE(config.setInfusionRate(0, 50));
    EXPECT_FALSE(config.setInfusionRate(6, 50));
    EXPECT_FALSE(config.setInfusionRate(1, 0));
    EXPECT_FALSE(config.setInfusionRate(1, 101));

    EXPECT_TRUE(config.setInfusionRate(1, 50));
    uint8_t rate;
    EXPECT_TRUE(config.getInfusionRate(1, rate));
    EXPECT_EQ(rate, 50);

    uint8_t out;
    EXPECT_FALSE(config.getInfusionRate(0, out));
}

TEST_F(ConfigStoreTestSuite, AlarmThresholdBoundsCheck) {
    ConfigStore& config = ConfigStore::getInstance();
    config.init();
    EXPECT_FALSE(config.setAlarmThreshold(0, 90));
    EXPECT_FALSE(config.setAlarmThreshold(6, 90));
    EXPECT_FALSE(config.setAlarmThreshold(1, 79));
    EXPECT_FALSE(config.setAlarmThreshold(1, 101));

    EXPECT_TRUE(config.setAlarmThreshold(1, 85));
    uint8_t out;
    EXPECT_FALSE(config.getAlarmThreshold(0, out));
}

TEST_F(ConfigStoreTestSuite, DeviceIdBoundsCheck) {
    ConfigStore& config = ConfigStore::getInstance();
    config.init();
    EXPECT_FALSE(config.setDeviceId(0, 123));
    uint32_t out;
    EXPECT_FALSE(config.getDeviceId(0, out));
}

TEST_F(ConfigStoreTestSuite, GenericGetFailsForUnknownKeys) {
    ConfigStore& config = ConfigStore::getInstance();
    config.init();
    uint32_t unknown_val = 0;
    EXPECT_FALSE(config.get(static_cast<ConfigKey>(9999), unknown_val));
}

TEST_F(ConfigStoreTestSuite, ValidateEnduranceThresholds) {
    ConfigStore& config = ConfigStore::getInstance();
    config.init();

    testing::internal::CaptureStdout();
    bool passed = config.validateEndurance(ConfigKey::FULL_CHARGE_LOG);
    const auto raw_out = testing::internal::GetCapturedStdout();
    std::string_view output(raw_out);

    EXPECT_TRUE(passed) << "Flash memory failed the 1,000 write-cycle test";
    EXPECT_TRUE(output.find("[WRN] Starting 1,000 Write-Cycle Endurance Test") != std::string_view::npos);
    EXPECT_TRUE(output.find("[INF] 1,000 Write-Cycle Endurance Test passed.") != std::string_view::npos);

    uint16_t final_value = 0;
    EXPECT_TRUE(config.get(ConfigKey::FULL_CHARGE_LOG, final_value));
    EXPECT_EQ(final_value, 999);
}

TEST_F(ConfigStoreTestSuite, EnduranceTestSkipsIfAlreadyCompleted) {
    ConfigStore& config = ConfigStore::getInstance();
    config.init();
    config.set(ConfigKey::FULL_CHARGE_LOG, (uint16_t)999);
    
    testing::internal::CaptureStdout();
    EXPECT_TRUE(config.validateEndurance(ConfigKey::FULL_CHARGE_LOG));
    const auto raw_out = testing::internal::GetCapturedStdout();
    std::string_view output(raw_out);
    
    EXPECT_TRUE(output.find("Endurance test already completed on a previous boot") != std::string_view::npos);
}

// NEW TEST: Coverage for endurance test reading an existing, non-999 value
TEST_F(ConfigStoreTestSuite, ValidateEnduranceRunsIfPreviousValueNot999) {
    ConfigStore& config = ConfigStore::getInstance();
    config.init();
    config.set(ConfigKey::FULL_CHARGE_LOG, (uint16_t)123);
    
    testing::internal::CaptureStdout();
    EXPECT_TRUE(config.validateEndurance(ConfigKey::FULL_CHARGE_LOG));
    const auto raw_out = testing::internal::GetCapturedStdout();
    std::string_view output(raw_out);
    
    EXPECT_TRUE(output.find("Starting 1,000 Write-Cycle Endurance Test") != std::string_view::npos);
}

TEST_F(ConfigStoreTestSuite, ValidateEnduranceFailsOnWriteError) {
    ConfigStore& config = ConfigStore::getInstance();
    config.init();

    mock_flash_write_fail = true;
    testing::internal::CaptureStdout();
    EXPECT_FALSE(config.validateEndurance(ConfigKey::FULL_CHARGE_LOG));
    const auto raw_out = testing::internal::GetCapturedStdout();
    std::string_view output(raw_out);

    EXPECT_TRUE(output.find("[ERR] Endurance Test failed to write at cycle 0") != std::string_view::npos);
}

TEST_F(ConfigStoreTestSuite, ValidateEnduranceFailsOnReadError) {
    ConfigStore& config = ConfigStore::getInstance();
    config.init();

    mock_flash_read_fail = true;
    EXPECT_FALSE(config.validateEndurance(ConfigKey::FULL_CHARGE_LOG));
}

TEST_F(ConfigStoreTestSuite, ValidateEnduranceFailsOnDataMismatch) {
    ConfigStore& config = ConfigStore::getInstance();
    config.init();

    mock_nvs_corrupt_data = true;

    testing::internal::CaptureStdout();
    EXPECT_FALSE(config.validateEndurance(ConfigKey::FULL_CHARGE_LOG));
    const auto raw_out = testing::internal::GetCapturedStdout();
    std::string_view output(raw_out);

    EXPECT_TRUE(output.find("[ERR] Endurance Test failed to verify at cycle 0") != std::string_view::npos);
}

TEST_F(ConfigStoreTestSuite, ValidateEnduranceFailsOnFcbGetNextError) {
    ConfigStore& config = ConfigStore::getInstance();
    config.init();

    // Inject failure during FCB traversal
    mock_fcb_getnext_fail = true;

    testing::internal::CaptureStdout();
    EXPECT_FALSE(config.validateEndurance(ConfigKey::FULL_CHARGE_LOG));
    const auto raw_out = testing::internal::GetCapturedStdout();
    std::string_view output(raw_out);

    // Verify it fails exactly at the get() verification on cycle 0
    EXPECT_TRUE(output.find("[ERR] Endurance Test failed to verify at cycle 0") != std::string_view::npos);
}

// seedDefaultDeviceIds()/seedDefaultAlarmThresholds(): bail out immediately in FAULT.
TEST_F(ConfigStoreTestSuite, SeedingIsSkippedWhenSystemFaulted) {
    ConfigStore& config = ConfigStore::getInstance();
    sys_context.current_state = SystemState::FAULT;

    testing::internal::CaptureStdout();
    ASSERT_TRUE(config.init());   // mount succeeds (no -ENOMSG), seeding returns early
    const std::string raw_out = testing::internal::GetCapturedStdout();

    EXPECT_NE(raw_out.find("FCB Configuration Store Mounted Successfully."), std::string::npos);
    EXPECT_EQ(raw_out.find("Provisioned slot"), std::string::npos);
    EXPECT_EQ(raw_out.find("Seeded slot"), std::string::npos);
    EXPECT_TRUE(mock_fcb_entries.empty()) << "Nothing may be written to flash while faulted";

    uint32_t devId = 0;
    uint8_t alarm = 0;
    EXPECT_FALSE(config.getDeviceId(1, devId));
    EXPECT_FALSE(config.getAlarmThreshold(1, alarm));
}

// validateEndurance(): FAULT detected inside the 1,000-cycle loop.
TEST_F(ConfigStoreTestSuite, ValidateEnduranceAbortsWhenSystemFaults) {
    ConfigStore& config = ConfigStore::getInstance();
    ASSERT_TRUE(config.init());
    const size_t entries_before = mock_fcb_entries.size();

    sys_context.current_state = SystemState::FAULT;

    testing::internal::CaptureStdout();
    EXPECT_FALSE(config.validateEndurance(ConfigKey::FULL_CHARGE_LOG));
    const std::string raw_out = testing::internal::GetCapturedStdout();

    EXPECT_NE(raw_out.find("[ERR] System FAULT detected. Aborting endurance test."), std::string::npos);
    EXPECT_EQ(raw_out.find("Endurance Test Progress"), std::string::npos);
    EXPECT_EQ(mock_fcb_entries.size(), entries_before) << "No flash writes allowed after FAULT";
}
