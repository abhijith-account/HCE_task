// test_Config_Fcb_Core.cpp
// Tests for the ConfigStore FCB core (Persistent_Configuration_System.cpp): singleton, the uninitialized-state guard, and init():
// flash-area open / sector / FCB mount failures, format-on-empty, erase and re-init failures, and the FAULT abort of the flash erase.

#include "test_config_common.h"

TEST_F(ConfigStoreTestSuite, SingletonEnforcesStrictUniqueness) {
    ConfigStore& instance_a = ConfigStore::getInstance();
    ConfigStore& instance_b = ConfigStore::getInstance();
    EXPECT_EQ(&instance_a, &instance_b) << "Singleton pattern violated!";
}

TEST_F(ConfigStoreTestSuite, UninitializedStateRejectsOperations) {
    ConfigStore& config = ConfigStore::getInstance();
    EXPECT_FALSE(config.validateEndurance(ConfigKey::ALARM_THRESHOLD_BASE));
    uint16_t dummy_val = 123;
    EXPECT_FALSE(config.set(ConfigKey::ALARM_THRESHOLD_BASE, dummy_val));
    uint16_t read_val = 0;
    EXPECT_FALSE(config.get(ConfigKey::ALARM_THRESHOLD_BASE, read_val));
}

TEST_F(ConfigStoreTestSuite, RejectsReInitializationIfHardwareFails) {
    mock_flash_area_open_err = -1;
    testing::internal::CaptureStdout();
    EXPECT_FALSE(ConfigStore::getInstance().init());
    const auto raw_out = testing::internal::GetCapturedStdout();
    std::string_view output(raw_out);
    EXPECT_TRUE(output.find("[ERR] Failed to open flash area") != std::string_view::npos);
}

TEST_F(ConfigStoreTestSuite, InitFailsWhenGetSectorsFails) {
    mock_flash_area_get_sectors_err = -1;
    testing::internal::CaptureStdout();
    EXPECT_FALSE(ConfigStore::getInstance().init());
    const auto raw_out = testing::internal::GetCapturedStdout();
    std::string_view output(raw_out);
    EXPECT_TRUE(output.find("[ERR] Failed to get flash sectors") != std::string_view::npos);
}

TEST_F(ConfigStoreTestSuite, InitFailsWhenFCBInitFails) {
    mock_fcb_init_err_first = -EIO;
    testing::internal::CaptureStdout();
    EXPECT_FALSE(ConfigStore::getInstance().init());
    const auto raw_out = testing::internal::GetCapturedStdout();
    std::string_view output(raw_out);
    EXPECT_TRUE(output.find("[ERR] FCB Mount failed") != std::string_view::npos);
}

TEST_F(ConfigStoreTestSuite, InitFailsWhenEraseFails) {
    mock_fcb_init_err_first = -ENOMSG; // trigger format
    mock_flash_area_erase_err = -1;
    testing::internal::CaptureStdout();
    EXPECT_FALSE(ConfigStore::getInstance().init());
    const auto raw_out = testing::internal::GetCapturedStdout();
    std::string_view output(raw_out);
    EXPECT_TRUE(output.find("[ERR] Failed to erase storage partition") != std::string_view::npos);
}

TEST_F(ConfigStoreTestSuite, InitFailsWhenFCBReinitFails) {
    mock_fcb_init_err_first = -ENOMSG; // trigger format
    mock_fcb_init_err_second = -1;     // fail on secondary init
    testing::internal::CaptureStdout();
    EXPECT_FALSE(ConfigStore::getInstance().init());
    const auto raw_out = testing::internal::GetCapturedStdout();
    std::string_view output(raw_out);
    EXPECT_TRUE(output.find("[ERR] FCB re-init failed after formatting") != std::string_view::npos);
}

TEST_F(ConfigStoreTestSuite, FormatsFlashWhenEmpty) {
    mock_fcb_init_err_first = -ENOMSG;
    testing::internal::CaptureStdout();
    EXPECT_TRUE(ConfigStore::getInstance().init());
    const auto raw_out = testing::internal::GetCapturedStdout();
    std::string_view output(raw_out);
    EXPECT_TRUE(output.find("[WRN] Flash storage is empty. Erasing and formatting FCB sectors...") != std::string_view::npos);
}

TEST_F(ConfigStoreTestSuite, InitAbortsFlashEraseWhenSystemFaulted) {
    mock_fcb_init_err_first = -ENOMSG;
    sys_context.current_state = SystemState::FAULT;

    testing::internal::CaptureStdout();
    EXPECT_FALSE(ConfigStore::getInstance().init());
    const std::string raw_out = testing::internal::GetCapturedStdout();

    EXPECT_NE(raw_out.find("[ERR] System FAULT. Aborting flash erase to prevent corruption."), std::string::npos);
    EXPECT_FALSE(ConfigStore::getInstance().initialized);
    EXPECT_EQ(fcb_init_call_count, 1) << "FCB must not be re-initialised after an aborted erase";
}
