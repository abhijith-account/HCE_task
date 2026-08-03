#include <gtest/gtest.h>
#include <array>
#include <cstring>
#include <cstdint>
#include <cerrno>
#include <vector>
#include <map>
#include <string>
#include <string_view>
#include <cstdio>

#define private public
#define protected public
#include "Persistent_Configuration_System.h"
#undef private
#undef protected

// --- FCB / Flash Mock Environment ---
static int mock_flash_area_open_err = 0;
static int mock_flash_area_get_sectors_err = 0;
static int mock_fcb_init_err_first = 0;
static int mock_fcb_init_err_second = 0;
static int fcb_init_call_count = 0;
static int mock_flash_area_erase_err = 0;
static bool mock_flash_read_fail = false;
static bool mock_flash_write_fail = false;
static bool mock_fcb_getnext_fail = false;
static bool mock_nvs_corrupt_data = false;

static std::map<off_t, uint8_t> mock_flash_memory;
static std::vector<fcb_entry> mock_fcb_entries;
static size_t mock_fcb_read_idx = 0;
static off_t current_append_offset = 0;

extern "C" {
    int flash_area_open(uint8_t id, const struct flash_area **fa) {
        static struct flash_area dummy = {0, 4096};
        *fa = &dummy;
        return mock_flash_area_open_err;
    }

    void flash_area_close(const struct flash_area *fa) {}

    int flash_area_get_sectors(int fa_id, uint32_t *count, struct flash_sector *sectors) {
        *count = 2;
        return mock_flash_area_get_sectors_err;
    }

    int fcb_init(int f_area_id, struct fcb *fcb) {
        fcb_init_call_count++;
        if (fcb_init_call_count == 1 && mock_fcb_init_err_first != 0) return mock_fcb_init_err_first;
        if (fcb_init_call_count == 2 && mock_fcb_init_err_second != 0) return mock_fcb_init_err_second;
        return 0;
    }

    int flash_area_erase(const struct flash_area *fa, off_t off, size_t len) {
        return mock_flash_area_erase_err;
    }

    int fcb_append(struct fcb *fcb, uint16_t len, struct fcb_entry *loc) {
        if (mock_flash_write_fail) return -1;
        loc->fe_elem_off = current_append_offset;
        current_append_offset += len;
        mock_fcb_entries.push_back(*loc);
        return 0;
    }

    int fcb_append_finish(struct fcb *fcb, struct fcb_entry *append_loc) {
        return 0;
    }

    int flash_area_write(const struct flash_area *fa, off_t off, const void *src, size_t len) {
        if (mock_flash_write_fail) return -1;
        const uint8_t* ptr = static_cast<const uint8_t*>(src);
        for (size_t i = 0; i < len; ++i) {
            mock_flash_memory[off + i] = ptr[i];
        }
        return 0;
    }

    int flash_area_read(const struct flash_area *fa, off_t off, void *dst, size_t len) {
        if (mock_flash_read_fail) return -1;
        uint8_t* ptr = static_cast<uint8_t*>(dst);
        for (size_t i = 0; i < len; ++i) {
            ptr[i] = mock_flash_memory[off + i];
        }
        
        // Corrupt only the payload (uint16_t is 2 bytes), leaving headers intact
        if (mock_nvs_corrupt_data && len == sizeof(uint16_t)) {
            ptr[0] ^= 0xFF;
        }
        return 0;
    }

    int fcb_getnext(struct fcb *fcb, struct fcb_entry *loc) {
        if (mock_fcb_getnext_fail) return -1;
        if (loc->fe_sector == nullptr) {
            mock_fcb_read_idx = 0;
        }
        if (mock_fcb_read_idx < mock_fcb_entries.size()) {
            *loc = mock_fcb_entries[mock_fcb_read_idx++];
            loc->fe_sector = (void*)0x1234; // Prevent infinite re-init
            return 0;
        }
        return -1;
    }
}

// --- Test Suite ---
class ConfigStoreTestSuite : public ::testing::Test {
protected:
    void SetUp() override {
        mock_flash_memory.clear();
        mock_fcb_entries.clear();
        mock_fcb_read_idx = 0;
        current_append_offset = 0;

        mock_flash_area_open_err = 0;
        mock_flash_area_get_sectors_err = 0;
        mock_fcb_init_err_first = 0;
        mock_fcb_init_err_second = 0;
        fcb_init_call_count = 0;
        mock_flash_area_erase_err = 0;
        
        mock_flash_read_fail = false;
        mock_flash_write_fail = false;
        mock_fcb_getnext_fail = false;
        mock_nvs_corrupt_data = false;

        ConfigStore::getInstance().initialized = false;
    }
};

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

TEST_F(ConfigStoreTestSuite, LogFunctionsFailWhenUninitialized) {
    ConfigStore& config = ConfigStore::getInstance();
    EXPECT_EQ(config.getStoredLogCount(), 0);
    ConfigStore::LogEntry entry{};
    EXPECT_FALSE(config.getLogEntry(0, entry));
}

// NEW TEST: Covers the read-failure branch during FCB transversal 
TEST_F(ConfigStoreTestSuite, LogRetrievalHandlesFlashReadErrors) {
    ConfigStore& config = ConfigStore::getInstance();
    config.init();

    FcbRecordHeader header{};
    header.key = ConfigKey::FULL_CHARGE_LOG;
    header.length = sizeof(ConfigStore::LogEntry);
    ConfigStore::LogEntry entry1{};

    fcb_entry loc1{};
    loc1.fe_sector = (void*)1;
    fcb_append(nullptr, sizeof(header) + sizeof(ConfigStore::LogEntry), &loc1);
    flash_area_write(nullptr, loc1.fe_elem_off, &header, sizeof(header));
    flash_area_write(nullptr, loc1.fe_elem_off + sizeof(header), &entry1, sizeof(ConfigStore::LogEntry));

    // Inject read failure
    mock_flash_read_fail = true; 

    EXPECT_EQ(config.getStoredLogCount(), 0);
    ConfigStore::LogEntry read_entry{};
    EXPECT_FALSE(config.getLogEntry(0, read_entry));
}

// NEW TEST: Validates the branch that skips corrupted lengths and unrecognized keys
TEST_F(ConfigStoreTestSuite, LogRetrievalIgnoresOtherKeysAndBadLengths) {
    ConfigStore& config = ConfigStore::getInstance();
    config.init();

    // Entry 1: Wrong Key
    FcbRecordHeader h1{};
    h1.key = ConfigKey::ALARM_THRESHOLD_BASE;
    h1.length = sizeof(ConfigStore::LogEntry);
    fcb_entry loc1{};
    loc1.fe_sector = (void*)1;
    fcb_append(nullptr, sizeof(h1) + sizeof(ConfigStore::LogEntry), &loc1);
    flash_area_write(nullptr, loc1.fe_elem_off, &h1, sizeof(h1));

    // Entry 2: Correct Key, Wrong Length
    FcbRecordHeader h2{};
    h2.key = ConfigKey::FULL_CHARGE_LOG;
    h2.length = 9999;
    fcb_entry loc2{};
    loc2.fe_sector = (void*)1;
    fcb_append(nullptr, sizeof(h2) + 4, &loc2);
    flash_area_write(nullptr, loc2.fe_elem_off, &h2, sizeof(h2));

    EXPECT_EQ(config.getStoredLogCount(), 0);
    ConfigStore::LogEntry read_entry{};
    EXPECT_FALSE(config.getLogEntry(0, read_entry));
}

TEST_F(ConfigStoreTestSuite, LogRetrievalWorks) {
    ConfigStore& config = ConfigStore::getInstance();
    config.init();

    // Manually inject 2 valid log entries directly into the FCB byte structure
    FcbRecordHeader header{};
    header.key = ConfigKey::FULL_CHARGE_LOG;
    header.length = sizeof(ConfigStore::LogEntry);
    
    ConfigStore::LogEntry entry1{};
    ConfigStore::LogEntry entry2{};

    // Mocking Entry 1
    fcb_entry loc1{};
    loc1.fe_sector = (void*)1;
    fcb_append(nullptr, sizeof(header) + sizeof(ConfigStore::LogEntry), &loc1);
    flash_area_write(nullptr, loc1.fe_elem_off, &header, sizeof(header));
    flash_area_write(nullptr, loc1.fe_elem_off + sizeof(header), &entry1, sizeof(ConfigStore::LogEntry));

    // Mocking Entry 2
    fcb_entry loc2{};
    loc2.fe_sector = (void*)1;
    fcb_append(nullptr, sizeof(header) + sizeof(ConfigStore::LogEntry), &loc2);
    flash_area_write(nullptr, loc2.fe_elem_off, &header, sizeof(header));
    flash_area_write(nullptr, loc2.fe_elem_off + sizeof(header), &entry2, sizeof(ConfigStore::LogEntry));

    EXPECT_EQ(config.getStoredLogCount(), 2);

    ConfigStore::LogEntry read_entry{};
    EXPECT_TRUE(config.getLogEntry(0, read_entry));
    EXPECT_TRUE(config.getLogEntry(1, read_entry));
    
    // Out-of-bounds check
    EXPECT_FALSE(config.getLogEntry(2, read_entry));
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
