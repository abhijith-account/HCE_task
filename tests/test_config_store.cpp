#include <gtest/gtest.h>
#include <array>
#include <cstring>
#include <cstdint>
#include <cerrno>
#include <sys/types.h>
#include <string>
#include <string_view>
#include <cstdio>
// White-box testing bypass to easily reset the Singleton's internal state
#define private public
#define protected public
#include "Persistent_Configuration_System.h"
#undef private
#undef protected

// ==============================================================================
// Zephyr NVS & Hardware Mocks
// ==============================================================================
struct MockNVSEntry {
    uint16_t id;
    uint8_t data[16];
    size_t len;
    bool active;
};

// Increased to 50 to ensure we have room for all seeds and endurance tests
static std::array<MockNVSEntry, 50> mock_nvs_storage;
static bool mock_device_ready = true;
static bool mock_nvs_mount_fail = false;
static bool mock_nvs_read_fail = false;
static bool mock_nvs_write_fail = false;
static bool mock_nvs_corrupt_data = false;

extern "C" {
    bool device_is_ready(const struct device *dev) {
        return mock_device_ready;
    }
    
    int nvs_mount(struct nvs_fs *fs) {
        if (mock_nvs_mount_fail) return -ENODEV;
        return 0;
    }
    
    ssize_t nvs_read(struct nvs_fs *fs, uint16_t id, void *data, size_t len) {
        if (mock_nvs_read_fail) return -EIO;
        
        for (const auto& entry : mock_nvs_storage) {
            if (entry.active && entry.id == id) {
                size_t copy_len = (len < entry.len) ? len : entry.len;
                std::memcpy(data, entry.data, copy_len);
                
                if (mock_nvs_corrupt_data && copy_len > 0) {
                    ((uint8_t*)data)[0] ^= 0xFF; // Flip bits to corrupt
                }
                return copy_len;
            }
        }
        return -ENOENT;
    }
    
    ssize_t nvs_write(struct nvs_fs *fs, uint16_t id, const void *data, size_t len) {
        if (mock_nvs_write_fail) return -EIO;
        if (len > 16) return -EINVAL;
        
        for (auto& entry : mock_nvs_storage) {
            if (entry.active && entry.id == id) {
                std::memcpy(entry.data, data, len);
                entry.len = len;
                return len;
            }
        }
        
        for (auto& entry : mock_nvs_storage) {
            if (!entry.active) {
                entry.active = true;
                entry.id = id;
                std::memcpy(entry.data, data, len);
                entry.len = len;
                return len;
            }
        }
        return -ENOSPC;
    }
}

// ==============================================================================
// Test Suite Setup
// ==============================================================================
class ConfigStoreTestSuite : public ::testing::Test {
protected:
    void SetUp() override {
        // Purge mock NVS memory
        for (auto& entry : mock_nvs_storage) {
            entry.active = false;
        }
        mock_device_ready = true;
        mock_nvs_mount_fail = false;
        mock_nvs_write_fail = false;
        mock_nvs_read_fail = false;
        mock_nvs_corrupt_data = false;
        
        // Reset Singleton State using whitebox access
        ConfigStore::getInstance().initialized = false;
    }
};

// ==============================================================================
// Core Lifecycle & Uninitialized States
// ==============================================================================
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
    mock_device_ready = false;
    testing::internal::CaptureStdout();
    EXPECT_FALSE(ConfigStore::getInstance().init());
    const auto raw_out = testing::internal::GetCapturedStdout();
    std::string_view output(raw_out);
    EXPECT_TRUE(output.find("[ERR] Flash device not ready") != std::string_view::npos);
}

TEST_F(ConfigStoreTestSuite, InitFailsWhenNVSMountFails) {
    mock_nvs_mount_fail = true;
    testing::internal::CaptureStdout();
    EXPECT_FALSE(ConfigStore::getInstance().init());
    const auto raw_out = testing::internal::GetCapturedStdout();
    std::string_view output(raw_out);
    EXPECT_TRUE(output.find("[ERR] NVS Mount failed:") != std::string_view::npos);
}

// ==============================================================================
// Seeding & Provisioning Branch Coverage
// ==============================================================================
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
    mock_nvs_write_fail = true; // Force failures during the seed loops
    
    testing::internal::CaptureStdout();
    ASSERT_TRUE(config.init()); // Init succeeds, but seeds will log warnings
    const auto raw_out = testing::internal::GetCapturedStdout();
    std::string_view output(raw_out);
    
    EXPECT_TRUE(output.find("[WRN] Failed to provision default Device ID") != std::string_view::npos);
    EXPECT_TRUE(output.find("[WRN] Failed to seed default alarm threshold") != std::string_view::npos);
}

TEST_F(ConfigStoreTestSuite, SeedingSkipsExistingValues) {
    ConfigStore& config = ConfigStore::getInstance();
    
    // Pre-populate specific slot data
    config.initialized = true; 
    
    // FIX: Wrap the [[nodiscard]] setup calls in EXPECT_TRUE
    EXPECT_TRUE(config.setDeviceId(1, 9999));
    EXPECT_TRUE(config.setAlarmThreshold(1, 88));
    
    config.initialized = false; 

    testing::internal::CaptureStdout();
    ASSERT_TRUE(config.init());
    const auto raw_out = testing::internal::GetCapturedStdout();
    std::string_view output(raw_out);

    uint32_t devId;
    // FIX: Wrap the [[nodiscard]] retrieval call in EXPECT_TRUE
    EXPECT_TRUE(config.getDeviceId(1, devId));
    EXPECT_EQ(devId, 9999); // Was not overwritten

    uint8_t alarm;
    // FIX: Wrap the [[nodiscard]] retrieval call in EXPECT_TRUE
    EXPECT_TRUE(config.getAlarmThreshold(1, alarm));
    EXPECT_EQ(alarm, 88); // Was not overwritten
    
    // Ensure we did not log a provision event for slot 1
    EXPECT_TRUE(output.find("Provisioned slot 1") == std::string_view::npos);
}

TEST_F(ConfigStoreTestSuite, SeedingOverwritesUnprovisionedIdSentinel) {
    ConfigStore& config = ConfigStore::getInstance();
    config.initialized = true;
    
    // FIX: Wrap the [[nodiscard]] setup call in EXPECT_TRUE
    EXPECT_TRUE(config.setDeviceId(2, InfusionDeviceConfig::UnprovisionedId)); // Explicitly zeroed
    
    config.initialized = false;
    
    config.init(); // Should detect sentinel and overwrite with Default
    
    uint32_t devId = 0;
    // FIX: Wrap the [[nodiscard]] retrieval call in EXPECT_TRUE
    EXPECT_TRUE(config.getDeviceId(2, devId));
    EXPECT_EQ(devId, InfusionDeviceConfig::DefaultDeviceIds[1]); // Reverted to default
}

// ==============================================================================
// Functional Limits & Lookups
// ==============================================================================
TEST_F(ConfigStoreTestSuite, FindSlotByDeviceId) {
    ConfigStore& config = ConfigStore::getInstance();
    config.init(); // Seeds 1001 to 1005

    uint8_t slot;
    EXPECT_TRUE(config.findSlotByDeviceId(1003, slot));
    EXPECT_EQ(slot, 3);

    EXPECT_FALSE(config.findSlotByDeviceId(9999, slot)); // Doesn't exist
}

TEST_F(ConfigStoreTestSuite, InfusionRateBoundsCheck) {
    ConfigStore& config = ConfigStore::getInstance();
    config.init();

    EXPECT_FALSE(config.setInfusionRate(0, 50)); // invalid slot
    EXPECT_FALSE(config.setInfusionRate(6, 50)); // invalid slot
    EXPECT_FALSE(config.setInfusionRate(1, 0));  // limit too low
    EXPECT_FALSE(config.setInfusionRate(1, 101)); // limit too high

    // Valid bounds
    EXPECT_TRUE(config.setInfusionRate(1, 50));
    uint8_t rate;
    EXPECT_TRUE(config.getInfusionRate(1, rate));
    EXPECT_EQ(rate, 50);

    // Invalid Get
    uint8_t out;
    EXPECT_FALSE(config.getInfusionRate(0, out)); 
}

TEST_F(ConfigStoreTestSuite, AlarmThresholdBoundsCheck) {
    ConfigStore& config = ConfigStore::getInstance();
    config.init();

    EXPECT_FALSE(config.setAlarmThreshold(0, 90)); // invalid slot
    EXPECT_FALSE(config.setAlarmThreshold(6, 90)); // invalid slot
    EXPECT_FALSE(config.setAlarmThreshold(1, 79)); // limit too low
    EXPECT_FALSE(config.setAlarmThreshold(1, 101)); // limit too high

    // Valid bounds
    EXPECT_TRUE(config.setAlarmThreshold(1, 85));
    uint8_t out;
    EXPECT_FALSE(config.getAlarmThreshold(0, out)); // invalid get
}

TEST_F(ConfigStoreTestSuite, DeviceIdBoundsCheck) {
    ConfigStore& config = ConfigStore::getInstance();
    config.init();

    EXPECT_FALSE(config.setDeviceId(0, 123)); // invalid slot
    uint32_t out;
    EXPECT_FALSE(config.getDeviceId(0, out)); // invalid get
}

TEST_F(ConfigStoreTestSuite, GenericGetFailsForUnknownKeys) {
    ConfigStore& config = ConfigStore::getInstance();
    config.init();
    uint32_t unknown_val = 0;
    EXPECT_FALSE(config.get(static_cast<ConfigKey>(9999), unknown_val));
}

// ==============================================================================
// Endurance Validation FSM
// ==============================================================================
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

TEST_F(ConfigStoreTestSuite, ValidateEnduranceFailsOnWriteError) {
    ConfigStore& config = ConfigStore::getInstance();
    config.init();

    mock_nvs_write_fail = true;
    testing::internal::CaptureStdout();
    EXPECT_FALSE(config.validateEndurance(ConfigKey::FULL_CHARGE_LOG));
    const auto raw_out = testing::internal::GetCapturedStdout();
    std::string_view output(raw_out);
    
    EXPECT_TRUE(output.find("[ERR] Endurance Test failed to write at cycle 0") != std::string_view::npos);
}

TEST_F(ConfigStoreTestSuite, ValidateEnduranceFailsOnReadError) {
    ConfigStore& config = ConfigStore::getInstance();
    config.init();
    
    mock_nvs_read_fail = true;
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
