// test_Smart_Battery_Fetch.cpp
// Tests for the I2C register fetch layer of Smart_Battery_System.cpp: fetch*WithRetry() (Retry_Policy.h backoff and retry
// count, watchdog feeding), degraded-data metadata propagation, and the I2C fault -> CommFault mapping.

#include "test_battery_common.h"

TEST_F(SmartBatteryTestSuite, FetchWithRetry_ExactRetryCount) {
    g_i2c_consecutive_failures = 9;
    auto res = battery.fetchBusVoltageRawWithRetry();
    EXPECT_TRUE(res.success);
    EXPECT_EQ(battery.getStats().retries, 3);
}

TEST_F(SmartBatteryTestSuite, FetchCurrent_RetryAndFail) {
    g_i2c_force_errno = -EIO;
    auto res = battery.fetchCurrentRawWithRetry();
    EXPECT_FALSE(res.success);
    EXPECT_EQ(res.error, CommFault::I2C_NACK);
    g_i2c_force_errno = 0;
}

TEST_F(SmartBatteryTestSuite, I2CFaultMapping_AllBranches) {
    auto test_map = [this](int errno_val) {
        g_i2c_force_errno = errno_val;
        auto res = battery.fetchBusVoltageRawWithRetry();
        g_i2c_force_errno = 0;
        return res.error;
    };

    EXPECT_EQ(test_map(0), CommFault::NONE);
    EXPECT_EQ(test_map(-ENODEV), CommFault::DEVICE_NOT_READY);
    EXPECT_EQ(test_map(-ETIMEDOUT), CommFault::I2C_TIMEOUT);
    EXPECT_EQ(test_map(-EBUSY), CommFault::I2C_BUS_BUSY);
    EXPECT_EQ(test_map(-EAGAIN), CommFault::I2C_ARBITRATION_LOST);
    EXPECT_EQ(test_map(-EIO), CommFault::I2C_NACK);
    EXPECT_EQ(test_map(-EPERM), CommFault::I2C_NACK);
}

TEST_F(SmartBatteryTestSuite, Fetch_NullWatchdogHookDuringRetry) {
    battery.setWatchdogFeedHook(nullptr);

    g_i2c_force_errno = -EIO;
    battery.fetchBusVoltageRawWithRetry();
    battery.fetchCurrentRawWithRetry();
    battery.fetchShuntVoltageRawWithRetry();
    g_i2c_force_errno = 0;
}

TEST_F(SmartBatteryTestSuite, MapI2CFault_AllBranches_Direct) {
    extern CommFault test_mapI2CFault(I2CFault fault);

    EXPECT_EQ(test_mapI2CFault(I2CFault::NACK), CommFault::I2C_NACK);
    EXPECT_EQ(test_mapI2CFault(I2CFault::TIMEOUT), CommFault::I2C_TIMEOUT);
    EXPECT_EQ(test_mapI2CFault(I2CFault::BUS_BUSY), CommFault::I2C_BUS_BUSY);
    EXPECT_EQ(test_mapI2CFault(I2CFault::ARBITRATION_LOST), CommFault::I2C_ARBITRATION_LOST);
    EXPECT_EQ(test_mapI2CFault(I2CFault::DEVICE_NOT_READY), CommFault::DEVICE_NOT_READY);

    EXPECT_EQ(test_mapI2CFault(static_cast<I2CFault>(99)), CommFault::I2C_NACK);
}

TEST_F(SmartBatteryTestSuite, FetchShunt_RetryAndFail) {
    g_i2c_force_errno = -EIO;
    auto res = battery.fetchShuntVoltageRawWithRetry();
    EXPECT_FALSE(res.success);
    EXPECT_EQ(res.error, CommFault::I2C_NACK);
    g_i2c_force_errno = 0;
}

TEST_F(SmartBatteryTestSuite, DegradedI2CFallbackPropagatesMetadataThroughDriverAndBmsCache) {
    extern void resetI2CCacheForTests();
    resetI2CCacheForTests();

    // 1. Seed I2C fallback cache with a healthy live poll
    g_i2c_mock_v_val = 10000;
    g_i2c_mock_i_val = 0;
    g_adc_mock_mv_val = 1650;
    battery.pollHardwareAndUpdateCache();

    auto live_v = battery.getVoltage();
    ASSERT_TRUE(live_v.success);
    EXPECT_FALSE(live_v.isDegraded());
    EXPECT_EQ(live_v.reliability_score, 100U);

    // 2. Force transient I2C failures across all 3 registers so I2CManager serves degraded cache entries
    g_i2c_consecutive_failures = 30;
    battery.pollHardwareAndUpdateCache();
    g_i2c_consecutive_failures = 0;

    auto deg_v = battery.getVoltage();
    auto deg_i = battery.getCurrent();
    auto deg_shunt = battery.getShuntVoltage();
    auto deg_soc = battery.getStateOfCharge();

    ASSERT_TRUE(deg_v.success);
    EXPECT_TRUE(deg_v.isDegraded());
    EXPECT_EQ(deg_v.reliability_score, 90U);
    EXPECT_TRUE(deg_i.isDegraded());
    EXPECT_TRUE(deg_shunt.isDegraded());
    EXPECT_TRUE(deg_soc.isDegraded());
}
