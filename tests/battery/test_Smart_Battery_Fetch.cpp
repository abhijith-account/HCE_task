// test_Smart_Battery_Fetch.cpp
// Tests for the I2C register fetch layer of Smart_Battery_System.cpp: fetch*WithRetry() (Retry_Policy.h backoff and retry
// count, watchdog feeding) and the I2C fault -> CommFault mapping.

#include "test_battery_common.h"

TEST_F(SmartBatteryTestSuite, FetchWithRetry_ExactRetryCount) {
    g_i2c_consecutive_failures = 3;
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

TEST_F(SmartBatteryTestSuite, Fetch_NullWatchdogHook_DuringRetry) {
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
