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

TEST_F(SmartBatteryTestSuite, DegradedFirstPollDoesNotSeedUninitializedSoc) {
    extern void resetI2CCacheForTests();
    extern void test_update_cache(uint32_t key, uint64_t value, bool calibrated);
    resetI2CCacheForTests();

    virtual_uptime = 1000U;
    // Pre-populate the I2C fallback cache directly (simulating another consumer or prior run)
    // while SbsBattery itself has soc_initialized == false.
    const uint32_t v_key = (static_cast<uint32_t>(INA226::I2C_ADDR) << 8U) | INA226::REG_BUS_VOLT;
    const uint32_t s_key = (static_cast<uint32_t>(INA226::I2C_ADDR) << 8U) | INA226::REG_SHUNT_VOLT;
    const uint32_t c_key = (static_cast<uint32_t>(INA226::I2C_ADDR) << 8U) | INA226::REG_CURRENT;
    test_update_cache(v_key, 0x1027U, true); // 10000 raw big-endian -> 12500 mV
    test_update_cache(s_key, 0x0000U, true);
    test_update_cache(c_key, 0x0000U, true);

    battery.soc_initialized = false;
    battery.cache.valid = false;
    g_adc_mock_mv_val = 1650;

    virtual_uptime = 1500U;
    g_i2c_consecutive_failures = 30;
    battery.pollHardwareAndUpdateCache();
    g_i2c_consecutive_failures = 0;

    EXPECT_FALSE(battery.soc_initialized);
    EXPECT_FALSE(battery.cache.valid);
    EXPECT_EQ(battery.cache.last_error, CommFault::CACHE_INVALID);
}

TEST_F(SmartBatteryTestSuite, DegradedAggregationUsesMinimumReliabilityAndOldestTimestamp) {
    extern void resetI2CCacheForTests();
    extern void test_set_reliability(uint32_t key, uint8_t score);
    resetI2CCacheForTests();

    // 1. Live poll at t = 2000 ms seeds soc_initialized and populates I2C cache at t = 2000 ms
    virtual_uptime = 2000U;
    g_i2c_mock_v_val = 10000;
    g_i2c_mock_i_val = 0;
    g_adc_mock_mv_val = 1650;
    battery.pollHardwareAndUpdateCache();

    // Lower bus-voltage reliability below shunt/current (70 -> decays to 60 on fallback,
    // while shunt & current stay at 100 -> 90).
    const uint32_t v_key = (static_cast<uint32_t>(INA226::I2C_ADDR) << 8U) | INA226::REG_BUS_VOLT;
    test_set_reliability(v_key, 70U);

    // 2. Advance uptime to t = 2500 ms and force degraded fallback across all 3 registers
    virtual_uptime = 2500U;
    g_i2c_consecutive_failures = 30;
    battery.pollHardwareAndUpdateCache();
    g_i2c_consecutive_failures = 0;

    auto deg_v = battery.getVoltage();
    ASSERT_TRUE(deg_v.success);
    EXPECT_TRUE(deg_v.isDegraded());
    EXPECT_EQ(deg_v.reliability_score, 60U);  // Minimum across V (60), Shunt (90), Current (90)
    EXPECT_EQ(deg_v.timestamp_ms, 2000);      // Original cached timestamp, not 2500 ms publication time
}

TEST_F(SmartBatteryTestSuite, ConcurrentPollAndGettersStress) {
    g_i2c_mock_v_val = 10000;
    g_i2c_mock_i_val = 0;
    g_adc_mock_mv_val = 1650;
    battery.pollHardwareAndUpdateCache();

    std::thread poller([this]() {
        for (int i = 0; i < 40; ++i) {
            battery.pollHardwareAndUpdateCache();
        }
    });

    std::thread readers[3];
    for (int r = 0; r < 3; ++r) {
        readers[r] = std::thread([this]() {
            for (int i = 0; i < 40; ++i) {
                auto v = battery.getVoltage();
                auto c = battery.getCurrent();
                auto s = battery.getStateOfCharge();
                EXPECT_TRUE(v.success);
                EXPECT_TRUE(c.success);
                EXPECT_TRUE(s.success);
            }
        });
    }

    poller.join();
    for (int r = 0; r < 3; ++r) {
        readers[r].join();
    }
}
