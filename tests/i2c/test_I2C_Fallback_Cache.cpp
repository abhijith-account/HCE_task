// test_I2C_Fallback_Cache.cpp
// Tests for the last-known-good fallback cache (Fault_Tolerant_I2C_Communication_Layer.cpp): serving cached data on a bus error, expiry,
// reliability decay, calibration, sample-count and key handling, pool exhaustion, and the state-gated "Fallback Rejected" logging.

#include "test_i2c_common.h"

TEST_F(I2CFaultRecoveryTestSuite, CachedReadRegister) {
    I2CManager manager(mock_dev);
    mock_read_data[0] = 77;
    auto first = manager.readRegister(0x10,0x20);
    EXPECT_TRUE(first.success);

    mock_i2c_err_code = -ETIMEDOUT;
    auto second = manager.readRegister(0x10,0x20);
    EXPECT_TRUE(second.success);
    EXPECT_EQ(second.value,77);
}

TEST_F(I2CFaultRecoveryTestSuite, Read24BitUsesCache) {
    I2CManager mgr(mock_dev);
    mock_i2c_err_code = 0;
    EXPECT_TRUE(mgr.read24Bit(1,2).success);

    mock_i2c_err_code = -ETIMEDOUT;
    auto r = mgr.read24Bit(1,2);
    EXPECT_TRUE(r.success);
}

TEST_F(I2CFaultRecoveryTestSuite, Read64BitUsesCache) {
    I2CManager mgr(mock_dev);
    EXPECT_TRUE(mgr.read64Bit(1,2).success);

    mock_i2c_err_code = -ETIMEDOUT;
    auto r = mgr.read64Bit(1,2);
    EXPECT_TRUE(r.success);
}

TEST_F(I2CFaultRecoveryTestSuite, CacheFullLogsError) {
    I2CManager mgr(mock_dev);
    mock_i2c_err_code = 0;

    // Fill all 64 available cache slots
    for(int i = 0; i < 64; i++) {
        mgr.readRegister(1, i);
    }

    // The 65th unique read will trigger the full cache log
    testing::internal::CaptureStdout();
    mgr.readRegister(1, 99);
    auto output = testing::internal::GetCapturedStdout();
    EXPECT_NE(output.find("cache table full"), std::string::npos);
}

TEST_F(I2CFaultRecoveryTestSuite, CacheExpired) {
    REQUIRE_SYSTEM_RUNNING();
    I2CManager mgr(mock_dev);
    mock_read_data[0] = 15;
    mgr.readRegister(1,2);

    virtual_uptime += 5000;
    mock_i2c_err_code = -EIO;

    testing::internal::CaptureStdout();
    auto r = mgr.readRegister(1,2);
    const auto raw_out = testing::internal::GetCapturedStdout();
    std::string_view out(raw_out);
    EXPECT_NE(out.find("Fallback Rejected: Data Stale"), std::string_view::npos);
    EXPECT_FALSE(r.success);
}

TEST_F(I2CFaultRecoveryTestSuite, ReliabilityDecay) {
    REQUIRE_SYSTEM_RUNNING();
    I2CManager mgr(mock_dev);
    mock_read_data[0] = 88;
    mgr.readRegister(1,2);

    for(int i = 0; i < 6; i++) {
        mock_i2c_err_code = -EIO;
        mgr.readRegister(1,2);
    }

    testing::internal::CaptureStdout();
    auto r = mgr.readRegister(1,2);
    const auto raw_out = testing::internal::GetCapturedStdout();
    std::string_view out(raw_out);
    EXPECT_NE(out.find("Fallback Rejected: Reliability Score too low"), std::string_view::npos);
    EXPECT_FALSE(r.success);
}

TEST_F(I2CFaultRecoveryTestSuite, CacheUpdatedMultipleTimes) {
    I2CManager mgr(mock_dev);

    mock_read_data[0] = 10;
    mgr.readRegister(1,2);

    mock_read_data[0] = 20;
    mgr.readRegister(1,2);

    mock_i2c_err_code = -EIO;
    auto r = mgr.readRegister(1,2);
    EXPECT_TRUE(r.success);
    EXPECT_EQ(r.value,20);
}

TEST_F(I2CFaultRecoveryTestSuite, ReadWordUsesCache) {
    I2CManager mgr(mock_dev);
    mock_read_data[0] = 0x34; mock_read_data[1] = 0x12;
    EXPECT_TRUE(mgr.readWord(1, 2).success);

    mock_i2c_err_code = -ETIMEDOUT;
    auto r = mgr.readWord(1, 2);
    EXPECT_TRUE(r.success);
    EXPECT_EQ(r.value, 0x1234);
}

TEST_F(I2CFaultRecoveryTestSuite, CachePoolExhausted) {

    exhaustI2CCachePool();

    I2CManager mgr(mock_dev);
    mock_i2c_err_code = 0;

    auto r = mgr.readRegister(1, 2);
    EXPECT_TRUE(r.success);

    resetI2CCachePool();
}

TEST_F(I2CFaultRecoveryTestSuite, CacheUncalibratedRejection) {
    REQUIRE_SYSTEM_RUNNING();
    uint32_t key = (1U << 8) | 2;
    test_update_cache(key, 123, false);

    I2CManager mgr(mock_dev);
    mock_i2c_err_code = -EIO;

    testing::internal::CaptureStdout();
    auto r = mgr.readRegister(1, 2);
    const auto raw_out = testing::internal::GetCapturedStdout();
    std::string_view out(raw_out);
    EXPECT_NE(out.find("Fallback Rejected: Data is uncalibrated."), std::string_view::npos);
    EXPECT_FALSE(r.success);
}

TEST_F(I2CFaultRecoveryTestSuite, CacheSampleCountOverflow) {
    uint32_t key = 0x1234;

    test_update_cache(key, 100, true);

}

TEST_F(I2CFaultRecoveryTestSuite, CacheReliabilityScoreRejectsLowScore)
{
    resetI2CCacheForTests();

    uint32_t key = 0x5555;

    test_update_cache(key, 10, true);
    test_set_reliability(key, 10);

    uint64_t val = 0;

    EXPECT_FALSE(test_get_cached_value(key, &val, 1000));
}

TEST_F(I2CFaultRecoveryTestSuite, SampleCountMaxBranch)
{
    uint32_t key = 0x1111;

    test_update_cache(key, 55, true);

    test_set_sample_count(key, UINT32_MAX);

    test_update_cache(key, 66, true);

    uint64_t value=0;
    EXPECT_TRUE(test_get_cached_value(key,&value,1000));
    EXPECT_EQ(value,66);
}

TEST_F(I2CFaultRecoveryTestSuite, CachedValueWrongKeyBranch)
{
    resetI2CCacheForTests();

    test_update_cache(0x1111,44,true);

    uint64_t out=0;

    EXPECT_FALSE(test_get_cached_value(0x2222,&out,1000));
}

TEST_F(I2CFaultRecoveryTestSuite, ReliabilityShimKeyNotFound)
{
    testing::internal::CaptureStderr();

    test_set_reliability(0xDEADBEEF,10);

    const auto raw_err = testing::internal::GetCapturedStderr();
    std::string_view err(raw_err);

    EXPECT_NE(err.find("Shim: Key"),
              std::string_view::npos);
}

TEST_F(I2CFaultRecoveryTestSuite, ReliabilityWrongKeyBranch)
{
    resetI2CCacheForTests();

    test_update_cache(0x1111, 10, true);

    testing::internal::CaptureStderr();

    test_set_reliability(0x2222, 50);

    const auto raw_err = testing::internal::GetCapturedStderr();
    std::string_view err(raw_err);

    EXPECT_NE(err.find("Shim: Key"),
              std::string_view::npos);
}

TEST_F(I2CFaultRecoveryTestSuite, SampleCountWrongKey)
{
    resetI2CCacheForTests();

    test_update_cache(0x1111, 99, true);

    test_set_sample_count(0x2222, 5);

    uint64_t value = 0;

    EXPECT_TRUE(test_get_cached_value(0x1111,
                                      &value,
                                      1000));

    EXPECT_EQ(value, 99);
}

TEST_F(I2CFaultRecoveryTestSuite, SampleCountEmptyCache)
{
    resetI2CCacheForTests();

    EXPECT_NO_FATAL_FAILURE(
        test_set_sample_count(0x1234, 10));
}

TEST_F(I2CFaultRecoveryTestSuite, GetCachedValueNullPointer) {
    uint64_t val = 0;

    EXPECT_FALSE(test_get_cached_value(0x1234, nullptr, 1000));

    test_update_cache(0x1234, 100, true);
    EXPECT_TRUE(test_get_cached_value(0x1234, &val, 1000));
}

TEST_F(I2CFaultRecoveryTestSuite, ResetCachePoolWithNullBlocks) {

    EXPECT_NO_FATAL_FAILURE(resetI2CCachePool());
}

TEST_F(I2CFaultRecoveryTestSuite, CacheRejectionSilentWhenNotRunning) {
    const uint32_t key = (7U << 8) | 9U;
    uint64_t val = 0;

    /* INIT is a legal exit from RUNNING and is "not RUNNING". */
    ASSERT_TRUE(sys_context.requestTransition(SystemState::INIT));
    ASSERT_NE(sys_context.getState(), SystemState::RUNNING);

    /* 1. Stale entry, state != RUNNING. */
    test_update_cache(key, 0x11, true);
    virtual_uptime += 5000;
    testing::internal::CaptureStdout();
    EXPECT_FALSE(test_get_cached_value(key, &val, 1000));
    auto out_stale = testing::internal::GetCapturedStdout();
    EXPECT_EQ(std::string_view(out_stale).find("Fallback Rejected"), std::string_view::npos);

    /* 2. Reliability too low, state != RUNNING. */
    test_update_cache(key, 0x22, true);          // fresh timestamp, score back to MAX
    test_set_reliability(key, 10);
    testing::internal::CaptureStdout();
    EXPECT_FALSE(test_get_cached_value(key, &val, 1000));
    auto out_rel = testing::internal::GetCapturedStdout();
    EXPECT_EQ(std::string_view(out_rel).find("Fallback Rejected"), std::string_view::npos);

    /* 3. Uncalibrated, state != RUNNING. */
    test_update_cache(key, 0x33, false);
    testing::internal::CaptureStdout();
    EXPECT_FALSE(test_get_cached_value(key, &val, 1000));
    auto out_cal = testing::internal::GetCapturedStdout();
    EXPECT_EQ(std::string_view(out_cal).find("Fallback Rejected"), std::string_view::npos);

    walkBackToRunning();
    ASSERT_EQ(sys_context.getState(), SystemState::RUNNING);
}
