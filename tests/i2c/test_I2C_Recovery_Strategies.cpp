// test_I2C_Recovery_Strategies.cpp
// Tests for the recovery strategies (Fault_Tolerant_I2C_Communication_Layer.cpp): RetryStrategy, BusResetStrategy, FailSafeStrategy,
// the I2CStrategy base / virtual destructors and StaticPool allocation of strategy objects. None of these touch I2CManager.

#include "test_i2c_common.h"

TEST_F(I2CFaultRecoveryTestSuite, FailSafeFallback) {
    FailSafeStrategy failsafe;
    EXPECT_EQ(failsafe.getLastGood(), 0);
    failsafe.updateLastGood(120);
    failsafe.executeRecovery(mock_dev);
    EXPECT_EQ(failsafe.getLastGood(), 120);
}

TEST_F(I2CFaultRecoveryTestSuite, StrategySwitchMidStream) {
    RetryStrategy retry;
    BusResetStrategy hard_reset;
    FailSafeStrategy failsafe;

    failsafe.updateLastGood(99);
    I2CStrategy* active_strategy = nullptr;

    active_strategy = &retry;
    EXPECT_NO_FATAL_FAILURE(active_strategy->executeRecovery(mock_dev));

    active_strategy = &hard_reset;
    EXPECT_NO_FATAL_FAILURE(active_strategy->executeRecovery(mock_dev));

    active_strategy = &failsafe;
    EXPECT_NO_FATAL_FAILURE(active_strategy->executeRecovery(mock_dev));

    auto* casted_failsafe = dynamic_cast<FailSafeStrategy*>(active_strategy);
    ASSERT_NE(casted_failsafe, nullptr);
    EXPECT_EQ(casted_failsafe->getLastGood(), 99);
}

TEST_F(I2CFaultRecoveryTestSuite, VirtualDestructorCoverage){
    StaticPool<RetryStrategy,1> strategy_pool;
    void* raw_memory = strategy_pool.allocate();
    ASSERT_NE(raw_memory, nullptr);
    RetryStrategy* strategy = new(raw_memory) RetryStrategy();
    strategy->~RetryStrategy();
    strategy_pool.deallocate(raw_memory);
}

TEST_F(I2CFaultRecoveryTestSuite, DeallocateNullPointerIsSafe){
    StaticPool<RetryStrategy,1> strategy_pool;
    EXPECT_NO_FATAL_FAILURE(strategy_pool.deallocate(nullptr));
}

TEST_F(I2CFaultRecoveryTestSuite, RetryStrategyLogsWarning){
    RetryStrategy strategy;
    testing::internal::CaptureStdout();
    strategy.executeRecovery(mock_dev);
    const auto raw_output = testing::internal::GetCapturedStdout();
    std::string_view output(raw_output);
    EXPECT_TRUE(output.find("[WRN] I2C Fault Detected. Executing Exponential Backoff Retry...")!= std::string_view::npos);
}

TEST_F(I2CFaultRecoveryTestSuite, BusRestStrategyLogsError){
    BusResetStrategy strategy;
    testing::internal::CaptureStdout();
    strategy.executeRecovery(mock_dev);
    const auto raw_output = testing::internal::GetCapturedStdout();
    std::string_view output(raw_output);
    EXPECT_TRUE(output.find("[ERR] I2C Bus Hard-Locked. Toggling SCL to recover")!=std::string_view::npos);
}

TEST_F(I2CFaultRecoveryTestSuite, FailSafeStrategyLogsErrorWithValue){
    FailSafeStrategy strategy;
    uint8_t dummy_last_good = 188;
    strategy.updateLastGood(dummy_last_good);
    testing::internal::CaptureStdout();
    strategy.executeRecovery(mock_dev);
    const auto raw_output = testing::internal::GetCapturedStdout();
    std::string_view output(raw_output);
    EXPECT_TRUE(output.find("Falling back to Last-known-Good data: 188")!= std::string_view::npos);
}

TEST_F(I2CFaultRecoveryTestSuite, BaseDestructor) {
    StaticPool<RetryStrategy,1> pool;
    void* mem = pool.allocate();
    I2CStrategy* ptr = new(mem) RetryStrategy;
    ptr->~I2CStrategy();
    pool.deallocate(mem);
}

TEST_F(I2CFaultRecoveryTestSuite, ExponentialBackoffDoublesAndCapsAtMax) {
    RetryStrategy strategy;
    EXPECT_EQ(strategy.getBackoffMs(), 10U);
    strategy.executeRecovery(mock_dev);
    EXPECT_EQ(strategy.getBackoffMs(), 20U);
    strategy.executeRecovery(mock_dev);
    EXPECT_EQ(strategy.getBackoffMs(), 40U);
    strategy.executeRecovery(mock_dev);
    EXPECT_EQ(strategy.getBackoffMs(), 80U);
    strategy.executeRecovery(mock_dev);
    EXPECT_EQ(strategy.getBackoffMs(), 80U);
    strategy.resetBackoff();
    EXPECT_EQ(strategy.getBackoffMs(), 10U);
}

TEST_F(I2CFaultRecoveryTestSuite, BusResetStrategyInvokesHardwareRecoverBus) {
    BusResetStrategy strategy;
    g_i2c_bus_recovery_attempts = 0;
    strategy.executeRecovery(mock_dev);
    EXPECT_EQ(g_i2c_bus_recovery_attempts, 1);
}

TEST_F(I2CFaultRecoveryTestSuite, TransientFaultRecoversOnRetryAndCacheMarksDegraded) {
    I2CManager manager(mock_dev);
    virtual_uptime = 1234U;
    mock_read_data[0] = 0x5A;
    mock_i2c_transient_failures = 2; // Fails attempt 0 (+10ms) & attempt 1 (+20ms), succeeds on attempt 2 at t=1264

    auto live_res = manager.readRegister(0x10, 0x20);
    EXPECT_TRUE(live_res.isOk());
    EXPECT_FALSE(live_res.isDegraded());
    EXPECT_EQ(live_res.unwrap(), 0x5A);
    EXPECT_EQ(live_res.timestamp_ms, 1264);
    EXPECT_EQ(live_res.reliability_score, 100U);

    // Force permanent bus lockup: 3 attempts call i2c_recover_bus, then fallback returns Degraded
    g_i2c_bus_recovery_attempts = 0;
    mock_i2c_err_code = -ETIMEDOUT;
    auto deg_res = manager.readRegister(0x10, 0x20);
    EXPECT_TRUE(deg_res.isOk());
    EXPECT_TRUE(deg_res.isDegraded());
    EXPECT_EQ(deg_res.error, I2CFault::TIMEOUT);
    EXPECT_EQ(deg_res.unwrap(), 0x5A);
    EXPECT_EQ(deg_res.timestamp_ms, 1264);
    EXPECT_EQ(deg_res.reliability_score, 90U);
    EXPECT_EQ(g_i2c_bus_recovery_attempts, 3);
}
