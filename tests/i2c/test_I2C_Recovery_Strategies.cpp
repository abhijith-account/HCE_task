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
