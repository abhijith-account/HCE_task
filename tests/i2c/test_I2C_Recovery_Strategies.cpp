// test_I2C_Recovery_Strategies.cpp
// Tests for the recovery strategies (Fault_Tolerant_I2C_Communication_Layer.cpp): RetryStrategy, BusResetStrategy, FailSafeStrategy,
// the I2CStrategy base / virtual destructors and StaticPool allocation of strategy objects.

#include "test_i2c_common.h"
#include "I2C_Internal.h"
#include <thread>
#include <vector>

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
    i2c_internal::reset_mock_gpiob();
    strategy.executeRecovery(mock_dev);
    EXPECT_EQ(g_i2c_bus_recovery_attempts, 1);

    /* Verify STM32 GPIOB 9-clock open-drain SCL waveform + STOP condition + AF4 restoration */
    const auto& gpio = i2c_internal::g_mock_gpiob;
    EXPECT_TRUE(gpio.rcc_gpiob_clk_enabled);
    EXPECT_EQ(gpio.OTYPER & ((1U << 8) | (1U << 9)), (1U << 8) | (1U << 9));
    EXPECT_EQ(gpio.PUPDR & ((3U << 16) | (3U << 18)), (1U << 16) | (1U << 18));
    ASSERT_EQ(gpio.bsrr_count, 22U);
    EXPECT_EQ(gpio.bsrr_log[0], (1U << 8) | (1U << 9));
    for (uint32_t pulse = 0U; pulse < 9U; ++pulse) {
        EXPECT_EQ(gpio.bsrr_log[1U + (pulse * 2U)], (1U << 24)); // PB8 SCL low
        EXPECT_EQ(gpio.bsrr_log[2U + (pulse * 2U)], (1U << 8));  // PB8 SCL high
    }
    EXPECT_EQ(gpio.bsrr_log[19], (1U << 25)); // PB9 SDA low
    EXPECT_EQ(gpio.bsrr_log[20], (1U << 8));  // PB8 SCL high
    EXPECT_EQ(gpio.bsrr_log[21], (1U << 9));  // PB9 SDA high while SCL high (STOP)
    EXPECT_EQ(gpio.MODER & ((3U << 16) | (3U << 18)), (2U << 16) | (2U << 18)); // AF4 restored
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

TEST_F(I2CFaultRecoveryTestSuite, ConcurrentMultiThreadBusAndCacheStress) {
    I2CManager shared_mgr(mock_dev);
    mock_read_data[0] = 0x42;
    mock_read_data[1] = 0x12;

    std::vector<std::thread> workers;
    for (int t = 0; t < 4; ++t) {
        workers.emplace_back([&shared_mgr, t]() {
            for (int iter = 0; iter < 50; ++iter) {
                const uint8_t reg = static_cast<uint8_t>((t * 8) + (iter & 7));
                auto r8 = shared_mgr.readRegister(0x40, reg);
                EXPECT_TRUE(r8.isOk());
                auto r16 = shared_mgr.readWord(0x40, reg);
                EXPECT_TRUE(r16.isOk());
                auto w16 = shared_mgr.writeWord(0x40, reg, 0x4527U);
                EXPECT_TRUE(w16.isOk());
            }
        });
    }
    for (auto& w : workers) {
        w.join();
    }
}

extern int g_i2c_recover_bus_ret;

TEST_F(I2CFaultRecoveryTestSuite, BusResetLogsEnosysAndFailureAndSleepLockEdgeCases) {
    const struct device local_dev{};
    BusResetStrategy reset;

    // Covers I2C_Recovery_Strategies.cpp lines 121-124
    g_i2c_recover_bus_ret = -ENOSYS;
    reset.executeRecovery(&local_dev);
    g_i2c_recover_bus_ret = -EIO;
    reset.executeRecovery(&local_dev);
    g_i2c_recover_bus_ret = 0;

    // Covers I2C_Manager.cpp lines 205 & 212 (redundant lockForSleep / unlockAfterSleep)
    {
        I2CManager temp(&local_dev);
        temp.lockForSleep();
        temp.lockForSleep();      // sleep_mutex_held already true
        temp.unlockAfterSleep();
        temp.unlockAfterSleep();  // sleep_mutex_held already false

        // Covers I2C_Manager.cpp lines 90-92 (detachManager while locked_manager == mgr)
        PowerManager::getInstance().notifyBeforeSleep();
    } // ~I2CManager() calls detachManager() while locked_manager == &temp
    PowerManager::getInstance().notifyAfterWakeup();

    // Covers I2C_Manager.cpp line 101 (beforeSleep when active_manager == nullptr)
    PowerManager::getInstance().notifyBeforeSleep();
    PowerManager::getInstance().notifyAfterWakeup();
    SUCCEED();
}
