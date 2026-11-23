// test_Smart_Battery_Soc.cpp
// Tests for the state-of-charge path of Smart_Battery_System.cpp: seedOrResyncCoulombCounter() and
// updateStateAndPublish() (including the Kalman predict/update and its rest/active measurement noise).

#include "test_battery_common.h"
#include "Kalman_Filter.h"

TEST_F(SmartBatteryTestSuite, CoulombCounter_AtRestDuration) {
    battery.seedOrResyncCoulombCounter(11000, 10, false);
    EXPECT_EQ(battery.rest_period_start_ms, virtual_uptime);

    battery.seedOrResyncCoulombCounter(11000, 500, false);
    EXPECT_EQ(battery.rest_period_start_ms, 0);

    battery.soc_initialized = false;
    battery.seedOrResyncCoulombCounter(BatteryLimits::PACK_MAX_VOLTAGE_MV, 10, false);
    EXPECT_TRUE(battery.soc_initialized);

    battery.soc_initialized = false;
    battery.seedOrResyncCoulombCounter(BatteryLimits::PACK_MIN_VOLTAGE_MV, 10, false);
    EXPECT_TRUE(battery.soc_initialized);
}

TEST_F(SmartBatteryTestSuite, UpdateStateAndPublish_ClampLimits) {
    battery.soc_initialized = true;
    battery.last_poll_time_ms = virtual_uptime;

    battery.kf_soc_pct = 200.0f;
    battery.updateStateAndPublish(13000, 0, 250, 0);
    EXPECT_EQ(battery.cache.soc.value, 100);

    battery.kf_soc_pct = -50.0f;
    battery.updateStateAndPublish(8000, 0, 250, 0);
    EXPECT_EQ(battery.cache.soc.value, 0);
    
    battery.updateStateAndPublish(13000, 5000, 250, 0);
}

TEST_F(SmartBatteryTestSuite, CoulombCounter_LongRestResync) {
    const uint32_t now = virtual_uptime;
    battery.seedOrResyncCoulombCounter(11000, 0, false);
    EXPECT_EQ(battery.rest_period_start_ms, now);

    virtual_uptime += (31 * 60 * 1000);

    battery.soc_initialized = false;
    battery.seedOrResyncCoulombCounter(11000, 0, false);
    EXPECT_TRUE(battery.soc_initialized);
}

TEST_F(SmartBatteryTestSuite, CoulombCounter_AtRestBranchCombos) {
    battery.rest_period_start_ms = 12345;
    battery.seedOrResyncCoulombCounter(11000, -500, false);
    EXPECT_EQ(battery.rest_period_start_ms, 0U);

    battery.soc_initialized = false;
    battery.seedOrResyncCoulombCounter(BatteryLimits::PACK_MAX_VOLTAGE_MV, 500, false);
    EXPECT_FALSE(battery.soc_initialized);

    battery.soc_initialized = false;
    battery.seedOrResyncCoulombCounter(BatteryLimits::PACK_MIN_VOLTAGE_MV, -500, false);
    EXPECT_FALSE(battery.soc_initialized);
}

TEST_F(SmartBatteryTestSuite, UpdateStateAndPublish_FirstSeedBranch) {
    battery.soc_initialized = false;
    battery.accumulated_uAh = 0;
    battery.last_poll_time_ms = 0;

    battery.updateStateAndPublish(11000, 10, 250, 0);

    EXPECT_TRUE(battery.soc_initialized);
    EXPECT_EQ(battery.last_poll_time_ms, virtual_uptime);
}

TEST_F(SmartBatteryTestSuite, UpdateStateAndPublish_NoClampNeeded) {
    battery.soc_initialized = true;
    battery.accumulated_uAh = 1000 * 1000LL;
    battery.updateStateAndPublish(11000, 0, 250, 0);
    EXPECT_GT(battery.cache.soc.value, 0);
    EXPECT_LT(battery.cache.soc.value, 100);
}

TEST_F(SmartBatteryTestSuite, CoulombCounter_IntermediateRest) {
    battery.rest_period_start_ms = virtual_uptime;
    virtual_uptime += 1000; // 1 second elapsed (< REST_RESYNC_DURATION_MS)

    battery.seedOrResyncCoulombCounter(11000, 0, false);
    EXPECT_NE(battery.rest_period_start_ms, 0U);
}

// Line 347: Kalman measurement noise, "active" (non-rest) branch
TEST_F(SmartBatteryTestSuite, UpdateStateAndPublish_ActiveShunt_UsesActiveNoise) {
    battery.soc_initialized = true;
    battery.last_poll_time_ms = virtual_uptime;
    battery.kf_soc_pct = 50.0f;

    battery.updateStateAndPublish(11000, 500, 250, 50000);
    EXPECT_TRUE(battery.cache.valid);

    battery.updateStateAndPublish(11000, -500, 250, -50000);
    EXPECT_TRUE(battery.cache.valid);
}

TEST_F(SmartBatteryTestSuite, UpdateStateAndPublish_ReleasesMutexAndPreservesCommHealthOnDegraded) {
    battery.soc_initialized = true;
    battery.last_poll_time_ms = virtual_uptime;
    battery.last_valid_comm_time = 1000U;
    battery.consecutive_comm_failures = 1U;
    virtual_uptime = 2000U;

    // 1. Live publish returns true, updates last_valid_comm_time, and releases cache_mutex for another thread
    ASSERT_TRUE(battery.updateStateAndPublish(11000, 0, 250, 0, false, 100U, 2000));
    EXPECT_EQ(battery.last_valid_comm_time, 2000U);
    EXPECT_EQ(battery.consecutive_comm_failures, 0U);

    BmsCache worker_snap{};
    std::thread reader([this, &worker_snap]() {
        worker_snap = battery.getCacheSnapshot();
    });
    reader.join();
    EXPECT_TRUE(worker_snap.valid);

    // 2. Degraded publish returns true but must NOT reset consecutive_comm_failures or advance last_valid_comm_time
    battery.consecutive_comm_failures = 1U;
    virtual_uptime = 3000U;
    ASSERT_TRUE(battery.updateStateAndPublish(11000, 0, 250, 0, true, 80U, 2000));
    EXPECT_EQ(battery.last_valid_comm_time, 2000U);
    EXPECT_EQ(battery.consecutive_comm_failures, 1U);
}

TEST_F(SmartBatteryTestSuite, KalmanFilterGuardsZeroCapacityAndZeroDenom) {
    float soc = 50.0f;
    float p = 0.0f;

    // Covers KalmanFilter::predict() capacity_mah <= 0.0f guard
    KalmanFilter::predict(soc, p, 500.0f, 0.1f, 0.0f, 0.01f);
    EXPECT_FLOAT_EQ(soc, 50.0f);
    EXPECT_FLOAT_EQ(p, 0.0f);

    // Covers KalmanFilter::update() denom <= 0.0f guard
    KalmanFilter::update(soc, p, 80.0f, 0.0f);
    EXPECT_FLOAT_EQ(soc, 50.0f);
    EXPECT_FLOAT_EQ(p, 0.0f);
}
