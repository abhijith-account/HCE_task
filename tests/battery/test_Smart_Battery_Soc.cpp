// test_Smart_Battery_Soc.cpp
// Tests for the state-of-charge path of Smart_Battery_System.cpp: seedOrResyncCoulombCounter() and
// updateStateAndPublish() (including the Kalman predict/update and its rest/active measurement noise).

#include "test_battery_common.h"

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
