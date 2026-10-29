// test_Smart_Battery_FSM.cpp
// Tests for SbsBattery::processFSM() (Smart_Battery_System.cpp): CUTOFF / re-enable / full-charge logging,
// the cache-failure branches and EMA/hysteresis classification (Fsm_classifier.cpp, only reachable through processFSM).

#include "test_battery_common.h"

TEST_F(SmartBatteryTestSuite, ProcessFSM_AllBranches) {
    battery.cache.voltage.value = 10000;
    battery.cache.soc.value = 50;

    battery.cache.voltage.value = 0; battery.processFSM(); battery.cache.voltage.value = 10000;

    battery.cache.shunt_voltage.value = 0;
    battery.current_state.store(BatteryFSM::CHARGING);
    battery.processFSM();
    EXPECT_EQ(battery.getState(), BatteryFSM::IDLE);

    battery.full_charge_logged = true;
    battery.cache.soc.value = 90;
    battery.processFSM();
    EXPECT_FALSE(battery.full_charge_logged);

    DeviceContext* backup = battery.sys_context;
    battery.sys_context = nullptr;
    battery.current_state.store(BatteryFSM::CUTOFF);
    battery.cache.soc.value = 90;
    battery.processFSM();
    battery.sys_context = backup;
}

TEST_F(SmartBatteryTestSuite, FSM_DeepCoverage) {
    battery.cache.valid = false;
    battery.processFSM();

    battery.cache.valid = true;
    battery.cache.voltage.value = 10000;
    battery.cache.soc.value = 100;
    battery.full_charge_logged.store(false);
    battery.processFSM();
    EXPECT_TRUE(battery.full_charge_logged);

    battery.processFSM();
    EXPECT_TRUE(battery.full_charge_logged);

    battery.cache.soc.value = 50;

    battery.cache.voltage.value = 0; battery.processFSM(); battery.cache.voltage.value = 10000;

    battery.cache.shunt_voltage.value = 50000; 
    battery.current_state.store(BatteryFSM::IDLE);
    battery.processFSM();
    EXPECT_EQ(battery.getState(), BatteryFSM::CHARGING);

    battery.cache.voltage.value = 0; battery.processFSM(); battery.cache.voltage.value = 10000;

    battery.cache.shunt_voltage.value = 0;
    battery.current_state.store(BatteryFSM::CHARGING);
    battery.processFSM();
    EXPECT_EQ(battery.getState(), BatteryFSM::IDLE);

    battery.cache.voltage.value = 0; battery.processFSM(); battery.cache.voltage.value = 10000;

    battery.cache.shunt_voltage.value = -50000;
    battery.current_state.store(BatteryFSM::IDLE);
    battery.processFSM();
    EXPECT_EQ(battery.getState(), BatteryFSM::DISCHARGING);

    battery.cache.soc.value = BatteryLimits::CUTOFF_SOC_PCT - 1;
    battery.current_state.store(BatteryFSM::DISCHARGING);
    battery.processFSM();
    EXPECT_EQ(battery.getState(), BatteryFSM::CUTOFF);

    battery.cache.soc.value = BatteryLimits::CUTOFF_SOC_PCT - 1;
    battery.processFSM();

    battery.cache.soc.value = BatteryLimits::REENABLE_SOC_PCT + 1;
    sys_context.requestTransition(SystemState::SAFE_HALT);
    battery.processFSM();

    battery.current_state.store(BatteryFSM::CUTOFF);
    battery.cache.soc.value = BatteryLimits::REENABLE_SOC_PCT + 1;
    sys_context.requestTransition(SystemState::RUNNING);
    battery.processFSM();

    battery.cache.voltage.value = 0; battery.processFSM(); battery.cache.voltage.value = 10000;

    battery.cache.shunt_voltage.value = 0;
    battery.cache.soc.value = BatteryLimits::REENABLE_SOC_PCT + 1;
    sys_context.requestTransition(SystemState::SAFE_HALT);
    battery.current_state.store(BatteryFSM::CUTOFF);
    battery.processFSM();
    EXPECT_EQ(battery.getState(), BatteryFSM::IDLE);
}

TEST_F(SmartBatteryTestSuite, ProcessFSM_MidBandsAndChargingRecovery) {
    battery.cache.voltage.value = 10000;
    battery.cache.valid = true;
    battery.cache.last_error = CommFault::NONE;
    battery.cache.timestamp_ms = virtual_uptime;

    battery.cache.current.value = 0;
    battery.cache.soc.value = (BatteryLimits::CUTOFF_SOC_PCT + BatteryLimits::REENABLE_SOC_PCT) / 2;
    battery.current_state.store(BatteryFSM::IDLE);
    battery.processFSM();
    EXPECT_EQ(battery.getState(), BatteryFSM::IDLE);

    battery.cache.soc.value = 97;
    battery.full_charge_logged.store(true);
    battery.processFSM();
    EXPECT_TRUE(battery.full_charge_logged);

    battery.cache.current.value = 100;
    battery.cache.soc.value = BatteryLimits::REENABLE_SOC_PCT + 1;
    battery.current_state.store(BatteryFSM::CUTOFF);
    battery.cache.shunt_voltage.value = 50000;
    sys_context.requestTransition(SystemState::SAFE_HALT);
    battery.processFSM();
    EXPECT_EQ(battery.getState(), BatteryFSM::CHARGING);
}

TEST_F(SmartBatteryTestSuite, ProcessFSM_CutoffTrigger_NullContextSkipsFaultCall) {
    DeviceContext* backup_ctx = battery.sys_context;
    battery.sys_context = nullptr;

    battery.cache.voltage.value = 10000;
    battery.cache.valid = true;
    battery.cache.last_error = CommFault::NONE;
    battery.cache.timestamp_ms = virtual_uptime;
    battery.cache.current.value = -100;
    battery.cache.soc.value = BatteryLimits::CUTOFF_SOC_PCT - 1;
    battery.current_state.store(BatteryFSM::DISCHARGING);

    battery.processFSM();
    EXPECT_EQ(battery.getState(), BatteryFSM::CUTOFF);

    battery.sys_context = backup_ctx;
}

TEST_F(SmartBatteryTestSuite, ProcessFSM_ReenableCheck_NullContextSkipsRecovery) {
    DeviceContext* backup_ctx = battery.sys_context;
    battery.sys_context = nullptr;

    battery.cache.voltage.value = 10000;
    battery.cache.valid = true;
    battery.cache.last_error = CommFault::NONE;
    battery.cache.timestamp_ms = virtual_uptime;
    battery.cache.current.value = 0;
    battery.cache.soc.value = BatteryLimits::REENABLE_SOC_PCT + 1;
    battery.current_state.store(BatteryFSM::CUTOFF);

    battery.processFSM();

    EXPECT_EQ(battery.getState(), BatteryFSM::CUTOFF);

    battery.sys_context = backup_ctx;
}

TEST_F(SmartBatteryTestSuite, ProcessFSM_CacheFailureReason_BothBranches) {
    testing::internal::CaptureStdout();

    battery.cache.valid = false;
    battery.cache.last_error = CommFault::I2C_NACK;
    battery.processFSM();

    battery.cache.valid = false;
    battery.cache.last_error = CommFault::NONE;
    battery.processFSM();

    testing::internal::GetCapturedStdout();
}

TEST_F(SmartBatteryTestSuite, ProcessFSM_Reenable_SysContextNonNull_WrongState_Direct) {
    battery.cache.voltage.value = 10000;
    battery.cache.valid = true;
    battery.cache.last_error = CommFault::NONE;
    battery.cache.timestamp_ms = virtual_uptime;
    battery.cache.current.value = 0;
    battery.cache.soc.value = BatteryLimits::REENABLE_SOC_PCT + 1;
    battery.current_state.store(BatteryFSM::CUTOFF);

    sys_context.requestTransition(SystemState::INIT);
    battery.processFSM();
    EXPECT_EQ(battery.getState(), BatteryFSM::CUTOFF);
}

TEST_F(SmartBatteryTestSuite, FullChargeLog_NvsWriteFailureCoversLine550) {
    extern int g_nvs_write_force_errno;

    ConfigStore::getInstance().initialized = true;

    battery.cache.valid = true;
    battery.cache.last_error = CommFault::NONE;
    battery.cache.timestamp_ms = virtual_uptime;
    battery.cache.soc.value = 100;
    battery.full_charge_logged.store(false);

    g_nvs_write_force_errno = -EIO;
    battery.processFSM();
    g_nvs_write_force_errno = 0;

    battery.full_charge_logged.store(false);
    battery.processFSM();
}

TEST_F(SmartBatteryTestSuite, FullChargeLog_ConfigStoreUninitializedCoversLine550) {
    const bool backup_initialized = ConfigStore::getInstance().initialized;
    ConfigStore::getInstance().initialized = false;

    battery.cache.valid = true;
    battery.cache.last_error = CommFault::NONE;
    battery.cache.timestamp_ms = virtual_uptime;
    battery.cache.soc.value = 100;
    battery.full_charge_logged.store(false);

    battery.processFSM();

    ConfigStore::getInstance().initialized = backup_initialized;
}

TEST_F(SmartBatteryTestSuite, ProcessFSM_FullChargeAndRecoveryBranches) {
    // NOTE: cache.voltage must be >= MIN_VALID_VOLTAGE_MV, otherwise processFSM()
    // takes the "battery disconnected" early exit and forces the state to IDLE.
    battery.cache.valid = true;
    battery.cache.last_error = CommFault::NONE;
    battery.cache.timestamp_ms = virtual_uptime;
    battery.cache.voltage.value = 10000;
    battery.cache.shunt_voltage.value = 0;

    // 1. Full Charge compare_exchange_strong failure branch (full_charge_logged already true)
    battery.cache.soc.value = 100;
    battery.full_charge_logged.store(true);
    battery.processFSM();

    // 2. Deadband coverage: 95% <= SOC < 100% (SOC = 97%)
    battery.cache.soc.value = 97;
    battery.processFSM();

    // 3. Recovery Ternary: CUTOFF -> CHARGING (shunt_uv < -IDLE_SHUNT_UV_THRESHOLD)
    battery.current_state.store(BatteryFSM::CUTOFF);
    battery.cache.soc.value = BatteryLimits::REENABLE_SOC_PCT + 5;
    battery.cache.shunt_voltage.value = 50000;
    sys_context.requestTransition(SystemState::SAFE_HALT);

    battery.processFSM();
    EXPECT_EQ(battery.getState(), BatteryFSM::CHARGING);

    // 4. Recovery Ternary: CUTOFF -> IDLE (shunt_uv >= -IDLE_SHUNT_UV_THRESHOLD)
    // Recovery moved the system SAFE_HALT -> INIT; go back to RUNNING before halting again.
    sys_context.requestTransition(SystemState::RUNNING);
    battery.current_state.store(BatteryFSM::CUTOFF);
    battery.cache.soc.value = BatteryLimits::REENABLE_SOC_PCT + 5;
    battery.cache.shunt_voltage.value = 0;
    sys_context.requestTransition(SystemState::SAFE_HALT);

    battery.processFSM();
    EXPECT_EQ(battery.getState(), BatteryFSM::IDLE);
}

TEST_F(SmartBatteryTestSuite, ProcessFSM_HysteresisTransitions) {
    // The FSM classifies direction from an EMA of the shunt voltage that persists between
    // calls, so every case first resets the EMA (voltage below MIN_VALID -> resetFsmClassifier)
    // and then feeds one large sample so the EMA equals that sample.
    auto run_case = [this](BatteryFSM start, int32_t shunt_uv) {
        battery.cache.valid = true;
        battery.cache.last_error = CommFault::NONE;
        battery.cache.timestamp_ms = virtual_uptime;
        battery.cache.soc.value = 50;

        battery.cache.voltage.value = 0;        // forces EMA reset
        battery.processFSM();

        battery.cache.voltage.value = 10000;
        battery.cache.shunt_voltage.value = shunt_uv;
        battery.current_state.store(start);
        battery.processFSM();
        return battery.getState();
    };
    constexpr int32_t BIG = 100000;

    EXPECT_EQ(run_case(BatteryFSM::CHARGING,      BIG), BatteryFSM::CHARGING);
    EXPECT_EQ(run_case(BatteryFSM::CHARGING,     -BIG), BatteryFSM::DISCHARGING);
    EXPECT_EQ(run_case(BatteryFSM::CHARGING,        0), BatteryFSM::IDLE);
    EXPECT_EQ(run_case(BatteryFSM::DISCHARGING,  -BIG), BatteryFSM::DISCHARGING);
    EXPECT_EQ(run_case(BatteryFSM::DISCHARGING,   BIG), BatteryFSM::CHARGING);
    EXPECT_EQ(run_case(BatteryFSM::DISCHARGING,     0), BatteryFSM::IDLE);
}

// Line 643: "active_state == CUTOFF" can only be true if another thread stores CUTOFF between
// the store at line 641 and the reload at line 642, so a racing thread is used.
// (Probabilistic: needs >= 2 cores. Alternative: change the source to test next_fsm_state.)
TEST_F(SmartBatteryTestSuite, ProcessFSM_ActiveStateReload_ConcurrentCutoff) {
    battery.cache.valid = true;
    battery.cache.last_error = CommFault::NONE;
    battery.cache.timestamp_ms = virtual_uptime;
    battery.cache.voltage.value = 10000;
    battery.cache.shunt_voltage.value = 0;
    battery.cache.soc.value = 50;
    battery.current_state.store(BatteryFSM::IDLE);

    testing::internal::CaptureStdout();
    std::atomic<bool> stop{false};
    std::thread racer([this, &stop]() {
        while (!stop.load(std::memory_order_relaxed)) {
            battery.current_state.store(BatteryFSM::CUTOFF);
            battery.current_state.store(static_cast<BatteryFSM>(0xFF));  // matches none of the 4 enumerators
            battery.current_state.store(BatteryFSM::IDLE);
        }
    });

    const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(3000);
    while (std::chrono::steady_clock::now() < deadline) {
        for (int i = 0; i < 1000; ++i) battery.processFSM();
    }

    stop.store(true, std::memory_order_relaxed);
    racer.join();
    testing::internal::GetCapturedStdout();
}
