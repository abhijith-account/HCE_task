// test_Bms_Comm_Thread.cpp
// Tests for bms_comm_thread() (Battery_Runtime.cpp): sensor init retry and escalation, sleep / SAFE_HALT gating,
// the BMS power observer it registers, and the status-log branches when getters fail.

#include "test_battery_common.h"

TEST_F(SmartBatteryTestSuite, BmsCommThread_InitRetryLog) {
    extern SbsBattery* smart_battery;
    auto backup = smart_battery;
    smart_battery = &battery;

    g_i2c_force_errno = -EIO;
    test_iterations_remaining = 1;
    bms_comm_thread();
    g_i2c_force_errno = 0;

    smart_battery = backup;
}

TEST_F(SmartBatteryTestSuite, PowerObserver_SleepAborted) {
    extern SbsBattery* smart_battery;
    auto backup = smart_battery;
    smart_battery = &battery;

    test_iterations_remaining = 1;
    bms_comm_thread();

    PowerManager::getInstance().notifySleepAborted();

    smart_battery = backup;
}

TEST_F(SmartBatteryTestSuite, BmsCommThread_SkipsPollWhileSleeping) {
    extern SbsBattery* smart_battery;
    auto backup = smart_battery;
    smart_battery = &battery;

    test_iterations_remaining = 0;
    bms_comm_thread();

    PowerManager::getInstance().notifyBeforeSleep();
    
    std::thread waker([]() {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        PowerManager::getInstance().notifyAfterWakeup();
    });
    
    test_iterations_remaining = 0;
    bms_comm_thread();
    waker.join();

    smart_battery = backup;
}

TEST_F(SmartBatteryTestSuite, PowerObserver_AfterWakeup_NullSmartBattery) {
    extern SbsBattery* smart_battery;
    auto backup = smart_battery;

    smart_battery = &battery;
    test_iterations_remaining = 0;
    bms_comm_thread();

    smart_battery = nullptr;
    PowerManager::getInstance().notifyBeforeSleep();
    PowerManager::getInstance().notifyAfterWakeup();

    smart_battery = backup;
}

TEST_F(SmartBatteryTestSuite, BmsCommThread_SleepBranch_OrderIndependent) {
    extern SbsBattery* smart_battery;
    auto backup = smart_battery;
    smart_battery = &battery;

    test_iterations_remaining = 0;
    bms_comm_thread();
    PowerManager::getInstance().notifyAfterWakeup();

    g_i2c_call_counter = 0;
    test_iterations_remaining = 0;
    bms_comm_thread();
    EXPECT_GT(g_i2c_call_counter, 0);

    PowerManager::getInstance().notifyBeforeSleep();
    g_i2c_call_counter = 0;
    
    std::thread waker([]() {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        PowerManager::getInstance().notifyAfterWakeup();
    });
    
    test_iterations_remaining = 0;
    bms_comm_thread();
    waker.join();
    
    EXPECT_GT(g_i2c_call_counter, 0);

    smart_battery = backup;
}

TEST_F(SmartBatteryTestSuite, BmsCommThread_SleepBranch_Diagnostic) {
    extern SbsBattery* smart_battery;
    extern bool isBmsObserverSleepingForTest();
    extern void resetI2CCacheForTests();
    auto backup = smart_battery;
    smart_battery = &battery;

    PowerManager::getInstance().resetForTest();
    resetI2CCacheForTests();

    test_iterations_remaining = 0;
    bms_comm_thread();
    ASSERT_FALSE(isBmsObserverSleepingForTest());

    g_i2c_call_counter = 0;
    test_iterations_remaining = 0;
    bms_comm_thread();
    const int awake_calls = g_i2c_call_counter;
    ASSERT_GT(awake_calls, 0);

    PowerManager::getInstance().notifyBeforeSleep();
    ASSERT_TRUE(isBmsObserverSleepingForTest());

    g_i2c_call_counter = 0;
    
    std::thread waker([]() {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        PowerManager::getInstance().notifyAfterWakeup();
    });

    test_iterations_remaining = 0;
    bms_comm_thread();
    waker.join();
    
    const int asleep_calls = g_i2c_call_counter;
    EXPECT_EQ(asleep_calls, awake_calls);

    smart_battery = backup;
}

TEST_F(SmartBatteryTestSuite, BmsCommThread_SafeHaltBranch) {
    extern SbsBattery* smart_battery;
    auto backup = smart_battery;
    smart_battery = &battery;

    sys_context.requestTransition(SystemState::SAFE_HALT);
    battery.current_state.store(BatteryFSM::IDLE);

    std::thread waker([]() {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        sys_context.requestTransition(SystemState::INIT);
        sys_context.requestTransition(SystemState::RUNNING);
    });

    test_iterations_remaining = 0;
    bms_comm_thread();
    waker.join();

    smart_battery = backup;
}

TEST_F(SmartBatteryTestSuite, BmsCommThread_SleepingButCharging) {
    extern SbsBattery* smart_battery;
    auto backup = smart_battery;
    smart_battery = &battery;

    PowerManager::getInstance().notifyBeforeSleep();
    battery.current_state.store(BatteryFSM::CHARGING);
    sys_context.requestTransition(SystemState::RUNNING);

    std::thread waker([]() {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        PowerManager::getInstance().notifyAfterWakeup();
    });

    test_iterations_remaining = 0;
    bms_comm_thread();
    waker.join();

    smart_battery = backup;
}

TEST_F(SmartBatteryTestSuite, BmsCommThread_SafeHaltButInCutoff) {
    extern SbsBattery* smart_battery;
    auto backup = smart_battery;
    smart_battery = &battery;

    sys_context.requestTransition(SystemState::SAFE_HALT);
    battery.current_state.store(BatteryFSM::CUTOFF); 
    PowerManager::getInstance().notifyAfterWakeup();

    std::thread waker([]() {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        sys_context.requestTransition(SystemState::INIT);
        sys_context.requestTransition(SystemState::RUNNING);
    });

    test_iterations_remaining = 0;
    bms_comm_thread(); 
    waker.join();

    smart_battery = backup;
}

TEST_F(SmartBatteryTestSuite, BmsCommThread_GettersFail_CoversElseLog) {
    extern SbsBattery* smart_battery;
    auto backup = smart_battery;
    smart_battery = &battery;

    sys_context.requestTransition(SystemState::RUNNING);
    PowerManager::getInstance().notifyAfterWakeup();
    battery.current_state.store(BatteryFSM::IDLE);
    
    g_mutex_lock_target_ptr = &battery.cache_mutex;
    g_mutex_lock_target_call_counter = 0;
    g_mutex_lock_target_fail_on_call_n = 4;

    test_iterations_remaining = 0;
    bms_comm_thread(); 
    
    g_mutex_lock_target_ptr = nullptr;
    g_mutex_lock_target_fail_on_call_n = 0;
    smart_battery = backup;
}

TEST_F(SmartBatteryTestSuite, BmsCommThread_SafeHalt_InitLoop) {
    extern SbsBattery* smart_battery;
    auto backup = smart_battery;
    smart_battery = &battery;

    sys_context.requestTransition(SystemState::SAFE_HALT);
    PowerManager::getInstance().notifyAfterWakeup(); 

    std::thread waker([]() {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        sys_context.requestTransition(SystemState::INIT);
        sys_context.requestTransition(SystemState::RUNNING);
    });

    test_iterations_remaining = 0;
    bms_comm_thread(); 
    waker.join();

    smart_battery = backup;
}

TEST_F(SmartBatteryTestSuite, BmsCommThread_InitFailure_AlreadySafeHalt)
{
    extern SbsBattery* smart_battery;
    auto backup = smart_battery;
    smart_battery = &battery;

    sys_context.requestTransition(SystemState::RUNNING);

    g_i2c_force_errno = -EIO;
    g_set_safe_halt_on_i2c_call = 5;
    test_iterations_remaining = 1;

    bms_comm_thread();

    g_set_safe_halt_on_i2c_call = -1;
    g_i2c_force_errno = 0;
    smart_battery = backup;
}

TEST_F(SmartBatteryTestSuite, BmsCommThread_InitFailure_Escalation) {
    extern SbsBattery* smart_battery;
    auto backup = smart_battery;
    smart_battery = &battery;

    sys_context.requestTransition(SystemState::RUNNING);

    // Force INA226 init to fail across multiple retries until fault escalation
    g_i2c_force_errno = -EIO;
    test_iterations_remaining = 5;

    bms_comm_thread();

    g_i2c_force_errno = 0;
    smart_battery = backup;
}

TEST_F(SmartBatteryTestSuite, BmsCommThread_GetterFailures_AllBranches) {
    extern SbsBattery* smart_battery;
    auto backup = smart_battery;
    smart_battery = &battery;

    sys_context.requestTransition(SystemState::RUNNING);
    PowerManager::getInstance().notifyAfterWakeup();
    battery.current_state.store(BatteryFSM::IDLE);

    // cache_mutex lock order: 1=wakeup, 2=poll snapshot, 3=publish, 4=v, 5=i, 6=soc, 7=temp, 8=cap
    for (int fail_call = 1; fail_call <= 8; ++fail_call) {
        battery.cache.valid = true;
        battery.cache.last_error = CommFault::NONE;
        battery.cache.timestamp_ms = virtual_uptime;

        g_mutex_lock_target_ptr = &battery.cache_mutex;
        g_mutex_lock_target_call_counter = 0;
        g_mutex_lock_target_fail_on_call_n = fail_call;

        test_iterations_remaining = 1;
        bms_comm_thread();

        g_mutex_lock_target_ptr = nullptr;
        g_mutex_lock_target_fail_on_call_n = 0;
    }

    smart_battery = backup;
}

// Lines 804-807: comm-thread wait loop, all condition combinations.
// bms_comm_thread runs on a worker thread; the test thread changes the conditions while it
// is cycling (the thread cannot be hooked between init() and the loop from the mocks).
TEST_F(SmartBatteryTestSuite, BmsCommThread_MainLoopWaitConditions_ThreadDriven) {
    extern SbsBattery* smart_battery;
    auto backup = smart_battery;
    smart_battery = &battery;

    PowerManager::getInstance().notifyAfterWakeup();
    battery.current_state.store(BatteryFSM::IDLE);

    testing::internal::CaptureStdout();   // the worker logs on every cycle
    std::atomic<bool> finished{false};
    test_iterations_remaining = 1000000000;
    std::thread worker([&finished]() {
        bms_comm_thread();
        finished.store(true);
    });

    auto settle = []() { std::this_thread::sleep_for(std::chrono::milliseconds(25)); };

    settle();
    PowerManager::getInstance().notifyBeforeSleep();             // sleeping, not charging -> wait loop body
    settle();
    battery.current_state.store(BatteryFSM::CHARGING);           // sleeping + charging -> leaves loop
    settle();
    PowerManager::getInstance().notifyAfterWakeup();
    battery.current_state.store(BatteryFSM::IDLE);
    settle();
    sys_context.requestTransition(SystemState::SAFE_HALT);       // SAFE_HALT, not cutoff -> wait loop body
    settle();
    battery.current_state.store(BatteryFSM::CUTOFF);             // SAFE_HALT + cutoff -> leaves loop
    settle();
    sys_context.requestTransition(SystemState::INIT);
    sys_context.requestTransition(SystemState::RUNNING);
    settle();

    while (!finished.load()) {
        test_iterations_remaining = 0;
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    worker.join();
    testing::internal::GetCapturedStdout();

    smart_battery = backup;
}

// Line 782: SAFE_HALT already active when init retries are exhausted.
// SAFE_HALT is toggled by a second thread while bms_comm_thread repeatedly runs its init-failure path.
TEST_F(SmartBatteryTestSuite, BmsCommThread_InitEscalation_SafeHaltRaisedConcurrently) {
    extern SbsBattery* smart_battery;
    auto backup = smart_battery;
    smart_battery = &battery;

    PowerManager::getInstance().notifyAfterWakeup();
    g_i2c_force_errno = -EIO;

    testing::internal::CaptureStdout();
    std::atomic<bool> stop{false};
    std::thread toggler([&stop]() {
        while (!stop.load()) {
            sys_context.requestTransition(SystemState::SAFE_HALT);
            for (int i = 0; i < 20; ++i) std::this_thread::yield();
            sys_context.requestTransition(SystemState::INIT);
            sys_context.requestTransition(SystemState::RUNNING);
            for (int i = 0; i < 20; ++i) std::this_thread::yield();
        }
    });

    for (int run = 0; run < 300; ++run) {
        test_iterations_remaining = 0;
        bms_comm_thread();
    }

    stop.store(true);
    toggler.join();
    testing::internal::GetCapturedStdout();

    g_i2c_force_errno = 0;
    smart_battery = backup;
}
