// test_Battery_Monitor_Thread.cpp
// Tests for battery_monitor_thread() (Battery_Runtime.cpp): sleep / SAFE_HALT gating, processFSM() scheduling and the
// null-instance guards. bms_comm_thread() is only used here as setup to register the power observer.

#include "test_battery_common.h"

TEST_F(SmartBatteryTestSuite, BatteryMonitorThread_SkipsProcessWhileSleeping) {
    extern SbsBattery* smart_battery;
    auto backup = smart_battery;
    smart_battery = &battery;

    PowerManager::getInstance().notifyBeforeSleep();
    
    std::thread waker([]() {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        PowerManager::getInstance().notifyAfterWakeup();
    });
    
    test_iterations_remaining = 0;
    battery_monitor_thread();
    waker.join();

    smart_battery = backup;
}

TEST_F(SmartBatteryTestSuite, BatteryMonitorThread_NonDischargingSkipsLog) {
    extern SbsBattery* smart_battery;
    auto backup = smart_battery;
    smart_battery = &battery;

    battery.current_state.store(BatteryFSM::IDLE);
    battery.cache.valid = true;
    battery.cache.last_error = CommFault::NONE;
    battery.cache.timestamp_ms = virtual_uptime;
    battery.cache.current.value = 0;
    battery.cache.soc.value = 50;

    test_iterations_remaining = 0;
    battery_monitor_thread();
    EXPECT_EQ(battery.getState(), BatteryFSM::IDLE);

    smart_battery = backup;
}

TEST_F(SmartBatteryTestSuite, BatteryMonitorThread_DischargingSocFailureSkipsLog) {
    extern SbsBattery* smart_battery;
    auto backup = smart_battery;
    smart_battery = &battery;

    battery.current_state.store(BatteryFSM::DISCHARGING);
    battery.cache.valid = true;
    battery.cache.last_error = CommFault::NONE;
    battery.cache.current.value = -100;
    battery.cache.soc.value = 50;
    battery.cache.timestamp_ms = virtual_uptime;

    virtual_uptime += (BatteryLimits::CACHE_STALE_MS + 1000);

    test_iterations_remaining = 0;
    battery_monitor_thread();

    smart_battery = backup;
}

TEST_F(SmartBatteryTestSuite, BatteryMonitorThread_DischargingSocFailure_MutexPath) {
    extern SbsBattery* smart_battery;
    auto backup = smart_battery;
    smart_battery = &battery;

    battery.current_state.store(BatteryFSM::DISCHARGING);
    battery.cache.valid = true;
    battery.cache.last_error = CommFault::NONE;
    battery.cache.current.value = -50;
    battery.cache.soc.value = 40;
    battery.cache.timestamp_ms = virtual_uptime;

    g_mutex_lock_force_errno = -EAGAIN;
    test_iterations_remaining = 0;
    battery_monitor_thread();
    g_mutex_lock_force_errno = 0;

    smart_battery = backup;
}

TEST_F(SmartBatteryTestSuite, MonitorThread_SocFailsAloneCoversLine688) {
    extern SbsBattery* smart_battery;
    auto backup = smart_battery;
    smart_battery = &battery;

    battery.cache.valid = true;
    battery.cache.last_error = CommFault::NONE;
    battery.cache.timestamp_ms = virtual_uptime;
    battery.cache.current.value = 0;
    battery.cache.soc.value = 50;

    g_mutex_lock_target_ptr = &battery.cache_mutex;
    g_mutex_lock_target_call_counter = 0;
    g_mutex_lock_target_fail_on_call_n = 3;
    test_iterations_remaining = 0;
    battery_monitor_thread();
    g_mutex_lock_target_ptr = nullptr;
    g_mutex_lock_target_fail_on_call_n = 0;

    smart_battery = backup;
}

TEST_F(SmartBatteryTestSuite, MonitorThread_TempFailsAloneCoversLine688) {
    extern SbsBattery* smart_battery;
    auto backup = smart_battery;
    smart_battery = &battery;

    battery.cache.valid = true;
    battery.cache.last_error = CommFault::NONE;
    battery.cache.timestamp_ms = virtual_uptime;
    battery.cache.current.value = 0;
    battery.cache.soc.value = 50;

    g_mutex_lock_target_ptr = &battery.cache_mutex;
    g_mutex_lock_target_call_counter = 0;
    g_mutex_lock_target_fail_on_call_n = 4;
    test_iterations_remaining = 0;
    battery_monitor_thread();
    g_mutex_lock_target_ptr = nullptr;
    g_mutex_lock_target_fail_on_call_n = 0;

    smart_battery = backup;
}

TEST_F(SmartBatteryTestSuite, MonitorThread_TempFailsAloneCoversLine704) {
    extern SbsBattery* smart_battery;
    auto backup = smart_battery;
    smart_battery = &battery;

    battery.cache.valid = true;
    battery.cache.last_error = CommFault::NONE;
    battery.cache.timestamp_ms = virtual_uptime;
    battery.cache.current.value = 0;
    battery.cache.soc.value = 50;

    g_mutex_lock_target_ptr = &battery.cache_mutex;
    g_mutex_lock_target_call_counter = 0;
    g_mutex_lock_target_fail_on_call_n = 5; 
    
    test_iterations_remaining = 0;
    battery_monitor_thread();
    
    g_mutex_lock_target_ptr = nullptr;
    g_mutex_lock_target_fail_on_call_n = 0;
    smart_battery = backup;
}

TEST_F(SmartBatteryTestSuite, BatteryMonitorThread_SafeHaltBranch) {
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
    battery_monitor_thread();
    waker.join();

    smart_battery = backup;
}

TEST_F(SmartBatteryTestSuite, BatteryMonitorThread_SleepingButCharging) {
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
    battery_monitor_thread(); 
    waker.join();

    smart_battery = backup;
}

TEST_F(SmartBatteryTestSuite, BatteryMonitorThread_SafeHaltButInCutoff) {
    extern SbsBattery* smart_battery;
    auto backup = smart_battery;
    smart_battery = &battery;

    sys_context.requestTransition(SystemState::SAFE_HALT);
    battery.current_state.store(BatteryFSM::CUTOFF); 

    std::thread waker([]() {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        sys_context.requestTransition(SystemState::INIT);
        sys_context.requestTransition(SystemState::RUNNING);
    });

    test_iterations_remaining = 0;
    battery_monitor_thread(); 
    waker.join();

    smart_battery = backup;
}

// Lines 696 / 700: helpers evaluated with a null smart_battery (monitor thread, sleeping)
TEST_F(SmartBatteryTestSuite, BatteryMonitorThread_NullInstance_WhileSleeping) {
    extern SbsBattery* smart_battery;
    auto backup = smart_battery;

    smart_battery = &battery;           // registers the power observer
    test_iterations_remaining = 0;
    bms_comm_thread();

    smart_battery = nullptr;
    PowerManager::getInstance().notifyBeforeSleep();

    std::thread waker([]() {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        PowerManager::getInstance().notifyAfterWakeup();
    });

    test_iterations_remaining = 0;
    battery_monitor_thread();
    waker.join();

    smart_battery = backup;
}

// Line 700: isBatteryInCutoff() evaluated with a null smart_battery (SAFE_HALT wait)
TEST_F(SmartBatteryTestSuite, BatteryMonitorThread_NullInstance_WhileSafeHalt) {
    extern SbsBattery* smart_battery;
    auto backup = smart_battery;

    smart_battery = &battery;
    test_iterations_remaining = 0;
    bms_comm_thread();                  // leaves the ready semaphore available

    smart_battery = nullptr;
    PowerManager::getInstance().notifyAfterWakeup();
    sys_context.requestTransition(SystemState::SAFE_HALT);

    std::thread waker([]() {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        sys_context.requestTransition(SystemState::INIT);
        sys_context.requestTransition(SystemState::RUNNING);
    });

    test_iterations_remaining = 0;
    battery_monitor_thread();
    waker.join();

    smart_battery = backup;
}
