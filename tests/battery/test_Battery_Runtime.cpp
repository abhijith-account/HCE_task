// test_Battery_Runtime.cpp
// Tests for Battery_Runtime.cpp that span both threads or the module as a whole: null-instance guards, the
// getSmartBatteryInstance() singleton and the combined sleep / SAFE_HALT loop conditions of bms_comm_thread and
// battery_monitor_thread. Per-thread tests are in test_Bms_Comm_Thread.cpp and test_Battery_Monitor_Thread.cpp.

#include "test_battery_common.h"

TEST_F(SmartBatteryTestSuite, Threads_NullGuardsAndSkips) {
    extern SbsBattery* smart_battery;
    auto backup = smart_battery;

    smart_battery = nullptr;
    
    test_iterations_remaining = 1;
    bms_comm_thread(); 
    
    test_iterations_remaining = 0;
    battery_monitor_thread();
    
    smart_battery = backup;

    test_iterations_remaining = 0;
    PowerManager::getInstance().notifyBeforeSleep();
    
    std::thread waker([]() {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        PowerManager::getInstance().notifyAfterWakeup();
    });
    
    bms_comm_thread();
    waker.join();

#if defined(IPOWER_OBSERVER_HAS_SLEEP_ABORTED) || 1
    PowerManager::getInstance().notifySleepAborted();
#endif
}

TEST_F(SmartBatteryTestSuite, ThreadExecution_And_Singletons) {
    extern SbsBattery* smart_battery;
    auto backup = smart_battery;
    smart_battery = &battery;

    EXPECT_EQ(getSmartBatteryInstance(), &battery);

    test_iterations_remaining = 1;
    bms_comm_thread();

    PowerManager::getInstance().notifyBeforeSleep();
    PowerManager::getInstance().notifyAfterWakeup();

    battery.cache.current.value = -500;
    battery.cache.soc.value = 60;
    battery.cache.timestamp_ms = virtual_uptime;
    battery.cache.valid = true;
    test_iterations_remaining = 1;
    battery_monitor_thread();

    battery.current_state.store(BatteryFSM::DISCHARGING);
    battery.cache.valid = false;
    test_iterations_remaining = 1;
    battery_monitor_thread();

    smart_battery = backup;
}

TEST_F(SmartBatteryTestSuite, ThreadLoops_SleepAwakeBranch_Robust) {
    extern SbsBattery* smart_battery;
    auto backup = smart_battery;
    smart_battery = &battery;

    PowerManager::getInstance().notifyAfterWakeup();

    battery.current_state.store(BatteryFSM::IDLE);
    battery.cache.valid = true;
    battery.cache.last_error = CommFault::NONE;
    battery.cache.timestamp_ms = virtual_uptime;
    battery.cache.current.value = 0;
    battery.cache.soc.value = 50;

    test_iterations_remaining = 0;
    bms_comm_thread();
    test_iterations_remaining = 0;
    battery_monitor_thread();

    PowerManager::getInstance().notifyBeforeSleep();
    
    std::thread waker1([]() {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        PowerManager::getInstance().notifyAfterWakeup();
    });
    test_iterations_remaining = 0;
    bms_comm_thread();
    waker1.join();

    PowerManager::getInstance().notifyBeforeSleep();
    
    std::thread waker2([]() {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        PowerManager::getInstance().notifyAfterWakeup();
    });
    test_iterations_remaining = 0;
    battery_monitor_thread();
    waker2.join();

    smart_battery = backup;
}

TEST_F(SmartBatteryTestSuite, ThreadLoops_FullBooleanConditionCoverage) {
    extern SbsBattery* smart_battery;
    auto backup = smart_battery;
    smart_battery = &battery;

    // Make sure the observer is registered and the system is awake/running.
    PowerManager::getInstance().notifyAfterWakeup();
    sys_context.requestTransition(SystemState::RUNNING);
    test_iterations_remaining = 0;
    bms_comm_thread();

    // The pre-init loop in bms_comm_thread blocks while sleeping or SAFE_HALT, so sleep /
    // SAFE_HALT must be raised *during init()* (via the i2c_write hooks), i.e. after that
    // loop has been passed and before the main do/while loop.

    // 1. Sleeping = true, Charging = true -> (!isBatteryCharging()) false, no waiting
    battery.current_state.store(BatteryFSM::CHARGING);
    g_i2c_call_counter = 0;
    g_sleep_on_i2c_call = 1;
    test_iterations_remaining = 1;
    bms_comm_thread();
    g_sleep_on_i2c_call = -1;

    test_iterations_remaining = 1;
    battery_monitor_thread();          // sleeping + charging: does not block

    // 2. SAFE_HALT = true, Cutoff = true -> (!isBatteryInCutoff()) false, no waiting
    PowerManager::getInstance().notifyAfterWakeup();
    battery.current_state.store(BatteryFSM::CUTOFF);
    g_i2c_call_counter = 0;
    g_set_safe_halt_on_i2c_call = 1;
    test_iterations_remaining = 1;
    bms_comm_thread();
    g_set_safe_halt_on_i2c_call = -1;

    test_iterations_remaining = 1;
    battery_monitor_thread();          // SAFE_HALT + cutoff: does not block

    sys_context.requestTransition(SystemState::INIT);
    sys_context.requestTransition(SystemState::RUNNING);

    // 3. Sleeping = true, Charging = false -> loop body executes until state becomes CHARGING
    PowerManager::getInstance().notifyAfterWakeup();
    battery.current_state.store(BatteryFSM::IDLE);
    g_i2c_call_counter = 0;
    g_sleep_on_i2c_call = 1;

    std::thread waker([this]() {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        battery.current_state.store(BatteryFSM::CHARGING);   // exits the wait loop
    });

    test_iterations_remaining = 1;
    bms_comm_thread();
    waker.join();
    g_sleep_on_i2c_call = -1;

    PowerManager::getInstance().notifyAfterWakeup();   // leave the observer awake for later tests
    smart_battery = backup;
}
