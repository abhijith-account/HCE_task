#include "test_power_common.h"

TEST_F(PowerManagementTestSuite, StopEntryRtcTimeNormalization) {
    PowerManager& pm = PowerManager::getInstance();
    pm.init(dummy_rtc, dummy_i2c, dummy_uart, dummy_usb, dummy_adc, &sys_context);

    mocks.rtc_time_val.tm_sec = 0;
    mocks.rtc_time_val.tm_min = 59;
    mocks.rtc_time_val.tm_hour = 23;

    virtual_uptime = 30000;
    pm.processFSM(); 

    virtual_uptime = 35000;
    pm.processFSM(); 

    EXPECT_EQ(std::string_view(pm.current_state->getName()), std::string_view("PM_STATE_SUSPEND_TO_RAM"));
}

TEST_F(PowerManagementTestSuite, StopEntryRtcTimeSecNoOverflow) {
    PowerManager& pm = PowerManager::getInstance();
    pm.init(dummy_rtc, dummy_i2c, dummy_uart, dummy_usb, dummy_adc, &sys_context);
    mocks.rtc_time_val.tm_sec = -60; 

    virtual_uptime = 30000;
    pm.processFSM(); 
    virtual_uptime = 35000;
    pm.processFSM(); 
    EXPECT_EQ(std::string_view(pm.current_state->getName()), std::string_view("PM_STATE_SUSPEND_TO_RAM"));
}

TEST_F(PowerManagementTestSuite, StopEntryRtcTimeHourNoOverflow) {
    PowerManager& pm = PowerManager::getInstance();
    pm.init(dummy_rtc, dummy_i2c, dummy_uart, dummy_usb, dummy_adc, &sys_context);
    mocks.rtc_time_val.tm_sec = 0;
    mocks.rtc_time_val.tm_min = 59;
    mocks.rtc_time_val.tm_hour = 10;

    virtual_uptime = 30000;
    pm.processFSM(); 
    virtual_uptime = 35000;
    pm.processFSM(); 
    EXPECT_EQ(std::string_view(pm.current_state->getName()), std::string_view("PM_STATE_SUSPEND_TO_RAM"));
}

TEST_F(PowerManagementTestSuite, StopEntryAndExitEdgeCaseBranches) {
    PowerManager& pm = PowerManager::getInstance();
    pm.init(dummy_rtc, dummy_i2c, dummy_uart, dummy_usb, dummy_adc, &sys_context);

    virtual_uptime = 35000;
    pm.processFSM();

    mocks.counter_cancel_ret = -ETIME;
    virtual_uptime = 40000;
    pm.processFSM();
    EXPECT_EQ(std::string_view(pm.current_state->getName()), std::string_view("PM_STATE_SUSPEND_TO_RAM"));

    mocks.counter_cancel_ret = -ENOTSUP;
    pm.reportActivity();
    pm.processFSM();
    EXPECT_EQ(std::string_view(pm.current_state->getName()), std::string_view("PM_STATE_ACTIVE"));

    virtual_uptime = 90000;
    pm.processFSM();
    virtual_uptime = 95000;

    mocks.counter_cancel_ret = 0;
    mocks.setAllSuspendRet(-EALREADY);
    pm.processFSM();
    EXPECT_EQ(std::string_view(pm.current_state->getName()), std::string_view("PM_STATE_SUSPEND_TO_RAM"));

    mocks.setAllResumeRet(-EALREADY);
    pm.reportActivity();
    pm.processFSM();
    EXPECT_EQ(std::string_view(pm.current_state->getName()), std::string_view("PM_STATE_ACTIVE"));
}

TEST_F(PowerManagementTestSuite, StopEntryFailuresFallbackToIdle) {
    PowerManager& pm = PowerManager::getInstance();
    pm.init(dummy_rtc, dummy_i2c, dummy_uart, dummy_usb, dummy_adc, &sys_context);

    virtual_uptime = 30000;
    pm.processFSM();
    EXPECT_EQ(std::string_view(pm.current_state->getName()), std::string_view("PM_STATE_SUSPEND_TO_IDLE"));

    virtual_uptime = 35000;
    mocks.auto_clear_errors = true; // Clear the error instantly so fallback succeeds
    mocks.pm_uart_suspend_ret = -EIO;
    pm.processFSM();
    
    // FSM survives the abort, and safely falls back to Idle
    ASSERT_NE(pm.current_state, nullptr) << "FSM Halted unexpectedly during fallback!";
    EXPECT_EQ(std::string_view(pm.current_state->getName()), std::string_view("PM_STATE_SUSPEND_TO_IDLE"));

    virtual_uptime = 85000;
    pm.consecutive_pm_failures = 0;
    
    pm.processFSM();
    ASSERT_NE(pm.current_state, nullptr);
    EXPECT_EQ(std::string_view(pm.current_state->getName()), std::string_view("PM_STATE_SUSPEND_TO_RAM"));
}

TEST_F(PowerManagementTestSuite, StopEntryCancelAlarmError) {
    PowerManager& pm = PowerManager::getInstance();
    pm.init(dummy_rtc, dummy_i2c, dummy_uart, dummy_usb, dummy_adc, &sys_context);

    virtual_uptime = 30000;
    pm.processFSM();

    virtual_uptime = 35000;
    mocks.counter_cancel_ret = -EIO; 

    pm.processFSM(); 
    EXPECT_EQ(std::string_view(pm.current_state->getName()), std::string_view("PM_STATE_SUSPEND_TO_RAM"));
}

TEST_F(PowerManagementTestSuite, SimTimerCallbackCoverage) {
    PowerManager& pm = PowerManager::getInstance();
    pm.init(nullptr, dummy_i2c, dummy_uart, dummy_usb, dummy_adc, &sys_context);

    if (g_sim_timer_cb) {
        g_sim_timer_cb(nullptr); 
        EXPECT_EQ(atomic_get(&pm.wake_pending), 1);
    }
}

TEST_F(PowerManagementTestSuite, SuspendFailuresStop) {
    PowerManager& pm = PowerManager::getInstance();
    pm.init(dummy_rtc, dummy_i2c, dummy_uart, dummy_usb, dummy_adc, &sys_context);
    mocks.pm_uart_suspend_ret = -EIO;
    EXPECT_FALSE(StopState::getInstance().enter(pm));
    mocks.pm_uart_suspend_ret = 0;

    mocks.pm_i2c_suspend_ret = -EIO;
    EXPECT_FALSE(StopState::getInstance().enter(pm));
    mocks.pm_i2c_suspend_ret = 0;
}

TEST_F(PowerManagementTestSuite, StopExecuteWakeReasons) {
    PowerManager& pm = PowerManager::getInstance();
    pm.init(dummy_rtc, dummy_i2c, dummy_uart, dummy_usb, dummy_adc, &sys_context);
    
    StopState& stop = StopState::getInstance();
    stop.sleep_prepared = true; 
    s_real_hw_sleep_active = false;

    // ACTIVITY wake
    atomic_set(&s_activity_pending, 1);
    k_sem_give(&stop_wake_sem);
    IPowerState& next1 = stop.execute(pm);
    EXPECT_EQ(std::string_view(next1.getName()), "PM_STATE_ACTIVE");

    // RTC wake
    stop.sleep_prepared = true;
    atomic_set(&pm.wake_pending, 1);
    k_sem_give(&stop_wake_sem);
    IPowerState& next2 = stop.execute(pm);
    EXPECT_EQ(std::string_view(next2.getName()), "PM_STATE_ACTIVE");

    // UNKNOWN wake
    stop.sleep_prepared = true;
    atomic_set(&s_activity_pending, 0);
    atomic_set(&pm.wake_pending, 0);
    k_sem_give(&stop_wake_sem);
    IPowerState& next3 = stop.execute(pm);
    EXPECT_EQ(std::string_view(next3.getName()), "PM_STATE_SUSPEND_TO_RAM");

    // TIMEOUT
    stop.sleep_prepared = true;
    IPowerState& next4 = stop.execute(pm);
    EXPECT_EQ(std::string_view(next4.getName()), "PM_STATE_SUSPEND_TO_RAM");

    // NOT PREPARED
    stop.sleep_prepared = false;
    IPowerState& next5 = stop.execute(pm);
    EXPECT_EQ(std::string_view(next5.getName()), "PM_STATE_SUSPEND_TO_RAM");
}

TEST_F(PowerManagementTestSuite, StopExecuteHwSleepLoop) {
    PowerManager& pm = PowerManager::getInstance();
    pm.init(dummy_rtc, dummy_i2c, dummy_uart, dummy_usb, dummy_adc, &sys_context);
    
    StopState& stop = StopState::getInstance();

    // Fast-forward wake_count abort
    s_real_hw_sleep_active = true;
    stop.sleep_prepared = true;
    atomic_set(&pm.wake_pending, 1);
    k_sem_give(&stop_wake_sem);
    IPowerState& next = stop.execute(pm);
    EXPECT_EQ(std::string_view(next.getName()), "PM_STATE_ACTIVE");
    
    // RTC timeout completion
    s_real_hw_sleep_active = true;
    stop.sleep_prepared = true; // MUST RESET for each block
    s_elapsed_sleep_time_us = 60000000;
    atomic_set(&pm.wake_pending, 1);
    k_sem_give(&stop_wake_sem);
    IPowerState& next2 = stop.execute(pm);
    EXPECT_EQ(std::string_view(next2.getName()), "PM_STATE_ACTIVE");

    // RTC set error branch inside loop
    s_real_hw_sleep_active = true;
    stop.sleep_prepared = true; // MUST RESET for each block
    s_elapsed_sleep_time_us = 0;
    atomic_set(&pm.wake_pending, 1);
    mocks.counter_set_ret = -1;
    k_sem_give(&stop_wake_sem);
    IPowerState& next3 = stop.execute(pm);
    EXPECT_EQ(std::string_view(next3.getName()), "PM_STATE_ACTIVE");
}

TEST_F(PowerManagementTestSuite, StopExitCoverage) {
    PowerManager& pm = PowerManager::getInstance();
    pm.init(dummy_rtc, dummy_i2c, dummy_uart, dummy_usb, dummy_adc, &sys_context);
    
    StopState& stop = StopState::getInstance();
    
    stop.sleep_prepared = true;
    s_real_hw_sleep_active = false;
    stop.exit(pm); 

    stop.sleep_prepared = true;
    s_real_hw_sleep_active = true;
    s_stop_wake_reason = StopWakeReason::WAKE_RTC;
    stop.exit(pm); 

    stop.sleep_prepared = true;
    s_real_hw_sleep_active = true;
    s_stop_wake_reason = StopWakeReason::WAKE_ACTIVITY;
    stop.exit(pm); 

    stop.sleep_prepared = true;
    s_real_hw_sleep_active = true;
    s_stop_wake_reason = StopWakeReason::WAKE_ABORTED;
    stop.exit(pm); 
    
    stop.sleep_prepared = true;
    s_real_hw_sleep_active = true;
    s_stop_wake_reason = StopWakeReason::WAKE_UNKNOWN;
    stop.exit(pm);
}

TEST_F(PowerManagementTestSuite, SimulatedStopTimerHandlerCoverage) {
    PowerManager& pm = PowerManager::getInstance();
    pm.init(dummy_rtc, dummy_i2c, dummy_uart, dummy_usb, dummy_adc, &sys_context);

    // K_TIMER_DEFINE stores the user-supplied expiry callback directly in
    // k_timer::expiry_fn — no need to scan memory for it.
    ASSERT_NE(mps2_stop_timer.expiry_fn, nullptr)
        << "Timer was not initialized with an expiry callback";

    mps2_stop_timer.expiry_fn(&mps2_stop_timer);

    // Verify the handler successfully executed by checking its side effect
    EXPECT_EQ(atomic_get(&pm.wake_pending), 1);
}
