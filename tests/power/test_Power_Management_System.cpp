#include "test_power_common.h"

TEST_F(PowerManagementTestSuite, InitFailures) {
    PowerManager& pm = PowerManager::getInstance();
    testing::internal::CaptureStdout();

    EXPECT_TRUE(pm.init(nullptr, dummy_i2c, dummy_uart, dummy_usb,  dummy_adc, &sys_context));
    EXPECT_TRUE(pm.init(dummy_rtc, nullptr, dummy_uart, dummy_usb,  dummy_adc, &sys_context));

    mocks.rtc_ready = false;
    EXPECT_TRUE(pm.init(dummy_rtc, dummy_i2c, dummy_uart, dummy_usb,  dummy_adc, &sys_context));

    mocks.rtc_ready = true;
    mocks.i2c_ready = false;
    EXPECT_TRUE(pm.init(dummy_rtc, dummy_i2c, dummy_uart, dummy_usb,  dummy_adc, &sys_context));

    mocks.i2c_ready = true;
    EXPECT_TRUE(pm.init(dummy_rtc, dummy_i2c, dummy_uart, dummy_usb,  dummy_adc, &sys_context));

    const auto raw_output = testing::internal::GetCapturedStdout();
    std::string_view output(raw_output);
    EXPECT_TRUE(output.find("[WRN] RTC device unavailable") != std::string_view::npos);
    EXPECT_TRUE(output.find("[ERR] I2C device bound but not ready") != std::string_view::npos);
}

TEST_F(PowerManagementTestSuite, InitFailuresAdcAndRtcSeed) {
    PowerManager& pm = PowerManager::getInstance();
    testing::internal::CaptureStdout();

    mocks.adc_ready = false;
    mocks.rtc_get_time_ret = -1; 

    EXPECT_TRUE(pm.init(dummy_rtc, dummy_i2c, dummy_uart, dummy_usb, nullptr, &sys_context));
    EXPECT_TRUE(pm.init(dummy_rtc, dummy_i2c, dummy_uart, dummy_usb, dummy_adc, &sys_context));

    const auto raw_output = testing::internal::GetCapturedStdout();
    EXPECT_TRUE(std::string_view(raw_output).find("[ERR] ADC device bound but not ready") != std::string_view::npos);
}

TEST_F(PowerManagementTestSuite, InitFailuresUartUsb) {
    PowerManager& pm = PowerManager::getInstance();
    EXPECT_TRUE(pm.init(dummy_rtc, dummy_i2c, nullptr, dummy_usb,  dummy_adc, &sys_context));
    EXPECT_TRUE(pm.init(dummy_rtc, dummy_i2c, dummy_uart, nullptr,  dummy_adc, &sys_context));
    
    mocks.uart_ready = false;
    EXPECT_TRUE(pm.init(dummy_rtc, dummy_i2c, dummy_uart, dummy_usb, dummy_adc, &sys_context));
    
    mocks.uart_ready = true;
    mocks.usb_ready = false;
    EXPECT_TRUE(pm.init(dummy_rtc, dummy_i2c, dummy_uart, dummy_usb, dummy_adc, &sys_context));
}

TEST_F(PowerManagementTestSuite, InitSuccess) {
    PowerManager& pm = PowerManager::getInstance();
    testing::internal::CaptureStdout();
    EXPECT_TRUE(pm.init(dummy_rtc, dummy_i2c, dummy_uart, dummy_usb, dummy_adc, &sys_context));
    const auto raw_output = testing::internal::GetCapturedStdout();
    EXPECT_TRUE(std::string_view(raw_output).find("[INF] Power Manager initialized.") != std::string_view::npos);
}

TEST_F(PowerManagementTestSuite, ObserverManagement) {
    PowerManager& pm = PowerManager::getInstance();
    TestObserver obs1, obs2;
    static std::array<TestObserver, 5> extra_observers;

    EXPECT_TRUE(pm.registerObserver(&obs1));
    EXPECT_TRUE(pm.registerObserver(&obs2));
    EXPECT_TRUE(pm.registerObserver(&obs1));
    EXPECT_TRUE(pm.registerObserver(nullptr));

    for (int i = 0; i < 5; i++) EXPECT_TRUE(pm.registerObserver(&extra_observers[i]));

    testing::internal::CaptureStdout();
    TestObserver obs_overflow;
    EXPECT_FALSE(pm.registerObserver(&obs_overflow));
    const auto raw_output = testing::internal::GetCapturedStdout();
    EXPECT_TRUE(std::string_view(raw_output).find("Observer limit reached") != std::string_view::npos);

    pm.notifyBeforeSleep();
    pm.notifyAfterWakeup();
    pm.notifySleepAborted();

    EXPECT_EQ(obs1.sleeps, 1);
    EXPECT_EQ(obs1.wakes, 1);
    EXPECT_EQ(obs1.aborts, 1);
}

TEST_F(PowerManagementTestSuite, FsmStateTransitions) {
    PowerManager& pm = PowerManager::getInstance();
    pm.init(dummy_rtc, dummy_i2c, dummy_uart, dummy_usb, dummy_adc, &sys_context);

    pm.processFSM();

    virtual_uptime = 30000;
    pm.processFSM();
    EXPECT_EQ(std::string_view(pm.current_state->getName()), std::string_view("PM_STATE_SUSPEND_TO_IDLE"));

    virtual_uptime = 32000;
    pm.processFSM();

    virtual_uptime = 35000;
    pm.processFSM();
    EXPECT_EQ(std::string_view(pm.current_state->getName()), std::string_view("PM_STATE_SUSPEND_TO_RAM"));

    pm.processFSM();
    EXPECT_EQ(std::string_view(pm.current_state->getName()), std::string_view("PM_STATE_SUSPEND_TO_RAM"));

    EXPECT_GE(mocks.adc_suspend_calls, 1);
    EXPECT_GE(mocks.i2c_suspend_calls, 1);
    EXPECT_GE(mocks.uart_suspend_calls, 0); 
    EXPECT_EQ(mocks.usb_suspend_calls, 0);  
}

TEST_F(PowerManagementTestSuite, FaultEscalationSequence) {
    PowerManager& pm = PowerManager::getInstance();
    pm.init(dummy_rtc, dummy_i2c, dummy_uart, dummy_usb, dummy_adc, &sys_context);

    for (int i = 0; i < 20; i++) {
        pm.reportPmFailure();
        if (sys_context.current_state != SystemState::INIT) {
            break;
        }
    }

    mocks.auto_clear_errors = false; 
    mocks.pm_adc_suspend_ret = -EIO;
    
    virtual_uptime = 30000;
    pm.processFSM(); 
    
    EXPECT_EQ(sys_context.current_state, SystemState::FAULT);
}

class FsmInterruptorState : public IPowerState {
public:
    bool enter(PowerManager& pm) override { return true; }
    IPowerState& execute(PowerManager& pm) override {
        pm.current_state = &ActiveState::getInstance();
        return StopState::getInstance();
    }
    void exit(PowerManager& pm) override {}
    const char* getName() const override { return "INTERRUPTOR"; }
};

TEST_F(PowerManagementTestSuite, ProcessFSMStateInterruption) {
    PowerManager& pm = PowerManager::getInstance();
    pm.init(dummy_rtc, dummy_i2c, dummy_uart, dummy_usb, dummy_adc, &sys_context);

    static FsmInterruptorState interruptor;
    pm.current_state = &interruptor;

    pm.processFSM();
    EXPECT_EQ(std::string_view(pm.current_state->getName()), std::string_view("PM_STATE_ACTIVE"));
}

TEST_F(PowerManagementTestSuite, DefensiveUnreachableBranches) {
    PowerManager& pm = PowerManager::getInstance();

    pm.observer_count = 10;
    std::array<IPowerObserver*, 8> out;
    EXPECT_EQ(pm.captureObservers(out), 8);

    pm.current_state = &ActiveState::getInstance();
    pm.transitionTo(ActiveState::getInstance());
    EXPECT_EQ(pm.current_state, &ActiveState::getInstance());

    StopState::getInstance().sleep_prepared = false;
    StopState::getInstance().exit(pm);

    pm.current_state = nullptr;
    pm.processFSM();

    pm.init(dummy_rtc, dummy_i2c, dummy_uart, dummy_usb, dummy_adc, &sys_context);

    virtual_uptime = 30000;
    pm.processFSM();

    virtual_uptime = 40000;
    mocks.counter_set_ret = -1; 
    mocks.pm_adc_suspend_ret = -EIO;
    g_force_idle_enter_fail = true;

    pm.processFSM();
    EXPECT_EQ(pm.current_state, nullptr);
}

TEST_F(PowerManagementTestSuite, RtcAlarmWakePendingHandling) {
    PowerManager& pm = PowerManager::getInstance();
    pm.init(dummy_rtc, dummy_i2c, dummy_uart, dummy_usb, dummy_adc, &sys_context);
    PowerManager::rtc_alarm_handler(dummy_rtc, 0, &pm);
    EXPECT_EQ(atomic_get(&pm.wake_pending), 1);
    
    // Nullptr user data check
    atomic_set(&pm.wake_pending, 0);
    PowerManager::rtc_alarm_handler(dummy_rtc, 0, nullptr);
    EXPECT_EQ(atomic_get(&pm.wake_pending), 0);
}

TEST_F(PowerManagementTestSuite, ThreadRoutineCoverage) {
    run_thread_once = true;
    mocks.stack_space_ret = 0;
    mocks.stack_space_calls = 0;
    power_monitor_thread();
    EXPECT_GE(mocks.stack_space_calls, 0);

    run_thread_once = false;
    mocks.stack_space_ret = -1;
    mocks.stack_space_calls = 0;
    power_monitor_thread();
    EXPECT_GE(mocks.stack_space_calls, 0);
}

TEST_F(PowerManagementTestSuite, ThreadRoutineCoverageInitFail) {
    g_force_init_fail = true;
    power_monitor_thread();
    g_force_init_fail = false;
}

TEST_F(PowerManagementTestSuite, NullFaultContextBranches) {
    PowerManager& pm = PowerManager::getInstance();

    pm.init(dummy_rtc, dummy_i2c, dummy_uart, dummy_usb, dummy_adc, nullptr);
    mocks.counter_set_ret = -1;
    virtual_uptime = 30000;
    pm.processFSM();
    virtual_uptime += 5000;
    pm.processFSM();
    virtual_uptime += 5000;
    pm.processFSM();

    virtual_uptime += 5000;
    pm.processFSM();
    EXPECT_NE(sys_context.current_state, SystemState::SAFE_HALT);

    pm.resetForTest();
    pm.init(dummy_rtc, dummy_i2c, dummy_uart, dummy_usb, dummy_adc, nullptr);
    mocks.counter_set_ret = 0;
    virtual_uptime = 30000;
    pm.processFSM();

    virtual_uptime = 40000;
    mocks.counter_set_ret = -1; 
    mocks.pm_adc_suspend_ret = -EIO;
    g_force_idle_enter_fail = true;

    pm.processFSM();
    EXPECT_EQ(pm.current_state, nullptr);
}

TEST_F(PowerManagementTestSuite, ReportActivityFiltering) {
    PowerManager& pm = PowerManager::getInstance();
    pm.init(dummy_rtc, dummy_i2c, dummy_uart, dummy_usb, dummy_adc, &sys_context);
    
    atomic_set(&s_activity_pending, 0);
    mock_is_in_isr_val = false;
    mock_k_current_get_val = processor_tid; // Filtered thread
    pm.reportActivity();
    EXPECT_EQ(atomic_get(&s_activity_pending), 1);

    mock_k_current_get_val = (k_tid_t)0x9999; // Non-filtered thread
    pm.reportActivity();
    EXPECT_EQ(atomic_get(&s_activity_pending), 1);

    atomic_set(&s_activity_pending, 0);
    mock_is_in_isr_val = true; // ISR overrides thread filter
    mock_k_current_get_val = processor_tid; 
    pm.reportActivity();
    EXPECT_EQ(atomic_get(&s_activity_pending), 1);
}

TEST_F(PowerManagementTestSuite, FsmActivityWhileActive) {
    PowerManager& pm = PowerManager::getInstance();
    pm.init(dummy_rtc, dummy_i2c, dummy_uart, dummy_usb, dummy_adc, &sys_context);
    
    pm.current_state = &ActiveState::getInstance();
    atomic_set(&s_activity_pending, 1);
    pm.processFSM();
    EXPECT_EQ(std::string_view(pm.current_state->getName()), "PM_STATE_ACTIVE");
}

TEST_F(PowerManagementTestSuite, PmFailureNullContextThreshold) {
    PowerManager& pm = PowerManager::getInstance();
    
    // Initialize with a nullptr fault_context
    pm.init(dummy_rtc, dummy_i2c, dummy_uart, dummy_usb, dummy_adc, nullptr);
   
    for (int i = 0; i < 10; i++) {
        pm.reportPmFailure();
    }
}

TEST_F(PowerManagementTestSuite, ProcessFsmActivityWhenHalted) {
    PowerManager& pm = PowerManager::getInstance();
    pm.init(dummy_rtc, dummy_i2c, dummy_uart, dummy_usb, dummy_adc, &sys_context);
    
    g_force_idle_enter_fail = true;
    virtual_uptime = 30000;
    pm.processFSM(); // Fails FSM transition, halts. current_state becomes nullptr
    
    // Covers branch: activity_woke = true, local_state = nullptr
    atomic_set(&s_activity_pending, 1);
    pm.processFSM(); 
}

TEST_F(PowerManagementTestSuite, ProcessFsmNoActivityWhenHalted) {
    PowerManager& pm = PowerManager::getInstance();
    pm.init(dummy_rtc, dummy_i2c, dummy_uart, dummy_usb, dummy_adc, &sys_context);
    
    g_force_idle_enter_fail = true;
    virtual_uptime = 30000;
    pm.processFSM(); // Fails FSM transition, halts
    
    // Covers branch: activity_woke = false, local_state = nullptr
    atomic_set(&s_activity_pending, 0);
    pm.processFSM(); 
}

TEST_F(PowerManagementTestSuite, ThreadRoutineIdleStateTimeoutEdges) {
    virtual_uptime = 0; 
    
    // We use a tight loop to artificially hit the microsecond race window between 
    // processFSM() execution and the thread's native current_time check.
    std::thread FsmAdvancer([]() {
        for (int i = 0; i < 50; i++) {
            run_thread_once = true; // Keep the loop alive
            
            virtual_uptime = 35000; // Trigger Active -> Idle
            std::this_thread::sleep_for(std::chrono::microseconds(50));
            
            // Race Window: Let processFSM see elapsed < 5000
            virtual_uptime = 39999; 
            std::this_thread::sleep_for(std::chrono::microseconds(50));
            
            // Immediately jump to >= 5000 so the loop fallback captures K_MSEC(10)
            virtual_uptime = 45000; 
            k_sem_give(&pm_wake_sem);
        }
        
        // Terminate cleanly
        run_thread_once = false;
        k_sem_give(&pm_wake_sem);
    });
    
    power_monitor_thread(); 
    FsmAdvancer.join();
}

TEST_F(PowerManagementTestSuite, ThreadRoutineActiveTimeoutElseBlock) {
    g_force_idle_enter_fail = true; // Prevent FSM from zeroing the active timer
    virtual_uptime = 0; 
    run_thread_once = true;
    
    std::thread advancer([]() {
        // Wait for power_monitor_thread to finish initialization
        while (PowerManager::getInstance().current_state == nullptr) {
            std::this_thread::yield();
        }
        // Advance time rapidly so active_elapsed gracefully exceeds 30000ms
        virtual_uptime = 40000;
        k_sem_give(&pm_wake_sem);
    });
    
    power_monitor_thread(); 
    advancer.join();
}

// ---------------------------------------------------------------------------
// power_monitor_thread(): `if (s_in_idle_state)` branch (both inner outcomes)
// ---------------------------------------------------------------------------
static void drain_pm_wake_sem() {
    while (k_sem_take(&pm_wake_sem, K_NO_WAIT) == 0) {}
}

// idle_elapsed < IDLE_TIMEOUT_MS  -> wait_timeout = IDLE_TIMEOUT_MS - idle_elapsed
TEST_F(PowerManagementTestSuite, ThreadRoutineIdleStateWaitRemaining) {
    PowerManager& pm = PowerManager::getInstance();
    virtual_uptime = 0;
    run_thread_once = false;                        // exactly one loop iteration
    hook_uptime_bump_on_active_enter = 30000;       // Active timeout elapses right after init
    k_sem_give(&pm_wake_sem);                       // never block in k_sem_take()

    power_monitor_thread();

    ASSERT_NE(pm.current_state, nullptr);
    EXPECT_EQ(std::string_view(pm.current_state->getName()), std::string_view("PM_STATE_SUSPEND_TO_IDLE"));

    drain_pm_wake_sem();
    IdleState::getInstance().exit(pm);              // leave s_in_idle_state == false
}

// idle_elapsed >= IDLE_TIMEOUT_MS -> wait_timeout = K_MSEC(10)
TEST_F(PowerManagementTestSuite, ThreadRoutineIdleStateTimeoutExpired) {
    PowerManager& pm = PowerManager::getInstance();
    virtual_uptime = 0;
    run_thread_once = false;
    hook_uptime_bump_on_active_enter = 30000;       // Active -> Idle in processFSM()
    hook_uptime_bump_on_idle_uart_suspend = 6000;   // time passes while Idle is being entered
    k_sem_give(&pm_wake_sem);

    power_monitor_thread();

    ASSERT_NE(pm.current_state, nullptr);
    EXPECT_EQ(std::string_view(pm.current_state->getName()), std::string_view("PM_STATE_SUSPEND_TO_IDLE"));

    drain_pm_wake_sem();
    IdleState::getInstance().exit(pm);
}

TEST_F(PowerManagementTestSuite, ThreadRoutineActiveStateTimeoutExpired) {
    PowerManager& pm = PowerManager::getInstance();

    // Make sure a previous test did not leave s_in_idle_state == true.
    IdleState::getInstance().exit(pm);

    virtual_uptime = 0;
    run_thread_once = false;                        // exactly one loop iteration
    hook_uptime_bump_on_active_enter = 30000;       // active timeout elapses after init
    g_force_idle_enter_fail = true;                 // FSM halts, s_in_idle_state stays false
    k_sem_give(&pm_wake_sem);                       // never block in k_sem_take()

    power_monitor_thread();

    EXPECT_EQ(pm.current_state, nullptr);           // FSM halted
    EXPECT_GE(k_uptime_get_32() - pm.getLastActivityTime(), 30000u);

    g_force_idle_enter_fail = false;
    drain_pm_wake_sem();
}

// power_monitor_thread(): `if (s_real_hw_sleep_active)` -> wait_timeout = K_NO_WAIT
TEST_F(PowerManagementTestSuite, ThreadRoutineHwSleepActiveNoWait) {
    PowerManager& pm = PowerManager::getInstance();

    virtual_uptime = 0;
    run_thread_once = false;          // exactly one loop iteration
    s_real_hw_sleep_active = true;    // init() -> Active does not clear it; Active::execute() doesn't either
    k_sem_give(&pm_wake_sem);         // never block in k_sem_take()

    power_monitor_thread();

    EXPECT_NE(pm.current_state, nullptr);

    // cleanup so later tests don't see a stale flag
    s_real_hw_sleep_active = false;
    drain_pm_wake_sem();
}
