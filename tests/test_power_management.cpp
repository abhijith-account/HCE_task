// =========================================================================
// Google Test Coverage Suite for PowerManager (100% Line & Branch Coverage)
// =========================================================================
#include <gtest/gtest.h>
#include <array>
#include <cstdint>
#include <string_view>

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/counter.h>
#include <zephyr/pm/device.h>
#include <zephyr/pm/policy.h>
#include <zephyr/sys/atomic.h>

#include <thread>
#include <vector>
#include <atomic>

#define private public
#include "Power_Management_System.h"
#include "Device_State_Machine+Watchdog.h" 
#undef private

// ---------------------------------------------------------
// System Mocks
// ---------------------------------------------------------
#ifndef ETIME
#define ETIME 62
#endif
#ifndef ENOTSUP
#define ENOTSUP 134
#endif
#ifndef EALREADY
#define EALREADY 114
#endif
#ifndef EIO
#define EIO 5
#endif

bool run_thread_once = false;
extern bool g_force_idle_enter_fail;// Mock flag for unreachable fallback branch

const struct device* dummy_rtc  = reinterpret_cast<const struct device*>(0x111);
const struct device* dummy_i2c  = reinterpret_cast<const struct device*>(0x222);
const struct device* dummy_uart = reinterpret_cast<const struct device*>(0x333);
const struct device* dummy_usb  = reinterpret_cast<const struct device*>(0x444);

const struct device* i2c_hardware  = dummy_i2c;
const struct device* uart_hardware = dummy_uart;
const struct device* usb_hardware  = dummy_usb;
const struct device* rtc_hardware  = dummy_rtc;

extern DeviceContext sys_context;

struct MockController {
    bool rtc_ready = true;
    bool i2c_ready = true;
    bool uart_ready = true;
    bool usb_ready = true;

    int counter_start_ret = 0;
    int counter_cancel_ret = 0;
    uint32_t counter_ticks_ret = 60000;
    int counter_set_ret = 0;

    // Per-device suspend/resume return codes -- now that PowerManager
    // suspends/resumes I2C, UART, and USB independently (each is a
    // separate pm_device_action_run() call site with its own branch),
    // each device needs its own controllable return code to isolate
    // that call site's true/false branches from the other two.
    int pm_i2c_suspend_ret  = 0;
    int pm_i2c_resume_ret   = 0;
    int pm_uart_suspend_ret = 0;
    int pm_uart_resume_ret  = 0;
    int pm_usb_suspend_ret  = 0;
    int pm_usb_resume_ret   = 0;

    // Per-device call counters, used by the rollback tests to confirm
    // exactly which devices were suspended/resumed (and how many times)
    // during a partial STOP-entry failure.
    int i2c_suspend_calls  = 0;
    int i2c_resume_calls   = 0;
    int uart_suspend_calls = 0;
    int uart_resume_calls  = 0;
    int usb_suspend_calls  = 0;
    int usb_resume_calls   = 0;

    int stack_space_ret = 0;
    int stack_space_calls = 0; 

    // Convenience setters for tests that don't care which specific device
    // fails/bypasses and just want to drive all three suspend or resume
    // call sites with the same return code in one line (e.g. the
    // -EALREADY bypass test).
    void setAllSuspendRet(int v) {
        pm_i2c_suspend_ret = v;
        pm_uart_suspend_ret = v;
        pm_usb_suspend_ret = v;
    }
    void setAllResumeRet(int v) {
        pm_i2c_resume_ret = v;
        pm_uart_resume_ret = v;
        pm_usb_resume_ret = v;
    }

    void reset() {
        rtc_ready = true;
        i2c_ready = true;
        uart_ready = true;
        usb_ready = true;
        counter_start_ret = 0;
        counter_cancel_ret = 0;
        counter_ticks_ret = 60000;
        counter_set_ret = 0;
        pm_i2c_suspend_ret = 0;
        pm_i2c_resume_ret = 0;
        pm_uart_suspend_ret = 0;
        pm_uart_resume_ret = 0;
        pm_usb_suspend_ret = 0;
        pm_usb_resume_ret = 0;
        i2c_suspend_calls = 0;
        i2c_resume_calls = 0;
        uart_suspend_calls = 0;
        uart_resume_calls = 0;
        usb_suspend_calls = 0;
        usb_resume_calls = 0;
        stack_space_ret = 0;
        stack_space_calls = 0;
    }
};
static MockController mocks;

enum class PmAction {
    NONE, LOCK_ACQUIRED, LOCK_RELEASED,
    I2C_SUSPENDED, I2C_RESUMED,
    UART_SUSPENDED, UART_RESUMED,
    USB_SUSPENDED, USB_RESUMED,
    RTC_SET
};
// Widened from 10 -> 64: a single STOP enter+exit cycle now produces 5
// entries on entry (RTC_SET + 3x SUSPEND + LOCK_RELEASED) and 4 on exit
// (LOCK_ACQUIRED + 3x RESUME) instead of 3 and 2, and several tests drive
// multiple cycles -- the old size-10 array would silently truncate mid-test.
static std::array<PmAction, 64> action_history;
static size_t action_idx = 0;
static void (*mock_rtc_alarm_cb)(const device*, uint8_t, uint32_t, void*) = nullptr;
static void* mock_rtc_user_data = nullptr;

static void record_action(PmAction a) {
    if (action_idx < action_history.size()) action_history[action_idx++] = a;
}

// ---------------------------------------------------------
// Zephyr API Stubs
// ---------------------------------------------------------
extern "C" {
    bool device_is_ready(const struct device *dev) {
        if (dev == dummy_rtc)  return mocks.rtc_ready;
        if (dev == dummy_i2c)  return mocks.i2c_ready;
        if (dev == dummy_uart) return mocks.uart_ready;
        if (dev == dummy_usb)  return mocks.usb_ready;
        return false;
    }
    int counter_start(const struct device*) { return mocks.counter_start_ret; }
    int counter_cancel_channel_alarm(const struct device*, uint8_t) { return mocks.counter_cancel_ret; }
    uint32_t counter_us_to_ticks(const struct device*, uint64_t) { return mocks.counter_ticks_ret; }
    
    int counter_set_channel_alarm(const struct device*, uint8_t, const struct counter_alarm_cfg *cfg) {
        mock_rtc_alarm_cb = cfg->callback;
        mock_rtc_user_data = cfg->user_data;
        record_action(PmAction::RTC_SET);
        return mocks.counter_set_ret;
    }
    void pm_policy_state_lock_get(uint8_t, uint8_t) {
        record_action(PmAction::LOCK_ACQUIRED);
    }
    void pm_policy_state_lock_put(uint8_t, uint8_t) {
        record_action(PmAction::LOCK_RELEASED);
    }
    int pm_device_action_run(const struct device* dev, enum pm_device_action action) {
        if (action == PM_DEVICE_ACTION_SUSPEND) {
            if (dev == dummy_i2c) {
                mocks.i2c_suspend_calls++;
                record_action(PmAction::I2C_SUSPENDED);
                return mocks.pm_i2c_suspend_ret;
            }
            if (dev == dummy_uart) {
                mocks.uart_suspend_calls++;
                record_action(PmAction::UART_SUSPENDED);
                return mocks.pm_uart_suspend_ret;
            }
            if (dev == dummy_usb) {
                mocks.usb_suspend_calls++;
                record_action(PmAction::USB_SUSPENDED);
                return mocks.pm_usb_suspend_ret;
            }
            return 0;
        } else if (action == PM_DEVICE_ACTION_RESUME) {
            if (dev == dummy_i2c) {
                mocks.i2c_resume_calls++;
                record_action(PmAction::I2C_RESUMED);
                return mocks.pm_i2c_resume_ret;
            }
            if (dev == dummy_uart) {
                mocks.uart_resume_calls++;
                record_action(PmAction::UART_RESUMED);
                return mocks.pm_uart_resume_ret;
            }
            if (dev == dummy_usb) {
                mocks.usb_resume_calls++;
                record_action(PmAction::USB_RESUMED);
                return mocks.pm_usb_resume_ret;
            }
            return 0;
        }
        return 0;
    }
    int k_thread_stack_space_get(const void*, size_t *unused_ptr) {
        mocks.stack_space_calls++;
        if (mocks.stack_space_ret == 0 && unused_ptr) *unused_ptr = 1024;
        return mocks.stack_space_ret;
    }
}

bool atomic_cas(atomic_t *target, atomic_val_t old_value, atomic_val_t new_value) {
    atomic_val_t expected = old_value;
    return target->compare_exchange_strong(expected, new_value);
}

// ---------------------------------------------------------
// Test Fixture
// ---------------------------------------------------------
class TestObserver : public IPowerObserver {
public:
    int sleeps = 0, wakes = 0, aborts = 0;
    void beforeSleep() override { sleeps++; }
    void afterWakeup() override { wakes++; }
    void sleepAborted() override { aborts++; }
    void reset() { sleeps = 0; wakes = 0; aborts = 0; }
};

class PowerManagementTestSuite : public ::testing::Test {
protected:
    void SetUp() override {
        PowerManager::getInstance().resetForTest();
        mocks.reset();
        virtual_uptime = 0;
        action_idx = 0;
        action_history.fill(PmAction::NONE);
        mock_rtc_alarm_cb = nullptr;
        mock_rtc_user_data = nullptr;
        sys_context.current_state = SystemState::INIT; 
        run_thread_once = false;
        g_force_idle_enter_fail = false;
    }
};

// ---------------------------------------------------------
// Test Cases
// ---------------------------------------------------------

TEST_F(PowerManagementTestSuite, InitFailures) {
    PowerManager& pm = PowerManager::getInstance();
    testing::internal::CaptureStdout();

    EXPECT_FALSE(pm.init(nullptr, dummy_i2c, dummy_uart, dummy_usb, &sys_context));
    EXPECT_FALSE(pm.init(dummy_rtc, nullptr, dummy_uart, dummy_usb, &sys_context));

    mocks.rtc_ready = false;
    EXPECT_FALSE(pm.init(dummy_rtc, dummy_i2c, dummy_uart, dummy_usb, &sys_context));
    
    mocks.rtc_ready = true;
    mocks.i2c_ready = false;
    EXPECT_FALSE(pm.init(dummy_rtc, dummy_i2c, dummy_uart, dummy_usb, &sys_context));
    
    mocks.i2c_ready = true;
    mocks.counter_start_ret = -1;
    EXPECT_FALSE(pm.init(dummy_rtc, dummy_i2c, dummy_uart, dummy_usb, &sys_context));
    
    const auto raw_output = testing::internal::GetCapturedStdout();
    std::string_view output(raw_output);
    EXPECT_TRUE(output.find("[ERR] RTC device not ready") != std::string_view::npos);
    EXPECT_TRUE(output.find("[ERR] I2C device not ready") != std::string_view::npos);
    EXPECT_TRUE(output.find("[ERR] Failed to start RTC counter") != std::string_view::npos);
}

// NEW: mirrors InitFailures' null/not-ready pattern for the two devices
// added to init() -- covers both halves of the `!uart_dev ||
// !device_is_ready(uart_dev)` and `!usb_dev || !device_is_ready(usb_dev)`
// compound conditions (null-pointer short-circuit, and ready-check
// false with a valid pointer).
TEST_F(PowerManagementTestSuite, InitFailuresUartUsb) {
    PowerManager& pm = PowerManager::getInstance();
    testing::internal::CaptureStdout();

    EXPECT_FALSE(pm.init(dummy_rtc, dummy_i2c, nullptr, dummy_usb, &sys_context));
    EXPECT_FALSE(pm.init(dummy_rtc, dummy_i2c, dummy_uart, nullptr, &sys_context));

    mocks.uart_ready = false;
    EXPECT_FALSE(pm.init(dummy_rtc, dummy_i2c, dummy_uart, dummy_usb, &sys_context));

    mocks.uart_ready = true;
    mocks.usb_ready = false;
    EXPECT_FALSE(pm.init(dummy_rtc, dummy_i2c, dummy_uart, dummy_usb, &sys_context));

    const auto raw_output = testing::internal::GetCapturedStdout();
    std::string_view output(raw_output);
    EXPECT_TRUE(output.find("[ERR] UART device not ready") != std::string_view::npos);
    EXPECT_TRUE(output.find("[ERR] USB device not ready") != std::string_view::npos);
}

TEST_F(PowerManagementTestSuite, InitSuccess) {
    PowerManager& pm = PowerManager::getInstance();
    testing::internal::CaptureStdout();
    EXPECT_TRUE(pm.init(dummy_rtc, dummy_i2c, dummy_uart, dummy_usb, &sys_context));
    const auto raw_output = testing::internal::GetCapturedStdout();
    EXPECT_TRUE(std::string_view(raw_output).find("[INF] Power Manager initialized. Deep Sleep Locked.") != std::string_view::npos);
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
    pm.init(dummy_rtc, dummy_i2c, dummy_uart, dummy_usb, &sys_context);

    // Stay ACTIVE
    pm.processFSM();
    
    // Transition to IDLE
    virtual_uptime = 30000;
    pm.processFSM();
    EXPECT_EQ(std::string_view(pm.current_state->getName()), std::string_view("IDLE"));

    // Execute within IDLE (covers IdleState return *this;)
    virtual_uptime = 32000;
    pm.processFSM();
    
    // Transition to STOP
    virtual_uptime = 35000;
    pm.processFSM();
    EXPECT_EQ(std::string_view(pm.current_state->getName()), std::string_view("STOP"));

    // Execute within STOP (covers StopState return *this;)
    pm.processFSM();
    EXPECT_EQ(std::string_view(pm.current_state->getName()), std::string_view("STOP"));

    // All three peripherals should have suspended exactly once on this
    // single clean STOP entry.
    EXPECT_EQ(mocks.i2c_suspend_calls, 1);
    EXPECT_EQ(mocks.uart_suspend_calls, 1);
    EXPECT_EQ(mocks.usb_suspend_calls, 1);
}

TEST_F(PowerManagementTestSuite, StopEntryAndExitEdgeCaseBranches) {
    PowerManager& pm = PowerManager::getInstance();
    pm.init(dummy_rtc, dummy_i2c, dummy_uart, dummy_usb, &sys_context);

    virtual_uptime = 35000;
    pm.processFSM(); // Jump to IDLE

    // STOP Enter edge case (-ETIME Bypass)
    mocks.counter_cancel_ret = -ETIME;
    virtual_uptime = 40000;
    pm.processFSM(); // Enters STOP
    EXPECT_EQ(std::string_view(pm.current_state->getName()), std::string_view("STOP"));

    // STOP Exit edge case (-ENOTSUP Bypass)
    mocks.counter_cancel_ret = -ENOTSUP; 
    pm.reportActivity(); // Wakes to ACTIVE
    EXPECT_EQ(std::string_view(pm.current_state->getName()), std::string_view("ACTIVE"));

    virtual_uptime = 90000;
    pm.processFSM(); // Jump back to IDLE
    virtual_uptime = 95000;

    // Suspend & Resume success-equivalent (-EALREADY Bypass), now exercised
    // across all three devices' suspend/resume call sites in one pass.
    mocks.counter_cancel_ret = 0;
    mocks.setAllSuspendRet(-EALREADY);
    pm.processFSM(); 
    EXPECT_EQ(std::string_view(pm.current_state->getName()), std::string_view("STOP"));

    mocks.setAllResumeRet(-EALREADY);
    pm.reportActivity(); 
    EXPECT_EQ(std::string_view(pm.current_state->getName()), std::string_view("ACTIVE"));
}

TEST_F(PowerManagementTestSuite, StopExitNormalWakeup) {
    PowerManager& pm = PowerManager::getInstance();
    pm.init(dummy_rtc, dummy_i2c, dummy_uart, dummy_usb, &sys_context);

    virtual_uptime = 30000;
    pm.processFSM(); // IDLE
    virtual_uptime = 35000;
    pm.processFSM(); // STOP

    // Move time significantly past 60s sleep expectation to trigger delay >= 0 branch
    virtual_uptime += 70000; 
    mocks.setAllResumeRet(0); // Success to hit resetPmFailures() branch

    testing::internal::CaptureStdout();
    pm.reportActivity(); // Wakes system
    const auto raw_output = testing::internal::GetCapturedStdout();
    
    EXPECT_TRUE(std::string_view(raw_output).find("System Awoken via RTC.") != std::string_view::npos);
    EXPECT_EQ(std::string_view(pm.current_state->getName()), std::string_view("ACTIVE"));
    EXPECT_EQ(pm.consecutive_pm_failures, 0); 
}

TEST_F(PowerManagementTestSuite, StopEntryFailuresFallbackToIdle) {
    PowerManager& pm = PowerManager::getInstance();
    pm.init(dummy_rtc, dummy_i2c, dummy_uart, dummy_usb, &sys_context);
    
    virtual_uptime = 30000;
    pm.processFSM(); // Enters IDLE

    // 1. Ticks = 0 overflow
    virtual_uptime = 35000;
    mocks.counter_ticks_ret = 0;
    pm.processFSM(); 
    EXPECT_EQ(std::string_view(pm.current_state->getName()), std::string_view("IDLE"));
    
    // 2. Alarm set fails
    virtual_uptime = 40000;
    mocks.counter_ticks_ret = 60000;
    mocks.counter_set_ret = -1;
    pm.processFSM(); 
    EXPECT_EQ(std::string_view(pm.current_state->getName()), std::string_view("IDLE"));
    
    // 3. pm_device_action_run fails (I2C -- the first suspend attempted)
    TestObserver obs;
    pm.registerObserver(&obs);
    virtual_uptime = 45000;
    mocks.counter_set_ret = 0;
    mocks.pm_i2c_suspend_ret = -EIO;
    pm.processFSM();
    EXPECT_EQ(std::string_view(pm.current_state->getName()), std::string_view("IDLE"));
    EXPECT_EQ(obs.aborts, 1); 
    
    // 4. Cancel channel alarm warns but continues
    virtual_uptime = 85000; 
    pm.consecutive_pm_failures = 0; 
    mocks.pm_i2c_suspend_ret = 0;
    mocks.counter_cancel_ret = -EIO; 
    
    testing::internal::CaptureStdout();
    pm.processFSM(); 
    const auto raw_output = testing::internal::GetCapturedStdout();
    EXPECT_TRUE(std::string_view(raw_output).find("Failed to cancel RTC alarm") != std::string_view::npos);
    EXPECT_EQ(std::string_view(pm.current_state->getName()), std::string_view("STOP"));
}

// NEW: UART suspend fails after I2C already succeeded -- exercises the
// rollback branch that resumes I2C before aborting STOP entry.
TEST_F(PowerManagementTestSuite, StopEntryUartSuspendFailureRollsBackI2c) {
    PowerManager& pm = PowerManager::getInstance();
    pm.init(dummy_rtc, dummy_i2c, dummy_uart, dummy_usb, &sys_context);

    TestObserver obs;
    pm.registerObserver(&obs);

    virtual_uptime = 30000;
    pm.processFSM(); // ACTIVE -> IDLE

    mocks.pm_uart_suspend_ret = -EIO;

    testing::internal::CaptureStdout();
    virtual_uptime = 35000;
    pm.processFSM(); // IDLE -> STOP attempt: I2C suspends, UART fails, rolls back
    const auto raw_output = testing::internal::GetCapturedStdout();
    std::string_view output(raw_output);

    EXPECT_TRUE(output.find("Failed to suspend UART peripheral") != std::string_view::npos);
    EXPECT_TRUE(output.find("Rolling back.") != std::string_view::npos);

    // Cascaded fallback lands on IDLE, not STOP.
    EXPECT_EQ(std::string_view(pm.current_state->getName()), std::string_view("IDLE"));

    // I2C was suspended once, then rolled back (resumed) once. UART was
    // attempted once and failed. USB was never reached.
    EXPECT_EQ(mocks.i2c_suspend_calls, 1);
    EXPECT_EQ(mocks.i2c_resume_calls, 1);
    EXPECT_EQ(mocks.uart_suspend_calls, 1);
    EXPECT_EQ(mocks.usb_suspend_calls, 0);

    EXPECT_EQ(obs.aborts, 1);
}

// NEW: USB suspend fails after I2C and UART already succeeded -- exercises
// the rollback branch that resumes both UART and I2C before aborting.
TEST_F(PowerManagementTestSuite, StopEntryUsbSuspendFailureRollsBackI2cAndUart) {
    PowerManager& pm = PowerManager::getInstance();
    pm.init(dummy_rtc, dummy_i2c, dummy_uart, dummy_usb, &sys_context);

    TestObserver obs;
    pm.registerObserver(&obs);

    virtual_uptime = 30000;
    pm.processFSM(); // ACTIVE -> IDLE

    mocks.pm_usb_suspend_ret = -EIO;

    testing::internal::CaptureStdout();
    virtual_uptime = 35000;
    pm.processFSM(); // IDLE -> STOP attempt: I2C+UART suspend, USB fails, rolls back both
    const auto raw_output = testing::internal::GetCapturedStdout();
    std::string_view output(raw_output);

    EXPECT_TRUE(output.find("Failed to suspend USB peripheral") != std::string_view::npos);
    EXPECT_TRUE(output.find("Rolling back.") != std::string_view::npos);

    EXPECT_EQ(std::string_view(pm.current_state->getName()), std::string_view("IDLE"));

    EXPECT_EQ(mocks.i2c_suspend_calls, 1);
    EXPECT_EQ(mocks.i2c_resume_calls, 1);
    EXPECT_EQ(mocks.uart_suspend_calls, 1);
    EXPECT_EQ(mocks.uart_resume_calls, 1);
    EXPECT_EQ(mocks.usb_suspend_calls, 1);
    EXPECT_EQ(mocks.usb_resume_calls, 0);

    EXPECT_EQ(obs.aborts, 1);
}

TEST_F(PowerManagementTestSuite, FaultEscalationSequence) {
    PowerManager& pm = PowerManager::getInstance();
    pm.init(dummy_rtc, dummy_i2c, dummy_uart, dummy_usb, &sys_context);
    
    mocks.counter_set_ret = -1;
    virtual_uptime = 30000;
    pm.processFSM(); 
    virtual_uptime += 5000; 
    pm.processFSM(); 
    virtual_uptime += 5000; 
    pm.processFSM(); 
    virtual_uptime += 5000; 
    
    testing::internal::CaptureStdout();
    pm.processFSM(); 
    const auto raw_output = testing::internal::GetCapturedStdout();
    EXPECT_TRUE(std::string_view(raw_output).find("Power management failure threshold reached") != std::string_view::npos);
    EXPECT_EQ(sys_context.current_state, SystemState::SAFE_HALT);
}

// Isolated interceptor to force transition mutation mathematically during branch evaluation
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
    pm.init(dummy_rtc, dummy_i2c, dummy_uart, dummy_usb, &sys_context);

    static FsmInterruptorState interruptor;
    pm.current_state = &interruptor;
    
    testing::internal::CaptureStdout();
    pm.processFSM();
    const auto raw_output = testing::internal::GetCapturedStdout();
    EXPECT_EQ(std::string_view(pm.current_state->getName()), std::string_view("ACTIVE"));
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

    // Force Idle Fallback to fail
    pm.init(dummy_rtc, dummy_i2c, dummy_uart, dummy_usb, &sys_context);
    
    // 1. Advance the FSM naturally to IDLE
    virtual_uptime = 30000;
    pm.processFSM(); 

    // 2. Advance time to trigger transition to STOP. 
    // mock ticks=0 fails Stop entry; g_force_idle_enter_fail fails Idle fallback.
    virtual_uptime = 40000;
    mocks.counter_ticks_ret = 0; 
    g_force_idle_enter_fail = true; 

    testing::internal::CaptureStdout();
    pm.processFSM();
    
    const auto raw_output = testing::internal::GetCapturedStdout();
    EXPECT_TRUE(std::string_view(raw_output).find("Power manager halted. Idle fallback failed.") != std::string_view::npos);
    EXPECT_EQ(pm.current_state, nullptr);
}


TEST_F(PowerManagementTestSuite, RtcAlarmWakePendingHandling) {
    PowerManager& pm = PowerManager::getInstance();
    pm.init(dummy_rtc, dummy_i2c, dummy_uart, dummy_usb, &sys_context);
    PowerManager::rtc_alarm_handler(dummy_rtc, 0, 0, &pm);
    
    testing::internal::CaptureStdout();
    pm.processFSM();
    const auto raw_output = testing::internal::GetCapturedStdout();
    EXPECT_TRUE(std::string_view(raw_output).find("=== RTC Wakeup Triggered ===") != std::string_view::npos);
}

extern void power_monitor_thread();

TEST_F(PowerManagementTestSuite, ThreadRoutineCoverage) {
    mocks.rtc_ready = false;
    testing::internal::CaptureStdout();
    power_monitor_thread();
    const auto raw_output1 = testing::internal::GetCapturedStdout();
    EXPECT_TRUE(std::string_view(raw_output1).find("Power Manager Init Failed. Thread halting.") != std::string_view::npos);
    
    mocks.rtc_ready = true;
    
    // Enable threading hook to run EXACTLY twice (Covers True, then False ternary branches)
    run_thread_once = true; 
    
    mocks.stack_space_ret = 0; 
    mocks.stack_space_calls = 0;
    power_monitor_thread();
    EXPECT_EQ(mocks.stack_space_calls, 2); 
    
    run_thread_once = false; 
    mocks.stack_space_ret = -1; 
    mocks.stack_space_calls = 0;
    power_monitor_thread();
    EXPECT_EQ(mocks.stack_space_calls, 1);
}

TEST_F(PowerManagementTestSuite, StopExitRemainingErrorBranches) {
    PowerManager& pm = PowerManager::getInstance();
    pm.init(dummy_rtc, dummy_i2c, dummy_uart, dummy_usb, &sys_context);

    // --- STOP *entry* with cancel err = -ENOTSUP (previously only tested on exit) ---
    virtual_uptime = 30000;
    pm.processFSM(); // ACTIVE -> IDLE
    mocks.counter_cancel_ret = -ENOTSUP;
    virtual_uptime = 35000;
    pm.processFSM(); // IDLE -> STOP, enter() bypasses -ENOTSUP silently
    EXPECT_EQ(std::string_view(pm.current_state->getName()), std::string_view("STOP"));

    // --- STOP *exit* with cancel err = -ETIME (previously only tested on entry) ---
    mocks.counter_cancel_ret = -ETIME;
    pm.reportActivity(); // STOP -> ACTIVE, exit() bypasses -ETIME silently
    EXPECT_EQ(std::string_view(pm.current_state->getName()), std::string_view("ACTIVE"));

    // --- STOP *exit* with a genuine, unmapped cancel error + I2C resume failure ---
    // This closes the previously 0-count LOG_WRN line, and I2C's resume
    // failure LOG_ERR + reportPmFailure() lines right after it. UART/USB
    // resume succeed here so this isolates the I2C-specific branch.
    virtual_uptime = 65000;
    pm.processFSM(); // ACTIVE -> IDLE
    mocks.counter_cancel_ret = 0;
    virtual_uptime = 70000;
    pm.processFSM(); // IDLE -> STOP (clean entry)
    EXPECT_EQ(std::string_view(pm.current_state->getName()), std::string_view("STOP"));

    mocks.counter_cancel_ret = -EIO;   // unmapped -> must log on exit
    mocks.pm_i2c_resume_ret = -EIO;    // I2C resume failure -> must log + reportPmFailure()

    testing::internal::CaptureStdout();
    pm.reportActivity(); // STOP -> ACTIVE
    const auto raw_output = testing::internal::GetCapturedStdout();
    std::string_view output(raw_output);

    EXPECT_TRUE(output.find("Failed to cancel RTC alarm on STOP exit (err: -5)") != std::string_view::npos);
    EXPECT_TRUE(output.find("Failed to resume I2C (err: -5)") != std::string_view::npos);
    EXPECT_EQ(std::string_view(pm.current_state->getName()), std::string_view("ACTIVE"));
    EXPECT_EQ(pm.consecutive_pm_failures, 1);
}

// NEW: isolates the UART resume failure branch in StopState::exit() --
// I2C and USB resume succeed, only UART fails.
TEST_F(PowerManagementTestSuite, StopExitUartResumeFailureReportsPmFailure) {
    PowerManager& pm = PowerManager::getInstance();
    pm.init(dummy_rtc, dummy_i2c, dummy_uart, dummy_usb, &sys_context);

    virtual_uptime = 30000;
    pm.processFSM(); // ACTIVE -> IDLE
    virtual_uptime = 35000;
    pm.processFSM(); // IDLE -> STOP
    EXPECT_EQ(std::string_view(pm.current_state->getName()), std::string_view("STOP"));

    mocks.pm_uart_resume_ret = -EIO;

    testing::internal::CaptureStdout();
    pm.reportActivity(); // STOP -> ACTIVE
    const auto raw_output = testing::internal::GetCapturedStdout();
    std::string_view output(raw_output);

    EXPECT_TRUE(output.find("Failed to resume UART (err: -5)") != std::string_view::npos);
    EXPECT_EQ(std::string_view(pm.current_state->getName()), std::string_view("ACTIVE"));
    EXPECT_EQ(pm.consecutive_pm_failures, 1);

    // All three resumes were still attempted despite UART's failure.
    EXPECT_EQ(mocks.usb_resume_calls, 1);
    EXPECT_EQ(mocks.uart_resume_calls, 1);
    EXPECT_EQ(mocks.i2c_resume_calls, 1);
}

// NEW: isolates the USB resume failure branch in StopState::exit() --
// I2C and UART resume succeed, only USB fails.
TEST_F(PowerManagementTestSuite, StopExitUsbResumeFailureReportsPmFailure) {
    PowerManager& pm = PowerManager::getInstance();
    pm.init(dummy_rtc, dummy_i2c, dummy_uart, dummy_usb, &sys_context);

    virtual_uptime = 30000;
    pm.processFSM(); // ACTIVE -> IDLE
    virtual_uptime = 35000;
    pm.processFSM(); // IDLE -> STOP
    EXPECT_EQ(std::string_view(pm.current_state->getName()), std::string_view("STOP"));

    mocks.pm_usb_resume_ret = -EIO;

    testing::internal::CaptureStdout();
    pm.reportActivity(); // STOP -> ACTIVE
    const auto raw_output = testing::internal::GetCapturedStdout();
    std::string_view output(raw_output);

    EXPECT_TRUE(output.find("Failed to resume USB (err: -5)") != std::string_view::npos);
    EXPECT_EQ(std::string_view(pm.current_state->getName()), std::string_view("ACTIVE"));
    EXPECT_EQ(pm.consecutive_pm_failures, 1);

    EXPECT_EQ(mocks.usb_resume_calls, 1);
    EXPECT_EQ(mocks.uart_resume_calls, 1);
    EXPECT_EQ(mocks.i2c_resume_calls, 1);
}

TEST_F(PowerManagementTestSuite, NullFaultContextBranches) {
    PowerManager& pm = PowerManager::getInstance();

    // --- reportPmFailure(): drive 3 consecutive failures with fault_context == nullptr ---
    pm.init(dummy_rtc, dummy_i2c, dummy_uart, dummy_usb, nullptr);
    mocks.counter_set_ret = -1;
    virtual_uptime = 30000;
    pm.processFSM(); // ACTIVE -> IDLE
    virtual_uptime += 5000;
    pm.processFSM(); // STOP fails (1)
    virtual_uptime += 5000;
    pm.processFSM(); // STOP fails (2)

    testing::internal::CaptureStdout();
    virtual_uptime += 5000;
    pm.processFSM(); // STOP fails (3) -> hits threshold, fault_context is null
    const auto raw_output1 = testing::internal::GetCapturedStdout();
    EXPECT_TRUE(std::string_view(raw_output1).find("Power management failure threshold reached") != std::string_view::npos);
    EXPECT_NE(sys_context.current_state, SystemState::SAFE_HALT); // triggerFault() never called

    // --- transitionTo(): cascaded IDLE fallback also fails, with fault_context == nullptr ---
    pm.resetForTest();
    pm.init(dummy_rtc, dummy_i2c, dummy_uart, dummy_usb, nullptr);
    mocks.counter_set_ret = 0;
    virtual_uptime = 30000;
    pm.processFSM(); // ACTIVE -> IDLE

    virtual_uptime = 40000;
    mocks.counter_ticks_ret = 0;      // force STOP entry to fail
    g_force_idle_enter_fail = true;   // force IDLE fallback to also fail

    testing::internal::CaptureStdout();
    pm.processFSM();
    const auto raw_output2 = testing::internal::GetCapturedStdout();
    EXPECT_TRUE(std::string_view(raw_output2).find("Power manager halted. Idle fallback failed.") != std::string_view::npos);
    EXPECT_EQ(pm.current_state, nullptr);
}

namespace {

// Forces genuine contention on the very first call to a singleton's
// getInstance(), so the compiler's double-checked-locking guard has a
// real chance to take its "lost the race, someone already finished
// construction" branch. This must run before ANY other code in the
// process calls the same getInstance() - after the first call ever,
// the guard is permanently initialized and this branch becomes
// unreachable for the rest of the process lifetime.
template <typename Fn>
void RaceForFirstInit(Fn get_instance, int num_threads = 64) {
    std::atomic<bool> start{false};
    std::vector<std::thread> threads;
    threads.reserve(num_threads);
    for (int i = 0; i < num_threads; ++i) {
        threads.emplace_back([&] {
            while (!start.load(std::memory_order_acquire)) {
                std::this_thread::yield();
            }
            get_instance();
        });
    }
    start.store(true, std::memory_order_release);
    for (auto& t : threads) t.join();
}

class SingletonRaceEnvironment : public ::testing::Environment {
public:
    void SetUp() override {
        RaceForFirstInit([] { return &PowerManager::getInstance(); });
        RaceForFirstInit([] { return &ActiveState::getInstance(); });
        RaceForFirstInit([] { return &IdleState::getInstance(); });
        RaceForFirstInit([] { return &StopState::getInstance(); });
    }
};

::testing::Environment* const race_env =
    ::testing::AddGlobalTestEnvironment(new SingletonRaceEnvironment);

}  // namespace
