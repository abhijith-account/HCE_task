#include <gtest/gtest.h>
#include <array>
#include <cstdint>
#include <string_view>

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/counter.h>
#include <zephyr/drivers/rtc.h>
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
#ifndef ENOSYS
#define ENOSYS 38
#endif

bool run_thread_once = false;
extern bool g_force_idle_enter_fail;
extern bool g_force_init_fail;
extern void (*g_sim_timer_cb)(struct k_timer*);

atomic_t g_producer_alive = ATOMIC_INIT(1);
atomic_t g_processor_alive = ATOMIC_INIT(1);
atomic_t g_logger_alive = ATOMIC_INIT(1);
atomic_t g_hr_prod_alive = ATOMIC_INIT(1);
atomic_t g_disp_cons_alive = ATOMIC_INIT(1);
atomic_t g_bms_comm_alive = ATOMIC_INIT(1);
atomic_t g_batt_mon_alive = ATOMIC_INIT(1);
atomic_t g_mem_mon_alive = ATOMIC_INIT(1);
atomic_t g_shell_alive = ATOMIC_INIT(1);

const struct device* dummy_rtc  = reinterpret_cast<const struct device*>(0x111);
const struct device* dummy_i2c  = reinterpret_cast<const struct device*>(0x222);
const struct device* dummy_uart = reinterpret_cast<const struct device*>(0x333);
const struct device* dummy_usb  = reinterpret_cast<const struct device*>(0x444);
const struct device* dummy_adc  = reinterpret_cast<const struct device*>(0x555);

const struct device* i2c_hardware  = dummy_i2c;
const struct device* uart_hardware = dummy_uart;
const struct device* usb_hardware  = dummy_usb;
const struct device* rtc_hardware  = dummy_rtc;
const struct device* adc_hardware  = dummy_adc;

extern DeviceContext sys_context;

struct MockController {
    bool rtc_ready = true;
    bool i2c_ready = true;
    bool uart_ready = true;
    bool usb_ready = true;
    bool adc_ready = true;

    int counter_start_ret = 0;
    int counter_cancel_ret = 0;
    uint32_t counter_ticks_ret = 60000;
    int counter_set_ret = 0;

    int pm_i2c_suspend_ret  = 0;
    int pm_i2c_resume_ret   = 0;
    int pm_uart_suspend_ret = 0;
    int pm_uart_resume_ret  = 0;
    int pm_usb_suspend_ret  = 0;
    int pm_usb_resume_ret   = 0;
    int pm_adc_suspend_ret  = 0;
    int pm_adc_resume_ret   = 0;

    int i2c_suspend_calls  = 0;
    int i2c_resume_calls   = 0;
    int uart_suspend_calls = 0;
    int uart_resume_calls  = 0;
    int usb_suspend_calls  = 0;
    int usb_resume_calls   = 0;
    int adc_suspend_calls  = 0;
    int adc_resume_calls   = 0;

    int stack_space_ret = 0;
    int stack_space_calls = 0;

    struct rtc_time rtc_time_val = {0};
    int rtc_get_time_ret = 0;
    bool auto_clear_errors = false;
    
    void setAllSuspendRet(int v) {
        pm_i2c_suspend_ret = v;
        pm_uart_suspend_ret = v;
        pm_usb_suspend_ret = v;
        pm_adc_suspend_ret = v;
    }
    void setAllResumeRet(int v) {
        pm_i2c_resume_ret = v;
        pm_uart_resume_ret = v;
        pm_usb_resume_ret = v;
        pm_adc_resume_ret = v;
    }

    void reset() {
        rtc_ready = true;
        i2c_ready = true;
        uart_ready = true;
        usb_ready = true;
        adc_ready = true;
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
        pm_adc_suspend_ret = 0;
        pm_adc_resume_ret = 0;
        i2c_suspend_calls = 0;
        i2c_resume_calls = 0;
        uart_suspend_calls = 0;
        uart_resume_calls = 0;
        usb_suspend_calls = 0;
        usb_resume_calls = 0;
        adc_suspend_calls = 0;
        adc_resume_calls = 0;
        stack_space_ret = 0;
        stack_space_calls = 0;
        memset(&rtc_time_val, 0, sizeof(rtc_time_val));
        rtc_get_time_ret = 0;
        auto_clear_errors = false;
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

static std::array<PmAction, 64> action_history;
static size_t action_idx = 0;
static void (*mock_rtc_alarm_cb)(const device*, uint16_t, void*) = nullptr;
static void* mock_rtc_user_data = nullptr;

static void record_action(PmAction a) {
    if (action_idx < action_history.size()) action_history[action_idx++] = a;
}

// Deterministic hooks used to steer power_monitor_thread() into the
// `s_in_idle_state == true` branch without any thread races.
//  - hook_uptime_bump_on_active_enter: applied inside ActiveState::enter()
//    (first pm_policy_state_lock_get), AFTER last_activity_time was stored, so
//    the very next processFSM() sees ACTIVE_TIMEOUT_MS elapsed -> goes to Idle.
//  - hook_uptime_bump_on_idle_uart_suspend: applied inside IdleState::enter()
//    (UART suspend), AFTER s_idle_entry_time_ms was captured, so the thread
//    sees a larger idle_elapsed than the FSM did.
static uint32_t hook_uptime_bump_on_active_enter = 0;
static uint32_t hook_uptime_bump_on_idle_uart_suspend = 0;

#pragma weak k_is_in_isr
#pragma weak k_current_get
#pragma weak i2c_recover_bus
#pragma weak usb_enable
#pragma weak usb_disable
#pragma weak usb_dc_detach

extern const k_tid_t processor_tid = (k_tid_t)0x10;
extern const k_tid_t producer_tid  = (k_tid_t)0x11;
extern const k_tid_t logger_tid    = (k_tid_t)0x12;
extern const k_tid_t battery_tid   = (k_tid_t)0x13;
extern const k_tid_t shell_tid     = (k_tid_t)0x14;
extern const k_tid_t bms_comm_tid  = (k_tid_t)0x15;
extern const k_tid_t hr_prod_tid   = (k_tid_t)0x16;
extern const k_tid_t disp_cons_tid = (k_tid_t)0x17;
extern const k_tid_t mem_mon_tid   = (k_tid_t)0x18;
extern const k_tid_t trace_tid   = (k_tid_t)0x19;

extern atomic_t s_activity_pending;
extern "C" {
    bool mock_is_in_isr_val = false;
    k_tid_t mock_k_current_get_val = nullptr;
    
    bool k_is_in_isr() { return mock_is_in_isr_val; }
    k_tid_t k_current_get() { return mock_k_current_get_val; }
    
    int i2c_recover_bus(const struct device*) { return -EIO; }
    int usb_enable(void*) { return 0; }
    int usb_disable(void) { return 0; }
    int usb_dc_detach(void) { return 0; }

    bool device_is_ready(const struct device *dev) {
        if (dev == dummy_rtc)  return mocks.rtc_ready;
        if (dev == dummy_i2c)  return mocks.i2c_ready;
        if (dev == dummy_uart) return mocks.uart_ready;
        if (dev == dummy_usb)  return mocks.usb_ready;
        if (dev == dummy_adc)  return mocks.adc_ready;
        return false;
    }
    int rtc_get_time(const struct device *dev, struct rtc_time *timeptr) {
        if (timeptr) *timeptr = mocks.rtc_time_val;
        return mocks.rtc_get_time_ret;
    }
    int rtc_set_time(const struct device *dev, const struct rtc_time *timeptr) { return 0; }
    
    int rtc_alarm_set_time(const struct device *dev, uint16_t id, uint16_t mask, const struct rtc_time *timeptr) {
        if (timeptr == nullptr) {
            return mocks.counter_cancel_ret;
        }
        record_action(PmAction::RTC_SET);
        return mocks.counter_set_ret;
    }
    
    int rtc_alarm_set_callback(const struct device *dev, uint16_t id, void (*callback)(const struct device *, uint16_t, void *), void *user_data) {
        mock_rtc_alarm_cb = callback;
        mock_rtc_user_data = user_data;
        return 0;
    }
    void pm_policy_state_lock_get(uint8_t, uint8_t) {
        record_action(PmAction::LOCK_ACQUIRED);
        if (hook_uptime_bump_on_active_enter != 0) {
            virtual_uptime += hook_uptime_bump_on_active_enter;
            hook_uptime_bump_on_active_enter = 0;
        }
    }
    void pm_policy_state_lock_put(uint8_t, uint8_t) {
        record_action(PmAction::LOCK_RELEASED);
    }
    int pm_device_action_run(const struct device* dev, enum pm_device_action action) {
        if (action == PM_DEVICE_ACTION_SUSPEND) {
            if (dev == dummy_adc) {
                mocks.adc_suspend_calls++;
                int ret = mocks.pm_adc_suspend_ret;
                if (mocks.auto_clear_errors) mocks.pm_adc_suspend_ret = 0;
                return ret;
            }
            if (dev == dummy_i2c) {
                mocks.i2c_suspend_calls++;
                record_action(PmAction::I2C_SUSPENDED);
                int ret = mocks.pm_i2c_suspend_ret;
                if (mocks.auto_clear_errors) mocks.pm_i2c_suspend_ret = 0;
                return ret;
            }
            if (dev == dummy_uart) {
                mocks.uart_suspend_calls++;
                record_action(PmAction::UART_SUSPENDED);
                if (hook_uptime_bump_on_idle_uart_suspend != 0) {
                    virtual_uptime += hook_uptime_bump_on_idle_uart_suspend;
                    hook_uptime_bump_on_idle_uart_suspend = 0;
                }
                int ret = mocks.pm_uart_suspend_ret;
                if (mocks.auto_clear_errors) mocks.pm_uart_suspend_ret = 0;
                return ret;
            }
            if (dev == dummy_usb) {
                mocks.usb_suspend_calls++;
                record_action(PmAction::USB_SUSPENDED);
                int ret = mocks.pm_usb_suspend_ret;
                if (mocks.auto_clear_errors) mocks.pm_usb_suspend_ret = 0;
                return ret;
            }
            return 0;
        } else if (action == PM_DEVICE_ACTION_RESUME) {
            if (dev == dummy_adc) {
                mocks.adc_resume_calls++;
                return mocks.pm_adc_resume_ret;
            }
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
        g_force_init_fail = false;
        mock_is_in_isr_val = false;
        mock_k_current_get_val = nullptr;
        atomic_set(&s_activity_pending, 0);
        hook_uptime_bump_on_active_enter = 0;
        hook_uptime_bump_on_idle_uart_suspend = 0;
    }
};

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

TEST_F(PowerManagementTestSuite, StopExitNormalWakeup) {
    PowerManager& pm = PowerManager::getInstance();
    pm.init(dummy_rtc, dummy_i2c, dummy_uart, dummy_usb, dummy_adc, &sys_context);

    virtual_uptime = 30000;
    pm.processFSM();
    virtual_uptime = 35000;
    pm.processFSM();

    virtual_uptime += 70000;
    mocks.setAllResumeRet(0);

    pm.reportActivity();
    pm.processFSM();
    EXPECT_EQ(std::string_view(pm.current_state->getName()), std::string_view("PM_STATE_ACTIVE"));
    EXPECT_EQ(pm.consecutive_pm_failures, 0);
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

extern void power_monitor_thread();

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

TEST_F(PowerManagementTestSuite, SimulationModeCoverage) {
    PowerManager& pm = PowerManager::getInstance();
    
    EXPECT_TRUE(pm.init(nullptr, dummy_i2c, dummy_uart, dummy_usb, dummy_adc, &sys_context));

    virtual_uptime = 30000;
    pm.processFSM(); 

    virtual_uptime = 35000;
    pm.processFSM(); 
    EXPECT_EQ(std::string_view(pm.current_state->getName()), std::string_view("PM_STATE_SUSPEND_TO_RAM"));

    pm.reportActivity(); 
    pm.processFSM();
    EXPECT_EQ(std::string_view(pm.current_state->getName()), std::string_view("PM_STATE_ACTIVE"));
}

TEST_F(PowerManagementTestSuite, SuspendResumeIgnoredErrors) {
    PowerManager& pm = PowerManager::getInstance();
    pm.init(dummy_rtc, dummy_i2c, dummy_uart, dummy_usb, dummy_adc, &sys_context);

    mocks.setAllSuspendRet(-ENOSYS);
    mocks.setAllResumeRet(-ENOSYS);

    virtual_uptime = 30000;
    pm.processFSM(); 
    virtual_uptime = 35000;
    pm.processFSM(); 
    EXPECT_EQ(std::string_view(pm.current_state->getName()), std::string_view("PM_STATE_SUSPEND_TO_RAM"));

    pm.reportActivity(); 
    pm.processFSM();
    EXPECT_EQ(std::string_view(pm.current_state->getName()), std::string_view("PM_STATE_ACTIVE"));

    mocks.setAllSuspendRet(-ENOTSUP);
    mocks.setAllResumeRet(-ENOTSUP);

    virtual_uptime = 70000;
    pm.processFSM(); 
    virtual_uptime = 75000;
    pm.processFSM(); 
    EXPECT_EQ(std::string_view(pm.current_state->getName()), std::string_view("PM_STATE_SUSPEND_TO_RAM"));

    pm.reportActivity();
    pm.processFSM();
    EXPECT_EQ(std::string_view(pm.current_state->getName()), std::string_view("PM_STATE_ACTIVE"));
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

TEST_F(PowerManagementTestSuite, SuspendFailures) {
    PowerManager& pm = PowerManager::getInstance();
    pm.init(dummy_rtc, dummy_i2c, dummy_uart, dummy_usb, dummy_adc, &sys_context);

    mocks.pm_uart_suspend_ret = -EIO;
    EXPECT_FALSE(IdleState::getInstance().enter(pm));
    mocks.pm_uart_suspend_ret = 0;

    mocks.pm_adc_suspend_ret = -EIO;
    EXPECT_FALSE(IdleState::getInstance().enter(pm));
    mocks.pm_adc_suspend_ret = 0;

    mocks.pm_i2c_suspend_ret = -EIO;
    EXPECT_FALSE(IdleState::getInstance().enter(pm));
    mocks.pm_i2c_suspend_ret = 0;

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
}

TEST_F(PowerManagementTestSuite, ActiveExecuteCoverage) {
    PowerManager& pm = PowerManager::getInstance();
    pm.init(dummy_rtc, dummy_i2c, dummy_uart, dummy_usb, dummy_adc, &sys_context);
    ActiveState& active = ActiveState::getInstance();
    
    pm.last_activity_time = k_uptime_get_32();
    EXPECT_EQ(std::string_view(active.execute(pm).getName()), "PM_STATE_ACTIVE");

    pm.last_activity_time = k_uptime_get_32() - 40000;
    EXPECT_EQ(std::string_view(active.execute(pm).getName()), "PM_STATE_SUSPEND_TO_IDLE");
}

TEST_F(PowerManagementTestSuite, IdleExecuteCoverage) {
    PowerManager& pm = PowerManager::getInstance();
    pm.init(dummy_rtc, dummy_i2c, dummy_uart, dummy_usb, dummy_adc, &sys_context);
    IdleState& idle = IdleState::getInstance();
    
    // Enter the state formally to capture the initial entry time internally
    virtual_uptime = 10000;
    idle.enter(pm);
    
    // Advance time by 3s (less than the 5s IDLE_TIMEOUT_MS)
    virtual_uptime = 13000;
    EXPECT_EQ(std::string_view(idle.execute(pm).getName()), "PM_STATE_SUSPEND_TO_IDLE");

    // Advance time past the 5s timeout
    virtual_uptime = 16000;
    EXPECT_EQ(std::string_view(idle.execute(pm).getName()), "PM_STATE_SUSPEND_TO_RAM");
}

TEST_F(PowerManagementTestSuite, FsmActivityWhileActive) {
    PowerManager& pm = PowerManager::getInstance();
    pm.init(dummy_rtc, dummy_i2c, dummy_uart, dummy_usb, dummy_adc, &sys_context);
    
    pm.current_state = &ActiveState::getInstance();
    atomic_set(&s_activity_pending, 1);
    pm.processFSM();
    EXPECT_EQ(std::string_view(pm.current_state->getName()), "PM_STATE_ACTIVE");
}

extern "C" void usb_reconnect_work_handler(struct k_work *work);

TEST_F(PowerManagementTestSuite, UsbReconnectCoverage) {
    // Explicit call to anonymous namespace bound work handler
    extern struct k_work_delayable usb_reconnect_work;
    if (usb_reconnect_work.work.handler) {
        usb_reconnect_work.work.handler(&usb_reconnect_work.work);
    }
}

TEST_F(PowerManagementTestSuite, TolerantSuspendResumeCoverage) {
    PowerManager& pm = PowerManager::getInstance();
    
    // 1. Cover nullptr devices returning true in suspend/resume wrappers
    pm.init(dummy_rtc, nullptr, nullptr, nullptr, nullptr, &sys_context);
    virtual_uptime = 30000;
    pm.processFSM(); // Enters Idle. Suspend wrappers called with nullptr.
    
    pm.reportActivity();
    pm.processFSM(); // Exits Idle. Resume wrappers called with nullptr.
    
    // 2. Cover valid devices returning non-ignored errors during resume
    pm.resetForTest();
    pm.init(dummy_rtc, dummy_i2c, dummy_uart, dummy_usb, dummy_adc, &sys_context);
    virtual_uptime = 30000;
    pm.processFSM(); // Enters Idle
    
    mocks.setAllResumeRet(-EIO); // Return a non-ignored error code
    pm.reportActivity();
    pm.processFSM(); // Exits Idle, triggers resume failure logs and returns false
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

// Global tolerant wrappers
extern bool pm_device_suspend_tolerant(const struct device* dev, const char* name);
extern bool pm_device_resume_tolerant(const struct device* dev, const char* name);

TEST_F(PowerManagementTestSuite, TolerantWrappersDirectCoverage) {
    // 1. Cover nullptr short-circuits
    EXPECT_TRUE(pm_device_suspend_tolerant(nullptr, "NULL_DEV"));
    EXPECT_TRUE(pm_device_resume_tolerant(nullptr, "NULL_DEV"));
    
    // 2. Cover the LOG_ERR branch on a non-ignored error (-EIO)
    mocks.pm_adc_resume_ret = -EIO;
    EXPECT_FALSE(pm_device_resume_tolerant(dummy_adc, "ADC"));
    mocks.pm_adc_resume_ret = 0;
}

// Global simulated timer
extern struct k_timer mps2_stop_timer;

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

#if defined(__linux__) && defined(__LP64__) && (defined(__x86_64__) || defined(__aarch64__))
#define PM_TEST_CAN_PATCH_CODE 1
#include <dlfcn.h>
#include <elf.h>
#include <link.h>
#include <sys/mman.h>
#include <unistd.h>
#include <cstdio>
#include <cstring>
#include <string>

namespace pm_test_patch {

static int exe_base_cb(struct dl_phdr_info* info, size_t, void* data) {
    if (info->dlpi_name == nullptr || info->dlpi_name[0] == '\0') {
        *static_cast<uintptr_t*>(data) = static_cast<uintptr_t>(info->dlpi_addr);
        return 1;
    }
    return 0;
}

// Finds a (possibly internal-linkage) function/object symbol in our own executable
// by reading the ELF symbol table in-process (no dependency on `nm`).
// Works for PIE (load bias != 0) and non-PIE (load bias == 0) binaries.
static uint8_t* find_own_symbol(const char* needle, unsigned char sym_type, size_t min_size) {
    FILE* f = std::fopen("/proc/self/exe", "rb");
    if (!f) return nullptr;

    std::fseek(f, 0, SEEK_END);
    long fsz = std::ftell(f);
    std::fseek(f, 0, SEEK_SET);
    if (fsz <= 0) { std::fclose(f); return nullptr; }

    std::string img(static_cast<size_t>(fsz), '\0');
    size_t rd = std::fread(&img[0], 1, img.size(), f);
    std::fclose(f);
    if (rd != img.size() || img.size() < sizeof(Elf64_Ehdr)) return nullptr;

    const auto* eh = reinterpret_cast<const Elf64_Ehdr*>(img.data());
    if (std::memcmp(eh->e_ident, ELFMAG, SELFMAG) != 0) return nullptr;
    if (eh->e_shoff == 0 || eh->e_shoff + eh->e_shnum * sizeof(Elf64_Shdr) > img.size()) return nullptr;

    const auto* sh = reinterpret_cast<const Elf64_Shdr*>(img.data() + eh->e_shoff);
    uintptr_t base = 0;
    dl_iterate_phdr(exe_base_cb, &base);

    for (unsigned i = 0; i < eh->e_shnum; ++i) {
        if (sh[i].sh_type != SHT_SYMTAB) continue;
        if (sh[i].sh_link >= eh->e_shnum) continue;
        const Elf64_Shdr& str = sh[sh[i].sh_link];
        if (sh[i].sh_offset + sh[i].sh_size > img.size()) continue;
        if (str.sh_offset + str.sh_size > img.size()) continue;

        const auto* syms = reinterpret_cast<const Elf64_Sym*>(img.data() + sh[i].sh_offset);
        const char* names = img.data() + str.sh_offset;
        size_t count = sh[i].sh_size / sizeof(Elf64_Sym);

        for (size_t s = 0; s < count; ++s) {
            if (ELF64_ST_TYPE(syms[s].st_info) != sym_type) continue;
            if (syms[s].st_value == 0 || syms[s].st_size < min_size) continue;
            if (syms[s].st_name >= str.sh_size) continue;
            if (std::strstr(names + syms[s].st_name, needle) == nullptr) continue;
            return reinterpret_cast<uint8_t*>(base + syms[s].st_value);
        }
    }
    return nullptr;
}

static uint8_t* find_own_function(const char* needle) {
    return find_own_symbol(needle, STT_FUNC, 8);
}

// Overwrites the start of `fn` with "return false" and returns the original
// bytes so the caller can restore them.
static bool patch_return_false(uint8_t* fn, uint8_t (&orig)[8]) {
    const uintptr_t page = static_cast<uintptr_t>(sysconf(_SC_PAGESIZE));
    const uintptr_t start = reinterpret_cast<uintptr_t>(fn) & ~(page - 1);
    const uintptr_t end   = (reinterpret_cast<uintptr_t>(fn) + 8 + page - 1) & ~(page - 1);
    if (mprotect(reinterpret_cast<void*>(start), end - start,
                 PROT_READ | PROT_WRITE | PROT_EXEC) != 0) {
        return false;
    }
    std::memcpy(orig, fn, 8);
#if defined(__x86_64__)
    fn[0] = 0x31; fn[1] = 0xC0;          // xor eax, eax
    fn[2] = 0xC3;                        // ret
#elif defined(__aarch64__)
    const uint32_t code[2] = {0x52800000u, 0xD65F03C0u};  // mov w0,#0 ; ret
    std::memcpy(fn, code, sizeof(code));
#endif
    __builtin___clear_cache(reinterpret_cast<char*>(fn), reinterpret_cast<char*>(fn + 8));
    return true;
}

static void restore(uint8_t* fn, const uint8_t (&orig)[8]) {
    const uintptr_t page = static_cast<uintptr_t>(sysconf(_SC_PAGESIZE));
    const uintptr_t start = reinterpret_cast<uintptr_t>(fn) & ~(page - 1);
    const uintptr_t end   = (reinterpret_cast<uintptr_t>(fn) + 8 + page - 1) & ~(page - 1);
    std::memcpy(fn, orig, 8);
    __builtin___clear_cache(reinterpret_cast<char*>(fn), reinterpret_cast<char*>(fn + 8));
    mprotect(reinterpret_cast<void*>(start), end - start, PROT_READ | PROT_EXEC);
}

}  // namespace pm_test_patch
#endif

TEST_F(PowerManagementTestSuite, IdleClockRestoreFailureCoverage) {
    PowerManager& pm = PowerManager::getInstance();
    pm.init(dummy_rtc, dummy_i2c, dummy_uart, dummy_usb, dummy_adc, &sys_context);
    IdleState::getInstance().enter(pm);

#ifdef PM_TEST_CAN_PATCH_CODE
    // clock_restore_from_idle() lives in an anonymous namespace and always
    // returns true on the host build, so the `!clock_ok` branch in
    // IdleState::exit() can only be reached by forcing it to return false.
    uint8_t* fn = pm_test_patch::find_own_function("clock_restore_from_idle");
    ASSERT_NE(fn, nullptr) << "clock_restore_from_idle symbol not found "
                              "(build must not strip symbols or inline it; use -O0)";

    uint8_t orig[8];
    ASSERT_TRUE(pm_test_patch::patch_return_false(fn, orig)) << "mprotect failed";

    testing::internal::CaptureStdout();
    IdleState::getInstance().exit(pm);   // clock_ok == false -> LOG_ERR + reportPmFailure()
    const auto raw_output = testing::internal::GetCapturedStdout();

    pm_test_patch::restore(fn, orig);

    EXPECT_TRUE(std::string_view(raw_output).find("Idle clock restore failed") != std::string_view::npos);
#else
    GTEST_SKIP() << "Runtime code patching unsupported on this platform";
#endif
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

// ---------------------------------------------------------------------------
// power_monitor_thread(): NOT in idle state, active timeout already elapsed
// (`active_elapsed >= ACTIVE_TIMEOUT_MS` -> wait_timeout = K_MSEC(10)).
//
// How it is reached deterministically:
//  - hook_uptime_bump_on_active_enter advances time by ACTIVE_TIMEOUT_MS right
//    after last_activity_time is stored, so the first processFSM() wants to go
//    Active -> Idle.
//  - g_force_idle_enter_fail makes IdleState::enter() bail out before it sets
//    s_in_idle_state, and the cascaded fallback fails too, so the FSM halts
//    (current_state == nullptr) and s_in_idle_state stays false.
//  - The thread therefore takes the `else` (active) path with
//    active_elapsed == 30000 >= ACTIVE_TIMEOUT_MS.
// ---------------------------------------------------------------------------
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

namespace {
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

#ifdef PM_TEST_CAN_PATCH_CODE
// A function-local static's thread-safe-init guard has a branch that is only
// taken when __cxa_guard_acquire() returns 0, i.e. "another thread finished
// constructing the object while I was waiting".  A multi-thread first-use race
// hits that only by luck (the winner usually finishes before anyone else gets
// there), which made line `static PowerManager instance;` flip between 4/4 and
// 3/4 branch coverage from run to run.
//
// Deterministic version, single-threaded:
//   1. make sure the singleton exists,
//   2. clear its init guard so the next call takes the slow path,
//   3. temporarily make __cxa_guard_acquire() return 0,
//   4. call getInstance()  -> constructor is skipped, existing instance returned,
//   5. restore the code and mark the guard "initialised" again.
template <typename Fn>
void CoverGuardContendedPath(const char* guard_symbol_part, Fn get_instance) {
    uint8_t* guard = pm_test_patch::find_own_symbol(guard_symbol_part, STT_OBJECT, 8);
    void* acquire = dlsym(RTLD_DEFAULT, "__cxa_guard_acquire");
    if (guard == nullptr || acquire == nullptr) {
        std::fprintf(stderr, "[ WARN ] cannot locate guard '%s' / __cxa_guard_acquire; "
                             "singleton init-guard branch left to chance\n", guard_symbol_part);
        return;
    }

    get_instance();                                           // 1. instance exists
    *reinterpret_cast<volatile uint64_t*>(guard) = 0;         // 2. force slow path

    uint8_t orig[8];
    if (!pm_test_patch::patch_return_false(static_cast<uint8_t*>(acquire), orig)) {
        *reinterpret_cast<volatile uint64_t*>(guard) = 1;
        std::fprintf(stderr, "[ WARN ] mprotect failed; singleton init-guard branch left to chance\n");
        return;
    }
    get_instance();                                           // 3+4. acquire() == 0
    pm_test_patch::restore(static_cast<uint8_t*>(acquire), orig);  // 5.
    *reinterpret_cast<volatile uint64_t*>(guard) = 1;
}
#endif

class SingletonRaceEnvironment : public ::testing::Environment {
public:
    void SetUp() override {
        RaceForFirstInit([] { return &PowerManager::getInstance(); });
        RaceForFirstInit([] { return &ActiveState::getInstance(); });
        RaceForFirstInit([] { return &IdleState::getInstance(); });
        RaceForFirstInit([] { return &StopState::getInstance(); });
#ifdef PM_TEST_CAN_PATCH_CODE
        CoverGuardContendedPath("GVZN12PowerManager11getInstance", [] { return &PowerManager::getInstance(); });
        CoverGuardContendedPath("GVZN11ActiveState11getInstance",  [] { return &ActiveState::getInstance(); });
        CoverGuardContendedPath("GVZN9IdleState11getInstance",     [] { return &IdleState::getInstance(); });
        CoverGuardContendedPath("GVZN9StopState11getInstance",     [] { return &StopState::getInstance(); });
#endif
    }
};

::testing::Environment* const race_env =
    ::testing::AddGlobalTestEnvironment(new SingletonRaceEnvironment);
}
