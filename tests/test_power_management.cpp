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

bool run_thread_once = false;
extern bool g_force_idle_enter_fail;
extern bool g_force_init_fail;

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
static void (*mock_rtc_alarm_cb)(const device*, uint8_t, uint32_t, void*) = nullptr;
static void* mock_rtc_user_data = nullptr;

static void record_action(PmAction a) {
    if (action_idx < action_history.size()) action_history[action_idx++] = a;
}

extern "C" {
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
        return 0;
    }
    void pm_policy_state_lock_get(uint8_t, uint8_t) {
        record_action(PmAction::LOCK_ACQUIRED);
    }
    void pm_policy_state_lock_put(uint8_t, uint8_t) {
        record_action(PmAction::LOCK_RELEASED);
    }
    int pm_device_action_run(const struct device* dev, enum pm_device_action action) {
        if (action == PM_DEVICE_ACTION_SUSPEND) {
            if (dev == dummy_adc) {
                mocks.adc_suspend_calls++;
                return mocks.pm_adc_suspend_ret;
            }
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
    EXPECT_TRUE(output.find("[WRN] RTC device not ready") != std::string_view::npos);
    EXPECT_TRUE(output.find("[WRN] I2C device not ready") != std::string_view::npos);
}

TEST_F(PowerManagementTestSuite, InitFailuresAdcAndRtcSeed) {
    PowerManager& pm = PowerManager::getInstance();
    testing::internal::CaptureStdout();

    mocks.adc_ready = false;
    mocks.rtc_get_time_ret = -1; 

    // Re-added the branch to cover adc_dev == nullptr explicitly.
    EXPECT_TRUE(pm.init(dummy_rtc, dummy_i2c, dummy_uart, dummy_usb, nullptr, &sys_context));
    EXPECT_TRUE(pm.init(dummy_rtc, dummy_i2c, dummy_uart, dummy_usb, dummy_adc, &sys_context));

    const auto raw_output = testing::internal::GetCapturedStdout();
    EXPECT_TRUE(std::string_view(raw_output).find("[WRN] ADC device not ready") != std::string_view::npos);
}

TEST_F(PowerManagementTestSuite, InitFailuresUartUsb) {
    PowerManager& pm = PowerManager::getInstance();
    testing::internal::CaptureStdout();

    EXPECT_TRUE(pm.init(dummy_rtc, dummy_i2c, nullptr, dummy_usb,  dummy_adc, &sys_context));
    EXPECT_TRUE(pm.init(dummy_rtc, dummy_i2c, dummy_uart, nullptr,  dummy_adc, &sys_context));

    mocks.uart_ready = false;
    EXPECT_TRUE(pm.init(dummy_rtc, dummy_i2c, dummy_uart, dummy_usb, dummy_adc, &sys_context));

    mocks.uart_ready = true;
    mocks.usb_ready = false;
    EXPECT_TRUE(pm.init(dummy_rtc, dummy_i2c, dummy_uart, dummy_usb, dummy_adc, &sys_context));

    const auto raw_output = testing::internal::GetCapturedStdout();
    std::string_view output(raw_output);
    EXPECT_TRUE(output.find("[WRN] UART device not ready") != std::string_view::npos);
    EXPECT_TRUE(output.find("[WRN] USB device not ready") != std::string_view::npos);
}

TEST_F(PowerManagementTestSuite, InitSuccess) {
    PowerManager& pm = PowerManager::getInstance();
    testing::internal::CaptureStdout();
    EXPECT_TRUE(pm.init(dummy_rtc, dummy_i2c, dummy_uart, dummy_usb, dummy_adc, &sys_context));
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
    pm.init(dummy_rtc, dummy_i2c, dummy_uart, dummy_usb, dummy_adc, &sys_context);

    pm.processFSM();

    virtual_uptime = 30000;
    pm.processFSM();
    EXPECT_EQ(std::string_view(pm.current_state->getName()), std::string_view("IDLE"));

    virtual_uptime = 32000;
    pm.processFSM();

    virtual_uptime = 35000;
    pm.processFSM();
    EXPECT_EQ(std::string_view(pm.current_state->getName()), std::string_view("STOP"));

    pm.processFSM();
    EXPECT_EQ(std::string_view(pm.current_state->getName()), std::string_view("STOP"));

    EXPECT_EQ(mocks.adc_suspend_calls, 1);
    EXPECT_EQ(mocks.i2c_suspend_calls, 1);
    EXPECT_EQ(mocks.uart_suspend_calls, 0); 
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

    EXPECT_EQ(std::string_view(pm.current_state->getName()), std::string_view("STOP"));
}

TEST_F(PowerManagementTestSuite, StopEntryRtcTimeSecNoOverflow) {
    PowerManager& pm = PowerManager::getInstance();
    pm.init(dummy_rtc, dummy_i2c, dummy_uart, dummy_usb, dummy_adc, &sys_context);

    // Forces tm_sec + 60 to evaluate to false (0 >= 60 is false)
    mocks.rtc_time_val.tm_sec = -60; 

    virtual_uptime = 30000;
    pm.processFSM(); 
    virtual_uptime = 35000;
    pm.processFSM(); 

    EXPECT_EQ(std::string_view(pm.current_state->getName()), std::string_view("STOP"));
}

TEST_F(PowerManagementTestSuite, StopEntryRtcTimeHourNoOverflow) {
    PowerManager& pm = PowerManager::getInstance();
    pm.init(dummy_rtc, dummy_i2c, dummy_uart, dummy_usb, dummy_adc, &sys_context);

    // Forces tm_hour to evaluate to false (11 >= 24 is false) 
    mocks.rtc_time_val.tm_sec = 0;
    mocks.rtc_time_val.tm_min = 59;
    mocks.rtc_time_val.tm_hour = 10;

    virtual_uptime = 30000;
    pm.processFSM(); 
    virtual_uptime = 35000;
    pm.processFSM(); 

    EXPECT_EQ(std::string_view(pm.current_state->getName()), std::string_view("STOP"));
}

TEST_F(PowerManagementTestSuite, StopEntryAndExitEdgeCaseBranches) {
    PowerManager& pm = PowerManager::getInstance();
    pm.init(dummy_rtc, dummy_i2c, dummy_uart, dummy_usb, dummy_adc, &sys_context);

    virtual_uptime = 35000;
    pm.processFSM();

    mocks.counter_cancel_ret = -ETIME;
    virtual_uptime = 40000;
    pm.processFSM();
    EXPECT_EQ(std::string_view(pm.current_state->getName()), std::string_view("STOP"));

    mocks.counter_cancel_ret = -ENOTSUP;
    pm.reportActivity();
    EXPECT_EQ(std::string_view(pm.current_state->getName()), std::string_view("ACTIVE"));

    virtual_uptime = 90000;
    pm.processFSM();
    virtual_uptime = 95000;

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
    pm.init(dummy_rtc, dummy_i2c, dummy_uart, dummy_usb, dummy_adc, &sys_context);

    virtual_uptime = 30000;
    pm.processFSM();
    virtual_uptime = 35000;
    pm.processFSM();

    virtual_uptime += 70000;
    mocks.setAllResumeRet(0);

    testing::internal::CaptureStdout();
    pm.reportActivity();
    const auto raw_output = testing::internal::GetCapturedStdout();

    EXPECT_TRUE(std::string_view(raw_output).find("System Awoken via RTC.") != std::string_view::npos);
    EXPECT_EQ(std::string_view(pm.current_state->getName()), std::string_view("ACTIVE"));
    EXPECT_EQ(pm.consecutive_pm_failures, 0);
}

TEST_F(PowerManagementTestSuite, StopEntryFailuresFallbackToIdle) {
    PowerManager& pm = PowerManager::getInstance();
    pm.init(dummy_rtc, dummy_i2c, dummy_uart, dummy_usb, dummy_adc, &sys_context);

    virtual_uptime = 30000;
    pm.processFSM();
    EXPECT_EQ(std::string_view(pm.current_state->getName()), std::string_view("IDLE"));

    virtual_uptime = 35000;
    mocks.counter_set_ret = -1;
    pm.processFSM();
    EXPECT_EQ(std::string_view(pm.current_state->getName()), std::string_view("IDLE"));

    TestObserver obs;
    pm.registerObserver(&obs);
    
    virtual_uptime = 40000;
    mocks.counter_set_ret = 0;
    mocks.pm_adc_suspend_ret = -EIO;
    pm.processFSM();
    EXPECT_EQ(std::string_view(pm.current_state->getName()), std::string_view("IDLE"));
    EXPECT_EQ(obs.aborts, 1);

    virtual_uptime = 85000;
    pm.consecutive_pm_failures = 0;
    mocks.pm_adc_suspend_ret = 0;
    
    pm.processFSM();
    EXPECT_EQ(std::string_view(pm.current_state->getName()), std::string_view("STOP"));
}

TEST_F(PowerManagementTestSuite, StopEntryAdcSuspendFailureRollsBackRtc) {
    PowerManager& pm = PowerManager::getInstance();
    pm.init(dummy_rtc, dummy_i2c, dummy_uart, dummy_usb, dummy_adc, &sys_context);

    TestObserver obs;
    pm.registerObserver(&obs);

    virtual_uptime = 30000;
    pm.processFSM();

    mocks.pm_adc_suspend_ret = -EIO;

    testing::internal::CaptureStdout();
    virtual_uptime = 35000;
    pm.processFSM();
    const auto raw_output = testing::internal::GetCapturedStdout();
    std::string_view output(raw_output);

    EXPECT_TRUE(output.find("Failed to suspend ADC peripheral") != std::string_view::npos);
    EXPECT_TRUE(output.find("Rolling back.") != std::string_view::npos);

    EXPECT_EQ(std::string_view(pm.current_state->getName()), std::string_view("IDLE"));

    EXPECT_EQ(mocks.adc_suspend_calls, 1);
    EXPECT_EQ(mocks.i2c_suspend_calls, 0);
    EXPECT_EQ(obs.aborts, 1);
}

TEST_F(PowerManagementTestSuite, StopEntryAdcSuspendFailureSimMode) {
    PowerManager& pm = PowerManager::getInstance();
    pm.init(nullptr, dummy_i2c, dummy_uart, dummy_usb, dummy_adc, &sys_context);

    TestObserver obs;
    pm.registerObserver(&obs);

    virtual_uptime = 30000;
    pm.processFSM(); 

    mocks.pm_adc_suspend_ret = -EIO;

    testing::internal::CaptureStdout();
    virtual_uptime = 35000;
    pm.processFSM(); 
    const auto raw_output = testing::internal::GetCapturedStdout();

    EXPECT_TRUE(std::string_view(raw_output).find("Failed to suspend ADC peripheral") != std::string_view::npos);
    EXPECT_EQ(std::string_view(pm.current_state->getName()), std::string_view("IDLE"));
    EXPECT_EQ(mocks.adc_suspend_calls, 1);
    EXPECT_EQ(obs.aborts, 1);
}

TEST_F(PowerManagementTestSuite, StopEntryI2cSuspendFailureRollsBackRtc) {
    PowerManager& pm = PowerManager::getInstance();
    pm.init(dummy_rtc, dummy_i2c, dummy_uart, dummy_usb, dummy_adc, &sys_context);

    TestObserver obs;
    pm.registerObserver(&obs);

    virtual_uptime = 30000;
    pm.processFSM();

    mocks.pm_i2c_suspend_ret = -EIO;

    testing::internal::CaptureStdout();
    virtual_uptime = 35000;
    pm.processFSM();
    const auto raw_output = testing::internal::GetCapturedStdout();
    std::string_view output(raw_output);

    EXPECT_TRUE(output.find("Failed to suspend I2C peripheral") != std::string_view::npos);
    EXPECT_TRUE(output.find("Rolling back.") != std::string_view::npos);

    EXPECT_EQ(std::string_view(pm.current_state->getName()), std::string_view("IDLE"));

    EXPECT_EQ(mocks.adc_suspend_calls, 1);
    EXPECT_EQ(mocks.i2c_suspend_calls, 1);
    EXPECT_EQ(obs.aborts, 1);
}

TEST_F(PowerManagementTestSuite, StopEntryI2cSuspendFailureSimMode) {
    PowerManager& pm = PowerManager::getInstance();
    pm.init(nullptr, dummy_i2c, dummy_uart, dummy_usb, dummy_adc, &sys_context);

    TestObserver obs;
    pm.registerObserver(&obs);

    virtual_uptime = 30000;
    pm.processFSM(); 

    mocks.pm_i2c_suspend_ret = -EIO;

    testing::internal::CaptureStdout();
    virtual_uptime = 35000;
    pm.processFSM(); 
    const auto raw_output = testing::internal::GetCapturedStdout();

    EXPECT_TRUE(std::string_view(raw_output).find("Failed to suspend I2C peripheral") != std::string_view::npos);
    EXPECT_EQ(std::string_view(pm.current_state->getName()), std::string_view("IDLE"));
    EXPECT_EQ(mocks.i2c_suspend_calls, 1);
    EXPECT_EQ(obs.aborts, 1);
}

TEST_F(PowerManagementTestSuite, FaultEscalationSequence) {
    PowerManager& pm = PowerManager::getInstance();
    pm.init(dummy_rtc, dummy_i2c, dummy_uart, dummy_usb, dummy_adc, &sys_context);

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

    pm.init(dummy_rtc, dummy_i2c, dummy_uart, dummy_usb, dummy_adc, &sys_context);

    virtual_uptime = 30000;
    pm.processFSM();

    virtual_uptime = 40000;
    mocks.counter_set_ret = -1; 
    g_force_idle_enter_fail = true;

    testing::internal::CaptureStdout();
    pm.processFSM();

    const auto raw_output = testing::internal::GetCapturedStdout();
    EXPECT_TRUE(std::string_view(raw_output).find("Power manager halted. Idle fallback failed.") != std::string_view::npos);
    EXPECT_EQ(pm.current_state, nullptr);
}

TEST_F(PowerManagementTestSuite, RtcAlarmWakePendingHandling) {
    PowerManager& pm = PowerManager::getInstance();
    pm.init(dummy_rtc, dummy_i2c, dummy_uart, dummy_usb, dummy_adc, &sys_context);
    PowerManager::rtc_alarm_handler(dummy_rtc, 0, &pm);

    testing::internal::CaptureStdout();
    pm.processFSM();
    const auto raw_output = testing::internal::GetCapturedStdout();
    EXPECT_TRUE(std::string_view(raw_output).find("=== RTC Wakeup Triggered ===") != std::string_view::npos);
}

extern void power_monitor_thread();

TEST_F(PowerManagementTestSuite, ThreadRoutineCoverage) {
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

TEST_F(PowerManagementTestSuite, ThreadRoutineCoverageInitFail) {
    // Restored the execution. The SUT patch allows this to pass.
    g_force_init_fail = true;
    testing::internal::CaptureStdout();
    power_monitor_thread();
    const auto raw_output = testing::internal::GetCapturedStdout();
    EXPECT_TRUE(std::string_view(raw_output).find("Power Manager Init Failed. Thread halting.") != std::string_view::npos);
    g_force_init_fail = false;
}

TEST_F(PowerManagementTestSuite, StopExitRemainingErrorBranches) {
    PowerManager& pm = PowerManager::getInstance();
    pm.init(dummy_rtc, dummy_i2c, dummy_uart, dummy_usb, dummy_adc, &sys_context);

    virtual_uptime = 30000;
    pm.processFSM();
    mocks.counter_cancel_ret = -ENOTSUP;
    virtual_uptime = 35000;
    pm.processFSM();
    EXPECT_EQ(std::string_view(pm.current_state->getName()), std::string_view("STOP"));

    mocks.counter_cancel_ret = -ETIME;
    pm.reportActivity();
    EXPECT_EQ(std::string_view(pm.current_state->getName()), std::string_view("ACTIVE"));

    virtual_uptime = 65000;
    pm.processFSM();
    mocks.counter_cancel_ret = 0;
    virtual_uptime = 70000;
    pm.processFSM();
    EXPECT_EQ(std::string_view(pm.current_state->getName()), std::string_view("STOP"));

    mocks.counter_cancel_ret = -EIO;
    mocks.pm_i2c_resume_ret = -EIO;

    testing::internal::CaptureStdout();
    pm.reportActivity();
    const auto raw_output = testing::internal::GetCapturedStdout();
    std::string_view output(raw_output);

    EXPECT_TRUE(output.find("Failed to cancel RTC alarm on STOP exit (err: -5)") != std::string_view::npos);
    EXPECT_TRUE(output.find("Failed to resume I2C (err: -5)") != std::string_view::npos);
    EXPECT_EQ(std::string_view(pm.current_state->getName()), std::string_view("ACTIVE"));
    EXPECT_EQ(pm.consecutive_pm_failures, 1);
}

TEST_F(PowerManagementTestSuite, StopExitAdcResumeFailureReportsPmFailure) {
    PowerManager& pm = PowerManager::getInstance();
    pm.init(dummy_rtc, dummy_i2c, dummy_uart, dummy_usb, dummy_adc, &sys_context);

    virtual_uptime = 30000;
    pm.processFSM();
    virtual_uptime = 35000;
    pm.processFSM();
    EXPECT_EQ(std::string_view(pm.current_state->getName()), std::string_view("STOP"));

    mocks.pm_adc_resume_ret = -EIO;

    testing::internal::CaptureStdout();
    pm.reportActivity();
    const auto raw_output = testing::internal::GetCapturedStdout();
    std::string_view output(raw_output);

    EXPECT_TRUE(output.find("Failed to resume ADC (err: -5)") != std::string_view::npos);
    EXPECT_EQ(std::string_view(pm.current_state->getName()), std::string_view("ACTIVE"));
    EXPECT_EQ(pm.consecutive_pm_failures, 1);

    EXPECT_EQ(mocks.adc_resume_calls, 1);
    EXPECT_EQ(mocks.i2c_resume_calls, 1);
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

    testing::internal::CaptureStdout();
    virtual_uptime += 5000;
    pm.processFSM();
    const auto raw_output1 = testing::internal::GetCapturedStdout();
    EXPECT_TRUE(std::string_view(raw_output1).find("Power management failure threshold reached") != std::string_view::npos);
    EXPECT_NE(sys_context.current_state, SystemState::SAFE_HALT);

    pm.resetForTest();
    pm.init(dummy_rtc, dummy_i2c, dummy_uart, dummy_usb, dummy_adc, nullptr);
    mocks.counter_set_ret = 0;
    virtual_uptime = 30000;
    pm.processFSM();

    virtual_uptime = 40000;
    mocks.counter_set_ret = -1; 
    g_force_idle_enter_fail = true;

    testing::internal::CaptureStdout();
    pm.processFSM();
    const auto raw_output2 = testing::internal::GetCapturedStdout();
    EXPECT_TRUE(std::string_view(raw_output2).find("Power manager halted. Idle fallback failed.") != std::string_view::npos);
    EXPECT_EQ(pm.current_state, nullptr);
}

TEST_F(PowerManagementTestSuite, SimulationModeCoverage) {
    PowerManager& pm = PowerManager::getInstance();
    
    EXPECT_TRUE(pm.init(nullptr, dummy_i2c, dummy_uart, dummy_usb, dummy_adc, &sys_context));

    virtual_uptime = 30000;
    pm.processFSM(); 

    virtual_uptime = 35000;
    pm.processFSM(); 
    EXPECT_EQ(std::string_view(pm.current_state->getName()), std::string_view("STOP"));

    pm.reportActivity(); 
    EXPECT_EQ(std::string_view(pm.current_state->getName()), std::string_view("ACTIVE"));
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
    EXPECT_EQ(std::string_view(pm.current_state->getName()), std::string_view("STOP"));

    pm.reportActivity(); 
    EXPECT_EQ(std::string_view(pm.current_state->getName()), std::string_view("ACTIVE"));

    mocks.setAllSuspendRet(-ENOTSUP);
    mocks.setAllResumeRet(-ENOTSUP);

    virtual_uptime = 70000;
    pm.processFSM(); 
    virtual_uptime = 75000;
    pm.processFSM(); 
    EXPECT_EQ(std::string_view(pm.current_state->getName()), std::string_view("STOP"));

    pm.reportActivity();
    EXPECT_EQ(std::string_view(pm.current_state->getName()), std::string_view("ACTIVE"));
}

TEST_F(PowerManagementTestSuite, StopEntryCancelAlarmError) {
    PowerManager& pm = PowerManager::getInstance();
    pm.init(dummy_rtc, dummy_i2c, dummy_uart, dummy_usb, dummy_adc, &sys_context);

    virtual_uptime = 30000;
    pm.processFSM();

    virtual_uptime = 35000;
    mocks.counter_cancel_ret = -EIO; 

    pm.processFSM(); 
    
    EXPECT_EQ(std::string_view(pm.current_state->getName()), std::string_view("STOP"));
}

TEST_F(PowerManagementTestSuite, SimTimerCallbackCoverage) {
    PowerManager& pm = PowerManager::getInstance();
    pm.init(nullptr, dummy_i2c, dummy_uart, dummy_usb, dummy_adc, &sys_context);

    if (g_sim_timer_cb) {
        g_sim_timer_cb(nullptr); 
        EXPECT_EQ(atomic_get(&pm.wake_pending), 1);
    }
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
}
