// test_power_common.h
// Shared declarations for the Power_* gtest files: includes, the mock controller, the
// action log, the test observer and the PowerManagementTestSuite fixture.
// Definitions of the mocks themselves live in test_power_mocks.cpp.
#pragma once

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

// ---- globals defined in test_power_mocks.cpp / the production code under test ----
extern bool run_thread_once;
extern bool g_force_idle_enter_fail;
extern bool g_force_init_fail;
extern void (*g_sim_timer_cb)(struct k_timer*);
extern DeviceContext sys_context;

extern const struct device* dummy_rtc;
extern const struct device* dummy_i2c;
extern const struct device* dummy_uart;
extern const struct device* dummy_usb;
extern const struct device* dummy_adc;

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

extern MockController mocks;

enum class PmAction {
    NONE, LOCK_ACQUIRED, LOCK_RELEASED,
    I2C_SUSPENDED, I2C_RESUMED,
    UART_SUSPENDED, UART_RESUMED,
    USB_SUSPENDED, USB_RESUMED,
    RTC_SET
};

extern std::array<PmAction, 64> action_history;
extern size_t action_idx;
extern void (*mock_rtc_alarm_cb)(const device*, uint16_t, void*);
extern void* mock_rtc_user_data;

void record_action(PmAction a);

// Deterministic hooks used to steer power_monitor_thread() into the
// `s_in_idle_state == true` branch without any thread races.
//  - hook_uptime_bump_on_active_enter: applied inside ActiveState::enter()
//    (first pm_policy_state_lock_get), AFTER last_activity_time was stored, so
//    the very next processFSM() sees ACTIVE_TIMEOUT_MS elapsed -> goes to Idle.
//  - hook_uptime_bump_on_idle_uart_suspend: applied inside IdleState::enter()
//    (UART suspend), AFTER s_idle_entry_time_ms was captured, so the thread
//    sees a larger idle_elapsed than the FSM did.
extern uint32_t hook_uptime_bump_on_active_enter;
extern uint32_t hook_uptime_bump_on_idle_uart_suspend;

extern "C" {
    extern bool mock_is_in_isr_val;
    extern k_tid_t mock_k_current_get_val;
}

// ---- production entry points exercised directly by the tests ----
extern void power_monitor_thread();
extern bool pm_device_suspend_tolerant(const struct device* dev, const char* name);
extern bool pm_device_resume_tolerant(const struct device* dev, const char* name);
extern struct k_work_delayable usb_reconnect_work;
extern struct k_timer mps2_stop_timer;

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
