// test_power_mocks.cpp
// Definitions of every global, stub and Zephyr-API mock shared by the Power_* gtest files.
// Link this file into every test binary exactly once.

#include "test_power_common.h"

bool run_thread_once = false;

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

MockController mocks;

std::array<PmAction, 64> action_history;
size_t action_idx = 0;
void (*mock_rtc_alarm_cb)(const device*, uint16_t, void*) = nullptr;
void* mock_rtc_user_data = nullptr;

void record_action(PmAction a) {
    if (action_idx < action_history.size()) action_history[action_idx++] = a;
}

uint32_t hook_uptime_bump_on_active_enter = 0;
uint32_t hook_uptime_bump_on_idle_uart_suspend = 0;

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
