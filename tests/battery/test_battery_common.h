// test_battery_common.h
// Shared includes, extern declarations and the SmartBatteryTestSuite fixture for the Battery gtest files.
// The mock/global definitions live in test_battery_mocks.cpp (link it into the test binary exactly once).
#pragma once

#include <gtest/gtest.h>
#include <array>
#include <cstring>
#include <string_view>
#include <cstdint>
#include <thread>
#include <atomic>
#include <chrono>
#include <string>

#include "Fault_Tolerant_I2C_Communication_Layer.h"
#include "Power_Management_System.h"

#define private public
#define protected public
#include "Smart_Battery_System.h"
#include "Persistent_Configuration_System.h"
#undef private
#undef protected

extern int g_i2c_force_errno;
extern int g_i2c_call_counter;
extern int g_i2c_fail_on_call_n;
extern int g_i2c_fail_on_call_errno;
extern uint32_t virtual_uptime;
extern DeviceContext sys_context;
extern int g_adc_sequence_init_dt_errno;
extern int test_iterations_remaining;
extern int g_mutex_lock_force_errno;
extern bool g_adc_ready_mock;
extern int g_adc_read_force_errno;
extern int g_mutex_lock_call_counter;
extern int g_mutex_lock_fail_on_call_n;
extern void* g_mutex_lock_target_ptr;
extern int g_mutex_lock_target_call_counter;
extern int g_mutex_lock_target_fail_on_call_n;

// ---- globals defined in test_battery_mocks.cpp ----
extern uint16_t g_i2c_mock_read_val;
extern uint16_t g_i2c_mock_v_val;
extern uint16_t g_i2c_mock_i_val;
extern int32_t g_adc_mock_mv_val;
extern int g_adc_raw_to_mv_errno;
extern int g_i2c_consecutive_failures;
extern int g_i2c_fail_after_reads;
extern int g_nvs_write_force_errno;
extern int g_set_safe_halt_on_i2c_call;
extern int g_sleep_on_i2c_call;

extern I2CManager i2c_manager;
extern bool custom_hook_called;
void custom_watchdog_hook(void);

// ---- production entry points exercised directly by the tests (Battery_Runtime.cpp) ----
extern void bms_comm_thread(void);
extern void battery_monitor_thread(void);
extern void resetI2CCacheForTests();

class SmartBatteryTestSuite : public ::testing::Test {
protected:
    SbsBattery battery{&i2c_manager, &sys_context};

    void SetUp() override {   
        resetI2CCacheForTests();
        sys_context = DeviceContext();
        sys_context.requestTransition(SystemState::RUNNING);

        virtual_uptime = 10000;
        g_i2c_force_errno = 0;
        g_i2c_call_counter = 0;
        g_i2c_fail_on_call_n = 0;
        g_i2c_fail_after_reads = -1;
        g_i2c_mock_read_val = 0;
        g_i2c_mock_v_val = 10000;
        g_i2c_mock_i_val = 0;
        g_i2c_consecutive_failures = 0;
        g_adc_ready_mock = true;
        g_adc_mock_mv_val = 2500;
        g_adc_raw_to_mv_errno = 0;
        g_mutex_lock_force_errno = 0;
        custom_hook_called = false;
        g_adc_sequence_init_dt_errno = 0;
        g_mutex_lock_call_counter = 0;
        g_mutex_lock_fail_on_call_n = 0;
        g_mutex_lock_target_ptr = nullptr;
        g_mutex_lock_target_call_counter = 0;
        g_mutex_lock_target_fail_on_call_n = 0;
        g_nvs_write_force_errno = 0;
        g_set_safe_halt_on_i2c_call = -1;
        g_sleep_on_i2c_call = -1;

        battery.cache = BmsCache{};
        battery.cache.valid = true;
        battery.cache.last_error = CommFault::NONE;
        battery.cache.timestamp_ms = k_uptime_get_32();
        battery.cache.temperature.value = Thermistor::KELVIN_OFFSET_TENTHS + 13;

        battery.current_state = BatteryFSM::IDLE;
        battery.consecutive_comm_failures = 0;
        battery.consecutive_mutex_failures = 0;
        battery.last_valid_comm_time = k_uptime_get_32();
        battery.soc_initialized = false;
        battery.accumulated_uAh = 0;
        battery.consecutive_jump_rejects = 0;
        battery.rest_period_start_ms = 0;
    }
};
