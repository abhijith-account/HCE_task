#pragma once

#include <gtest/gtest.h>
#include <zephyr/kernel.h>
#include <array>
#include <atomic>
#include <new>
#include <string>
#include <string_view>
#include <thread>
#include <chrono>
#include <cstdint>
#include <cstddef>
#include <cstdio>
#include <fstream>
#include <pthread.h>
#include <sys/mman.h>
#include <unistd.h>

#define private public
#define protected public
#include "RTOS_Command_Based_Thread_System.h"
#undef private
#undef protected

#include "Static_Memory+MISRA_Compliance_Layer.h"
#include "Device_State_Machine+Watchdog.h"
#include "Power_Management_System.h"
#include "Fault_Tolerant_I2C_Communication_Layer.h"

extern "C" {
    void producer_thread(void *arg1, void *arg2, void *arg3);
    void processor_thread(void *arg1, void *arg2, void *arg3);
    void logger_thread(void *arg1, void *arg2, void *arg3);
}

#define producer_thread()  producer_thread(nullptr, nullptr, nullptr)
#define processor_thread() processor_thread(nullptr, nullptr, nullptr)
#define logger_thread()    logger_thread(nullptr, nullptr, nullptr)

extern int test_iterations_remaining;
extern bool run_thread_once;
extern int g_i2c_call_counter;
extern int g_i2c_fail_on_call_n;
extern int g_i2c_fail_on_call_errno;

extern DeviceContext sys_context;
extern I2CManager i2c_manager;
extern PowerManager pwr_manager;
extern void resetRtosCommandTestState() noexcept;
extern void resetI2CCacheForTests() noexcept;

extern std::array<int, 2> execution_order;
extern std::atomic<size_t> exec_index;
extern std::atomic<uintptr_t> g_spoofed_current_thread;

extern uint16_t g_i2c_mock_word_val;
extern int g_i2c_force_errno;

class PreemptionTestCmd final : public ICommand {
private:
    int thread_priority;
public:
    PreemptionTestCmd(int prio) : thread_priority(prio) {}

    void execute() noexcept override final {
        size_t idx = exec_index.fetch_add(1);
        if (idx < execution_order.size()) {
            execution_order[idx] = thread_priority;
        }
    }
};
extern bool g_device_ready_override;

namespace {
    extern BME280Calibration g_bme280Calib;
}

extern void resetSensorReadCmdLastValsForTest() noexcept;

inline void drainAndDestroy(k_msgq* q) {
    ICommand* cmd = nullptr;
    while (k_msgq_get(q, &cmd, K_NO_WAIT) == 0) {
        cmd->destroy();
    }
}

class RTOSCommandsTestSuite : public ::testing::Test {
protected:
    void SetUp() override {
    sys_context = DeviceContext();
    drainAndDestroy(PROCESSOR_Q);
    drainAndDestroy(LOGGER_Q);
    g_queueStats = QueueStats{};
    exec_index = 0;
    execution_order.fill(0);

    resetRtosCommandTestState();
    resetI2CCacheForTests();
    resetSensorReadCmdLastValsForTest(); 
    g_i2c_force_errno = 0;
    g_i2c_call_counter = 0;
    g_i2c_fail_on_call_n = 0;
    g_device_ready_override = true;
    g_i2c_mock_word_val = 0; 
}

};

inline void msSleep(int ms) { std::this_thread::sleep_for(std::chrono::milliseconds(ms)); }
