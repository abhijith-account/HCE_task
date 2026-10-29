// test_rtos_mocks.cpp
// Definitions of every global and Zephyr-API mock (I2C burst read, k_current_get) shared by the RTOS gtest files.
// Link this file into the test binary exactly once.

#include "test_rtos_common.h"

int test_iterations_remaining = 0;
bool run_thread_once = false;
static const struct device dummy_dev;

__attribute__((weak)) DeviceContext sys_context;
I2CManager i2c_manager(&dummy_dev);
__attribute__((weak)) PowerManager pwr_manager;

std::array<int, 2> execution_order{};
std::atomic<size_t> exec_index{0};
std::atomic<uintptr_t> g_spoofed_current_thread{0};

uint16_t g_i2c_mock_word_val = 0; 
extern "C" {
    int i2c_burst_read(const struct device *dev, uint16_t dev_addr,
                        uint8_t start_addr, uint8_t *buf, uint32_t num_bytes) {
        ++g_i2c_call_counter;
        if (g_i2c_force_errno != 0) return g_i2c_force_errno;
        if (g_i2c_fail_on_call_n != 0 && g_i2c_call_counter == g_i2c_fail_on_call_n) {
            return g_i2c_fail_on_call_errno;
        }
        if (buf && num_bytes == 2) {
            buf[0] = g_i2c_mock_word_val & 0xFF;
            buf[1] = (g_i2c_mock_word_val >> 8) & 0xFF;
        } else if (buf && num_bytes > 0) {
            memset(buf, 0, num_bytes);   // Triple/Block reads unaffected -- same as before
        }
        return 0;
    }
}

extern "C" k_tid_t k_current_get(void) {
    return reinterpret_cast<k_tid_t>(g_spoofed_current_thread.load());
}
