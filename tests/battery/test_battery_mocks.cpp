// test_battery_mocks.cpp
// Definitions of every global, Zephyr-API mock (NVS, I2C, ADC) and helper shared by the Battery gtest files.
// Link this file into the test binary exactly once.

#include "test_battery_common.h"

uint16_t g_i2c_mock_read_val = 0;
uint16_t g_i2c_mock_v_val = 10000;
uint16_t g_i2c_mock_i_val = 0;
int32_t g_adc_mock_mv_val = 2500;
int g_adc_raw_to_mv_errno = 0;
int g_i2c_consecutive_failures = 0;
int g_i2c_fail_after_reads = -1;
int g_nvs_write_force_errno = 0;
int g_set_safe_halt_on_i2c_call = -1;
int g_sleep_on_i2c_call = -1;

extern "C" {
    int nvs_mount(struct nvs_fs *fs) {
        return 0;
    }
    ssize_t nvs_write(struct nvs_fs *fs, uint16_t id, const void *data, size_t len) {
    if (g_nvs_write_force_errno != 0) return g_nvs_write_force_errno;
    return len;
    }

    ssize_t nvs_read(struct nvs_fs *fs, uint16_t id, void *data, size_t len) {
        return -ENOENT;
    }

    int i2c_burst_read(const struct device *dev, uint16_t dev_addr, uint8_t start_addr, uint8_t *buf, uint32_t num_bytes) {
        ++g_i2c_call_counter;
        if (g_i2c_force_errno != 0) return g_i2c_force_errno;
        if (g_i2c_fail_on_call_n != 0 && g_i2c_call_counter == g_i2c_fail_on_call_n) return g_i2c_fail_on_call_errno;
        if (g_i2c_consecutive_failures > 0) {
            g_i2c_consecutive_failures--;
            return -EIO;
        }
        if (g_i2c_fail_after_reads >= 0 && g_i2c_call_counter > g_i2c_fail_after_reads) return -EIO;

        if (buf && num_bytes >= 2) {
            uint16_t val = g_i2c_mock_read_val;

            if (start_addr == 0x02) {
                val = g_i2c_mock_v_val;
            } else if (start_addr == 0x04) {
                val = g_i2c_mock_i_val;
            }

            buf[0] = (val >> 8) & 0xFF;
            buf[1] = val & 0xFF;
        } else if (buf && num_bytes > 0) {
            memset(buf, 0, num_bytes);
        }
        return 0;
    }

    int i2c_write_read(const struct device *dev, uint16_t addr, const void *write_buf, size_t num_write, void *read_buf, size_t num_read) {
        ++g_i2c_call_counter;
        if (g_i2c_force_errno != 0) return g_i2c_force_errno;
        if (g_i2c_fail_on_call_n != 0 && g_i2c_call_counter == g_i2c_fail_on_call_n) return g_i2c_fail_on_call_errno;
        if (g_i2c_consecutive_failures > 0) {
            g_i2c_consecutive_failures--;
            return -EIO;
        }
        if (g_i2c_fail_after_reads >= 0 && g_i2c_call_counter > g_i2c_fail_after_reads) return -EIO;

        if (read_buf && num_read >= 2) {
            uint8_t* b = static_cast<uint8_t*>(read_buf);
            uint16_t val = g_i2c_mock_read_val;

            if (write_buf && num_write >= 1) {
                uint8_t reg = static_cast<const uint8_t*>(write_buf)[0];
                if (reg == 0x02) val = g_i2c_mock_v_val;
                else if (reg == 0x04) val = g_i2c_mock_i_val;
            }

            b[0] = (val >> 8) & 0xFF;
            b[1] = val & 0xFF;
        } else if (read_buf && num_read > 0) {
            memset(read_buf, 0, num_read);
        }
        return 0;
    }

    int i2c_write(const struct device *dev, const uint8_t *buf, uint32_t num_bytes, uint16_t addr) {
        ++g_i2c_call_counter;
        if (g_set_safe_halt_on_i2c_call > 0 && g_i2c_call_counter == g_set_safe_halt_on_i2c_call) {sys_context.requestTransition(SystemState::SAFE_HALT);}
        if (g_sleep_on_i2c_call > 0 && g_i2c_call_counter == g_sleep_on_i2c_call) {PowerManager::getInstance().notifyBeforeSleep();}
        if (g_i2c_force_errno != 0) return g_i2c_force_errno;
        if (g_i2c_fail_on_call_n != 0 && g_i2c_call_counter == g_i2c_fail_on_call_n) return g_i2c_fail_on_call_errno;
        if (g_i2c_fail_after_reads >= 0 && g_i2c_call_counter > g_i2c_fail_after_reads) return -EIO;
        return 0;
    }

    int adc_raw_to_millivolts_dt(const struct adc_dt_spec *spec, int32_t *val) {
        if (g_adc_raw_to_mv_errno != 0) return g_adc_raw_to_mv_errno;
        if (val) *val = g_adc_mock_mv_val;
        return 0;
    }
}

static const device dummy_i2c_dev;
I2CManager i2c_manager(&dummy_i2c_dev);

bool custom_hook_called = false;
void custom_watchdog_hook(void) { custom_hook_called = true; }
