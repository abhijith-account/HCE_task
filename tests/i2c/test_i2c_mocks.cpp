#include "test_i2c_common.h"

int mock_i2c_err_code = 0;
int mock_i2c_transient_failures = 0;
uint8_t mock_read_data[8] = {0};

extern "C" {
int i2c_reg_read_byte(const struct device *dev, uint16_t dev_addr, uint8_t reg_addr, uint8_t *value) {
    (void)dev; (void)dev_addr; (void)reg_addr;
    if (mock_i2c_transient_failures > 0) {
        --mock_i2c_transient_failures;
        return -EAGAIN;
    }
    if (value && mock_i2c_err_code == 0) {
        *value = mock_read_data[0];
    }
    return mock_i2c_err_code;
}

int i2c_reg_write_byte(const struct device *dev, uint16_t dev_addr, uint8_t reg_addr, uint8_t value) {
    (void)dev; (void)dev_addr; (void)reg_addr; (void)value;
    if (mock_i2c_transient_failures > 0) {
        --mock_i2c_transient_failures;
        return -EAGAIN;
    }
    return mock_i2c_err_code;
}

int i2c_write(const struct device *dev, const uint8_t *buf, uint32_t num_bytes, uint16_t addr) {
    (void)dev; (void)buf; (void)num_bytes; (void)addr;
    if (mock_i2c_transient_failures > 0) {
        --mock_i2c_transient_failures;
        return -EAGAIN;
    }
    return mock_i2c_err_code;
}

int i2c_burst_read(const struct device *dev, uint16_t dev_addr, uint8_t start_addr, uint8_t *buff, uint32_t num_bytes) {
    (void)dev; (void)dev_addr; (void)start_addr;
    if (mock_i2c_transient_failures > 0) {
        --mock_i2c_transient_failures;
        return -EAGAIN;
    }
    if (buff && mock_i2c_err_code == 0 && num_bytes >= 2) {
        for (uint32_t i = 0; i < num_bytes; i++) {
            buff[i] = mock_read_data[i];
        }
    }
    return mock_i2c_err_code;
}
}

const device* mock_dev = reinterpret_cast<const device*>(0xDEADBEEF);
