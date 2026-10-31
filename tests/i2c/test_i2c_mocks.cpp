// test_i2c_mocks.cpp
// Definitions of the mocked Zephyr I2C driver calls (strong overrides of the weak ones in mock_globals.cpp) and the shared
// mock state used by the I2C gtest files. Link this file into the test binary exactly once.

#include "test_i2c_common.h"

int mock_i2c_err_code = 0;
uint8_t mock_read_data[8] = {0};

extern "C" {
    int i2c_reg_read_byte(const struct device *dev, uint16_t dev_addr, uint8_t reg_addr, uint8_t *value){
       if (value && mock_i2c_err_code == 0) {
          *value = mock_read_data[0];
       }
       return mock_i2c_err_code;
    }

    int i2c_reg_write_byte(const struct device *dev, uint16_t dev_addr, uint8_t reg_addr, uint8_t value){
       return mock_i2c_err_code;
    }

    int i2c_burst_read(const struct device *dev, uint16_t dev_addr, uint8_t start_addr, uint8_t *buff, uint32_t num_bytes){
      if (buff && mock_i2c_err_code == 0 && num_bytes >= 2){
          for(uint32_t i=0; i<num_bytes; i++) {
              buff[i] = mock_read_data[i];
          }
      }
      return mock_i2c_err_code;
    }
}

const device* mock_dev = reinterpret_cast<const device*>(0xDEADBEEF);
