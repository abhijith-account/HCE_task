#include <cstdint>

#include "Fault_Tolerant_I2C_Communication_Layer.h"

namespace INA226 {
  static constexpr uint16_t I2C_ADDR=0x40U;
  static constexpr uint8_t REG_CONFIG=0x00U;
  static constexpr uint8_t REG_SHUNT_VOLT=0x01U;
  static constexpr uint8_t REG_BUS_VOLT=0x02U;
  static constexpr uint8_t REG_POWER=0x03U;
  static constexpr uint8_t REG_CURRENT=0x04U;
  static constexpr uint8_t REG_CALIBRATION=0x05U;
  static constexpr uint16_t CONFIG_VALUE=0x4527U;
  static constexpr uint32_t R_SHUNT_MILLIOHMS=100U;
  static constexpr uint32_t MAX_MEASURABLE_CURRENT_UA=81920000U/R_SHUNT_MILLIOHMS;
  static constexpr uint32_t  CURRENT_LSB_UA=MAX_MEASURABLE_CURRENT_UA/32768U;
  static constexpr uint16_t CALIBRATION_VALUE=static_cast<uint16_t>(5120000ULL/(static_cast<uint64_t>(CURRENT_LSB_UA)*R_SHUNT_MILLIOHMS));
  
  class Driver {
  public:
    explicit Driver(I2CManager* i2c);
    bool init();
    
    Result<int16_t> readBusVoltageRaw();
    Result<int16_t> readCurrentRaw();
    Result<int16_t> readShuntVoltageRaw();
    
  private:
    I2CManager* i2c;
  };
}
