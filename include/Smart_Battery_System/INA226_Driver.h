#include <cstdint>

#include "Fault_Tolerant_I2C_Communication_Layer.h"

namespace INA226 {
  constexpr uint16_t I2C_ADDR=0x40U;
  constexpr uint8_t REG_CONFIG=0x00U;
  constexpr uint8_t REG_SHUNT_VOLT=0x01U;
  constexpr uint8_t REG_BUS_VOLT=0x02U;
  constexpr uint8_t REG_POWER=0x03U;
  constexpr uint8_t REG_CURRENT=0x04U;
  constexpr uint8_t REG_CALIBRATION=0x05U;
  constexpr uint16_t CONFIG_VALUE=0x4527U;
  constexpr uint32_t R_SHUNT_MILLIOHMS=100U;
  constexpr uint32_t MAX_MEASURABLE_CURRENT_UA=81920000U/R_SHUNT_MILLIOHMS;
  constexpr uint32_t  CURRENT_LSB_UA=MAX_MEASURABLE_CURRENT_UA/32768U;
  constexpr uint16_t CALIBRATION_VALUE=static_cast<uint16_t>(5120000ULL/(static_cast<uint64_t>(CURRENT_LSB_UA)*R_SHUNT_MILLIOHMS));
  
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
