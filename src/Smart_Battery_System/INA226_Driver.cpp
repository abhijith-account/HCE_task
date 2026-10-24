#include "INA226_Driver.h"
#include <zephyr/sys/byteorder.h>

namespace INA226 {
  Driver::Driver(I2CManager* i2c):i2c(i2c){}
  
  bool Driver::init(){
    if (i2c==nullptr){
      return false;
    }
    
    const Result<bool> cfg=i2c->writeWord(I2C_ADDR,REG_CONFIG,sys_cpu_to_be16(CONFIG_VALUE));
    
    if (!cfg.isOk()){
      return false;
    }
    
    return i2c->writeWord(I2C_ADDR,REG_CALIBRATION,sys_cpu_to_be16(CALIBRATION_VALUE)).isOk();
  }
  
  Result<int16_t> Driver::readBusVoltageRaw(){
    const Result<uint16_t> r= i2c->readWord(I2C_ADDR, REG_BUS_VOLT);
    
    if (!r.isOk()){
      return Result<int16_t>::Err(r.error);
    }
    
    uint16_t raw_val=r.unwrap();
    return Result<int16_t>::Ok(static_cast<int16_t>((raw_val<<8)|(raw_val>>8)));
  }
  
  Result<int16_t> Driver::readShuntVoltageRaw(){
    const Result<uint16_t> r=i2c->readWord(I2C_ADDR,REG_SHUNT_VOLT);
    
    if (!r.isOk()){
      return Result<int16_t>::Err(r.error);
    }
    
    uint16_t raw_val=r.unwrap();
    return Result<int16_t>::Ok(static_cast<int16_t>((raw_val<<8)|(raw_val>>8)));
  }
  
  Result<int16_t> Driver::readCurrentRaw(){
    const Result<uint16_t> r=i2c->readWord(I2C_ADDR,REG_CURRENT);
    
    if (!r.isOk()){
      return Result<int16_t>::Err(r.error);
    }
    
    uint16_t raw_val=r.unwrap();
    return Result<int16_t>::Ok(static_cast<int16_t>((raw_val<<8)|(raw_val>>8)));
  }
}
