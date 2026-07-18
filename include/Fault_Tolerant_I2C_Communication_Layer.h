#include <zephyr/drivers/i2c.h>
#include <cstdint>
#include "Static_Memory+MISRA_Compliance_Layer.h"

#pragma once

enum class I2CFault{NONE,NACK,TIMEOUT,BUS_BUSY,ARBITRATION_LOST,DEVICE_NOT_READY};

template <typename T>
struct Result{
    T value;
    I2CFault error;
    bool success;
    
    static Result<T> Ok(T val){
        return {val,I2CFault::NONE,true};
    }
    
    static Result<T> Err(I2CFault err){
        return {T(),err,false};
    }
    
    bool isOk() const {return success; }
    T unwrap() const{ return value; }
};

class I2CStrategy{
  public:
        virtual ~I2CStrategy()=default;
        virtual void executeRecovery(const device* i2c_dev)=0;
};

class RetryStrategy:public I2CStrategy{
  public:
      void executeRecovery(const device* i2c_dev) override;
};

class BusResetStrategy:public I2CStrategy{
  public:
      void executeRecovery(const device* i2c_dev) override;
};

class FailSafeStrategy:public I2CStrategy{
  private:
      uint64_t last_known_good_value=0U;
  public:
      void executeRecovery(const device* i2c_dev) override;
      void updateLastGood(uint64_t val);
      uint64_t getLastGood() const;
};

class I2CManager{
  private:
      const device* i2c_dev;
      RetryStrategy retry_strategy;
      BusResetStrategy reset_strategy; 
      FailSafeStrategy failsafe_strategy;
  public:
      explicit I2CManager(const device* dev);
      Result<uint8_t> readRegister(uint16_t sensor_addr,uint8_t reg_addr);
      Result<bool> writeRegister(uint16_t sensor_addr,uint8_t reg_addr,uint8_t val);
      Result<uint16_t> readWord(uint16_t sensor_addr,uint8_t reg_addr);
      Result<uint32_t> read24Bit(uint16_t sensor_addr,uint8_t reg_addr);
      Result<uint64_t> read64Bit(uint16_t sensor_addr,uint8_t reg_addr);
};
