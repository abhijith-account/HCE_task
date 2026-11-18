#include "INA226_Driver.h"
#include <zephyr/sys/byteorder.h>

namespace INA226 {

namespace {
Result<int16_t> decodeBigEndianWord(const Result<uint16_t>& r) {
    if (!r.isOk()) {
        return Result<int16_t>::Err(r.error);
    }
    const uint16_t raw_val = r.unwrap();
    const int16_t swapped = static_cast<int16_t>((raw_val << 8U) | (raw_val >> 8U));
    return {swapped, r.error, true, r.degraded, r.timestamp_ms, r.reliability_score};
}
} // namespace

Driver::Driver(I2CManager* i2c):i2c(i2c){}

bool Driver::init(){
    if (i2c==nullptr){
        return false;
    }
    const Result<bool> cfg=i2c->writeWord(I2C_ADDR,REG_CONFIG,CONFIG_VALUE);
    if (!cfg.isOk()){
        return false;
    }
    return i2c->writeWord(I2C_ADDR,REG_CALIBRATION,CALIBRATION_VALUE).isOk();
}

Result<int16_t> Driver::readBusVoltageRaw(){
    return decodeBigEndianWord(i2c->readWord(I2C_ADDR, REG_BUS_VOLT));
}

Result<int16_t> Driver::readShuntVoltageRaw(){
    return decodeBigEndianWord(i2c->readWord(I2C_ADDR, REG_SHUNT_VOLT));
}

Result<int16_t> Driver::readCurrentRaw(){
    return decodeBigEndianWord(i2c->readWord(I2C_ADDR, REG_CURRENT));
}

}
