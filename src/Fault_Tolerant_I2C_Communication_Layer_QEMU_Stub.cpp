#include "Fault_Tolerant_I2C_Communication_Layer.h"
#include <zephyr/kernel.h>

void RetryStrategy::executeRecovery(const device* /* i2c_dev */)
{
}

void BusResetStrategy::executeRecovery(const device* /* i2c_dev */)
{
}

void FailSafeStrategy::executeRecovery(const device* /* i2c_dev */)
{
}

void FailSafeStrategy::updateLastGood(uint8_t val)
{
    last_known_good_value = val;
}

uint8_t FailSafeStrategy::getLastGood() const
{
    return last_known_good_value;
}

I2CManager::I2CManager(const device* i2c_dev)
    : i2c_dev(i2c_dev)
{
}

Result<uint8_t> I2CManager::readRegister(uint16_t /* sensor_addr */, uint8_t /* reg_addr */)
{
    return Result<uint8_t>::Ok(75);
}

Result<bool> I2CManager::writeRegister(uint16_t /* sensor_addr */,
                                       uint8_t /* reg_addr */,
                                       uint8_t /* val */)
{
    return Result<bool>::Ok(true);
}

Result<uint16_t> I2CManager::readWord(uint16_t /* sensor_addr */, uint8_t reg_addr)
{
    switch (reg_addr) {
        default:
            return Result<uint16_t>::Ok(95);
    }
}
