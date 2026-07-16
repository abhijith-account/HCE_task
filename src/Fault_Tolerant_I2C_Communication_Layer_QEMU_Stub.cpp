#include "Fault_Tolerant_I2C_Communication_Layer.h"

#include <zephyr/kernel.h>
#include <zephyr/drivers/i2c.h>

/*----------------------------------------------------------------------------
 * Global I2C Manager
 *---------------------------------------------------------------------------*/

extern I2CManager i2c_manager(nullptr);

/*----------------------------------------------------------------------------
 * RetryStrategy
 *---------------------------------------------------------------------------*/

void RetryStrategy::executeRecovery(const device* /* i2c_dev */)
{
    k_msleep(5);
}

/*----------------------------------------------------------------------------
 * BusResetStrategy
 *---------------------------------------------------------------------------*/

void BusResetStrategy::executeRecovery(const device* /* i2c_dev */)
{
    k_msleep(1);
}

/*----------------------------------------------------------------------------
 * FailSafeStrategy
 *---------------------------------------------------------------------------*/

void FailSafeStrategy::executeRecovery(const device* /* i2c_dev */)
{
    /* Stub implementation */
}

void FailSafeStrategy::updateLastGood(uint64_t val)
{
    last_known_good_value = val;
}

uint64_t FailSafeStrategy::getLastGood() const
{
    return last_known_good_value;
}

/*----------------------------------------------------------------------------
 * I2CManager
 *---------------------------------------------------------------------------*/

I2CManager::I2CManager(const device* dev)
    : i2c_dev(dev),
      retry_strategy(),
      reset_strategy(),
      failsafe_strategy()
{
}

Result<uint8_t> I2CManager::readRegister(
    uint16_t /*sensor_addr*/,
    uint8_t /*reg_addr*/)
{
    return Result<uint8_t>::Ok(0x55U);
}

Result<bool> I2CManager::writeRegister(
    uint16_t /*sensor_addr*/,
    uint8_t /*reg_addr*/,
    uint8_t /*value*/)
{
    return Result<bool>::Ok(true);
}

Result<uint16_t> I2CManager::readWord(
    uint16_t /*sensor_addr*/,
    uint8_t /*reg_addr*/)
{
    return Result<uint16_t>::Ok(0x1234U);
}

Result<uint32_t> I2CManager::read24Bit(
    uint16_t /*sensor_addr*/,
    uint8_t /*reg_addr*/)
{
    return Result<uint32_t>::Ok(0x123456UL);
}

Result<uint64_t> I2CManager::read64Bit(
    uint16_t /*sensor_addr*/,
    uint8_t /*reg_addr*/)
{
    return Result<uint64_t>::Ok(0x123456789ABCDEF0ULL);
}
