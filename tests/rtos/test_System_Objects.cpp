// test_System_Objects.cpp
// Tests for System_Objects.cpp: the SystemObjects::power() accessor, and that SystemObjects::i2c() hands back a ready, readable
// I2C manager (the two I2C tests are readiness checks of the accessor; they assert on the mocked device).

#include "test_rtos_common.h"

TEST_F(RTOSCommandsTestSuite, SystemObjectsPowerAccessor) {
    EXPECT_EQ(&SystemObjects::power(), &PowerManager::getInstance());
}

TEST_F(RTOSCommandsTestSuite, DiagnosticI2C) {

    const device* dev = SystemObjects::i2c().i2c_dev;
    ASSERT_NE(dev, nullptr) << "i2c_dev is nullptr!";

    ASSERT_TRUE(device_is_ready(dev))
        << "device_is_ready() returned false. g_device_ready_override = "
        << g_device_ready_override;

    const uint16_t addr = static_cast<uint16_t>(SensorID::LPS22HB);
    auto res = SystemObjects::i2c().read24Bit(addr, SensorReg::LPS_P_DESC.reg);
    ASSERT_TRUE(res.isOk())
        << "read24Bit failed. Error = " << static_cast<int>(res.error);
}

TEST_F(RTOSCommandsTestSuite, PinpointI2cFailure) {
    const device* dev = SystemObjects::i2c().i2c_dev;
    ASSERT_NE(dev, nullptr) << "i2c_dev is nullptr!";
    ASSERT_TRUE(device_is_ready(dev))
        << "device_is_ready() returns false. g_device_ready_override = "
        << g_device_ready_override;
    SUCCEED();
}
