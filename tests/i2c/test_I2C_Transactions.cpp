// test_I2C_Transactions.cpp
// Tests for the I2CManager transaction API (Fault_Tolerant_I2C_Communication_Layer.cpp): readRegister / writeRegister / readWord /
// read24Bit / read64Bit / writeWord, errno -> I2CFault mapping, null / not-ready device, the test-env forced errno, the Result<T> type,
// and the lockout gating (sleep, FAULT, SAFE_HALT -> DEVICE_NOT_READY).

#include "test_i2c_common.h"

/* Every bus-facing API must refuse to touch the bus and report DEVICE_NOT_READY. */
static void expectAllApisLockedOut(I2CManager& mgr) {
    auto r8 = mgr.readRegister(1, 2);
    EXPECT_FALSE(r8.success);
    EXPECT_EQ(r8.error, I2CFault::DEVICE_NOT_READY);

    auto w8 = mgr.writeRegister(1, 2, 0x55);
    EXPECT_FALSE(w8.success);
    EXPECT_EQ(w8.error, I2CFault::DEVICE_NOT_READY);

    auto r16 = mgr.readWord(1, 3);
    EXPECT_FALSE(r16.success);
    EXPECT_EQ(r16.error, I2CFault::DEVICE_NOT_READY);

    auto r24 = mgr.read24Bit(1, 4);
    EXPECT_FALSE(r24.success);
    EXPECT_EQ(r24.error, I2CFault::DEVICE_NOT_READY);

    auto r64 = mgr.read64Bit(1, 5);
    EXPECT_FALSE(r64.success);
    EXPECT_EQ(r64.error, I2CFault::DEVICE_NOT_READY);
}

TEST_F(I2CFaultRecoveryTestSuite, HandlesNACKEncoding) {
    Result<uint8_t> res = Result<uint8_t>::Err(I2CFault::NACK);
    EXPECT_FALSE(res.success);
    EXPECT_EQ(res.error, I2CFault::NACK);
    EXPECT_EQ(res.value, 0);
}

TEST_F(I2CFaultRecoveryTestSuite, HandlesTimeoutEncoding) {
      Result<uint16_t> res = Result<uint16_t>::Err(I2CFault::TIMEOUT);
      EXPECT_FALSE(res.success);
      EXPECT_EQ(res.error, I2CFault::TIMEOUT);
}

TEST_F(I2CFaultRecoveryTestSuite, HandlesBusBusyEncoding) {
      Result<bool> res = Result<bool>::Err(I2CFault::BUS_BUSY);
      EXPECT_FALSE(res.success);
      EXPECT_EQ(res.error, I2CFault::BUS_BUSY);
}

TEST_F(I2CFaultRecoveryTestSuite, HandlesArbitrationLostEncoding) {
      Result<uint8_t> res = Result<uint8_t>::Err(I2CFault::ARBITRATION_LOST);
      EXPECT_FALSE(res.success);
      EXPECT_EQ(res.error, I2CFault::ARBITRATION_LOST);
}

TEST_F(I2CFaultRecoveryTestSuite, HandlesSuccessfulRead) {
      uint16_t expected_temp = 3650;
      Result<uint16_t> res = Result<uint16_t>::Ok(expected_temp);
      EXPECT_TRUE(res.success);
      EXPECT_EQ(res.error, I2CFault::NONE);
      EXPECT_EQ(res.value, expected_temp);
}

TEST_F(I2CFaultRecoveryTestSuite, ManagerReadRegisterSuccess){
    I2CManager manager(mock_dev);
    mock_read_data[0] = 42;
    auto res = manager.readRegister(0x10,0x20);
    EXPECT_TRUE(res.success);
    EXPECT_EQ(res.value,42);
}

TEST_F(I2CFaultRecoveryTestSuite, ManagerReadRegisterTimeout){
    I2CManager manager(mock_dev);
    mock_i2c_err_code = -ETIMEDOUT;
    auto res = manager.readRegister(0x10,0x20);
    EXPECT_FALSE(res.success);
    EXPECT_EQ(res.error, I2CFault::TIMEOUT);
}

TEST_F(I2CFaultRecoveryTestSuite, ManagerReadRegisterNACK){
    I2CManager manager(mock_dev);
    mock_i2c_err_code = -EIO;
    auto res = manager.readRegister(0x10,0x20);
    EXPECT_FALSE(res.success);
    EXPECT_EQ(res.error, I2CFault::NACK);
}

TEST_F(I2CFaultRecoveryTestSuite, ManagerWriteRegisterSuccess){
    I2CManager manager(mock_dev);
    auto res = manager.writeRegister(0x10,0x20,0xFF);
    EXPECT_TRUE(res.success);
}

TEST_F(I2CFaultRecoveryTestSuite, ManagerWriteRegisterNACK){
    I2CManager manager(mock_dev);
    mock_i2c_err_code = -EIO;
    auto res = manager.writeRegister(0x10,0x20,0xFF);
    EXPECT_FALSE(res.success);
    EXPECT_EQ(res.error, I2CFault::NACK);
}

TEST_F(I2CFaultRecoveryTestSuite, ManagerReadWordSuccess){
    I2CManager manager(mock_dev);
    mock_read_data[0] = 0xAA;
    mock_read_data[1] = 0xBB;
    auto res = manager.readWord(0x10,0x20);
    EXPECT_TRUE(res.success);
    EXPECT_EQ(res.value, 0xBBAA);
}

TEST_F(I2CFaultRecoveryTestSuite, ManagerReadWordTimeout){
    I2CManager manager(mock_dev);
    mock_i2c_err_code = -ETIMEDOUT;
    auto res = manager.readWord(0x10,0x20);
    EXPECT_FALSE(res.success);
    EXPECT_EQ(res.error, I2CFault::TIMEOUT);
}

TEST_F(I2CFaultRecoveryTestSuite, ManagerReadWordNACK){
    I2CManager manager(mock_dev);
    mock_i2c_err_code = -EIO;
    auto res = manager.readWord(0x10,0x20);
    EXPECT_FALSE(res.success);
    EXPECT_EQ(res.error, I2CFault::NACK);
}

TEST_F(I2CFaultRecoveryTestSuite, ManagerReadRegisterBusBusy) {
    I2CManager manager(mock_dev);
    mock_i2c_err_code = -EBUSY;
    auto res = manager.readRegister(0x10,0x20);
    EXPECT_FALSE(res.success);
    EXPECT_EQ(res.error,I2CFault::BUS_BUSY);
}

TEST_F(I2CFaultRecoveryTestSuite, ManagerReadRegisterArbitrationLost) {
    I2CManager manager(mock_dev);
    mock_i2c_err_code = -EAGAIN;
    auto res = manager.readRegister(0x10,0x20);
    EXPECT_FALSE(res.success);
    EXPECT_EQ(res.error,I2CFault::ARBITRATION_LOST);
}

TEST_F(I2CFaultRecoveryTestSuite, ManagerWriteRegisterBusBusy) {
    I2CManager manager(mock_dev);
    mock_i2c_err_code = -EBUSY;
    auto res = manager.writeRegister(0x10,0x20,0x55);
    EXPECT_FALSE(res.success);
    EXPECT_EQ(res.error,I2CFault::BUS_BUSY);
}

TEST_F(I2CFaultRecoveryTestSuite, ManagerReadWordBusBusy) {
    I2CManager manager(mock_dev);
    mock_i2c_err_code = -EBUSY;
    auto res = manager.readWord(0x10,0x20);
    EXPECT_FALSE(res.success);
    EXPECT_EQ(res.error,I2CFault::BUS_BUSY);
}

TEST_F(I2CFaultRecoveryTestSuite, ReadWordArbitrationLost) {
    I2CManager mgr(mock_dev);
    mock_i2c_err_code = -EAGAIN;
    auto r=mgr.readWord(1,2);
    EXPECT_FALSE(r.success);
    EXPECT_EQ(r.error,I2CFault::ARBITRATION_LOST);
}

TEST_F(I2CFaultRecoveryTestSuite, Read24BitSuccess) {
    I2CManager manager(mock_dev);
    mock_i2c_err_code = 0;
    mock_read_data[0] = 0x11;
    mock_read_data[1] = 0x22;
    mock_read_data[2] = 0x33;
    auto res = manager.read24Bit(0x10,0x20);
    EXPECT_TRUE(res.success);
    EXPECT_EQ(res.value, 0x112233);
}

TEST_F(I2CFaultRecoveryTestSuite, Read24BitTimeout) {
    I2CManager manager(mock_dev);
    mock_i2c_err_code = -ETIMEDOUT;
    auto res = manager.read24Bit(0x10,0x20);
    EXPECT_FALSE(res.success);
    EXPECT_EQ(res.error,I2CFault::TIMEOUT);
}

TEST_F(I2CFaultRecoveryTestSuite, Read24BitBusBusy) {
    I2CManager mgr(mock_dev);
    mock_i2c_err_code = -EBUSY;
    auto r = mgr.read24Bit(1,2);
    EXPECT_FALSE(r.success);
    EXPECT_EQ(r.error,I2CFault::BUS_BUSY);
}

TEST_F(I2CFaultRecoveryTestSuite, Read24BitArbitrationLost) {
    I2CManager mgr(mock_dev);
    mock_i2c_err_code = -EAGAIN;
    auto r = mgr.read24Bit(1,2);
    EXPECT_FALSE(r.success);
    EXPECT_EQ(r.error,I2CFault::ARBITRATION_LOST);
}

TEST_F(I2CFaultRecoveryTestSuite, Read24BitNACK) {
    I2CManager mgr(mock_dev);
    mock_i2c_err_code = -EIO;
    auto r = mgr.read24Bit(1,2);
    EXPECT_FALSE(r.success);
    EXPECT_EQ(r.error,I2CFault::NACK);
}

TEST_F(I2CFaultRecoveryTestSuite, Read64BitSuccess) {
    I2CManager manager(mock_dev);
    mock_i2c_err_code = 0;
    for(int i=0; i<8; i++) mock_read_data[i] = i+1;
    auto res = manager.read64Bit(0x10,0x20);
    EXPECT_TRUE(res.success);
    EXPECT_EQ(res.value, 0x0102030405060708ULL);
}

TEST_F(I2CFaultRecoveryTestSuite, Read64BitTimeout) {
    I2CManager manager(mock_dev);
    mock_i2c_err_code = -ETIMEDOUT;
    auto res = manager.read64Bit(0x10,0x20);
    EXPECT_FALSE(res.success);
    EXPECT_EQ(res.error,I2CFault::TIMEOUT);
}

TEST_F(I2CFaultRecoveryTestSuite, Read64BitBusBusy) {
    I2CManager manager(mock_dev);
    mock_i2c_err_code = -EBUSY;
    auto res = manager.read64Bit(0x10,0x20);
    EXPECT_FALSE(res.success);
    EXPECT_EQ(res.error,I2CFault::BUS_BUSY);
}

TEST_F(I2CFaultRecoveryTestSuite, Read64BitArbitrationLost) {
    I2CManager manager(mock_dev);
    mock_i2c_err_code = -EAGAIN;
    auto res = manager.read64Bit(0x10,0x20);
    EXPECT_FALSE(res.success);
    EXPECT_EQ(res.error,I2CFault::ARBITRATION_LOST);
}

TEST_F(I2CFaultRecoveryTestSuite, Read64BitNACK) {
    I2CManager mgr(mock_dev);
    mock_i2c_err_code = -EIO;
    auto r = mgr.read64Bit(1,2);
    EXPECT_FALSE(r.success);
    EXPECT_EQ(r.error,I2CFault::NACK);
}

TEST_F(I2CFaultRecoveryTestSuite, ReadRegisterNullDevice) {
    I2CManager mgr(nullptr);
    auto r = mgr.readRegister(1,2);
    EXPECT_FALSE(r.success);
    EXPECT_EQ(r.error, I2CFault::DEVICE_NOT_READY);
}

TEST_F(I2CFaultRecoveryTestSuite, WriteRegisterNullDevice) {
    I2CManager mgr(nullptr);
    auto r = mgr.writeRegister(1,2,3);
    EXPECT_FALSE(r.success);
    EXPECT_EQ(r.error,I2CFault::DEVICE_NOT_READY);
}

TEST_F(I2CFaultRecoveryTestSuite, ReadWordNullDevice) {
    I2CManager mgr(nullptr);
    auto r = mgr.readWord(1,2);
    EXPECT_FALSE(r.success);
    EXPECT_EQ(r.error,I2CFault::DEVICE_NOT_READY);
}

TEST_F(I2CFaultRecoveryTestSuite, Read24BitNullDevice) {
    I2CManager mgr(nullptr);
    auto r = mgr.read24Bit(1,2);
    EXPECT_FALSE(r.success);
    EXPECT_EQ(r.error,I2CFault::DEVICE_NOT_READY);
}

TEST_F(I2CFaultRecoveryTestSuite, Read64BitNullDevice) {
    I2CManager mgr(nullptr);
    auto r = mgr.read64Bit(1,2);
    EXPECT_FALSE(r.success);
    EXPECT_EQ(r.error,I2CFault::DEVICE_NOT_READY);
}

TEST_F(I2CFaultRecoveryTestSuite, ReadRegisterDeviceNotReadyOverride) {
    g_device_ready_override = false;
    I2CManager mgr(mock_dev);
    auto r = mgr.readRegister(1,2);
    EXPECT_FALSE(r.success);
    EXPECT_EQ(r.error,I2CFault::DEVICE_NOT_READY);
    g_device_ready_override = true;
}

TEST_F(I2CFaultRecoveryTestSuite, WriteRegisterDeviceNotReadyOverride) {
    g_device_ready_override = false;
    I2CManager mgr(mock_dev);
    auto r = mgr.writeRegister(1,2,3);
    EXPECT_FALSE(r.success);
    EXPECT_EQ(r.error, I2CFault::DEVICE_NOT_READY);
    g_device_ready_override = true;
}

TEST_F(I2CFaultRecoveryTestSuite, ReadWordDeviceNotReadyOverride) {
    g_device_ready_override = false;
    I2CManager mgr(mock_dev);
    auto r = mgr.readWord(1,2);
    EXPECT_FALSE(r.success);
    EXPECT_EQ(r.error, I2CFault::DEVICE_NOT_READY);
    g_device_ready_override = true;
}

TEST_F(I2CFaultRecoveryTestSuite, Read24BitDeviceNotReadyOverride) {
    g_device_ready_override = false;
    I2CManager mgr(mock_dev);
    auto r = mgr.read24Bit(1,2);
    EXPECT_FALSE(r.success);
    EXPECT_EQ(r.error, I2CFault::DEVICE_NOT_READY);
    g_device_ready_override = true;
}

TEST_F(I2CFaultRecoveryTestSuite, Read64BitDeviceNotReadyOverride) {
    g_device_ready_override = false;
    I2CManager mgr(mock_dev);
    auto r = mgr.read64Bit(1,2);
    EXPECT_FALSE(r.success);
    EXPECT_EQ(r.error, I2CFault::DEVICE_NOT_READY);
    g_device_ready_override = true;
}

TEST(ResultCoverage, Unwrap) {
    auto r = Result<int>::Ok(15);
    EXPECT_TRUE(r.isOk());
    EXPECT_EQ(r.unwrap(),15);
}

TEST(ResultCoverage, ErrUnwrap) {
    auto r = Result<int>::Err(I2CFault::TIMEOUT);
    EXPECT_FALSE(r.isOk());
    EXPECT_EQ(r.unwrap(),0);
}

TEST_F(I2CFaultRecoveryTestSuite, ReadRegister_ENODEV) {
    I2CManager mgr(mock_dev);
    mock_i2c_err_code = -ENODEV;
    auto r = mgr.readRegister(0x10, 0x20);
    EXPECT_FALSE(r.success);
    EXPECT_EQ(r.error, I2CFault::DEVICE_NOT_READY);
}

TEST_F(I2CFaultRecoveryTestSuite, Read64BitTestEnvForceErrno) {
    g_i2c_force_errno = -ETIMEDOUT;
    I2CManager mgr(mock_dev);
    auto r = mgr.read64Bit(1, 2);
    EXPECT_FALSE(r.success);
    EXPECT_EQ(r.error, I2CFault::TIMEOUT);
    g_i2c_force_errno = 0;
}

TEST_F(I2CFaultRecoveryTestSuite, ReadRegisterTestEnvForceErrno) {
    g_i2c_force_errno = -ETIMEDOUT;
    I2CManager mgr(mock_dev);

    auto r = mgr.readRegister(1, 2);

    EXPECT_FALSE(r.success);
    EXPECT_EQ(r.error, I2CFault::TIMEOUT);

    g_i2c_force_errno = 0;
}

TEST_F(I2CFaultRecoveryTestSuite, WriteRegisterTestEnvForceErrno) {
    g_i2c_force_errno = -EIO;
    I2CManager mgr(mock_dev);

    auto r = mgr.writeRegister(1, 2, 0xFF);

    EXPECT_FALSE(r.success);
    EXPECT_EQ(r.error, I2CFault::NACK);

    g_i2c_force_errno = 0;
}

TEST_F(I2CFaultRecoveryTestSuite, ReadWordTestEnvForceErrno) {
    g_i2c_force_errno = -EBUSY;
    I2CManager mgr(mock_dev);

    auto r = mgr.readWord(1, 2);

    EXPECT_FALSE(r.success);
    EXPECT_EQ(r.error, I2CFault::BUS_BUSY);

    g_i2c_force_errno = 0;
}

TEST_F(I2CFaultRecoveryTestSuite, Read24BitTestEnvForceErrno) {
    g_i2c_force_errno = -EAGAIN;
    I2CManager mgr(mock_dev);

    auto r = mgr.read24Bit(1, 2);

    EXPECT_FALSE(r.success);
    EXPECT_EQ(r.error, I2CFault::ARBITRATION_LOST);

    g_i2c_force_errno = 0;
}

TEST_F(I2CFaultRecoveryTestSuite, PowerObserverAndSleepGating) {
    I2CManager mgr(mock_dev);
    mock_i2c_err_code = 0;

    mgr.readRegister(1, 2);

    mgr.readRegister(1, 2);

    PowerManager::getInstance().notifyBeforeSleep();

    auto w_res = mgr.writeRegister(1, 2, 0xFF);
    EXPECT_FALSE(w_res.success);
    EXPECT_EQ(w_res.error, I2CFault::DEVICE_NOT_READY);

    resetI2CCacheForTests();

    EXPECT_FALSE(mgr.readRegister(1, 2).success);
    EXPECT_FALSE(mgr.readWord(1, 3).success);
    EXPECT_FALSE(mgr.read24Bit(1, 4).success);
    EXPECT_FALSE(mgr.read64Bit(1, 5).success);

    // Seed the cache while the bus is asleep.
    test_update_cache((1U << 8) | 2U, 0xAA, true);
    test_update_cache((1U << 8) | 3U, 0xAABB, true);
    test_update_cache((1U << 8) | 4U, 0xAABBCC, true);
    test_update_cache((1U << 8) | 5U, 0x1122334455667788ULL, true);

    // While sleeping, reads are rejected before the cache fallback is reached,
    // even though valid cached data exists.
    auto r8 = mgr.readRegister(1, 2);
    EXPECT_FALSE(r8.success);
    EXPECT_EQ(r8.error, I2CFault::DEVICE_NOT_READY);

    auto r16 = mgr.readWord(1, 3);
    EXPECT_FALSE(r16.success);
    EXPECT_EQ(r16.error, I2CFault::DEVICE_NOT_READY);

    auto r24 = mgr.read24Bit(1, 4);
    EXPECT_FALSE(r24.success);
    EXPECT_EQ(r24.error, I2CFault::DEVICE_NOT_READY);

    auto r64 = mgr.read64Bit(1, 5);
    EXPECT_FALSE(r64.success);
    EXPECT_EQ(r64.error, I2CFault::DEVICE_NOT_READY);

    PowerManager::getInstance().notifyAfterWakeup();
    EXPECT_TRUE(mgr.writeRegister(1, 2, 0xFF).success);

    // The cache survived the sleep cycle: a bus error now falls back to it.
    mock_i2c_err_code = -EIO;
    auto c8 = mgr.readRegister(1, 2);
    EXPECT_TRUE(c8.success);
    EXPECT_EQ(c8.value, 0xAA);

    auto c16 = mgr.readWord(1, 3);
    EXPECT_TRUE(c16.success);
    EXPECT_EQ(c16.value, 0xAABB);

    auto c24 = mgr.read24Bit(1, 4);
    EXPECT_TRUE(c24.success);
    EXPECT_EQ(c24.value, 0xAABBCCU);

    auto c64 = mgr.read64Bit(1, 5);
    EXPECT_TRUE(c64.success);
    EXPECT_EQ(c64.value, 0x1122334455667788ULL);
    mock_i2c_err_code = 0;

    PowerManager::getInstance().notifyBeforeSleep();
    EXPECT_FALSE(mgr.writeRegister(1, 2, 0xFF).success);

    PowerManager::getInstance().notifySleepAborted();
    EXPECT_TRUE(mgr.writeRegister(1, 2, 0xFF).success);
}

TEST_F(I2CFaultRecoveryTestSuite, LockOutBranchesSleepFaultSafeHalt) {
    REQUIRE_SYSTEM_RUNNING();
    I2CManager mgr(mock_dev);
    mock_i2c_err_code = 0;
    g_device_ready_override = true;

    /* All operands FALSE: the call goes through (also registers the observer). */
    EXPECT_TRUE(mgr.readRegister(1, 2).success);
    EXPECT_TRUE(mgr.writeRegister(1, 2, 0x55).success);
    EXPECT_TRUE(mgr.readWord(1, 3).success);
    EXPECT_TRUE(mgr.read24Bit(1, 4).success);
    EXPECT_TRUE(mgr.read64Bit(1, 5).success);

    /* Operand 1 TRUE: bus is asleep. */
    PowerManager::getInstance().notifyBeforeSleep();
    expectAllApisLockedOut(mgr);
    PowerManager::getInstance().notifyAfterWakeup();

    /* Operand 1 FALSE, operand 2 TRUE: FAULT. */
    ASSERT_TRUE(sys_context.requestTransition(SystemState::FAULT));
    ASSERT_EQ(sys_context.getState(), SystemState::FAULT);
    expectAllApisLockedOut(mgr);

    /* Operands 1 and 2 FALSE, operand 3 TRUE: SAFE_HALT. */
    ASSERT_TRUE(sys_context.requestTransition(SystemState::SAFE_HALT));
    ASSERT_EQ(sys_context.getState(), SystemState::SAFE_HALT);
    expectAllApisLockedOut(mgr);

    /* Back to RUNNING: all operands FALSE again. */
    walkBackToRunning();
    ASSERT_EQ(sys_context.getState(), SystemState::RUNNING);
    EXPECT_TRUE(mgr.readRegister(1, 2).success);
    EXPECT_TRUE(mgr.writeRegister(1, 2, 0x55).success);
    EXPECT_TRUE(mgr.readWord(1, 3).success);
    EXPECT_TRUE(mgr.read24Bit(1, 4).success);
    EXPECT_TRUE(mgr.read64Bit(1, 5).success);
}

/* ---------------------------------------------------------------------------
 * I2CManager::writeWord(): splits the value into high/low bytes and issues two
 * writeRegister() calls. Needs both outcomes of `if (!res1.isOk()) return res1;`.
 * ------------------------------------------------------------------------- */
TEST_F(I2CFaultRecoveryTestSuite, WriteWordSuccess) {
    I2CManager mgr(mock_dev);
    mock_i2c_err_code = 0;

    auto res = mgr.writeWord(0x10, 0x20, 0xABCD);   // both byte writes succeed
    EXPECT_TRUE(res.success);
    EXPECT_EQ(res.error, I2CFault::NONE);
}

TEST_F(I2CFaultRecoveryTestSuite, WriteWordFirstByteFailsNack) {
    I2CManager mgr(mock_dev);
    mock_i2c_err_code = -EIO;                        // first writeRegister() fails

    auto res = mgr.writeWord(0x10, 0x20, 0xABCD);    // early return of res1
    EXPECT_FALSE(res.success);
    EXPECT_EQ(res.error, I2CFault::NACK);
}

TEST_F(I2CFaultRecoveryTestSuite, WriteWordFirstByteFailsTimeout) {
    I2CManager mgr(mock_dev);
    mock_i2c_err_code = -ETIMEDOUT;

    auto res = mgr.writeWord(0x10, 0x20, 0x1234);
    EXPECT_FALSE(res.success);
    EXPECT_EQ(res.error, I2CFault::TIMEOUT);
}

TEST_F(I2CFaultRecoveryTestSuite, WriteWordLockedOutWhenFault) {
    I2CManager mgr(mock_dev);
    ASSERT_TRUE(sys_context.requestTransition(SystemState::FAULT));

    auto res = mgr.writeWord(0x10, 0x20, 0xFFFF);
    EXPECT_FALSE(res.success);
    EXPECT_EQ(res.error, I2CFault::DEVICE_NOT_READY);

    walkBackToRunning();
    ASSERT_EQ(sys_context.getState(), SystemState::RUNNING);
}
