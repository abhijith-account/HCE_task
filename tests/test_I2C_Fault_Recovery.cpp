#include <gtest/gtest.h>
#include "Fault_Tolerant_I2C_Communication_Layer.h"
#include "Static_Memory+MISRA_Compliance_Layer.h"
#include "Power_Management_System.h"
#include <new>
#include <string>
#include <string_view>

static int mock_i2c_err_code = 0;
static uint8_t mock_read_data[8] = {0};
extern bool g_device_ready_override;
extern uint32_t virtual_uptime;
extern int g_i2c_force_errno;
extern void test_update_cache(uint32_t key, uint64_t value, bool calibrated);
extern bool test_get_cached_value(uint32_t key, uint64_t* out_val, uint32_t max_age_ms);
extern void update_cache(uint32_t key, uint64_t value, bool calibrated);
extern void test_set_reliability(uint32_t key, uint8_t score);
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
extern void resetI2CCacheForTests() noexcept;
extern void exhaustI2CCachePool();
extern void resetI2CCachePool();
extern void ensure_mutex_initialized();

class I2CFaultRecoveryTestSuite : public ::testing::Test {
  protected:
      void SetUp() override {
          g_i2c_force_errno = 0;
          mock_i2c_err_code = 0;
          for(int i=0; i<8; i++) mock_read_data[i] = 0;
          resetI2CCacheForTests(); 
          ensure_mutex_initialized();
      }
};

TEST_F(I2CFaultRecoveryTestSuite, FailSafeFallback) {
    FailSafeStrategy failsafe;
    EXPECT_EQ(failsafe.getLastGood(), 0);
    failsafe.updateLastGood(120);
    failsafe.executeRecovery(mock_dev);
    EXPECT_EQ(failsafe.getLastGood(), 120);
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

TEST_F(I2CFaultRecoveryTestSuite, StrategySwitchMidStream) {
    RetryStrategy retry;
    BusResetStrategy hard_reset;
    FailSafeStrategy failsafe;
    
    failsafe.updateLastGood(99);
    I2CStrategy* active_strategy = nullptr;
    
    active_strategy = &retry;
    EXPECT_NO_FATAL_FAILURE(active_strategy->executeRecovery(mock_dev));
    
    active_strategy = &hard_reset;
    EXPECT_NO_FATAL_FAILURE(active_strategy->executeRecovery(mock_dev));
    
    active_strategy = &failsafe;
    EXPECT_NO_FATAL_FAILURE(active_strategy->executeRecovery(mock_dev));
    
    auto* casted_failsafe = dynamic_cast<FailSafeStrategy*>(active_strategy);
    ASSERT_NE(casted_failsafe, nullptr);
    EXPECT_EQ(casted_failsafe->getLastGood(), 99);
}

// --------------------------------------------------------------------------------
// Manager Core Reads and Writes 
// --------------------------------------------------------------------------------

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

// --------------------------------------------------------------------------------
// System Logs and Pool Validations
// --------------------------------------------------------------------------------

TEST_F(I2CFaultRecoveryTestSuite, VirtualDestructorCoverage){
    StaticPool<RetryStrategy,1> strategy_pool;
    void* raw_memory = strategy_pool.allocate();
    ASSERT_NE(raw_memory, nullptr);
    RetryStrategy* strategy = new(raw_memory) RetryStrategy();    
    strategy->~RetryStrategy();
    strategy_pool.deallocate(raw_memory);
}

TEST_F(I2CFaultRecoveryTestSuite, DeallocateNullPointerIsSafe){
    StaticPool<RetryStrategy,1> strategy_pool;
    EXPECT_NO_FATAL_FAILURE(strategy_pool.deallocate(nullptr));
}

TEST_F(I2CFaultRecoveryTestSuite, RetryStrategyLogsWarning){
    RetryStrategy strategy;
    testing::internal::CaptureStdout();
    strategy.executeRecovery(mock_dev);
    const auto raw_output = testing::internal::GetCapturedStdout();
    std::string_view output(raw_output);
    EXPECT_TRUE(output.find("[WRN] I2C Fault Detected. Executing Exponential Backoff Retry...")!= std::string_view::npos);
}

TEST_F(I2CFaultRecoveryTestSuite, BusRestStrategyLogsError){
    BusResetStrategy strategy;
    testing::internal::CaptureStdout();
    strategy.executeRecovery(mock_dev);
    const auto raw_output = testing::internal::GetCapturedStdout();
    std::string_view output(raw_output);
    EXPECT_TRUE(output.find("[ERR] I2C Bus Hard-Locked. Toggling SCL to recover")!=std::string_view::npos);
}

TEST_F(I2CFaultRecoveryTestSuite, FailSafeStrategyLogsErrorWithValue){
    FailSafeStrategy strategy;
    uint8_t dummy_last_good = 188;
    strategy.updateLastGood(dummy_last_good);
    testing::internal::CaptureStdout();
    strategy.executeRecovery(mock_dev);
    const auto raw_output = testing::internal::GetCapturedStdout();
    std::string_view output(raw_output);
    EXPECT_TRUE(output.find("Falling back to Last-known-Good data: 188")!= std::string_view::npos);
}

// --------------------------------------------------------------------------------
// Extended I2C Error Encoding Coverage 
// --------------------------------------------------------------------------------

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

// --------------------------------------------------------------------------------
// Nullptr Device / Hardware Not Ready Edge Cases (Line & Branch Coverage targets)
// --------------------------------------------------------------------------------

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

// --- Overrides ---

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

// --------------------------------------------------------------------------------
// Hardware Caching
// --------------------------------------------------------------------------------

TEST_F(I2CFaultRecoveryTestSuite, CachedReadRegister) {
    I2CManager manager(mock_dev);
    mock_read_data[0] = 77;
    auto first = manager.readRegister(0x10,0x20);
    EXPECT_TRUE(first.success);

    mock_i2c_err_code = -ETIMEDOUT;
    auto second = manager.readRegister(0x10,0x20);
    EXPECT_TRUE(second.success);
    EXPECT_EQ(second.value,77);
}

TEST_F(I2CFaultRecoveryTestSuite, Read24BitUsesCache) {
    I2CManager mgr(mock_dev);
    mock_i2c_err_code = 0;
    EXPECT_TRUE(mgr.read24Bit(1,2).success);

    mock_i2c_err_code = -ETIMEDOUT;
    auto r = mgr.read24Bit(1,2);
    EXPECT_TRUE(r.success);
}

TEST_F(I2CFaultRecoveryTestSuite, Read64BitUsesCache) {
    I2CManager mgr(mock_dev);
    EXPECT_TRUE(mgr.read64Bit(1,2).success);

    mock_i2c_err_code = -ETIMEDOUT;
    auto r = mgr.read64Bit(1,2);
    EXPECT_TRUE(r.success);
}

TEST_F(I2CFaultRecoveryTestSuite, CacheFullLogsError) {
    I2CManager mgr(mock_dev);
    mock_i2c_err_code = 0;

    // Fill the cache up to its max size limit (15 entries)
    for(int i = 0; i < 15; i++) {
        mgr.readRegister(1, i);
    }

    testing::internal::CaptureStdout();
    mgr.readRegister(1, 99); // Will exceed the limit
    auto output = testing::internal::GetCapturedStdout();
    EXPECT_NE(output.find("cache table full"), std::string_view::npos);
}

TEST_F(I2CFaultRecoveryTestSuite, CacheExpired) {
    I2CManager mgr(mock_dev);
    mock_read_data[0] = 15;
    mgr.readRegister(1,2);

    virtual_uptime += 5000;
    mock_i2c_err_code = -EIO;

    testing::internal::CaptureStdout();
    auto r = mgr.readRegister(1,2);
    const auto raw_out = testing::internal::GetCapturedStdout();
    std::string_view out(raw_out);
    EXPECT_NE(out.find("Fallback Rejected: Data Stale"), std::string_view::npos);
    EXPECT_FALSE(r.success);
}

TEST_F(I2CFaultRecoveryTestSuite, ReliabilityDecay) {
    I2CManager mgr(mock_dev);
    mock_read_data[0] = 88;
    mgr.readRegister(1,2);

    for(int i = 0; i < 6; i++) {
        mock_i2c_err_code = -EIO;
        mgr.readRegister(1,2);
    }

    testing::internal::CaptureStdout();
    auto r = mgr.readRegister(1,2);
    const auto raw_out = testing::internal::GetCapturedStdout();
    std::string_view out(raw_out);
    EXPECT_NE(out.find("Fallback Rejected: Reliability Score too low"), std::string_view::npos);
    EXPECT_FALSE(r.success);
}

TEST_F(I2CFaultRecoveryTestSuite, CacheUpdatedMultipleTimes) {
    I2CManager mgr(mock_dev);

    mock_read_data[0] = 10;
    mgr.readRegister(1,2);

    mock_read_data[0] = 20;
    mgr.readRegister(1,2);

    mock_i2c_err_code = -EIO;
    auto r = mgr.readRegister(1,2);
    EXPECT_TRUE(r.success);
    EXPECT_EQ(r.value,20);
}

// --------------------------------------------------------------------------------
// General Object Lifecycle
// --------------------------------------------------------------------------------

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

TEST_F(I2CFaultRecoveryTestSuite, BaseDestructor) {
    StaticPool<RetryStrategy,1> pool;
    void* mem = pool.allocate();
    I2CStrategy* ptr = new(mem) RetryStrategy;
    ptr->~I2CStrategy();
    pool.deallocate(mem);
}

TEST_F(I2CFaultRecoveryTestSuite, ReadRegister_ENODEV) {
    I2CManager mgr(mock_dev);
    mock_i2c_err_code = -ENODEV;
    auto r = mgr.readRegister(0x10, 0x20);
    EXPECT_FALSE(r.success);
    EXPECT_EQ(r.error, I2CFault::DEVICE_NOT_READY);
}

TEST_F(I2CFaultRecoveryTestSuite, ReadWordUsesCache) {
    I2CManager mgr(mock_dev);
    mock_read_data[0] = 0x34; mock_read_data[1] = 0x12;
    EXPECT_TRUE(mgr.readWord(1, 2).success);   // caches 0x1234

    mock_i2c_err_code = -ETIMEDOUT;
    auto r = mgr.readWord(1, 2);
    EXPECT_TRUE(r.success);
    EXPECT_EQ(r.value, 0x1234);
}

TEST_F(I2CFaultRecoveryTestSuite, Read64BitTestEnvForceErrno) {
    g_i2c_force_errno = -ETIMEDOUT;
    I2CManager mgr(mock_dev);
    auto r = mgr.read64Bit(1, 2);
    EXPECT_FALSE(r.success);
    EXPECT_EQ(r.error, I2CFault::TIMEOUT);
    g_i2c_force_errno = 0;   // reset
}

TEST_F(I2CFaultRecoveryTestSuite, CachePoolExhausted) {
    // Exhaust the pool without filling any active_entries slot.
    exhaustI2CCachePool();

    I2CManager mgr(mock_dev);
    mock_i2c_err_code = 0;
    // This read will succeed on the bus, but update_cache will fail to allocate
    // a pool block (because the pool is empty) and silently skip caching.
    auto r = mgr.readRegister(1, 2);
    EXPECT_TRUE(r.success);

    // Restore the pool so other tests are not affected.
    resetI2CCachePool();
}

TEST_F(I2CFaultRecoveryTestSuite, CacheUncalibratedRejection) {
    uint32_t key = (1U << 8) | 2;
    test_update_cache(key, 123, false);   // uncalibrated entry

    I2CManager mgr(mock_dev);
    mock_i2c_err_code = -EIO;             // trigger fallback attempt

    testing::internal::CaptureStdout();
    auto r = mgr.readRegister(1, 2);
    const auto raw_out = testing::internal::GetCapturedStdout();
    std::string_view out(raw_out);
    EXPECT_NE(out.find("Fallback Rejected: Data is uncalibrated."), std::string_view::npos);
    EXPECT_FALSE(r.success);
}

TEST_F(I2CFaultRecoveryTestSuite, CacheSampleCountOverflow) {
    uint32_t key = 0x1234;
    // Manually set sample count to max
    test_update_cache(key, 100, true);
    
    // Access the private/protected member through the shim or test logic
    // This requires forcing the entry's sample_count to UINT32_MAX.
    // If you cannot access CacheEntry directly, use a loop to call 
    // update_cache() 0xFFFFFFFF times (inefficient) or add a test-only 
    // setter shim in your source code.
}

TEST_F(I2CFaultRecoveryTestSuite, CacheReliabilityScoreRejectsLowScore)
{
    resetI2CCacheForTests();

    uint32_t key = 0x5555;

    test_update_cache(key, 10, true);
    test_set_reliability(key, 10);

    uint64_t val = 0;

    EXPECT_FALSE(test_get_cached_value(key, &val, 1000));
}


extern void test_set_sample_count(uint32_t,uint32_t);

TEST_F(I2CFaultRecoveryTestSuite, SampleCountMaxBranch)
{
    uint32_t key = 0x1111;

    test_update_cache(key, 55, true);

    test_set_sample_count(key, UINT32_MAX);

    test_update_cache(key, 66, true);

    uint64_t value=0;
    EXPECT_TRUE(test_get_cached_value(key,&value,1000));
    EXPECT_EQ(value,66);
}

TEST_F(I2CFaultRecoveryTestSuite, CachedValueWrongKeyBranch)
{
    resetI2CCacheForTests();

    test_update_cache(0x1111,44,true);

    uint64_t out=0;

    EXPECT_FALSE(test_get_cached_value(0x2222,&out,1000));
}

TEST_F(I2CFaultRecoveryTestSuite, ReliabilityShimKeyNotFound)
{
    testing::internal::CaptureStderr();

    test_set_reliability(0xDEADBEEF,10);

    const auto raw_err = testing::internal::GetCapturedStderr();
    std::string_view err(raw_err);

    EXPECT_NE(err.find("Shim: Key"),
              std::string_view::npos);
}

TEST_F(I2CFaultRecoveryTestSuite, ReliabilityWrongKeyBranch)
{
    resetI2CCacheForTests();

    test_update_cache(0x1111, 10, true);

    testing::internal::CaptureStderr();

    test_set_reliability(0x2222, 50);

    const auto raw_err = testing::internal::GetCapturedStderr();
    std::string_view err(raw_err);

    EXPECT_NE(err.find("Shim: Key"),
              std::string_view::npos);
}

TEST_F(I2CFaultRecoveryTestSuite, SampleCountWrongKey)
{
    resetI2CCacheForTests();

    test_update_cache(0x1111, 99, true);

    test_set_sample_count(0x2222, 5);

    uint64_t value = 0;

    EXPECT_TRUE(test_get_cached_value(0x1111,
                                      &value,
                                      1000));

    EXPECT_EQ(value, 99);
}

TEST_F(I2CFaultRecoveryTestSuite, SampleCountEmptyCache)
{
    resetI2CCacheForTests();

    EXPECT_NO_FATAL_FAILURE(
        test_set_sample_count(0x1234, 10));
}

TEST_F(I2CFaultRecoveryTestSuite, GetCachedValueNullPointer) {
    uint64_t val = 0;
    // This hits the 'if (out_val == nullptr)' true path
    EXPECT_FALSE(test_get_cached_value(0x1234, nullptr, 1000));
    
    // This hits the 'false' path (out_val is provided)
    // Note: requires key to be cached first
    test_update_cache(0x1234, 100, true);
    EXPECT_TRUE(test_get_cached_value(0x1234, &val, 1000));
}

TEST_F(I2CFaultRecoveryTestSuite, ResetCachePoolWithNullBlocks) {
    // Calling reset directly without exhausting the pool first means 
    // the exhausted_blocks array is filled with nullptrs.
    // This exercises the 'false' branch of: if (exhausted_blocks[i] != nullptr)
    EXPECT_NO_FATAL_FAILURE(resetI2CCachePool());
}

TEST_F(I2CFaultRecoveryTestSuite, ReadRegisterTestEnvForceErrno) {
    g_i2c_force_errno = -ETIMEDOUT;
    I2CManager mgr(mock_dev);
    
    auto r = mgr.readRegister(1, 2);
    
    EXPECT_FALSE(r.success);
    EXPECT_EQ(r.error, I2CFault::TIMEOUT);
    
    g_i2c_force_errno = 0;   // reset for subsequent tests
}

TEST_F(I2CFaultRecoveryTestSuite, WriteRegisterTestEnvForceErrno) {
    g_i2c_force_errno = -EIO;
    I2CManager mgr(mock_dev);
    
    auto r = mgr.writeRegister(1, 2, 0xFF);
    
    EXPECT_FALSE(r.success);
    EXPECT_EQ(r.error, I2CFault::NACK);
    
    g_i2c_force_errno = 0;   // reset
}

TEST_F(I2CFaultRecoveryTestSuite, ReadWordTestEnvForceErrno) {
    g_i2c_force_errno = -EBUSY;
    I2CManager mgr(mock_dev);
    
    auto r = mgr.readWord(1, 2);
    
    EXPECT_FALSE(r.success);
    EXPECT_EQ(r.error, I2CFault::BUS_BUSY);
    
    g_i2c_force_errno = 0;   // reset
}

TEST_F(I2CFaultRecoveryTestSuite, Read24BitTestEnvForceErrno) {
    g_i2c_force_errno = -EAGAIN;
    I2CManager mgr(mock_dev);
    
    auto r = mgr.read24Bit(1, 2);
    
    EXPECT_FALSE(r.success);
    EXPECT_EQ(r.error, I2CFault::ARBITRATION_LOST);
    
    g_i2c_force_errno = 0;   // reset
}

TEST_F(I2CFaultRecoveryTestSuite, PowerObserverAndSleepGating) {
    I2CManager mgr(mock_dev);
    mock_i2c_err_code = 0;
    
    // 1. Initial call triggers ensure_power_observer_registered() true branch (Registers observer)
    mgr.readRegister(1, 2);
    // 2. Second call triggers ensure_power_observer_registered() false branch
    mgr.readRegister(1, 2);
    
    // 3. Trigger sleep state (Covers I2CPowerObserver::beforeSleep and true branch of isSleeping)
    PowerManager::getInstance().notifyBeforeSleep();
    
    // 4. Test write in sleep state (Unconditionally rejected)
    // 4. Test write in sleep state (Unconditionally rejected)
    auto w_res = mgr.writeRegister(1, 2, 0xFF);
    EXPECT_FALSE(w_res.success);
    EXPECT_EQ(w_res.error, I2CFault::DEVICE_NOT_READY);
    
    // Clear the cache populated by Steps 1 & 2 to enforce the "NO cache" condition
    resetI2CCacheForTests();
    
    // 5. Test reads in sleep state with NO cache -> Should return DEVICE_NOT_READY
    EXPECT_FALSE(mgr.readRegister(1, 2).success);
    EXPECT_FALSE(mgr.readWord(1, 3).success);
    EXPECT_FALSE(mgr.read24Bit(1, 4).success);
    EXPECT_FALSE(mgr.read64Bit(1, 5).success);
    
    // 6. Populate cache via shim for the specific sleeping keys
    test_update_cache((1U << 8) | 2U, 0xAA, true);
    test_update_cache((1U << 8) | 3U, 0xAABB, true);
    test_update_cache((1U << 8) | 4U, 0xAABBCC, true);
    test_update_cache((1U << 8) | 5U, 0x1122334455667788ULL, true);
    
    // 7. Test reads in sleep state WITH valid cache (Graceful fallback)
    auto r8 = mgr.readRegister(1, 2);
    EXPECT_TRUE(r8.success);
    EXPECT_EQ(r8.value, 0xAA);
    
    auto r16 = mgr.readWord(1, 3);
    EXPECT_TRUE(r16.success);
    EXPECT_EQ(r16.value, 0xAABB);
    
    auto r24 = mgr.read24Bit(1, 4);
    EXPECT_TRUE(r24.success);
    EXPECT_EQ(r24.value, 0xAABBCC);
    
    auto r64 = mgr.read64Bit(1, 5);
    EXPECT_TRUE(r64.success);
    EXPECT_EQ(r64.value, 0x1122334455667788ULL);
    
    // 8. Wake up (Covers I2CPowerObserver::afterWakeup)
    PowerManager::getInstance().notifyAfterWakeup();
    EXPECT_TRUE(mgr.writeRegister(1, 2, 0xFF).success); // Should succeed now that it's awake
    
    // 9. Abort sleep (Covers I2CPowerObserver::sleepAborted)
    PowerManager::getInstance().notifyBeforeSleep(); 
    EXPECT_FALSE(mgr.writeRegister(1, 2, 0xFF).success); // Verify asleep again
    
    PowerManager::getInstance().notifySleepAborted(); 
    EXPECT_TRUE(mgr.writeRegister(1, 2, 0xFF).success); // Verify awake again
}
