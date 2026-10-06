#include <gtest/gtest.h>
#include <array>
#include <cstring>
#include <string_view>
#include <cstdint>
#include <thread>
#include <atomic>
#include <chrono>
#include <string>

#include "Fault_Tolerant_I2C_Communication_Layer.h"
#include "Power_Management_System.h"

#define private public
#define protected public
#include "Smart_Battery_System.h"
#include "Persistent_Configuration_System.h"
#undef private
#undef protected

extern int g_i2c_force_errno;
extern int g_i2c_call_counter;
extern int g_i2c_fail_on_call_n;
extern int g_i2c_fail_on_call_errno;
extern uint32_t virtual_uptime;
extern DeviceContext sys_context;
extern int g_adc_sequence_init_dt_errno;
extern int test_iterations_remaining;
extern int g_mutex_lock_force_errno;
extern bool g_adc_ready_mock;
extern int g_adc_read_force_errno;
extern int g_mutex_lock_call_counter;
extern int g_mutex_lock_fail_on_call_n;
extern void* g_mutex_lock_target_ptr;
extern int g_mutex_lock_target_call_counter;
extern int g_mutex_lock_target_fail_on_call_n;

uint16_t g_i2c_mock_read_val = 0;
uint16_t g_i2c_mock_v_val = 10000;
uint16_t g_i2c_mock_i_val = 0;
int32_t g_adc_mock_mv_val = 2500;
int g_adc_raw_to_mv_errno = 0;
int g_i2c_consecutive_failures = 0;
int g_i2c_fail_after_reads = -1;
int g_nvs_write_force_errno = 0;
int g_set_safe_halt_on_i2c_call = -1;
int g_sleep_on_i2c_call = -1;

extern "C" {
    int nvs_mount(struct nvs_fs *fs) {
        return 0;
    }
    ssize_t nvs_write(struct nvs_fs *fs, uint16_t id, const void *data, size_t len) {
    if (g_nvs_write_force_errno != 0) return g_nvs_write_force_errno;
    return len;
    }

    ssize_t nvs_read(struct nvs_fs *fs, uint16_t id, void *data, size_t len) {
        return -ENOENT;
    }

    int i2c_burst_read(const struct device *dev, uint16_t dev_addr, uint8_t start_addr, uint8_t *buf, uint32_t num_bytes) {
        ++g_i2c_call_counter;
        if (g_i2c_force_errno != 0) return g_i2c_force_errno;
        if (g_i2c_fail_on_call_n != 0 && g_i2c_call_counter == g_i2c_fail_on_call_n) return g_i2c_fail_on_call_errno;
        if (g_i2c_consecutive_failures > 0) {
            g_i2c_consecutive_failures--;
            return -EIO;
        }
        if (g_i2c_fail_after_reads >= 0 && g_i2c_call_counter > g_i2c_fail_after_reads) return -EIO;

        if (buf && num_bytes >= 2) {
            uint16_t val = g_i2c_mock_read_val;

            if (start_addr == 0x02) {
                val = g_i2c_mock_v_val;
            } else if (start_addr == 0x04) {
                val = g_i2c_mock_i_val;
            }

            buf[0] = (val >> 8) & 0xFF;
            buf[1] = val & 0xFF;
        } else if (buf && num_bytes > 0) {
            memset(buf, 0, num_bytes);
        }
        return 0;
    }

    int i2c_write_read(const struct device *dev, uint16_t addr, const void *write_buf, size_t num_write, void *read_buf, size_t num_read) {
        ++g_i2c_call_counter;
        if (g_i2c_force_errno != 0) return g_i2c_force_errno;
        if (g_i2c_fail_on_call_n != 0 && g_i2c_call_counter == g_i2c_fail_on_call_n) return g_i2c_fail_on_call_errno;
        if (g_i2c_consecutive_failures > 0) {
            g_i2c_consecutive_failures--;
            return -EIO;
        }
        if (g_i2c_fail_after_reads >= 0 && g_i2c_call_counter > g_i2c_fail_after_reads) return -EIO;

        if (read_buf && num_read >= 2) {
            uint8_t* b = static_cast<uint8_t*>(read_buf);
            uint16_t val = g_i2c_mock_read_val;

            if (write_buf && num_write >= 1) {
                uint8_t reg = static_cast<const uint8_t*>(write_buf)[0];
                if (reg == 0x02) val = g_i2c_mock_v_val;
                else if (reg == 0x04) val = g_i2c_mock_i_val;
            }

            b[0] = (val >> 8) & 0xFF;
            b[1] = val & 0xFF;
        } else if (read_buf && num_read > 0) {
            memset(read_buf, 0, num_read);
        }
        return 0;
    }

    int i2c_write(const struct device *dev, const uint8_t *buf, uint32_t num_bytes, uint16_t addr) {
        ++g_i2c_call_counter;
        if (g_set_safe_halt_on_i2c_call > 0 && g_i2c_call_counter == g_set_safe_halt_on_i2c_call) {sys_context.requestTransition(SystemState::SAFE_HALT);}
        if (g_sleep_on_i2c_call > 0 && g_i2c_call_counter == g_sleep_on_i2c_call) {PowerManager::getInstance().notifyBeforeSleep();}
        if (g_i2c_force_errno != 0) return g_i2c_force_errno;
        if (g_i2c_fail_on_call_n != 0 && g_i2c_call_counter == g_i2c_fail_on_call_n) return g_i2c_fail_on_call_errno;
        if (g_i2c_fail_after_reads >= 0 && g_i2c_call_counter > g_i2c_fail_after_reads) return -EIO;
        return 0;
    }

    int adc_raw_to_millivolts_dt(const struct adc_dt_spec *spec, int32_t *val) {
        if (g_adc_raw_to_mv_errno != 0) return g_adc_raw_to_mv_errno;
        if (val) *val = g_adc_mock_mv_val;
        return 0;
    }
}

static const device dummy_i2c_dev;
I2CManager i2c_manager(&dummy_i2c_dev);

static bool custom_hook_called = false;
static void custom_watchdog_hook(void) { custom_hook_called = true; }

class SmartBatteryTestSuite : public ::testing::Test {
protected:
    SbsBattery battery{&i2c_manager, &sys_context};

    void SetUp() override {
        sys_context = DeviceContext();
        sys_context.requestTransition(SystemState::RUNNING);

        virtual_uptime = 10000;
        g_i2c_force_errno = 0;
        g_i2c_call_counter = 0;
        g_i2c_fail_on_call_n = 0;
        g_i2c_fail_after_reads = -1;
        g_i2c_mock_read_val = 0;
        g_i2c_mock_v_val = 10000;
        g_i2c_mock_i_val = 0;
        g_i2c_consecutive_failures = 0;
        g_adc_ready_mock = true;
        g_adc_mock_mv_val = 2500;
        g_adc_raw_to_mv_errno = 0;
        g_mutex_lock_force_errno = 0;
        custom_hook_called = false;
        g_adc_sequence_init_dt_errno = 0;
        g_mutex_lock_call_counter = 0;
        g_mutex_lock_fail_on_call_n = 0;
        g_mutex_lock_target_ptr = nullptr;
        g_mutex_lock_target_call_counter = 0;
        g_mutex_lock_target_fail_on_call_n = 0;
        g_nvs_write_force_errno = 0;
        g_set_safe_halt_on_i2c_call = -1;
        g_sleep_on_i2c_call = -1;

        battery.cache = BmsCache{};
        battery.cache.valid = true;
        battery.cache.last_error = CommFault::NONE;
        battery.cache.timestamp_ms = k_uptime_get_32();
        battery.cache.temperature.value = Thermistor::KELVIN_OFFSET_TENTHS + 13;

        battery.current_state = BatteryFSM::IDLE;
        battery.consecutive_comm_failures = 0;
        battery.consecutive_mutex_failures = 0;
        battery.last_valid_comm_time = k_uptime_get_32();
        battery.soc_initialized = false;
        battery.accumulated_uAh = 0;
        battery.consecutive_jump_rejects = 0;
        battery.rest_period_start_ms = 0;
    }
};

TEST_F(SmartBatteryTestSuite, CurveFitting_LUT_Edges) {
    EXPECT_EQ(battery.estimateSocFromVoltage(8700), 0);
    EXPECT_EQ(battery.estimateSocFromVoltage(8600), 0);
    EXPECT_EQ(battery.estimateSocFromVoltage(12600), 100);
    EXPECT_EQ(battery.estimateSocFromVoltage(13000), 100);
    EXPECT_EQ(battery.estimateSocFromVoltage(9000), 1);

    g_adc_mock_mv_val = 3300;
    EXPECT_EQ(Thermistor::readCelsius().error, Thermistor::Fault::OUT_OF_RANGE);
    g_adc_mock_mv_val = 3220;
    EXPECT_EQ(Thermistor::readCelsius().value, -400);
    g_adc_mock_mv_val = 3250;
    EXPECT_EQ(Thermistor::readCelsius().value, -400);
    g_adc_mock_mv_val = 114;
    EXPECT_EQ(Thermistor::readCelsius().value, 1250);
    g_adc_mock_mv_val = 100;
    EXPECT_EQ(Thermistor::readCelsius().value, 1250);
}

TEST_F(SmartBatteryTestSuite, Thermistor_InitAndReadFailures) {
    g_adc_ready_mock = false;
    EXPECT_FALSE(Thermistor::init());
    EXPECT_EQ(Thermistor::readCelsius().error, Thermistor::Fault::ADC_NOT_READY);

    g_adc_ready_mock = true;
    g_adc_raw_to_mv_errno = -EIO;
    EXPECT_EQ(Thermistor::readCelsius().error, Thermistor::Fault::ADC_READ_ERROR);

    g_adc_raw_to_mv_errno = 0;
    g_adc_mock_mv_val = 0;
    EXPECT_EQ(Thermistor::readCelsius().error, Thermistor::Fault::OUT_OF_RANGE);
}

TEST_F(SmartBatteryTestSuite, INA226_InitFailures) {
    INA226::Driver drv(nullptr);
    EXPECT_FALSE(drv.init());

    INA226::Driver drv2(&i2c_manager);
    g_i2c_call_counter = 0;
    g_i2c_fail_on_call_n = 2;
    g_i2c_fail_on_call_errno = -EIO;
    EXPECT_FALSE(drv2.init());
}

TEST_F(SmartBatteryTestSuite, FetchWithRetry_ExactRetryCount) {
    g_i2c_consecutive_failures = 3;
    auto res = battery.fetchBusVoltageRawWithRetry();
    EXPECT_TRUE(res.success);
    EXPECT_EQ(battery.getStats().retries, 3);
}

TEST_F(SmartBatteryTestSuite, FetchCurrent_RetryAndFail) {
    g_i2c_force_errno = -EIO;
    auto res = battery.fetchCurrentRawWithRetry();
    EXPECT_FALSE(res.success);
    EXPECT_EQ(res.error, CommFault::I2C_NACK);
    g_i2c_force_errno = 0;
}

TEST_F(SmartBatteryTestSuite, PollHardware_ValidationRejects) {
    g_i2c_mock_v_val = 0;
    battery.pollHardwareAndUpdateCache();
    EXPECT_EQ(battery.cache.last_error, CommFault::NONE);

    battery.cache.last_error = CommFault::NONE;
    battery.cache.valid = true;
    g_i2c_mock_v_val = 0xFFFF;
    battery.pollHardwareAndUpdateCache();
    EXPECT_EQ(battery.cache.last_error, CommFault::VALIDATION_ERROR);
}

TEST_F(SmartBatteryTestSuite, JumpReject_ThresholdReached) {
    battery.cache.voltage.value = 10000; 
    battery.cache.current.value = 0;
    battery.cache.temperature.value = Thermistor::KELVIN_OFFSET_TENTHS + 250;
    battery.cache.valid = true;
    battery.cache.last_error = CommFault::NONE;
    battery.cache.timestamp_ms = virtual_uptime;
    
    g_i2c_mock_v_val = 8000;
    g_i2c_mock_i_val = 0; 
    g_adc_mock_mv_val = 1650; 

    battery.cache.current.value = -8001; 

    battery.pollHardwareAndUpdateCache();
    EXPECT_EQ(battery.consecutive_jump_rejects, 1);

    battery.consecutive_jump_rejects = 0;
    
    for(int i = 0; i < 10; ++i) { 
        battery.pollHardwareAndUpdateCache();
        if (battery.cache.last_error == CommFault::VALIDATION_ERROR) {
            break;
        }
    }
    EXPECT_EQ(battery.cache.last_error, CommFault::VALIDATION_ERROR);
}

TEST_F(SmartBatteryTestSuite, JumpDetection_NoJump_CoversRemainingLines)
{
    battery.cache.valid = true;
    battery.cache.last_error = CommFault::NONE;

    battery.cache.voltage.value = 10000;
    battery.cache.current.value = 0;
    battery.cache.temperature.value = Thermistor::KELVIN_OFFSET_TENTHS + 250;

    g_i2c_mock_v_val = 8000;
    g_i2c_mock_i_val = 0;
    g_adc_mock_mv_val = 1650;

    battery.pollHardwareAndUpdateCache();

    EXPECT_EQ(battery.consecutive_jump_rejects, 0);
}

TEST_F(SmartBatteryTestSuite, JumpDetection_CurrentOnly)
{
    battery.cache.valid = true;
    battery.cache.last_error = CommFault::NONE;

    battery.cache.voltage.value = 10000;
    battery.cache.current.value = -8001;
    battery.cache.temperature.value = Thermistor::KELVIN_OFFSET_TENTHS + 250;

    g_i2c_mock_v_val = 8000;
    g_i2c_mock_i_val = 0;
    g_adc_mock_mv_val = 1650;

    battery.pollHardwareAndUpdateCache();

    EXPECT_GT(battery.consecutive_jump_rejects,0);
}

TEST_F(SmartBatteryTestSuite, JumpDetection_TemperatureOnly)
{
    battery.cache.valid = true;
    battery.cache.last_error = CommFault::NONE;

    battery.cache.voltage.value = 10000;
    battery.cache.current.value = 0;
    battery.cache.temperature.value = Thermistor::KELVIN_OFFSET_TENTHS + 250;

    g_i2c_mock_v_val = 8000;
    g_i2c_mock_i_val = 0;
    g_adc_mock_mv_val = 114;

    battery.pollHardwareAndUpdateCache();

    EXPECT_GT(battery.consecutive_jump_rejects,0);
}

TEST_F(SmartBatteryTestSuite, CoulombCounter_AtRestDuration) {
    battery.seedOrResyncCoulombCounter(11000, 10, false);
    EXPECT_EQ(battery.rest_period_start_ms, virtual_uptime);

    battery.seedOrResyncCoulombCounter(11000, 500, false);
    EXPECT_EQ(battery.rest_period_start_ms, 0);

    battery.soc_initialized = false;
    battery.seedOrResyncCoulombCounter(BatteryLimits::PACK_MAX_VOLTAGE_MV, 10, false);
    EXPECT_TRUE(battery.soc_initialized);

    battery.soc_initialized = false;
    battery.seedOrResyncCoulombCounter(BatteryLimits::PACK_MIN_VOLTAGE_MV, 10, false);
    EXPECT_TRUE(battery.soc_initialized);
}

TEST_F(SmartBatteryTestSuite, UpdateStateAndPublish_ClampLimits) {
    battery.soc_initialized = true;
    battery.last_poll_time_ms = virtual_uptime;

    battery.kf_soc_pct = 200.0f;
    battery.updateStateAndPublish(13000, 0, 250, 0);
    EXPECT_EQ(battery.cache.soc.value, 100);

    battery.kf_soc_pct = -50.0f;
    battery.updateStateAndPublish(8000, 0, 250, 0);
    EXPECT_EQ(battery.cache.soc.value, 0);
    
    battery.updateStateAndPublish(13000, 5000, 250, 0);
}

TEST_F(SmartBatteryTestSuite, NotifySystemWakeup_Edges) {
    battery.cache.timestamp_ms = 0;
    battery.notifySystemWakeup();
    EXPECT_EQ(battery.cache.timestamp_ms, virtual_uptime);

    battery.cache.valid = false;
    battery.cache.timestamp_ms = 0;
    battery.notifySystemWakeup();
    EXPECT_EQ(battery.cache.timestamp_ms, 0);

    g_mutex_lock_force_errno = -EAGAIN;
    testing::internal::CaptureStdout();
    battery.notifySystemWakeup();
    const auto out = testing::internal::GetCapturedStdout();
    EXPECT_NE(out.find("Could not lock cache_mutex"), std::string::npos);
}

TEST_F(SmartBatteryTestSuite, PublishError_NullContext) {
    DeviceContext* backup = battery.sys_context;
    battery.sys_context = nullptr;
    battery.current_state.store(BatteryFSM::IDLE);
    battery.consecutive_comm_failures = 0;
    battery.last_valid_comm_time = 0;
    virtual_uptime = 999999;

    battery.publishError(CommFault::I2C_TIMEOUT);
    EXPECT_EQ(battery.getState(), BatteryFSM::CUTOFF);
    battery.sys_context = backup;
}

TEST_F(SmartBatteryTestSuite, ProcessFSM_AllBranches) {
    battery.cache.voltage.value = 10000;
    battery.cache.soc.value = 50;

    battery.cache.voltage.value = 0; battery.processFSM(); battery.cache.voltage.value = 10000;

    battery.cache.shunt_voltage.value = 0;
    battery.current_state.store(BatteryFSM::CHARGING);
    battery.processFSM();
    EXPECT_EQ(battery.getState(), BatteryFSM::IDLE);

    battery.full_charge_logged = true;
    battery.cache.soc.value = 90;
    battery.processFSM();
    EXPECT_FALSE(battery.full_charge_logged);

    DeviceContext* backup = battery.sys_context;
    battery.sys_context = nullptr;
    battery.current_state.store(BatteryFSM::CUTOFF);
    battery.cache.soc.value = 90;
    battery.processFSM();
    battery.sys_context = backup;
}

extern void bms_comm_thread(void);
extern void battery_monitor_thread(void);

TEST_F(SmartBatteryTestSuite, Threads_NullGuardsAndSkips) {
    extern SbsBattery* smart_battery;
    auto backup = smart_battery;

    smart_battery = nullptr;
    
    test_iterations_remaining = 1;
    bms_comm_thread(); 
    
    test_iterations_remaining = 0;
    battery_monitor_thread();
    
    smart_battery = backup;

    test_iterations_remaining = 0;
    PowerManager::getInstance().notifyBeforeSleep();
    
    std::thread waker([]() {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        PowerManager::getInstance().notifyAfterWakeup();
    });
    
    bms_comm_thread();
    waker.join();

#if defined(IPOWER_OBSERVER_HAS_SLEEP_ABORTED) || 1
    PowerManager::getInstance().notifySleepAborted();
#endif
}

TEST_F(SmartBatteryTestSuite, I2CFaultMapping_AllBranches) {
    auto test_map = [this](int errno_val) {
        g_i2c_force_errno = errno_val;
        auto res = battery.fetchBusVoltageRawWithRetry();
        g_i2c_force_errno = 0;
        return res.error;
    };

    EXPECT_EQ(test_map(0), CommFault::NONE);
    EXPECT_EQ(test_map(-ENODEV), CommFault::DEVICE_NOT_READY);
    EXPECT_EQ(test_map(-ETIMEDOUT), CommFault::I2C_TIMEOUT);
    EXPECT_EQ(test_map(-EBUSY), CommFault::I2C_BUS_BUSY);
    EXPECT_EQ(test_map(-EAGAIN), CommFault::I2C_ARBITRATION_LOST);
    EXPECT_EQ(test_map(-EIO), CommFault::I2C_NACK);
    EXPECT_EQ(test_map(-EPERM), CommFault::I2C_NACK);
}

TEST_F(SmartBatteryTestSuite, Getters_ValidAndInvalidCache) {
    battery.cache.valid = true;
    battery.cache.last_error = CommFault::NONE;
    battery.cache.timestamp_ms = virtual_uptime;

    EXPECT_TRUE(battery.getVoltage().success);
    EXPECT_TRUE(battery.getCurrent().success);
    EXPECT_TRUE(battery.getStateOfCharge().success);
    EXPECT_TRUE(battery.getTemperature().success);
    EXPECT_TRUE(battery.getCapacity().success);
    EXPECT_TRUE(battery.getShuntVoltage().success);

    virtual_uptime += 5000;
    EXPECT_FALSE(battery.getVoltage().success);
    EXPECT_EQ(battery.getVoltage().error, CommFault::CACHE_INVALID);

    virtual_uptime = battery.cache.timestamp_ms;
    battery.cache.last_error = CommFault::I2C_NACK;
    EXPECT_FALSE(battery.getCurrent().success);
    EXPECT_FALSE(battery.getStateOfCharge().success);
    EXPECT_FALSE(battery.getTemperature().success);
    EXPECT_FALSE(battery.getCapacity().success);
    EXPECT_FALSE(battery.getShuntVoltage().success);
}

TEST_F(SmartBatteryTestSuite, HardwarePoll_SequentialFaults) {
    g_i2c_consecutive_failures = 5;
    battery.pollHardwareAndUpdateCache();

    g_i2c_consecutive_failures = 0;
    g_i2c_call_counter = 0;
    g_i2c_fail_after_reads = 1;
    battery.pollHardwareAndUpdateCache();
    g_i2c_fail_after_reads = -1;

    g_adc_raw_to_mv_errno = -EIO;
    battery.pollHardwareAndUpdateCache();
    g_adc_raw_to_mv_errno = 0;
}

TEST_F(SmartBatteryTestSuite, MutexContention_AllPaths) {
    g_mutex_lock_force_errno = -EAGAIN;

    battery.updateStateAndPublish(10000, 100, 250, 0);

    BmsCache c = battery.getCacheSnapshot();
    EXPECT_FALSE(c.valid);
    EXPECT_EQ(c.last_error, CommFault::MUTEX_TIMEOUT);

    battery.publishError(CommFault::I2C_NACK);

    for (int i = 0; i < 5; ++i) battery.publishError(CommFault::MUTEX_TIMEOUT);
    EXPECT_EQ(battery.getState(), BatteryFSM::CUTOFF);

    g_mutex_lock_force_errno = 0;
}

TEST_F(SmartBatteryTestSuite, CoulombCounter_LongRestResync) {
    const uint32_t now = virtual_uptime;
    battery.seedOrResyncCoulombCounter(11000, 0, false);
    EXPECT_EQ(battery.rest_period_start_ms, now);

    virtual_uptime += (31 * 60 * 1000);

    battery.soc_initialized = false;
    battery.seedOrResyncCoulombCounter(11000, 0, false);
    EXPECT_TRUE(battery.soc_initialized);
}

TEST_F(SmartBatteryTestSuite, FSM_DeepCoverage) {
    battery.cache.valid = false;
    battery.processFSM();

    battery.cache.valid = true;
    battery.cache.voltage.value = 10000;
    battery.cache.soc.value = 100;
    battery.full_charge_logged.store(false);
    battery.processFSM();
    EXPECT_TRUE(battery.full_charge_logged);

    battery.processFSM();
    EXPECT_TRUE(battery.full_charge_logged);

    battery.cache.soc.value = 50;

    battery.cache.voltage.value = 0; battery.processFSM(); battery.cache.voltage.value = 10000;

    battery.cache.shunt_voltage.value = -50000; 
    battery.current_state.store(BatteryFSM::IDLE);
    battery.processFSM();
    EXPECT_EQ(battery.getState(), BatteryFSM::CHARGING);

    battery.cache.voltage.value = 0; battery.processFSM(); battery.cache.voltage.value = 10000;

    battery.cache.shunt_voltage.value = 0;
    battery.current_state.store(BatteryFSM::CHARGING);
    battery.processFSM();
    EXPECT_EQ(battery.getState(), BatteryFSM::IDLE);

    battery.cache.voltage.value = 0; battery.processFSM(); battery.cache.voltage.value = 10000;

    battery.cache.shunt_voltage.value = 50000;
    battery.current_state.store(BatteryFSM::IDLE);
    battery.processFSM();
    EXPECT_EQ(battery.getState(), BatteryFSM::DISCHARGING);

    battery.cache.soc.value = BatteryLimits::CUTOFF_SOC_PCT - 1;
    battery.current_state.store(BatteryFSM::DISCHARGING);
    battery.processFSM();
    EXPECT_EQ(battery.getState(), BatteryFSM::CUTOFF);

    battery.cache.soc.value = BatteryLimits::CUTOFF_SOC_PCT - 1;
    battery.processFSM();

    battery.cache.soc.value = BatteryLimits::REENABLE_SOC_PCT + 1;
    sys_context.requestTransition(SystemState::SAFE_HALT);
    battery.processFSM();

    battery.current_state.store(BatteryFSM::CUTOFF);
    battery.cache.soc.value = BatteryLimits::REENABLE_SOC_PCT + 1;
    sys_context.requestTransition(SystemState::RUNNING);
    battery.processFSM();

    battery.cache.voltage.value = 0; battery.processFSM(); battery.cache.voltage.value = 10000;

    battery.cache.shunt_voltage.value = 0;
    battery.cache.soc.value = BatteryLimits::REENABLE_SOC_PCT + 1;
    sys_context.requestTransition(SystemState::SAFE_HALT);
    battery.current_state.store(BatteryFSM::CUTOFF);
    battery.processFSM();
    EXPECT_EQ(battery.getState(), BatteryFSM::IDLE);
}

TEST_F(SmartBatteryTestSuite, BmsCommThread_InitRetryLog) {
    extern SbsBattery* smart_battery;
    auto backup = smart_battery;
    smart_battery = &battery;

    g_i2c_force_errno = -EIO;
    test_iterations_remaining = 1;
    bms_comm_thread();
    g_i2c_force_errno = 0;

    smart_battery = backup;
}

TEST_F(SmartBatteryTestSuite, ThreadExecution_And_Singletons) {
    extern SbsBattery* smart_battery;
    auto backup = smart_battery;
    smart_battery = &battery;

    EXPECT_EQ(getSmartBatteryInstance(), &battery);

    test_iterations_remaining = 1;
    bms_comm_thread();

    PowerManager::getInstance().notifyBeforeSleep();
    PowerManager::getInstance().notifyAfterWakeup();

    battery.cache.current.value = -500;
    battery.cache.soc.value = 60;
    battery.cache.timestamp_ms = virtual_uptime;
    battery.cache.valid = true;
    test_iterations_remaining = 1;
    battery_monitor_thread();

    battery.current_state.store(BatteryFSM::DISCHARGING);
    battery.cache.valid = false;
    test_iterations_remaining = 1;
    battery_monitor_thread();

    smart_battery = backup;
}

TEST_F(SmartBatteryTestSuite, Init_WatchdogHooks) {
    battery.setWatchdogFeedHook([]() { custom_hook_called = true; });
    battery.feedWatchdog();
    EXPECT_TRUE(custom_hook_called);

    battery.setWatchdogFeedHook(nullptr);
}

TEST_F(SmartBatteryTestSuite, Init_INA_Fail) {
    g_i2c_fail_on_call_n = 1;
    g_i2c_fail_on_call_errno = -EIO;
    EXPECT_FALSE(battery.init());
    g_i2c_fail_on_call_n = 0;
}

TEST_F(SmartBatteryTestSuite, Init_Thermistor_Fail) {
    g_i2c_fail_on_call_n = 0;
    g_i2c_force_errno = 0;
    g_adc_ready_mock = false;
    EXPECT_FALSE(battery.init());
}

TEST_F(SmartBatteryTestSuite, CoulombCounter_AtRestBranchCombos) {
    battery.rest_period_start_ms = 12345;
    battery.seedOrResyncCoulombCounter(11000, -500, false);
    EXPECT_EQ(battery.rest_period_start_ms, 0U);

    battery.soc_initialized = false;
    battery.seedOrResyncCoulombCounter(BatteryLimits::PACK_MAX_VOLTAGE_MV, 500, false);
    EXPECT_FALSE(battery.soc_initialized);

    battery.soc_initialized = false;
    battery.seedOrResyncCoulombCounter(BatteryLimits::PACK_MIN_VOLTAGE_MV, -500, false);
    EXPECT_FALSE(battery.soc_initialized);
}

TEST_F(SmartBatteryTestSuite, UpdateStateAndPublish_FirstSeedBranch) {
    battery.soc_initialized = false;
    battery.accumulated_uAh = 0;
    battery.last_poll_time_ms = 0;

    battery.updateStateAndPublish(11000, 10, 250, 0);

    EXPECT_TRUE(battery.soc_initialized);
    EXPECT_EQ(battery.last_poll_time_ms, virtual_uptime);
}

TEST_F(SmartBatteryTestSuite, UpdateStateAndPublish_NoClampNeeded) {
    battery.soc_initialized = true;
    battery.accumulated_uAh = 1000 * 1000LL;
    battery.updateStateAndPublish(11000, 0, 250, 0);
    EXPECT_GT(battery.cache.soc.value, 0);
    EXPECT_LT(battery.cache.soc.value, 100);
}

TEST_F(SmartBatteryTestSuite, WatchdogHook_ConstructorAndNullHook) {
    custom_hook_called = false;
    SbsBattery hooked_battery(&i2c_manager, &sys_context, &custom_watchdog_hook);
    hooked_battery.feedWatchdog();
    EXPECT_TRUE(custom_hook_called);

    battery.watchdog_feed_hook.store(nullptr);
    custom_hook_called = false;
    battery.feedWatchdog();
    EXPECT_FALSE(custom_hook_called);
}

TEST_F(SmartBatteryTestSuite, PollHardware_TotalI2CFailure_NoCacheFallback) {
    extern void resetI2CCacheForTests();
    resetI2CCacheForTests();

    g_i2c_force_errno = -EIO;
    battery.pollHardwareAndUpdateCache();
    EXPECT_EQ(battery.cache.last_error, CommFault::I2C_NACK);
    g_i2c_force_errno = 0;
}

TEST_F(SmartBatteryTestSuite, PollHardware_CurrentFetchTotalFailure) {
    extern void resetI2CCacheForTests();
    resetI2CCacheForTests();

    g_i2c_fail_after_reads = 2;
    battery.pollHardwareAndUpdateCache();
    g_i2c_fail_after_reads = -1;
    EXPECT_EQ(battery.cache.last_error, CommFault::I2C_NACK);
}

TEST_F(SmartBatteryTestSuite, CacheFreshness_ZeroTimestampShortCircuit) {
    battery.cache.valid = true;
    battery.cache.last_error = CommFault::NONE;
    battery.cache.timestamp_ms = 0;
    EXPECT_FALSE(battery.getVoltage().success);
    EXPECT_EQ(battery.getVoltage().error, CommFault::CACHE_INVALID);
}

TEST_F(SmartBatteryTestSuite, PublishError_MutexFaultWithSuccessfulLock) {
    g_mutex_lock_force_errno = 0;
    battery.consecutive_mutex_failures = 0;
    battery.current_state.store(BatteryFSM::IDLE);
    battery.publishError(CommFault::MUTEX_TIMEOUT);
    EXPECT_EQ(battery.consecutive_mutex_failures, 1U);
}

TEST_F(SmartBatteryTestSuite, PowerObserver_SleepAborted) {
    extern SbsBattery* smart_battery;
    auto backup = smart_battery;
    smart_battery = &battery;

    test_iterations_remaining = 1;
    bms_comm_thread();

    PowerManager::getInstance().notifySleepAborted();

    smart_battery = backup;
}

TEST_F(SmartBatteryTestSuite, CurveFitting_InteriorSegments) {
    static constexpr uint16_t ocv_probe_mv[] = {
        9100, 9900, 10500, 10950, 11250, 11550, 11850, 12150, 12450
    };
    for (uint16_t mv : ocv_probe_mv) {
        (void)battery.estimateSocFromVoltage(mv);
    }

    static constexpr int32_t ntc_probe_mv[] = {
        3200, 3160, 3110, 3050, 2970, 2870, 2750, 2610, 2460, 2290,
        2110, 1930, 1740, 1560, 1385, 1220, 1070,  935,  810,  700,
         605,  520,  450,  390,  340,  300,  260,  230,  200,  175,
         155,  135,  120
    };
    for (int32_t mv : ntc_probe_mv) {
        g_adc_mock_mv_val = mv;
        EXPECT_TRUE(Thermistor::readCelsius().success);
    }
}

TEST_F(SmartBatteryTestSuite, BmsCommThread_SkipsPollWhileSleeping) {
    extern SbsBattery* smart_battery;
    auto backup = smart_battery;
    smart_battery = &battery;

    test_iterations_remaining = 0;
    bms_comm_thread();

    PowerManager::getInstance().notifyBeforeSleep();
    
    std::thread waker([]() {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        PowerManager::getInstance().notifyAfterWakeup();
    });
    
    test_iterations_remaining = 0;
    bms_comm_thread();
    waker.join();

    smart_battery = backup;
}

TEST_F(SmartBatteryTestSuite, BatteryMonitorThread_SkipsProcessWhileSleeping) {
    extern SbsBattery* smart_battery;
    auto backup = smart_battery;
    smart_battery = &battery;

    PowerManager::getInstance().notifyBeforeSleep();
    
    std::thread waker([]() {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        PowerManager::getInstance().notifyAfterWakeup();
    });
    
    test_iterations_remaining = 0;
    battery_monitor_thread();
    waker.join();

    smart_battery = backup;
}

TEST_F(SmartBatteryTestSuite, BatteryMonitorThread_NonDischargingSkipsLog) {
    extern SbsBattery* smart_battery;
    auto backup = smart_battery;
    smart_battery = &battery;

    battery.current_state.store(BatteryFSM::IDLE);
    battery.cache.valid = true;
    battery.cache.last_error = CommFault::NONE;
    battery.cache.timestamp_ms = virtual_uptime;
    battery.cache.current.value = 0;
    battery.cache.soc.value = 50;

    test_iterations_remaining = 0;
    battery_monitor_thread();
    EXPECT_EQ(battery.getState(), BatteryFSM::IDLE);

    smart_battery = backup;
}

TEST_F(SmartBatteryTestSuite, PowerObserver_AfterWakeup_NullSmartBattery) {
    extern SbsBattery* smart_battery;
    auto backup = smart_battery;

    smart_battery = &battery;
    test_iterations_remaining = 0;
    bms_comm_thread();

    smart_battery = nullptr;
    PowerManager::getInstance().notifyBeforeSleep();
    PowerManager::getInstance().notifyAfterWakeup();

    smart_battery = backup;
}

TEST_F(SmartBatteryTestSuite, ProcessFSM_MidBandsAndChargingRecovery) {
    battery.cache.voltage.value = 10000;
    battery.cache.valid = true;
    battery.cache.last_error = CommFault::NONE;
    battery.cache.timestamp_ms = virtual_uptime;

    battery.cache.current.value = 0;
    battery.cache.soc.value = (BatteryLimits::CUTOFF_SOC_PCT + BatteryLimits::REENABLE_SOC_PCT) / 2;
    battery.current_state.store(BatteryFSM::IDLE);
    battery.processFSM();
    EXPECT_EQ(battery.getState(), BatteryFSM::IDLE);

    battery.cache.soc.value = 97;
    battery.full_charge_logged.store(true);
    battery.processFSM();
    EXPECT_TRUE(battery.full_charge_logged);

    battery.cache.current.value = 100;
    battery.cache.soc.value = BatteryLimits::REENABLE_SOC_PCT + 1;
    battery.current_state.store(BatteryFSM::CUTOFF);
    battery.cache.shunt_voltage.value = -50000;
    sys_context.requestTransition(SystemState::SAFE_HALT);
    battery.processFSM();
    EXPECT_EQ(battery.getState(), BatteryFSM::CHARGING);
}

TEST_F(SmartBatteryTestSuite, JumpDetection_AllConditions) {
    battery.cache.valid=true;
    battery.cache.last_error=CommFault::NONE;

    battery.cache.voltage.value=10000;
    battery.cache.current.value=0;
    battery.cache.temperature.value=Thermistor::KELVIN_OFFSET_TENTHS+250;

    g_i2c_mock_v_val=8000;
    g_i2c_mock_i_val=3000;
    g_adc_mock_mv_val=2023;

    battery.cache.voltage.value=10000;
    battery.cache.current.value=0;
    battery.cache.temperature.value=Thermistor::KELVIN_OFFSET_TENTHS+150;

    battery.pollHardwareAndUpdateCache();

    battery.consecutive_jump_rejects=0;

    g_i2c_mock_v_val=8000;
    g_i2c_mock_i_val=0;
    g_adc_mock_mv_val=114;

    battery.pollHardwareAndUpdateCache();

    battery.consecutive_jump_rejects=5;

    g_i2c_mock_v_val=8000;
    g_i2c_mock_i_val=0;
    g_adc_mock_mv_val=2023;

    battery.cache.voltage.value=10000;
    battery.cache.current.value=0;
    battery.cache.temperature.value=Thermistor::KELVIN_OFFSET_TENTHS+150;

    battery.pollHardwareAndUpdateCache();

    EXPECT_EQ(battery.consecutive_jump_rejects,0);
}

TEST_F(SmartBatteryTestSuite, Thermistor_AdcReadFailures) {
    g_adc_ready_mock = true;

    g_adc_sequence_init_dt_errno = -EIO;
    EXPECT_EQ(Thermistor::readCelsius().error, Thermistor::Fault::ADC_READ_ERROR);
    g_adc_sequence_init_dt_errno = 0;

    g_adc_read_force_errno = -EIO;
    EXPECT_EQ(Thermistor::readCelsius().error, Thermistor::Fault::ADC_READ_ERROR);
    g_adc_read_force_errno = 0;
}

TEST_F(SmartBatteryTestSuite, PollHardware_InvalidCache_NotValidationError) {
    battery.cache.valid = false;
    battery.cache.last_error = CommFault::I2C_NACK;

    g_i2c_mock_v_val = 10000;
    g_i2c_mock_i_val = 0;

    battery.pollHardwareAndUpdateCache();

    EXPECT_EQ(battery.cache.last_error, CommFault::NONE);
    EXPECT_TRUE(battery.cache.valid);
}

TEST_F(SmartBatteryTestSuite, Fetch_NullWatchdogHook_DuringRetry) {
    battery.setWatchdogFeedHook(nullptr);

    g_i2c_force_errno = -EIO;
    battery.fetchBusVoltageRawWithRetry();
    battery.fetchCurrentRawWithRetry();
    battery.fetchShuntVoltageRawWithRetry();
    g_i2c_force_errno = 0;
}

TEST_F(SmartBatteryTestSuite, ThreadLoops_SleepAwakeBranch_Robust) {
    extern SbsBattery* smart_battery;
    auto backup = smart_battery;
    smart_battery = &battery;

    PowerManager::getInstance().notifyAfterWakeup();

    battery.current_state.store(BatteryFSM::IDLE);
    battery.cache.valid = true;
    battery.cache.last_error = CommFault::NONE;
    battery.cache.timestamp_ms = virtual_uptime;
    battery.cache.current.value = 0;
    battery.cache.soc.value = 50;

    test_iterations_remaining = 0;
    bms_comm_thread();
    test_iterations_remaining = 0;
    battery_monitor_thread();

    PowerManager::getInstance().notifyBeforeSleep();
    
    std::thread waker1([]() {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        PowerManager::getInstance().notifyAfterWakeup();
    });
    test_iterations_remaining = 0;
    bms_comm_thread();
    waker1.join();

    PowerManager::getInstance().notifyBeforeSleep();
    
    std::thread waker2([]() {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        PowerManager::getInstance().notifyAfterWakeup();
    });
    test_iterations_remaining = 0;
    battery_monitor_thread();
    waker2.join();

    smart_battery = backup;
}

TEST_F(SmartBatteryTestSuite, BatteryMonitorThread_DischargingSocFailureSkipsLog) {
    extern SbsBattery* smart_battery;
    auto backup = smart_battery;
    smart_battery = &battery;

    battery.current_state.store(BatteryFSM::DISCHARGING);
    battery.cache.valid = true;
    battery.cache.last_error = CommFault::NONE;
    battery.cache.current.value = -100;
    battery.cache.soc.value = 50;
    battery.cache.timestamp_ms = virtual_uptime;

    virtual_uptime += (BatteryLimits::CACHE_STALE_MS + 1000);

    test_iterations_remaining = 0;
    battery_monitor_thread();

    smart_battery = backup;
}

TEST_F(SmartBatteryTestSuite, ProcessFSM_CutoffTrigger_NullContextSkipsFaultCall) {
    DeviceContext* backup_ctx = battery.sys_context;
    battery.sys_context = nullptr;

    battery.cache.voltage.value = 10000;
    battery.cache.valid = true;
    battery.cache.last_error = CommFault::NONE;
    battery.cache.timestamp_ms = virtual_uptime;
    battery.cache.current.value = -100;
    battery.cache.soc.value = BatteryLimits::CUTOFF_SOC_PCT - 1;
    battery.current_state.store(BatteryFSM::DISCHARGING);

    battery.processFSM();
    EXPECT_EQ(battery.getState(), BatteryFSM::CUTOFF);

    battery.sys_context = backup_ctx;
}

TEST_F(SmartBatteryTestSuite, ProcessFSM_ReenableCheck_NullContextSkipsRecovery) {
    DeviceContext* backup_ctx = battery.sys_context;
    battery.sys_context = nullptr;

    battery.cache.voltage.value = 10000;
    battery.cache.valid = true;
    battery.cache.last_error = CommFault::NONE;
    battery.cache.timestamp_ms = virtual_uptime;
    battery.cache.current.value = 0;
    battery.cache.soc.value = BatteryLimits::REENABLE_SOC_PCT + 1;
    battery.current_state.store(BatteryFSM::CUTOFF);

    battery.processFSM();

    EXPECT_EQ(battery.getState(), BatteryFSM::CUTOFF);

    battery.sys_context = backup_ctx;
}

TEST_F(SmartBatteryTestSuite, ProcessFSM_CacheFailureReason_BothBranches) {
    testing::internal::CaptureStdout();

    battery.cache.valid = false;
    battery.cache.last_error = CommFault::I2C_NACK;
    battery.processFSM();

    battery.cache.valid = false;
    battery.cache.last_error = CommFault::NONE;
    battery.processFSM();

    testing::internal::GetCapturedStdout();
}

TEST_F(SmartBatteryTestSuite, BatteryMonitorThread_DischargingSocFailure_MutexPath) {
    extern SbsBattery* smart_battery;
    auto backup = smart_battery;
    smart_battery = &battery;

    battery.current_state.store(BatteryFSM::DISCHARGING);
    battery.cache.valid = true;
    battery.cache.last_error = CommFault::NONE;
    battery.cache.current.value = -50;
    battery.cache.soc.value = 40;
    battery.cache.timestamp_ms = virtual_uptime;

    g_mutex_lock_force_errno = -EAGAIN;
    test_iterations_remaining = 0;
    battery_monitor_thread();
    g_mutex_lock_force_errno = 0;

    smart_battery = backup;
}

TEST_F(SmartBatteryTestSuite, ProcessFSM_Reenable_SysContextNonNull_WrongState_Direct) {
    battery.cache.voltage.value = 10000;
    battery.cache.valid = true;
    battery.cache.last_error = CommFault::NONE;
    battery.cache.timestamp_ms = virtual_uptime;
    battery.cache.current.value = 0;
    battery.cache.soc.value = BatteryLimits::REENABLE_SOC_PCT + 1;
    battery.current_state.store(BatteryFSM::CUTOFF);

    sys_context.requestTransition(SystemState::INIT);
    battery.processFSM();
    EXPECT_EQ(battery.getState(), BatteryFSM::CUTOFF);
}

TEST_F(SmartBatteryTestSuite, BmsCommThread_SleepBranch_OrderIndependent) {
    extern SbsBattery* smart_battery;
    auto backup = smart_battery;
    smart_battery = &battery;

    test_iterations_remaining = 0;
    bms_comm_thread();
    PowerManager::getInstance().notifyAfterWakeup();

    g_i2c_call_counter = 0;
    test_iterations_remaining = 0;
    bms_comm_thread();
    EXPECT_GT(g_i2c_call_counter, 0);

    PowerManager::getInstance().notifyBeforeSleep();
    g_i2c_call_counter = 0;
    
    std::thread waker([]() {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        PowerManager::getInstance().notifyAfterWakeup();
    });
    
    test_iterations_remaining = 0;
    bms_comm_thread();
    waker.join();
    
    EXPECT_GT(g_i2c_call_counter, 0);

    smart_battery = backup;
}

TEST_F(SmartBatteryTestSuite, BmsCommThread_SleepBranch_Diagnostic) {
    extern SbsBattery* smart_battery;
    extern bool isBmsObserverSleepingForTest();
    extern void resetI2CCacheForTests();
    auto backup = smart_battery;
    smart_battery = &battery;

    PowerManager::getInstance().resetForTest();
    resetI2CCacheForTests();

    test_iterations_remaining = 0;
    bms_comm_thread();
    ASSERT_FALSE(isBmsObserverSleepingForTest());

    g_i2c_call_counter = 0;
    test_iterations_remaining = 0;
    bms_comm_thread();
    const int awake_calls = g_i2c_call_counter;
    ASSERT_GT(awake_calls, 0);

    PowerManager::getInstance().notifyBeforeSleep();
    ASSERT_TRUE(isBmsObserverSleepingForTest());

    g_i2c_call_counter = 0;
    
    std::thread waker([]() {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        PowerManager::getInstance().notifyAfterWakeup();
    });

    test_iterations_remaining = 0;
    bms_comm_thread();
    waker.join();
    
    const int asleep_calls = g_i2c_call_counter;
    EXPECT_EQ(asleep_calls, awake_calls);

    smart_battery = backup;
}

TEST_F(SmartBatteryTestSuite, MapI2CFault_AllBranches_Direct) {
    extern CommFault test_mapI2CFault(I2CFault fault);

    EXPECT_EQ(test_mapI2CFault(I2CFault::NACK), CommFault::I2C_NACK);
    EXPECT_EQ(test_mapI2CFault(I2CFault::TIMEOUT), CommFault::I2C_TIMEOUT);
    EXPECT_EQ(test_mapI2CFault(I2CFault::BUS_BUSY), CommFault::I2C_BUS_BUSY);
    EXPECT_EQ(test_mapI2CFault(I2CFault::ARBITRATION_LOST), CommFault::I2C_ARBITRATION_LOST);
    EXPECT_EQ(test_mapI2CFault(I2CFault::DEVICE_NOT_READY), CommFault::DEVICE_NOT_READY);

    EXPECT_EQ(test_mapI2CFault(static_cast<I2CFault>(99)), CommFault::I2C_NACK);
}

TEST_F(SmartBatteryTestSuite, MonitorThread_SocFailsAloneCoversLine688) {
    extern SbsBattery* smart_battery;
    auto backup = smart_battery;
    smart_battery = &battery;

    battery.cache.valid = true;
    battery.cache.last_error = CommFault::NONE;
    battery.cache.timestamp_ms = virtual_uptime;
    battery.cache.current.value = 0;
    battery.cache.soc.value = 50;

    g_mutex_lock_target_ptr = &battery.cache_mutex;
    g_mutex_lock_target_call_counter = 0;
    g_mutex_lock_target_fail_on_call_n = 3;
    test_iterations_remaining = 0;
    battery_monitor_thread();
    g_mutex_lock_target_ptr = nullptr;
    g_mutex_lock_target_fail_on_call_n = 0;

    smart_battery = backup;
}

TEST_F(SmartBatteryTestSuite, MonitorThread_TempFailsAloneCoversLine688) {
    extern SbsBattery* smart_battery;
    auto backup = smart_battery;
    smart_battery = &battery;

    battery.cache.valid = true;
    battery.cache.last_error = CommFault::NONE;
    battery.cache.timestamp_ms = virtual_uptime;
    battery.cache.current.value = 0;
    battery.cache.soc.value = 50;

    g_mutex_lock_target_ptr = &battery.cache_mutex;
    g_mutex_lock_target_call_counter = 0;
    g_mutex_lock_target_fail_on_call_n = 4;
    test_iterations_remaining = 0;
    battery_monitor_thread();
    g_mutex_lock_target_ptr = nullptr;
    g_mutex_lock_target_fail_on_call_n = 0;

    smart_battery = backup;
}

TEST_F(SmartBatteryTestSuite, FullChargeLog_NvsWriteFailureCoversLine550) {
    extern int g_nvs_write_force_errno;

    ConfigStore::getInstance().initialized = true;

    battery.cache.valid = true;
    battery.cache.last_error = CommFault::NONE;
    battery.cache.timestamp_ms = virtual_uptime;
    battery.cache.soc.value = 100;
    battery.full_charge_logged.store(false);

    g_nvs_write_force_errno = -EIO;
    battery.processFSM();
    g_nvs_write_force_errno = 0;

    battery.full_charge_logged.store(false);
    battery.processFSM();
}

TEST_F(SmartBatteryTestSuite, FullChargeLog_ConfigStoreUninitializedCoversLine550) {
    const bool backup_initialized = ConfigStore::getInstance().initialized;
    ConfigStore::getInstance().initialized = false;

    battery.cache.valid = true;
    battery.cache.last_error = CommFault::NONE;
    battery.cache.timestamp_ms = virtual_uptime;
    battery.cache.soc.value = 100;
    battery.full_charge_logged.store(false);

    battery.processFSM();

    ConfigStore::getInstance().initialized = backup_initialized;
}

TEST_F(SmartBatteryTestSuite, PollHardware_JumpDetection_CacheCombinations) {
    battery.cache.timestamp_ms = 0;
    g_i2c_mock_v_val = 10000;
    battery.pollHardwareAndUpdateCache();

    battery.cache.timestamp_ms = virtual_uptime;
    battery.cache.valid = false;
    battery.cache.last_error = CommFault::VALIDATION_ERROR;
    battery.pollHardwareAndUpdateCache();
}

TEST_F(SmartBatteryTestSuite, MonitorThread_TempFailsAloneCoversLine704) {
    extern SbsBattery* smart_battery;
    auto backup = smart_battery;
    smart_battery = &battery;

    battery.cache.valid = true;
    battery.cache.last_error = CommFault::NONE;
    battery.cache.timestamp_ms = virtual_uptime;
    battery.cache.current.value = 0;
    battery.cache.soc.value = 50;

    g_mutex_lock_target_ptr = &battery.cache_mutex;
    g_mutex_lock_target_call_counter = 0;
    g_mutex_lock_target_fail_on_call_n = 5; 
    
    test_iterations_remaining = 0;
    battery_monitor_thread();
    
    g_mutex_lock_target_ptr = nullptr;
    g_mutex_lock_target_fail_on_call_n = 0;
    smart_battery = backup;
}

TEST_F(SmartBatteryTestSuite, BmsCommThread_SafeHaltBranch) {
    extern SbsBattery* smart_battery;
    auto backup = smart_battery;
    smart_battery = &battery;

    sys_context.requestTransition(SystemState::SAFE_HALT);
    battery.current_state.store(BatteryFSM::IDLE);

    std::thread waker([]() {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        sys_context.requestTransition(SystemState::INIT);
        sys_context.requestTransition(SystemState::RUNNING);
    });

    test_iterations_remaining = 0;
    bms_comm_thread();
    waker.join();

    smart_battery = backup;
}

TEST_F(SmartBatteryTestSuite, BatteryMonitorThread_SafeHaltBranch) {
    extern SbsBattery* smart_battery;
    auto backup = smart_battery;
    smart_battery = &battery;

    sys_context.requestTransition(SystemState::SAFE_HALT);
    battery.current_state.store(BatteryFSM::IDLE);

    std::thread waker([]() {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        sys_context.requestTransition(SystemState::INIT);
        sys_context.requestTransition(SystemState::RUNNING);
    });

    test_iterations_remaining = 0;
    battery_monitor_thread();
    waker.join();

    smart_battery = backup;
}

TEST_F(SmartBatteryTestSuite, BmsCommThread_SleepingButCharging) {
    extern SbsBattery* smart_battery;
    auto backup = smart_battery;
    smart_battery = &battery;

    PowerManager::getInstance().notifyBeforeSleep();
    battery.current_state.store(BatteryFSM::CHARGING);
    sys_context.requestTransition(SystemState::RUNNING);

    std::thread waker([]() {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        PowerManager::getInstance().notifyAfterWakeup();
    });

    test_iterations_remaining = 0;
    bms_comm_thread();
    waker.join();

    smart_battery = backup;
}

TEST_F(SmartBatteryTestSuite, BatteryMonitorThread_SleepingButCharging) {
    extern SbsBattery* smart_battery;
    auto backup = smart_battery;
    smart_battery = &battery;

    PowerManager::getInstance().notifyBeforeSleep();
    battery.current_state.store(BatteryFSM::CHARGING); 
    sys_context.requestTransition(SystemState::RUNNING);

    std::thread waker([]() {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        PowerManager::getInstance().notifyAfterWakeup();
    });

    test_iterations_remaining = 0;
    battery_monitor_thread(); 
    waker.join();

    smart_battery = backup;
}

TEST_F(SmartBatteryTestSuite, BmsCommThread_SafeHaltButInCutoff) {
    extern SbsBattery* smart_battery;
    auto backup = smart_battery;
    smart_battery = &battery;

    sys_context.requestTransition(SystemState::SAFE_HALT);
    battery.current_state.store(BatteryFSM::CUTOFF); 
    PowerManager::getInstance().notifyAfterWakeup();

    std::thread waker([]() {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        sys_context.requestTransition(SystemState::INIT);
        sys_context.requestTransition(SystemState::RUNNING);
    });

    test_iterations_remaining = 0;
    bms_comm_thread(); 
    waker.join();

    smart_battery = backup;
}

TEST_F(SmartBatteryTestSuite, BatteryMonitorThread_SafeHaltButInCutoff) {
    extern SbsBattery* smart_battery;
    auto backup = smart_battery;
    smart_battery = &battery;

    sys_context.requestTransition(SystemState::SAFE_HALT);
    battery.current_state.store(BatteryFSM::CUTOFF); 

    std::thread waker([]() {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        sys_context.requestTransition(SystemState::INIT);
        sys_context.requestTransition(SystemState::RUNNING);
    });

    test_iterations_remaining = 0;
    battery_monitor_thread(); 
    waker.join();

    smart_battery = backup;
}

TEST_F(SmartBatteryTestSuite, BmsCommThread_GettersFail_CoversElseLog) {
    extern SbsBattery* smart_battery;
    auto backup = smart_battery;
    smart_battery = &battery;

    sys_context.requestTransition(SystemState::RUNNING);
    PowerManager::getInstance().notifyAfterWakeup();
    battery.current_state.store(BatteryFSM::IDLE);
    
    g_mutex_lock_target_ptr = &battery.cache_mutex;
    g_mutex_lock_target_call_counter = 0;
    g_mutex_lock_target_fail_on_call_n = 4;

    test_iterations_remaining = 0;
    bms_comm_thread(); 
    
    g_mutex_lock_target_ptr = nullptr;
    g_mutex_lock_target_fail_on_call_n = 0;
    smart_battery = backup;
}

TEST_F(SmartBatteryTestSuite, BmsCommThread_SafeHalt_InitLoop) {
    extern SbsBattery* smart_battery;
    auto backup = smart_battery;
    smart_battery = &battery;

    sys_context.requestTransition(SystemState::SAFE_HALT);
    PowerManager::getInstance().notifyAfterWakeup(); 

    std::thread waker([]() {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        sys_context.requestTransition(SystemState::INIT);
        sys_context.requestTransition(SystemState::RUNNING);
    });

    test_iterations_remaining = 0;
    bms_comm_thread(); 
    waker.join();

    smart_battery = backup;
}

TEST_F(SmartBatteryTestSuite, Init_Success) {
    g_i2c_force_errno = 0;
    g_adc_ready_mock = true;
    EXPECT_TRUE(battery.init());
}

TEST_F(SmartBatteryTestSuite, FetchShunt_RetryAndFail) {
    g_i2c_force_errno = -EIO;
    auto res = battery.fetchShuntVoltageRawWithRetry();
    EXPECT_FALSE(res.success);
    EXPECT_EQ(res.error, CommFault::I2C_NACK);
    g_i2c_force_errno = 0;
}

TEST_F(SmartBatteryTestSuite, PollHardware_DisconnectAfterLargeVoltageDrop)
{
    battery.cache.valid = true;
    battery.cache.last_error = CommFault::NONE;
    battery.cache.timestamp_ms = virtual_uptime;
    battery.cache.voltage.value = 12000;

    g_i2c_mock_v_val = 6000;   // pack = 7500 mV

    battery.pollHardwareAndUpdateCache();

    EXPECT_EQ(battery.getState(), BatteryFSM::IDLE);
}

TEST_F(SmartBatteryTestSuite, BmsCommThread_InitFailure_AlreadySafeHalt)
{
    extern SbsBattery* smart_battery;
    auto backup = smart_battery;
    smart_battery = &battery;

    sys_context.requestTransition(SystemState::RUNNING);

    g_i2c_force_errno = -EIO;
    g_set_safe_halt_on_i2c_call = 5;
    test_iterations_remaining = 1;

    bms_comm_thread();

    g_set_safe_halt_on_i2c_call = -1;
    g_i2c_force_errno = 0;
    smart_battery = backup;
}

TEST_F(SmartBatteryTestSuite, JumpDetection_VoltageOnly)
{
    battery.cache.valid = true;
    battery.cache.last_error = CommFault::NONE;
    battery.cache.timestamp_ms = virtual_uptime;
    
    // Set a baseline cached voltage
    battery.cache.voltage.value = 10000;
    battery.cache.current.value = 0;
    battery.cache.temperature.value = Thermistor::KELVIN_OFFSET_TENTHS + 25;

    // Provide a new mock voltage that creates a delta larger than MAX_VOLTAGE_DELTA_MV
    g_i2c_mock_v_val = 15000; // Large jump from 10000
    g_i2c_mock_i_val = 0;
    g_adc_mock_mv_val = 2500;

    battery.pollHardwareAndUpdateCache();
    
    // Verify that the jump was rejected and the counter incremented
    EXPECT_GT(battery.consecutive_jump_rejects, 0);
}

TEST_F(SmartBatteryTestSuite, BmsCommThread_InitFailure_Escalation) {
    extern SbsBattery* smart_battery;
    auto backup = smart_battery;
    smart_battery = &battery;

    sys_context.requestTransition(SystemState::RUNNING);

    // Force INA226 init to fail across multiple retries until fault escalation
    g_i2c_force_errno = -EIO;
    test_iterations_remaining = 5;

    bms_comm_thread();

    g_i2c_force_errno = 0;
    smart_battery = backup;
}

TEST_F(SmartBatteryTestSuite, ThreadLoops_FullBooleanConditionCoverage) {
    extern SbsBattery* smart_battery;
    auto backup = smart_battery;
    smart_battery = &battery;

    // Make sure the observer is registered and the system is awake/running.
    PowerManager::getInstance().notifyAfterWakeup();
    sys_context.requestTransition(SystemState::RUNNING);
    test_iterations_remaining = 0;
    bms_comm_thread();

    // The pre-init loop in bms_comm_thread blocks while sleeping or SAFE_HALT, so sleep /
    // SAFE_HALT must be raised *during init()* (via the i2c_write hooks), i.e. after that
    // loop has been passed and before the main do/while loop.

    // 1. Sleeping = true, Charging = true -> (!isBatteryCharging()) false, no waiting
    battery.current_state.store(BatteryFSM::CHARGING);
    g_i2c_call_counter = 0;
    g_sleep_on_i2c_call = 1;
    test_iterations_remaining = 1;
    bms_comm_thread();
    g_sleep_on_i2c_call = -1;

    test_iterations_remaining = 1;
    battery_monitor_thread();          // sleeping + charging: does not block

    // 2. SAFE_HALT = true, Cutoff = true -> (!isBatteryInCutoff()) false, no waiting
    PowerManager::getInstance().notifyAfterWakeup();
    battery.current_state.store(BatteryFSM::CUTOFF);
    g_i2c_call_counter = 0;
    g_set_safe_halt_on_i2c_call = 1;
    test_iterations_remaining = 1;
    bms_comm_thread();
    g_set_safe_halt_on_i2c_call = -1;

    test_iterations_remaining = 1;
    battery_monitor_thread();          // SAFE_HALT + cutoff: does not block

    sys_context.requestTransition(SystemState::INIT);
    sys_context.requestTransition(SystemState::RUNNING);

    // 3. Sleeping = true, Charging = false -> loop body executes until state becomes CHARGING
    PowerManager::getInstance().notifyAfterWakeup();
    battery.current_state.store(BatteryFSM::IDLE);
    g_i2c_call_counter = 0;
    g_sleep_on_i2c_call = 1;

    std::thread waker([this]() {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        battery.current_state.store(BatteryFSM::CHARGING);   // exits the wait loop
    });

    test_iterations_remaining = 1;
    bms_comm_thread();
    waker.join();
    g_sleep_on_i2c_call = -1;

    PowerManager::getInstance().notifyAfterWakeup();   // leave the observer awake for later tests
    smart_battery = backup;
}

TEST_F(SmartBatteryTestSuite, BmsCommThread_GetterFailures_AllBranches) {
    extern SbsBattery* smart_battery;
    auto backup = smart_battery;
    smart_battery = &battery;

    sys_context.requestTransition(SystemState::RUNNING);
    PowerManager::getInstance().notifyAfterWakeup();
    battery.current_state.store(BatteryFSM::IDLE);

    // cache_mutex lock order: 1=wakeup, 2=poll snapshot, 3=publish, 4=v, 5=i, 6=soc, 7=temp, 8=cap
    for (int fail_call = 1; fail_call <= 8; ++fail_call) {
        battery.cache.valid = true;
        battery.cache.last_error = CommFault::NONE;
        battery.cache.timestamp_ms = virtual_uptime;

        g_mutex_lock_target_ptr = &battery.cache_mutex;
        g_mutex_lock_target_call_counter = 0;
        g_mutex_lock_target_fail_on_call_n = fail_call;

        test_iterations_remaining = 1;
        bms_comm_thread();

        g_mutex_lock_target_ptr = nullptr;
        g_mutex_lock_target_fail_on_call_n = 0;
    }

    smart_battery = backup;
}

TEST_F(SmartBatteryTestSuite, ProcessFSM_FullChargeAndRecoveryBranches) {
    // NOTE: cache.voltage must be >= MIN_VALID_VOLTAGE_MV, otherwise processFSM()
    // takes the "battery disconnected" early exit and forces the state to IDLE.
    battery.cache.valid = true;
    battery.cache.last_error = CommFault::NONE;
    battery.cache.timestamp_ms = virtual_uptime;
    battery.cache.voltage.value = 10000;
    battery.cache.shunt_voltage.value = 0;

    // 1. Full Charge compare_exchange_strong failure branch (full_charge_logged already true)
    battery.cache.soc.value = 100;
    battery.full_charge_logged.store(true);
    battery.processFSM();

    // 2. Deadband coverage: 95% <= SOC < 100% (SOC = 97%)
    battery.cache.soc.value = 97;
    battery.processFSM();

    // 3. Recovery Ternary: CUTOFF -> CHARGING (shunt_uv < -IDLE_SHUNT_UV_THRESHOLD)
    battery.current_state.store(BatteryFSM::CUTOFF);
    battery.cache.soc.value = BatteryLimits::REENABLE_SOC_PCT + 5;
    battery.cache.shunt_voltage.value = -50000;
    sys_context.requestTransition(SystemState::SAFE_HALT);

    battery.processFSM();
    EXPECT_EQ(battery.getState(), BatteryFSM::CHARGING);

    // 4. Recovery Ternary: CUTOFF -> IDLE (shunt_uv >= -IDLE_SHUNT_UV_THRESHOLD)
    // Recovery moved the system SAFE_HALT -> INIT; go back to RUNNING before halting again.
    sys_context.requestTransition(SystemState::RUNNING);
    battery.current_state.store(BatteryFSM::CUTOFF);
    battery.cache.soc.value = BatteryLimits::REENABLE_SOC_PCT + 5;
    battery.cache.shunt_voltage.value = 0;
    sys_context.requestTransition(SystemState::SAFE_HALT);

    battery.processFSM();
    EXPECT_EQ(battery.getState(), BatteryFSM::IDLE);
}

TEST_F(SmartBatteryTestSuite, Disconnect_BranchCoverage) {
    // 1. was_connected = true, pack_mv < MIN_VALID_VOLTAGE_MV, v_drop <= MAX_VOLTAGE_DELTA_MV
    battery.cache.valid = true;
    battery.cache.last_error = CommFault::NONE;
    battery.cache.timestamp_ms = virtual_uptime;
    battery.cache.voltage.value = 3000;

    g_i2c_mock_v_val = 2000; // Pack voltage = 2500 mV (v_drop = 500 mV)
    battery.pollHardwareAndUpdateCache();
    EXPECT_EQ(battery.getState(), BatteryFSM::IDLE);

    // 2. was_connected = false, pack_mv >= PACK_MIN_VOLTAGE_MV
    battery.cache.valid = false;
    battery.cache.voltage.value = 0;
    g_i2c_mock_v_val = 8000; // Pack voltage = 10000 mV
    battery.pollHardwareAndUpdateCache();

    // 3. Disconnect with mutex lock failure
    battery.cache.valid = true;
    battery.cache.voltage.value = 10000;
    g_i2c_mock_v_val = 0; // Pack voltage = 0 mV (disconnected)
    g_mutex_lock_force_errno = -EAGAIN;

    battery.pollHardwareAndUpdateCache();
    g_mutex_lock_force_errno = 0;
}

TEST_F(SmartBatteryTestSuite, JumpDetection_AllBranchCombinations) {
    // 1. snapshot.timestamp_ms == 0 (skips jump calculation)
    battery.cache.timestamp_ms = 0;
    battery.cache.valid = true;
    g_i2c_mock_v_val = 10000;
    battery.pollHardwareAndUpdateCache();

    // 2. snapshot.valid == false BUT snapshot.last_error == CommFault::VALIDATION_ERROR
    battery.cache.timestamp_ms = virtual_uptime;
    battery.cache.valid = false;
    battery.cache.last_error = CommFault::VALIDATION_ERROR;
    battery.cache.voltage.value = 10000;
    battery.cache.current.value = 0;
    battery.cache.temperature.value = Thermistor::KELVIN_OFFSET_TENTHS + 250;

    g_i2c_mock_v_val = 15000; // Voltage jump
    battery.pollHardwareAndUpdateCache();

    // 3. Jump rejection via Temperature Delta alone
    battery.cache.valid = true;
    battery.cache.last_error = CommFault::NONE;
    battery.cache.timestamp_ms = virtual_uptime;
    battery.cache.voltage.value = 10000;
    battery.cache.current.value = 0;
    battery.cache.temperature.value = Thermistor::KELVIN_OFFSET_TENTHS + 250;

    g_i2c_mock_v_val = 8000;
    g_i2c_mock_i_val = 0;
    g_adc_mock_mv_val = 100; // Temperature jump

    battery.pollHardwareAndUpdateCache();
    EXPECT_GT(battery.consecutive_jump_rejects, 0);
}

TEST_F(SmartBatteryTestSuite, ProcessFSM_HysteresisTransitions) {
    // The FSM classifies direction from an EMA of the shunt voltage that persists between
    // calls, so every case first resets the EMA (voltage below MIN_VALID -> resetFsmClassifier)
    // and then feeds one large sample so the EMA equals that sample.
    auto run_case = [this](BatteryFSM start, int32_t shunt_uv) {
        battery.cache.valid = true;
        battery.cache.last_error = CommFault::NONE;
        battery.cache.timestamp_ms = virtual_uptime;
        battery.cache.soc.value = 50;

        battery.cache.voltage.value = 0;        // forces EMA reset
        battery.processFSM();

        battery.cache.voltage.value = 10000;
        battery.cache.shunt_voltage.value = shunt_uv;
        battery.current_state.store(start);
        battery.processFSM();
        return battery.getState();
    };
    constexpr int32_t BIG = 100000;

    EXPECT_EQ(run_case(BatteryFSM::CHARGING,     -BIG), BatteryFSM::CHARGING);
    EXPECT_EQ(run_case(BatteryFSM::CHARGING,      BIG), BatteryFSM::DISCHARGING);
    EXPECT_EQ(run_case(BatteryFSM::CHARGING,        0), BatteryFSM::IDLE);
    EXPECT_EQ(run_case(BatteryFSM::DISCHARGING,   BIG), BatteryFSM::DISCHARGING);
    EXPECT_EQ(run_case(BatteryFSM::DISCHARGING, -BIG), BatteryFSM::CHARGING);
    EXPECT_EQ(run_case(BatteryFSM::DISCHARGING,     0), BatteryFSM::IDLE);
}

TEST_F(SmartBatteryTestSuite, PublishError_MutexLockFailure_Threshold) {
    battery.current_state.store(BatteryFSM::IDLE);
    battery.consecutive_mutex_failures = battery.WATCHDOG_MUTEX_FAILURE_THRESHOLD - 1;

    g_mutex_lock_force_errno = -EAGAIN;
    battery.publishError(CommFault::MUTEX_TIMEOUT);
    g_mutex_lock_force_errno = 0;

    EXPECT_EQ(battery.getState(), BatteryFSM::CUTOFF);
}

TEST_F(SmartBatteryTestSuite, CoulombCounter_IntermediateRest) {
    battery.rest_period_start_ms = virtual_uptime;
    virtual_uptime += 1000; // 1 second elapsed (< REST_RESYNC_DURATION_MS)

    battery.seedOrResyncCoulombCounter(11000, 0, false);
    EXPECT_NE(battery.rest_period_start_ms, 0U);
}


// ===========================================================================
// Remaining coverage closure tests
// ===========================================================================

// Line 347: Kalman measurement noise, "active" (non-rest) branch
TEST_F(SmartBatteryTestSuite, UpdateStateAndPublish_ActiveShunt_UsesActiveNoise) {
    battery.soc_initialized = true;
    battery.last_poll_time_ms = virtual_uptime;
    battery.kf_soc_pct = 50.0f;

    battery.updateStateAndPublish(11000, 500, 250, 50000);
    EXPECT_TRUE(battery.cache.valid);

    battery.updateStateAndPublish(11000, -500, 250, -50000);
    EXPECT_TRUE(battery.cache.valid);
}

// Line 400: big voltage drop while connected, but new pack voltage is still >= PACK_MIN
TEST_F(SmartBatteryTestSuite, PollHardware_LargeDropButPackAboveMin_NotDisconnect) {
    battery.cache.valid = true;
    battery.cache.last_error = CommFault::NONE;
    battery.cache.timestamp_ms = virtual_uptime;
    battery.cache.voltage.value = 30000;
    battery.cache.current.value = 0;
    battery.cache.temperature.value = Thermistor::KELVIN_OFFSET_TENTHS + 250;

    g_i2c_mock_v_val = 8000;     // pack = 10000 mV
    g_i2c_mock_i_val = 0;
    g_adc_mock_mv_val = 1650;

    battery.pollHardwareAndUpdateCache();
    EXPECT_GT(battery.consecutive_jump_rejects, 0U);
}

// Lines 440-442: shunt register read fails after the bus-voltage read succeeded
TEST_F(SmartBatteryTestSuite, PollHardware_ShuntFetchTotalFailure) {
    extern void resetI2CCacheForTests();
    resetI2CCacheForTests();

    g_i2c_fail_after_reads = 1;
    battery.pollHardwareAndUpdateCache();
    g_i2c_fail_after_reads = -1;
    EXPECT_EQ(battery.cache.last_error, CommFault::I2C_NACK);
}

// Lines 453-456: |current| above MAX_VALID_CURRENT_MA.
// The largest value the INA226 current register can produce is 32768 * CURRENT_LSB_UA / 1000 mA.
// If MAX_VALID_CURRENT_MA is not below that, the check can never fire from any test, so the
// test reports that (SKIPPED, with the numbers) instead of failing. To cover lines 454-456,
// lower BatteryLimits::MAX_VALID_CURRENT_MA (or raise CURRENT_LSB_UA) so the limit is reachable.
TEST_F(SmartBatteryTestSuite, PollHardware_CurrentOutOfRange_ValidationError) {
    const int32_t lsb_ua = static_cast<int32_t>(INA226::CURRENT_LSB_UA);
    const int32_t max_reachable_ma = (32767 * lsb_ua) / 1000;
    const int32_t limit_ma = static_cast<int32_t>(BatteryLimits::MAX_VALID_CURRENT_MA);

    if (max_reachable_ma <= limit_ma) {
        GTEST_SKIP() << "MAX_VALID_CURRENT_MA (" << limit_ma << " mA) is not below the largest current the INA226 "
                     << "register can report (" << max_reachable_ma << " mA at " << lsb_ua
                     << " uA/LSB): the check at Smart_Battery_System.cpp:453 is unreachable.";
    }

    g_i2c_mock_v_val = 8000;
    g_adc_mock_mv_val = 1650;

    const uint16_t raw_extremes[] = {0x7FFF, 0x8000};
    for (uint16_t raw : raw_extremes) {
        battery.cache.valid = true;
        battery.cache.last_error = CommFault::NONE;
        battery.cache.timestamp_ms = virtual_uptime;
        g_i2c_mock_i_val = raw;

        battery.pollHardwareAndUpdateCache();
        EXPECT_EQ(battery.cache.last_error, CommFault::VALIDATION_ERROR) << "raw current 0x" << std::hex << raw;
    }
}

// Lines 696 / 700: helpers evaluated with a null smart_battery (monitor thread, sleeping)
TEST_F(SmartBatteryTestSuite, BatteryMonitorThread_NullInstance_WhileSleeping) {
    extern SbsBattery* smart_battery;
    auto backup = smart_battery;

    smart_battery = &battery;           // registers the power observer
    test_iterations_remaining = 0;
    bms_comm_thread();

    smart_battery = nullptr;
    PowerManager::getInstance().notifyBeforeSleep();

    std::thread waker([]() {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        PowerManager::getInstance().notifyAfterWakeup();
    });

    test_iterations_remaining = 0;
    battery_monitor_thread();
    waker.join();

    smart_battery = backup;
}

// Line 700: isBatteryInCutoff() evaluated with a null smart_battery (SAFE_HALT wait)
TEST_F(SmartBatteryTestSuite, BatteryMonitorThread_NullInstance_WhileSafeHalt) {
    extern SbsBattery* smart_battery;
    auto backup = smart_battery;

    smart_battery = &battery;
    test_iterations_remaining = 0;
    bms_comm_thread();                  // leaves the ready semaphore available

    smart_battery = nullptr;
    PowerManager::getInstance().notifyAfterWakeup();
    sys_context.requestTransition(SystemState::SAFE_HALT);

    std::thread waker([]() {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        sys_context.requestTransition(SystemState::INIT);
        sys_context.requestTransition(SystemState::RUNNING);
    });

    test_iterations_remaining = 0;
    battery_monitor_thread();
    waker.join();

    smart_battery = backup;
}

// Lines 804-807: comm-thread wait loop, all condition combinations.
// bms_comm_thread runs on a worker thread; the test thread changes the conditions while it
// is cycling (the thread cannot be hooked between init() and the loop from the mocks).
TEST_F(SmartBatteryTestSuite, BmsCommThread_MainLoopWaitConditions_ThreadDriven) {
    extern SbsBattery* smart_battery;
    auto backup = smart_battery;
    smart_battery = &battery;

    PowerManager::getInstance().notifyAfterWakeup();
    battery.current_state.store(BatteryFSM::IDLE);

    testing::internal::CaptureStdout();   // the worker logs on every cycle
    std::atomic<bool> finished{false};
    test_iterations_remaining = 1000000000;
    std::thread worker([&finished]() {
        bms_comm_thread();
        finished.store(true);
    });

    auto settle = []() { std::this_thread::sleep_for(std::chrono::milliseconds(25)); };

    settle();
    PowerManager::getInstance().notifyBeforeSleep();             // sleeping, not charging -> wait loop body
    settle();
    battery.current_state.store(BatteryFSM::CHARGING);           // sleeping + charging -> leaves loop
    settle();
    PowerManager::getInstance().notifyAfterWakeup();
    battery.current_state.store(BatteryFSM::IDLE);
    settle();
    sys_context.requestTransition(SystemState::SAFE_HALT);       // SAFE_HALT, not cutoff -> wait loop body
    settle();
    battery.current_state.store(BatteryFSM::CUTOFF);             // SAFE_HALT + cutoff -> leaves loop
    settle();
    sys_context.requestTransition(SystemState::INIT);
    sys_context.requestTransition(SystemState::RUNNING);
    settle();

    while (!finished.load()) {
        test_iterations_remaining = 0;
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    worker.join();
    testing::internal::GetCapturedStdout();

    smart_battery = backup;
}

// Line 782: SAFE_HALT already active when init retries are exhausted.
// SAFE_HALT is toggled by a second thread while bms_comm_thread repeatedly runs its init-failure path.
TEST_F(SmartBatteryTestSuite, BmsCommThread_InitEscalation_SafeHaltRaisedConcurrently) {
    extern SbsBattery* smart_battery;
    auto backup = smart_battery;
    smart_battery = &battery;

    PowerManager::getInstance().notifyAfterWakeup();
    g_i2c_force_errno = -EIO;

    testing::internal::CaptureStdout();
    std::atomic<bool> stop{false};
    std::thread toggler([&stop]() {
        while (!stop.load()) {
            sys_context.requestTransition(SystemState::SAFE_HALT);
            for (int i = 0; i < 20; ++i) std::this_thread::yield();
            sys_context.requestTransition(SystemState::INIT);
            sys_context.requestTransition(SystemState::RUNNING);
            for (int i = 0; i < 20; ++i) std::this_thread::yield();
        }
    });

    for (int run = 0; run < 300; ++run) {
        test_iterations_remaining = 0;
        bms_comm_thread();
    }

    stop.store(true);
    toggler.join();
    testing::internal::GetCapturedStdout();

    g_i2c_force_errno = 0;
    smart_battery = backup;
}

// Line 643: "active_state == CUTOFF" can only be true if another thread stores CUTOFF between
// the store at line 641 and the reload at line 642, so a racing thread is used.
// (Probabilistic: needs >= 2 cores. Alternative: change the source to test next_fsm_state.)
TEST_F(SmartBatteryTestSuite, ProcessFSM_ActiveStateReload_ConcurrentCutoff) {
    battery.cache.valid = true;
    battery.cache.last_error = CommFault::NONE;
    battery.cache.timestamp_ms = virtual_uptime;
    battery.cache.voltage.value = 10000;
    battery.cache.shunt_voltage.value = 0;
    battery.cache.soc.value = 50;
    battery.current_state.store(BatteryFSM::IDLE);

    testing::internal::CaptureStdout();
    std::atomic<bool> stop{false};
    std::thread racer([this, &stop]() {
        while (!stop.load(std::memory_order_relaxed)) {
            battery.current_state.store(BatteryFSM::CUTOFF);
            battery.current_state.store(BatteryFSM::IDLE);
        }
    });

    const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(1500);
    while (std::chrono::steady_clock::now() < deadline) {
        for (int i = 0; i < 1000; ++i) battery.processFSM();
    }

    stop.store(true, std::memory_order_relaxed);
    racer.join();
    testing::internal::GetCapturedStdout();
}
