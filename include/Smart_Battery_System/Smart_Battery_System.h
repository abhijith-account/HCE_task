#pragma once

#include <cstdint>
#include <atomic>
#include <zephyr/kernel.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/drivers/adc.h>

#include "Device_State_Machine+Watchdog.h"
#include "Power_Management_System.h"
#include "Fault_Tolerant_I2C_Communication_Layer.h"
#include "Static_Memory+MISRA_Compliance_Layer.h"
#include "Thermistor.h"
#include "INA226_Driver.h"
#include "Kalman_Filter.h"
#include "Battery_Types.h"
#include "Battery_Config.h"

#ifdef INA226_TEST_BYPASS_STATIC_ASSERT
static_assert(INA226::MAX_MEASURABLE_CURRENT_UA >= (BatteryLimits::MAX_DISCHARGE_CURRENT_MA * 1000U),
    "INA226 shunt cannot measure the pack's rated discharge current -- "
    "see the HARDWARE MISMATCH comment on INA226::R_SHUNT_MILLIOHMS");
#endif

using WatchdogFeedHook = void (*)();

class SbsBattery {
public:
    SbsBattery(I2CManager* i2c_bus, DeviceContext* context, WatchdogFeedHook hook = nullptr);
    void setWatchdogFeedHook(WatchdogFeedHook hook);
    bool init();
    void pollHardwareAndUpdateCache();

    result<Millivolts> getVoltage() const;
    result<Milliamps> getCurrent() const;
    result<Microvolts> getShuntVoltage() const;
    result<Percent> getStateOfCharge() const;
    result<Kelvin> getTemperature() const;
    result<MilliAmpHours> getCapacity() const;

    void processFSM();
    BatteryFSM getState() const;
    CommStatistics getStats() const;
    void notifySystemWakeup();

private:
    INA226::Driver ina226;
    DeviceContext* sys_context;

    std::atomic<BatteryFSM> current_state;
    std::atomic<bool> full_charge_logged;
    std::atomic<WatchdogFeedHook> watchdog_feed_hook;

    mutable struct k_mutex cache_mutex;

    uint32_t last_valid_comm_time;
    uint32_t consecutive_comm_failures;
    uint32_t consecutive_mutex_failures;

    static constexpr uint32_t COMM_TIMEOUT_MS = 5000U;
    static constexpr uint32_t MAX_RETRIES = 3U;
    static constexpr uint32_t INITIAL_BACKOFF_MS = 20U;
    static constexpr uint32_t MAX_BACKOFF_MS = 160U;
    static constexpr uint32_t WATCHDOG_FAILURE_THRESHOLD = 2U;
    static constexpr uint32_t WATCHDOG_MUTEX_FAILURE_THRESHOLD = 5U;

    BmsCache cache;
    AtomicCommStatistics stats;

    bool soc_initialized;
    int64_t accumulated_uAh;
    uint32_t last_poll_time_ms;
    uint8_t consecutive_jump_rejects;
    uint32_t rest_period_start_ms;

    result<int16_t> fetchBusVoltageRawWithRetry();
    result<int16_t> fetchCurrentRawWithRetry();
    result<int16_t> fetchShuntVoltageRawWithRetry();
    result<int16_t> fetchI2cRegisterWithRetry(Result<int16_t> (INA226::Driver::*reader)());
    result<int16_t> fetchTemperatureTenthsWithRetry();

    uint8_t estimateSocFromVoltage(uint16_t pack_mv) const;

    void seedOrResyncCoulombCounter(uint16_t pack_mv, int32_t shunt_uv, bool force_seed);
    void updateStateAndPublish(uint16_t pack_mv, int32_t current_ma, int16_t temp_tenths, int32_t shunt_uv);

    void feedWatchdog() const;
    void publishError(CommFault fault);
    BmsCache getCacheSnapshot() const;
    
    float kf_soc_pct;
    float kf_p_covariance;
};

SbsBattery* getSmartBatteryInstance();
#ifndef IS_TEST_ENVIRONMENT
I2CManager* getI2cBusManagerInstance();
#endif
