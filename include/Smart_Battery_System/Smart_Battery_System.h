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

static_assert(BatteryLimits::MAX_CHARGE_CURRENT_MA <= static_cast<uint32_t>(BatteryLimits::MAX_VALID_CURRENT_MA),
    "Charge current exceeds software validation limit");
static_assert(BatteryLimits::MAX_DISCHARGE_CURRENT_MA <= static_cast<uint32_t>(BatteryLimits::MAX_VALID_CURRENT_MA),
    "Discharge current exceeds software validation limit");
static_assert(static_cast<uint64_t>(BatteryLimits::MAX_VALID_CURRENT_MA) * 1000ULL <= INA226::MAX_MEASURABLE_CURRENT_UA,
    "INA226 shunt measurement range is insufficient");

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
    DeviceContext* sys_context{nullptr};

    std::atomic<BatteryFSM> current_state{BatteryFSM::IDLE};
    std::atomic<bool> full_charge_logged{false};
    std::atomic<WatchdogFeedHook> watchdog_feed_hook{nullptr};

    mutable struct k_mutex cache_mutex{};

    uint32_t last_valid_comm_time{0U};
    uint32_t consecutive_comm_failures{0U};
    uint32_t consecutive_mutex_failures{0U};

    static constexpr uint32_t COMM_TIMEOUT_MS = 5000U;
    static constexpr uint32_t MAX_RETRIES = 3U;
    static constexpr uint32_t INITIAL_BACKOFF_MS = 20U;
    static constexpr uint32_t MAX_BACKOFF_MS = 160U;
    static constexpr uint32_t WATCHDOG_FAILURE_THRESHOLD = 2U;
    static constexpr uint32_t WATCHDOG_MUTEX_FAILURE_THRESHOLD = 5U;

    BmsCache cache{};
    std::atomic<bool> cache_requires_live_refresh{false};
    AtomicCommStatistics stats{};

    bool soc_initialized{false};
    int64_t accumulated_uAh{0};
    uint32_t last_poll_time_ms{0U};
    uint8_t consecutive_jump_rejects{0U};
    uint32_t rest_period_start_ms{0U};

    result<int16_t> fetchBusVoltageRawWithRetry();
    result<int16_t> fetchCurrentRawWithRetry();
    result<int16_t> fetchShuntVoltageRawWithRetry();
    result<int16_t> fetchI2cRegisterWithRetry(Result<int16_t> (INA226::Driver::*reader)());
    result<int16_t> fetchTemperatureTenthsWithRetry();

    uint8_t estimateSocFromVoltage(uint16_t pack_mv) const;

    void seedOrResyncCoulombCounter(uint16_t pack_mv, int32_t shunt_uv, bool force_seed);
    bool updateStateAndPublish(uint16_t pack_mv, int32_t current_ma, int16_t temp_tenths, int32_t shunt_uv,
                               bool degraded = false, uint8_t reliability_score = 100U,
                               int64_t sample_timestamp_ms = 0);

    void feedWatchdog() const;
    void publishError(CommFault fault);
    BmsCache getCacheSnapshot() const;

    float kf_soc_pct{0.0f};
    float kf_p_covariance{1.0f};
};

SbsBattery* getSmartBatteryInstance() noexcept;
#ifndef IS_TEST_ENVIRONMENT
I2CManager* getI2cBusManagerInstance();
#endif
