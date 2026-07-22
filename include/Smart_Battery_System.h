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

// -----------------------------------------------------------------------------
// Thermistor Sensor Module
// -----------------------------------------------------------------------------
namespace Thermistor {

enum class Fault : uint8_t { NONE, ADC_NOT_READY, ADC_READ_ERROR, OUT_OF_RANGE };

template <typename T>
struct Reading {
    T value{};
    Fault error{Fault::NONE};
    bool success{false};

    static Reading Ok(T v) { return {v, Fault::NONE, true}; }
    static Reading Err(Fault e) { return {T{}, e, false}; }
};

static constexpr int16_t MIN_VALID_CELSIUS = -40;
static constexpr int16_t MAX_VALID_CELSIUS = 125;
static constexpr int16_t KELVIN_OFFSET_TENTHS = 2731;
static constexpr uint8_t OVERSAMPLE_COUNT = 8U;
static constexpr int32_t SUPPLY_MILLIVOLTS = 3300; // 3.3V divider supply

// NTC_LUT (in the .cpp) assumes an NTC-to-GND divider: VCC -> R_FIXED(10K) ->
// ADC_NODE -> NTC -> GND. Voltage at the ADC node falls as temperature rises
// (NTC resistance drops). If your board wires the NTC on top instead
// (VCC -> NTC -> ADC_NODE -> R_FIXED -> GND), the voltage-vs-temperature
// relationship inverts and the table needs to be re-derived/reversed.

#ifndef CONFIG_BOARD_QEMU_CORTEX_M3
extern const struct adc_dt_spec thermistor_adc_chan;
#endif

bool init();
Reading<int16_t> readCelsius();

} // namespace Thermistor

// -----------------------------------------------------------------------------
// INA226 Driver Module
// -----------------------------------------------------------------------------
namespace INA226 {

static constexpr uint16_t I2C_ADDR = 0x40U;

static constexpr uint8_t REG_CONFIG      = 0x00U;
static constexpr uint8_t REG_SHUNT_VOLT  = 0x01U;
static constexpr uint8_t REG_BUS_VOLT    = 0x02U;
static constexpr uint8_t REG_POWER       = 0x03U;
static constexpr uint8_t REG_CURRENT     = 0x04U;
static constexpr uint8_t REG_CALIBRATION = 0x05U;

// AVG=16, VBUSCT=1.1ms, VSHCT=1.1ms, continuous shunt+bus mode (datasheet Table 7)
static constexpr uint16_t CONFIG_VALUE = 0x4527U;

// Bus voltage LSB is fixed at 1.25mV/bit. pack_mv in the .cpp approximates
// raw * 1.25 via the integer expression raw + raw/4 rather than a float.

// --- Calibration (integer/fixed-point, no floating point) ---
// R_SHUNT_MILLIOHMS is the resistor actually populated on the INA226 module
// (as given: 0.1 ohm = 100 milliohms).
//
// !! HARDWARE MISMATCH !! The INA226's shunt ADC input saturates at
// +/-81.92mV (2.5uV LSB x 32768 codes). With a 0.1 ohm shunt that caps the
// *measurable* current at 81.92mV / 0.1ohm = ~0.82A. This pack is rated for
// 6A continuous discharge (3C) and 2A charge (1C) -- both well past that
// limit, so current/SoC readings will clip and be wrong under any real load.
// CALIBRATION_VALUE/CURRENT_LSB_UA below are computed to be internally
// consistent with a 0.1 ohm shunt (so nothing silently overflows), but for
// the pack's real 6A range you need a smaller shunt -- e.g. 0.01 ohm (10
// milliohms) gives headroom to ~8.19A. If you swap the shunt, just update
// R_SHUNT_MILLIOHMS to match; everything else recomputes from it.
static constexpr uint32_t R_SHUNT_MILLIOHMS = 100U; // 0.1 ohm

// I_max[uA] = 81.92mV / R_shunt = 81,920,000 / R_SHUNT_MILLIOHMS
static constexpr uint32_t MAX_MEASURABLE_CURRENT_UA = 81920000U / R_SHUNT_MILLIOHMS;

// Current_LSB[uA] = I_max[uA] / 32768 codes
static constexpr uint32_t CURRENT_LSB_UA = MAX_MEASURABLE_CURRENT_UA / 32768U;

// Cal = 0.00512 / (Current_LSB[A] * R_SHUNT[ohm])
//     = 5,120,000 / (Current_LSB[uA] * R_SHUNT[milliohm])   (integer form)
static constexpr uint16_t CALIBRATION_VALUE =
    static_cast<uint16_t>(5120000ULL / (static_cast<uint64_t>(CURRENT_LSB_UA) * R_SHUNT_MILLIOHMS));

class Driver {
public:
    explicit Driver(I2CManager* i2c);
    bool init();
    Result<int16_t> readBusVoltageRaw();
    Result<int16_t> readCurrentRaw();

private:
    I2CManager* i2c;
};

} // namespace INA226

// -----------------------------------------------------------------------------
// Smart Battery System
// -----------------------------------------------------------------------------

enum class CommFault : uint8_t {
    NONE, I2C_NACK, I2C_TIMEOUT, I2C_BUS_BUSY, I2C_ARBITRATION_LOST,
    DEVICE_NOT_READY, THERMISTOR_FAULT, CACHE_INVALID, VALIDATION_ERROR,
    MUTEX_TIMEOUT // Added for Phase 3 bounded lock timeouts
};

enum class BatteryFSM : uint8_t { IDLE, CHARGING, DISCHARGING, CUTOFF };

template<typename T>
struct result {
    T value{};
    CommFault error{CommFault::NONE};
    bool success{false};

    static constexpr result Ok(const T& value) { return {value, CommFault::NONE, true}; }
    static constexpr result Err(CommFault error) { return {T{}, error, false}; }
};

struct Millivolts { uint16_t value = 0U; };
struct Milliamps { int32_t value = 0; }; // sign convention: positive = charging, negative = discharging (see processFSM())
struct Percent { uint8_t value = 0U; };
struct Kelvin { uint16_t value = 0U; };
struct MilliAmpHours { uint32_t value = 0U; };

struct CommStatistics {
    uint32_t reads = 0U;
    uint32_t i2c_faults = 0U;
    uint32_t thermistor_faults = 0U;
    uint32_t retries = 0U;
    uint32_t validation_errors = 0U;
    uint32_t successful_publishes = 0U; // full poll cycles that reached updateStateAndPublish() and committed
};

struct AtomicCommStatistics {
    atomic_t reads{};
    atomic_t i2c_faults{};
    atomic_t thermistor_faults{};
    atomic_t retries{};
    atomic_t validation_errors{};
    atomic_t successful_publishes{};
};

struct BatteryLimits {
    static constexpr uint8_t CUTOFF_SOC_PCT = 10U;
    static constexpr uint8_t REENABLE_SOC_PCT = 15U;
    static constexpr uint32_t CACHE_STALE_MS = 3000U;
    static constexpr uint16_t PACK_MIN_VOLTAGE_MV = 8700U;
    static constexpr uint16_t PACK_MAX_VOLTAGE_MV = 12600U;
    static constexpr uint32_t NOMINAL_CAPACITY_MAH = 2000U;
    static constexpr uint32_t MAX_CHARGE_CURRENT_MA = 800U;  
    static constexpr uint32_t MAX_DISCHARGE_CURRENT_MA = 800U;
    static constexpr int32_t REST_CURRENT_THRESHOLD_MA = 50;

    // Phase 3 Hard-Limits & Constants
    static constexpr uint32_t MUTEX_TIMEOUT_MS = 10U;
    static constexpr int BMS_THREAD_PRIO = 10;  // producer: bms_comm_thread, polls hardware
    static constexpr int MONITOR_THREAD_PRIO = 11; // consumer: battery_monitor_thread, runs the FSM

    static constexpr uint16_t MAX_VALID_VOLTAGE_MV = 20000U;
    static constexpr int32_t MAX_VALID_CURRENT_MA = 12000;

    // Jump detection constraints (per sample cycle)
    static constexpr uint16_t MAX_VOLTAGE_DELTA_MV = 2000U;
    static constexpr int32_t MAX_CURRENT_DELTA_MA = 8000;
    static constexpr int16_t MAX_TEMP_DELTA_TENTHS = 200; // 20.0C per 1s sample; catches ADC glitches

    // A single noisy sample shouldn't be enough to reject data and count
    // toward the comm-failure watchdog. Only escalate to publishError() once
    // this many consecutive samples fail the rate-of-change check.
    static constexpr uint8_t MAX_CONSECUTIVE_JUMP_REJECTS = 3U;

    // bms_comm_thread retries sensor init on failure; after this many
    // consecutive failures it escalates a fault instead of retrying forever
    // silently.
    static constexpr uint32_t MAX_INIT_RETRIES = 10U;
};

// Compile-time check: the INA226's shunt ADC must actually be able to measure
// the pack's rated discharge current, or every reading above the shunt's
// saturation point silently clips. This currently FAILS with the 0.1 ohm
// shunt documented in the INA226 namespace above (max measurable ~0.82A vs a
// 6A rating) -- that's intentional: it's surfacing a real hardware mismatch
// as a build error instead of letting it hide in a comment. Resolve it by
// either swapping to a smaller shunt (recommended, e.g. 0.01 ohm) or, if
// 0.82A is accepted as the actual operating ceiling, lowering
// MAX_DISCHARGE_CURRENT_MA above to match reality.
#ifndef INA226_TEST_BYPASS_STATIC_ASSERT
static_assert(INA226::MAX_MEASURABLE_CURRENT_UA >= (BatteryLimits::MAX_DISCHARGE_CURRENT_MA * 1000U),
    "INA226 shunt cannot measure the pack's rated discharge current -- "
    "see the HARDWARE MISMATCH comment on INA226::R_SHUNT_MILLIOHMS");
#endif

struct BmsCache {
    Millivolts voltage{};
    Milliamps current{};
    Percent soc{};
    Kelvin temperature{};
    MilliAmpHours capacity{};
    uint32_t timestamp_ms = 0U;
    CommFault last_error = CommFault::CACHE_INVALID;
    bool valid = false;
};

using WatchdogFeedHook = void (*)();

class SbsBattery {
public:
    SbsBattery(I2CManager* i2c_bus, DeviceContext* context, WatchdogFeedHook hook = nullptr);
    void setWatchdogFeedHook(WatchdogFeedHook hook);
    bool init();
    void pollHardwareAndUpdateCache();

    result<Millivolts> getVoltage() const;
    result<Milliamps> getCurrent() const;
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

    // Thread-safe lock-free state
    std::atomic<BatteryFSM> current_state;
    std::atomic<bool> full_charge_logged;
    std::atomic<WatchdogFeedHook> watchdog_feed_hook;

    mutable struct k_mutex cache_mutex;

    uint32_t last_valid_comm_time;
    uint32_t consecutive_comm_failures;

    static constexpr uint32_t COMM_TIMEOUT_MS = 5000U;
    static constexpr uint32_t MAX_RETRIES = 3U;
    static constexpr uint32_t INITIAL_BACKOFF_MS = 20U;
    static constexpr uint32_t MAX_BACKOFF_MS = 160U;
    static constexpr uint32_t WATCHDOG_FAILURE_THRESHOLD = 2U;

    BmsCache cache;
    AtomicCommStatistics stats;

    // Mutex-Protected Internal State Variables
    bool soc_initialized;
    int64_t accumulated_uAh;
    uint32_t last_poll_time_ms;
    uint8_t consecutive_jump_rejects;

    result<int16_t> fetchBusVoltageRawWithRetry();
    result<int16_t> fetchCurrentRawWithRetry();
    result<int16_t> fetchTemperatureTenthsWithRetry();

    uint8_t estimateSocFromVoltage(uint16_t pack_mv) const;

    // Core state managers
    void seedOrResyncCoulombCounter(uint16_t pack_mv, int32_t current_ma, bool force_seed);
    void updateStateAndPublish(uint16_t pack_mv, int32_t current_ma, int16_t temp_tenths);

    void feedWatchdog() const;
    void publishError(CommFault fault);
    BmsCache getCacheSnapshot() const;
};

SbsBattery* getSmartBatteryInstance();
#ifndef IS_TEST_ENVIRONMENT
I2CManager* getI2cBusManagerInstance();
#endif
