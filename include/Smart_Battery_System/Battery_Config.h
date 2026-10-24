#pragma once

#include <cstddef>
#include <cstdint>

/* Pack-specific configuration: operating limits, filter tuning and the OCV curve.
 * Edit this file when the battery pack or tuning changes. */

struct BatteryLimits {
    static constexpr uint8_t CUTOFF_SOC_PCT = 10U;
    static constexpr uint8_t REENABLE_SOC_PCT = 15U;
    static constexpr uint32_t CACHE_STALE_MS = 3000U;
    static constexpr uint16_t PACK_MIN_VOLTAGE_MV = 8700U;
    static constexpr uint16_t PACK_MAX_VOLTAGE_MV = 12600U;
    static constexpr uint32_t NOMINAL_CAPACITY_MAH = 2000U;
    static constexpr uint32_t MAX_CHARGE_CURRENT_MA = 2000U;
    static constexpr uint32_t MAX_DISCHARGE_CURRENT_MA = 800U;
    static constexpr int32_t REST_CURRENT_THRESHOLD_MA = 50; 

    static constexpr int32_t IDLE_SHUNT_UV_THRESHOLD = 150;

    static constexpr uint32_t MUTEX_TIMEOUT_MS = 10U;
    static constexpr int BMS_THREAD_PRIO = 10;
    static constexpr int MONITOR_THREAD_PRIO = 11;

    static constexpr uint16_t MAX_VALID_VOLTAGE_MV = 20000U;
    static constexpr int32_t MAX_VALID_CURRENT_MA = 800;

    static constexpr uint16_t MAX_VOLTAGE_DELTA_MV = 2000U;
    static constexpr int32_t MAX_CURRENT_DELTA_MA = 8000;
    static constexpr int16_t MAX_TEMP_DELTA_TENTHS = 200;

    static constexpr uint8_t MAX_CONSECUTIVE_JUMP_REJECTS = 3U;

    static constexpr uint32_t MAX_INIT_RETRIES = 10U;

    static constexpr uint32_t REST_RESYNC_DURATION_MS = 30U * 60U * 1000U;
    static constexpr uint16_t MIN_VALID_VOLTAGE_MV = 4000U; 
    static constexpr int32_t ESTIMATED_PACK_IR_MILLIOHMS = 150; 
    static constexpr float KF_PROCESS_NOISE = 0.0001f;          
    static constexpr float KF_MEAS_NOISE_REST = 2.0f;           
    static constexpr float KF_MEAS_NOISE_ACTIVE = 50.0f;
};

/* Open-circuit-voltage -> state-of-charge curve for the 3S pack. */
namespace CurveFitting {
    struct OcvPoint { uint16_t mv{}; uint8_t soc_pct{}; };

    static constexpr OcvPoint OCV_LUT[] = {
        { 8700, 0 },    { 9600, 5 },    { 10200, 10 },
        { 10800, 25 },  { 11100, 40 },  { 11400, 60 },
        { 11700, 75 },  { 12000, 85 },  { 12300, 95 },
        { 12600, 100 }
    };

    template <size_t N>
    uint8_t interpolateOcv(const OcvPoint (&lut)[N], uint16_t mv) {
        if (mv <= lut[0].mv) { return lut[0].soc_pct;}

        for (size_t i = 0; i < N - 1; ++i) {
            if (mv <= lut[i+1].mv) {
                uint32_t v_range = lut[i+1].mv - lut[i].mv;
                uint32_t s_range = lut[i+1].soc_pct - lut[i].soc_pct;
                uint32_t v_offset = mv - lut[i].mv;
                return lut[i].soc_pct + static_cast<uint8_t>((v_offset * s_range) / v_range);
            }
        }
        return lut[N-1].soc_pct;
    }
}
