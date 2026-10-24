#pragma once

#include <cstddef>
#include <cstdint>
#include <zephyr/drivers/adc.h>

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
static constexpr int32_t SUPPLY_MILLIVOLTS = 3300;

#ifndef CONFIG_BOARD_MPS2_AN386
extern const struct adc_dt_spec thermistor_adc_chan;
#endif

bool init();
Reading<int16_t> readCelsius();

} // namespace Thermistor

/* NTC voltage -> temperature curve. Kept in the CurveFitting namespace so
 * existing references keep compiling. */
namespace CurveFitting {

struct NtcPoint { int32_t mv; int16_t temp_tenths; };

static constexpr NtcPoint NTC_LUT[] = {
    { 3220, -400 }, { 3187, -350 }, { 3143, -300 }, { 3086, -250 },
    { 3014, -200 }, { 2925, -150 }, { 2816, -100 }, { 2689,  -50 },
    { 2543,    0 }, { 2381,   50 }, { 2206,  100 }, { 2023,  150 },
    { 1836,  200 }, { 1650,  250 }, { 1470,  300 }, { 1301,  350 },
    { 1143,  400 }, { 1000,  450 }, {  871,  500 }, {  757,  550 },
    {  657,  600 }, {  570,  650 }, {  494,  700 }, {  428,  750 },
    {  372,  800 }, {  323,  850 }, {  282,  900 }, {  246,  950 },
    {  215, 1000 }, {  189, 1050 }, {  166, 1100 }, {  146, 1150 },
    {  129, 1200 }, {  114, 1250 }
};

template <size_t N>
int16_t interpolateNtc(const NtcPoint (&lut)[N], int32_t mv) {
    if (mv >= lut[0].mv) { return lut[0].temp_tenths; }

    for (size_t i = 0; i < N - 1; ++i) {
        if (mv >= lut[i+1].mv) {
            int32_t v_range = lut[i].mv - lut[i+1].mv;
            int32_t t_range = lut[i+1].temp_tenths - lut[i].temp_tenths;
            int32_t v_offset = lut[i].mv - mv;
            return lut[i].temp_tenths + static_cast<int16_t>((v_offset * t_range) / v_range);
        }
    }
    return lut[N-1].temp_tenths;
}

} // namespace CurveFitting
