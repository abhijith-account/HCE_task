#include "Thermistor.h"

#include <zephyr/device.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(THERMISTOR, LOG_LEVEL_INF);

namespace Thermistor {

const struct adc_dt_spec thermistor_adc_chan = ADC_DT_SPEC_GET_BY_IDX(DT_NODELABEL(zephyr_user), 0);

bool init() {
    if (!adc_is_ready_dt(&thermistor_adc_chan)) {
        LOG_ERR("Thermistor ADC channel not ready");
        return false;
    }
    return adc_channel_setup_dt(&thermistor_adc_chan) == 0;
}

Reading<int16_t> readCelsius() {
    if (!adc_is_ready_dt(&thermistor_adc_chan)) return Reading<int16_t>::Err(Fault::ADC_NOT_READY);

    int32_t sum_mv = 0;
    for (uint8_t i = 0; i < OVERSAMPLE_COUNT; ++i) {
        int16_t raw = 0;
        struct adc_sequence sequence{};
        sequence.buffer = &raw;
        sequence.buffer_size = sizeof(raw);

        if (adc_sequence_init_dt(&thermistor_adc_chan, &sequence) != 0 ||
            adc_read(thermistor_adc_chan.dev, &sequence) != 0) {
            return Reading<int16_t>::Err(Fault::ADC_READ_ERROR);
        }

        int32_t mv = raw;
        if (adc_raw_to_millivolts_dt(&thermistor_adc_chan, &mv) != 0) return Reading<int16_t>::Err(Fault::ADC_READ_ERROR);
        sum_mv += mv;
    }

    int32_t avg_mv = sum_mv / static_cast<int32_t>(OVERSAMPLE_COUNT);

    if ((avg_mv <= 0) || (avg_mv >= SUPPLY_MILLIVOLTS)) return Reading<int16_t>::Err(Fault::OUT_OF_RANGE);

    const int16_t celsius_tenths = CurveFitting::interpolateNtc(CurveFitting::NTC_LUT, avg_mv);

    return Reading<int16_t>::Ok(celsius_tenths);
}

} // namespace Thermistor
