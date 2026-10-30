#pragma once

#include <stdint.h>

#include <stddef.h>


struct device;


struct adc_dt_spec {

    const struct device *dev;

    uint8_t channel_id;

    uint8_t resolution;

};


#define ADC_DT_SPEC_GET_BY_IDX(node, idx) { .dev = nullptr, .channel_id = (idx), .resolution = 12 }


#ifdef __cplusplus

extern "C" {

#endif


extern bool g_adc_ready_mock;

static inline int adc_is_ready_dt(const struct adc_dt_spec *spec) { return g_adc_ready_mock ? 1 : 0; }

static inline int adc_channel_setup_dt(const struct adc_dt_spec *spec) { return 0; }


struct adc_sequence {

    uint32_t channels;

    void *buffer;

    size_t buffer_size;

    uint8_t resolution;

};

extern int g_adc_sequence_init_dt_errno;

__attribute__((weak)) int adc_sequence_init_dt(const struct adc_dt_spec *spec, struct adc_sequence *seq) {
    return g_adc_sequence_init_dt_errno;
}

int adc_read(const struct device *dev, const struct adc_sequence *sequence);

int adc_raw_to_millivolts_dt(const struct adc_dt_spec *spec, int32_t *val);


#ifdef __cplusplus

}

#endif

