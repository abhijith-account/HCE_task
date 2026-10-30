#pragma once
#include <stdint.h>
struct counter_alarm_cfg { uint32_t flags; uint32_t ticks; void (*callback)(const struct device *, uint8_t, uint32_t, void *); void *user_data; };
extern "C" {
int counter_start(const struct device *dev);
int counter_cancel_channel_alarm(const struct device *dev, uint8_t chan_id);
uint32_t counter_us_to_ticks(const struct device *dev, uint64_t us);
int counter_set_channel_alarm(const struct device *dev, uint8_t chan_id, const struct counter_alarm_cfg *alarm_cfg);
}
