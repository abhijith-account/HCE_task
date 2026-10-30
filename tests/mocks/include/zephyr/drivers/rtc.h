#pragma once
#include <stdint.h>
#include <stddef.h>
struct rtc_time { int tm_sec; int tm_min; int tm_hour; int tm_mday; int tm_mon; int tm_year; int tm_wday; int tm_yday; int tm_isdst; int tm_nsec; };
#define RTC_ALARM_TIME_MASK_SECOND (1 << 0)
#define RTC_ALARM_TIME_MASK_MINUTE (1 << 1)
#define RTC_ALARM_TIME_MASK_HOUR   (1 << 2)
extern "C" {
int rtc_get_time(const struct device *dev, struct rtc_time *timeptr);
int rtc_set_time(const struct device *dev, const struct rtc_time *timeptr);
int rtc_alarm_set_time(const struct device *dev, uint16_t id, uint16_t mask, const struct rtc_time *timeptr);
int rtc_alarm_set_callback(const struct device *dev, uint16_t id, void (*callback)(const struct device *, uint16_t, void *), void *user_data);
}
