// Power_RTC.cpp
// RTC alarm helpers (advance_rtc_time, set_rtc_alarm, clear_rtc_alarm, ...).
// These are the free functions declared in Power_Management_System.h. Real hardware only.

#include "Power_Internal.h"

#ifndef CONFIG_BOARD_MPS2_AN386

namespace pm_internal {
uint32_t s_last_alraf_cycles = 0;
uint32_t s_last_alraf_timeout = 0;
}
using namespace pm_internal;

namespace {
// The HAL RTC macros only need a handle carrying the register-block pointer.
RTC_HandleTypeDef s_rtc_hal = []() { RTC_HandleTypeDef h{}; h.Instance = RTC; return h; }();
uint32_t s_last_alarm_interval_us = 0;
uint32_t s_last_alarm_seconds = 0;
int s_last_alarm_pending = 0;
struct rtc_time s_last_rtc_now{};
struct rtc_time s_last_rtc_alarm{};
}

void advance_rtc_time(struct rtc_time& time, uint32_t seconds_to_add)
{
    uint32_t total_seconds =
        static_cast<uint32_t>(time.tm_hour) * 3600U +
        static_cast<uint32_t>(time.tm_min) * 60U +
        static_cast<uint32_t>(time.tm_sec) +
        seconds_to_add;

    time.tm_hour = (total_seconds / 3600U) % 24U;
    time.tm_min  = (total_seconds / 60U) % 60U;
    time.tm_sec  = total_seconds % 60U;

    if (total_seconds >= 24U * 3600U) {
        time.tm_mday += total_seconds / (24U * 3600U);
    }
}

bool clear_rtc_alarm_a_pending() {
    uint32_t prev_demcr = CoreDebug->DEMCR;
    uint32_t prev_dwt   = DWT->CTRL;

    __HAL_RTC_WRITEPROTECTION_DISABLE(&s_rtc_hal);
    __HAL_RTC_ALARM_CLEAR_FLAG(&s_rtc_hal, RTC_FLAG_ALRAF);   // clear RTC Alarm A flag

    // Temporarily enable DWT Cycle Counter hardware
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    uint32_t start_cycles = DWT->CYCCNT;
    uint32_t timeout = 100000;

    while (__HAL_RTC_ALARM_GET_FLAG(&s_rtc_hal, RTC_FLAG_ALRAF) && --timeout) {
        // wait
    }

    uint32_t end_cycles = DWT->CYCCNT;

    // Store silently instead of logging
    s_last_alraf_cycles = (end_cycles - start_cycles);
    s_last_alraf_timeout = timeout;

    // Restore DWT and CoreDebug so TRCENA is not left enabled during STOP!
    DWT->CTRL = prev_dwt;
    CoreDebug->DEMCR = prev_demcr;

    __HAL_RTC_WRITEPROTECTION_ENABLE(&s_rtc_hal);   // re-lock
    __HAL_RTC_ALARM_EXTI_CLEAR_FLAG();              // EXTI line 17, write-1-to-clear
    NVIC_ClearPendingIRQ(RTC_Alarm_IRQn);   // CMSIS inline; HAL_NVIC_* is not linked in this build
    return timeout != 0;
}

int set_rtc_alarm(const struct device* rtc_dev, uint32_t interval_us, void* user_data)
{
    if (!rtc_dev) {
        return -ENODEV;
    }

    int ret = rtc_alarm_set_time(rtc_dev, 0, 0, nullptr);
    if (ret != 0) {
        return ret;
    }

    rtc_alarm_set_callback(rtc_dev, 0, nullptr, nullptr);

    struct rtc_time now;
    if (rtc_get_time(rtc_dev, &now) != 0) {
        return -EIO;
    }

    uint32_t seconds_to_add = (interval_us + 999999U) / 1000000U;
    if (seconds_to_add == 0) {
        seconds_to_add = 1;
    }

    struct rtc_time alarm = now;
    advance_rtc_time(alarm, seconds_to_add);

    s_last_alarm_interval_us = interval_us;
    s_last_alarm_seconds = seconds_to_add;
    s_last_rtc_now = now;
    s_last_rtc_alarm = alarm;

    rtc_alarm_set_callback(
        rtc_dev,
        0,
        PowerManager::rtc_alarm_handler,
        user_data
    );

    uint16_t mask =
        RTC_ALARM_TIME_MASK_SECOND |
        RTC_ALARM_TIME_MASK_MINUTE |
        RTC_ALARM_TIME_MASK_HOUR;

    if (!clear_rtc_alarm_a_pending()) {
        return -EIO;
    }

    ret = rtc_alarm_set_time(
        rtc_dev,
        0,
        mask,
        &alarm
    );

    if (ret == 0) {
        s_last_alarm_pending = 0;
    }

    return ret;
}

void clear_rtc_alarm(const struct device* rtc_dev) {
    if (!rtc_dev) return;
    rtc_alarm_set_callback(rtc_dev, 0, nullptr, nullptr);
    rtc_alarm_set_time(rtc_dev, 0, 0, nullptr);
}

#endif // !CONFIG_BOARD_MPS2_AN386
