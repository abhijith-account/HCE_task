#pragma once
#include <stdint.h>
#define WDT_FLAG_RESET_SOC 1
#define WDT_OPT_PAUSE_HALTED_BY_DBG 1
#define WDT_OPT_PAUSE_IN_SLEEP 2
struct wdt_window { uint32_t min; uint32_t max; };
struct wdt_timeout_cfg { struct wdt_window window; void (*callback)(const struct device *, int); uint8_t flags; };
extern "C" {
int wdt_install_timeout(const struct device *dev, const struct wdt_timeout_cfg *cfg);
int wdt_setup(const struct device *dev, uint8_t options);
int wdt_feed(const struct device *dev, int channel_id);
}
