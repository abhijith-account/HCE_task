#include <zephyr/kernel.h>
#include <zephyr/pm/pm.h>
#include <zephyr/pm/policy.h>
#include <zephyr/drivers/counter.h>
#include <zephyr/pm/device.h>
#include <zephyr/device.h>
#include <cstdint>
#pragma once

enum class PowerMode{ACTIVE,IDLE,STOP};

class PowerManager{
  private:
      PowerMode current_mode;
      uint32_t last_activity_time;
      
      const device* rtc_dev;
      const device* i2c_dev;
      
      static void rtc_alarm_handler(const device *dev,uint8_t chan_id,uint32_t ticks,void *user_data);
      
  public:
      PowerManager(const device* rtc,const device* i2c);
      
      bool init();
      
      void reportActivity();
      
      void processFSM();
      
      PowerMode getMode() const;
};
      
