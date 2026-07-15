#include <zephyr/kernel.h>
#include <array>
#include <cstdint>
#pragma once

class ZephyrSemaphore {
  private:
      k_sem sem;
  public:
    ZephyrSemaphore(unsigned int initial,unsigned int limit);
    void give();
    int take(k_timeout_t timeout);
};

class ZephyrMutex{
  private:
      k_mutex mutex;
  public:
      ZephyrMutex();
      void lock();
      void unlock();
};

class ZephyrWorkQueue {
  private:
      k_work_delayable work;
      void (*callback)();
      static void execute_callback(struct k_work *w);
  public:
      explicit ZephyrWorkQueue(void (*cb)());
      void schedule(k_timeout_t delay);   
};

struct SharedHeartRateBuffer{
      std::array<uint32_t,10> data;
      size_t head=0;
      size_t tail=0;
      ZephyrMutex mutex;
};
extern SharedHeartRateBuffer hr_buffer;
extern ZephyrSemaphore display_sem;      
