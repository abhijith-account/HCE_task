#pragma once
#include <cstdint>
#include <zephyr/sys/atomic.h>

enum class CommFault : uint8_t {NONE,I2C_NACK,I2C_TIMEOUT,I2C_BUS_BUSY,I2C_ARBITRATION_LOST,DEVICE_NOT_READY,THERMISTOR_FAULT,CACHE_INVALID,VALIDATION_ERROR,MUTEX_TIMEOUT};

enum class BatteryFSM : uint8_t {IDLE,CHARGING,DISCHARGING,CUTOFF};

template <typename T>
struct result {
    T value{};
    CommFault error{CommFault::NONE};
    bool success{false};
    
    static constexpr result Ok(const T& value){
      return {value,CommFault::NONE,true};
    }
    
    static constexpr result Err(CommFault error){
      return {T{},error,false};
    }
};

struct Millivolts {
  uint16_t value=0U;
};

struct Milliamps {
  int32_t value=0;
};

struct Microvolts {
  int32_t value=0; 
};

struct Percent {
  uint8_t value=0U;
};

struct Kelvin {
  uint16_t value=0U;
};

struct MilliAmpHours {
  uint32_t value=0U;
};

struct CommStatistics {
  uint32_t reads=0U;
  uint32_t i2c_faults=0U;
  uint32_t thermistor_faults=0U;
  uint32_t retries=0U;
  uint32_t validation_errors=0U;
  uint32_t successful_publishes=0U; 
};

struct AtomicCommStatistics {
  atomic_t reads{};
  atomic_t i2c_faults{};
  atomic_t thermistor_faults{};
  atomic_t retries{};
  atomic_t validation_errors{};
  atomic_t successful_publishes{};
};

struct BmsCache{
  Millivolts voltage{};
  Milliamps current{};
  Microvolts shunt_voltage{};
  Percent soc{};
  Kelvin temperature{};
  MilliAmpHours capacity{};
  uint32_t timestamp_ms=0U;
  CommFault last_error= CommFault::CACHE_INVALID;
  bool valid=false;
};
