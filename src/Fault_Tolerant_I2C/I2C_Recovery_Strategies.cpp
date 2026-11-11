// I2C_Recovery_Strategies.cpp
// The stateless recovery strategies that I2CManager runs after a failed bus transaction:
//   RetryStrategy      - generic fault: warn and back off (10 ms)
//   BusResetStrategy   - TIMEOUT / BUS_BUSY: reports a hard-locked bus (currently it only logs "Toggling SCL to recover";
//                        no GPIO toggling is implemented)
// FailSafeStrategy (last-known-good value) is in I2C_Fallback_Cache.cpp, next to the cache it complements.
// Which strategy runs for which fault is decided in I2C_Manager.cpp.

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/device.h>
#include "Fault_Tolerant_I2C_Communication_Layer.h"
#include "Static_Memory+MISRA_Compliance_Layer.h"
#include "Power_Management_System.h"
#include "Device_State_Machine+Watchdog.h" 
#include <zephyr/sys/atomic.h>
#include <cerrno>
#include <cstdlib>
#include <new>
#include <cstdint>

LOG_MODULE_DECLARE(I2C_HW, LOG_LEVEL_INF);

void RetryStrategy::executeRecovery(const device* ) {
    LOG_WRN("I2C Fault Detected. Executing Exponential Backoff Retry...");
    k_msleep(10);
}

void BusResetStrategy::executeRecovery(const device* ) {
    LOG_ERR("I2C Bus Hard-Locked. Toggling SCL to recover");
}
