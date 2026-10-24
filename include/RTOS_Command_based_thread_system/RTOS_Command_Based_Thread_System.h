#pragma once

/* Umbrella header: includes every part of the command-based thread system,
 * so existing code (and the unit tests) can keep including just this file.
 *
 *   RTOS_Config.h         queue / pool / thread tuning constants
 *   Sensor_Common.h       SensorID, ReadLength, descriptors, log tags
 *   BME280.h, LPS22HB.h,
 *   PAV3015.h             one module per sensor (math + device access)
 *   System_Objects.h      state machine / I2C / power access, sleep observer
 *   Command_Core.h        ICommand, memory pool, queues, enqueueCommand<>
 *   Sensor_Commands.h     SensorReadCmd, ComputeCmd
 *   Print_Commands.h      PrintCmd and friends
 *   Threads.h             producer / processor / logger thread declarations
 */

#include <zephyr/kernel.h>
#include <cstdint>
#include <utility>
#include "Fault_Tolerant_I2C_Communication_Layer.h"
#include "Device_State_Machine+Watchdog.h"
#include "Power_Management_System.h"

#include "Sensor_Common.h"
#include "BME280.h"
#include "LPS22HB.h"
#include "PAV3015.h"
#include "System_Objects.h"
#include "Command_Core.h"
#include "Sensor_Commands.h"
#include "Print_Commands.h"
#include "Threads.h"
