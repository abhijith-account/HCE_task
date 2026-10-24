#include "Command_Core.h"
#include "Sensor_Commands.h"
#include "Print_Commands.h"

static_assert(sizeof(SensorReadCmd) <= sizeof(MaxCommandSize), "SensorReadCmd exceeds memory pool size");
static_assert(sizeof(ComputeCmd) <= sizeof(MaxCommandSize), "ComputeCmd exceeds memory pool size");
static_assert(sizeof(PrintCmd) <= sizeof(MaxCommandSize), "PrintCmd exceeds memory pool size");
static_assert(sizeof(PrintBME280Cmd) <= sizeof(MaxCommandSize), "PrintBME280Cmd exceeds memory pool size");
static_assert(sizeof(PrintLPS22HBCmd) <= sizeof(MaxCommandSize), "PrintLPS22HBCmd exceeds memory pool size");

static_assert(alignof(MaxCommandSize) >= alignof(PrintLPS22HBCmd), "Alignment mismatch on PrintLPS22HBCmd");
static_assert(alignof(MaxCommandSize) >= alignof(SensorReadCmd), "Alignment mismatch on SensorReadCmd");
static_assert(alignof(MaxCommandSize) >= alignof(ComputeCmd), "Alignment mismatch on ComputeCmd");
static_assert(alignof(MaxCommandSize) >= alignof(PrintCmd), "Alignment mismatch on PrintCmd");
static_assert(alignof(MaxCommandSize) >= alignof(PrintBME280Cmd), "Alignment mismatch on PrintBME280Cmd");
