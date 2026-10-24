#pragma once

#include <cstdint>

/* Definitions shared by every sensor module and by the command system. */

enum class SensorID : uint16_t {
    BME280       = 0x76,
    BME280_PRESS = 0x176,
    BME280_HUM   = 0x276,
    LPS22HB      = 0x5C,
    LPS22HB_TEMP = 0x15C,
    PAV3015      = 0x28
};

enum class ReadLength : uint8_t {
    Word   = 2,
    Triple = 3,
    Block  = 8
};

static_assert(sizeof(SensorID) == 2, "SensorID must be 16-bit");
static_assert(sizeof(ReadLength) == 1, "ReadLength must be 8-bit");

struct SensorDescriptor {
    SensorID id;
    uint8_t reg;
    ReadLength len;
};

namespace LogTags {
    constexpr char READ[]     = "READ";
    constexpr char COMPUTE[]  = "COMPUTE";
    constexpr char PRINT[]    = "PRINT";
    constexpr char PRODUCER[] = "PRODUCER";
}

/* Each sensor header re-opens these namespaces to add its own entries,
 * so existing code such as SensorReg::LPS_P_DESC keeps compiling. */
namespace SensorReg {}
namespace MockValues {
    constexpr uint16_t STEP = 17;
}
