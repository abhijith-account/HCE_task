#pragma once

#include <cstdint>
#include "Sensor_Common.h"
#include "Fault_Tolerant_I2C_Communication_Layer.h"

namespace SensorReg {
    constexpr SensorDescriptor LPS_P_DESC = { SensorID::LPS22HB, 0x28, ReadLength::Triple };
    constexpr SensorDescriptor LPS_T_DESC = { SensorID::LPS22HB, 0x2B, ReadLength::Word };
}

namespace LPS22HBConfig {
    constexpr uint8_t CTRL_REG1       = 0x10;
    constexpr uint8_t CTRL_REG1_VALUE = 0x10;
}

namespace MockValues {
    constexpr uint64_t LPS_P_BASE = 4000000ULL;
    constexpr uint64_t LPS_T_BASE = 2500ULL;
}

struct LPS22HBData {
    float temperature;
    float pressure;
};

/* Pure math: no I/O, no RTOS dependency. */
namespace LPS22HBMath {
    [[nodiscard]] float decodeTemperature(uint64_t raw) noexcept;
    [[nodiscard]] float decodePressure(uint64_t raw) noexcept;
}

namespace LPS22HBDriver {
    constexpr uint64_t ActivityThresholdPressure    = 0x50;
    constexpr uint64_t ActivityThresholdTemperature = 0x10;

    [[nodiscard]] bool initDevice(I2CManager& i2c) noexcept;

    /* Pressure and temperature arrive as two separate reads. Feed each raw
     * value in; returns true (and fills `out`) once a pressure followed by a
     * temperature has been received. */
    [[nodiscard]] bool accumulate(uint8_t reg, uint64_t raw, LPS22HBData& out) noexcept;
}
