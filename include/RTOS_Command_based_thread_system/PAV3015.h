#pragma once

#include <cstdint>
#include "Sensor_Common.h"
#include "Fault_Tolerant_I2C_Communication_Layer.h"

namespace SensorReg {
    constexpr SensorDescriptor PAV_DESC = { SensorID::PAV3015, 0x00, ReadLength::Word };
}

namespace MockValues {
    constexpr uint64_t PAV_BASE = 409ULL;
}

/* Pure math: no I/O, no RTOS dependency. */
namespace PAV3015Math {
    [[nodiscard]] float decodeAirflow(uint64_t raw) noexcept;
}

namespace PAV3015Driver {
    constexpr uint64_t ActivityThreshold = 0x05;

    /* The PAV3015 has no config register; init is a probe read of register 0. */
    [[nodiscard]] bool initDevice(I2CManager& i2c) noexcept;

#ifdef CONFIG_BOARD_MPS2_AN386
    /* Simulated airflow for the emulated board (no real sensor attached). */
    [[nodiscard]] uint64_t readMock() noexcept;
#endif
}
