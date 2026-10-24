#pragma once

#include <cstdint>
#include "Sensor_Common.h"
#include "Fault_Tolerant_I2C_Communication_Layer.h"

namespace SensorReg {
    constexpr uint8_t BME280_CTRL_HUM   = 0xF2;
    constexpr uint8_t BME280_CTRL_MEAS  = 0xF4;
    constexpr uint8_t BME280_DATA_START = 0xF7;
    constexpr SensorDescriptor BME_DESC = { SensorID::BME280, 0xF7, ReadLength::Block };
}

namespace BME280CalibReg {
    constexpr uint8_t DIG_T1 = 0x88; constexpr uint8_t DIG_T2 = 0x8A; constexpr uint8_t DIG_T3 = 0x8C;
    constexpr uint8_t DIG_P1 = 0x8E; constexpr uint8_t DIG_P2 = 0x90; constexpr uint8_t DIG_P3 = 0x92;
    constexpr uint8_t DIG_P4 = 0x94; constexpr uint8_t DIG_P5 = 0x96; constexpr uint8_t DIG_P6 = 0x98;
    constexpr uint8_t DIG_P7 = 0x9A; constexpr uint8_t DIG_P8 = 0x9C; constexpr uint8_t DIG_P9 = 0x9E;
    constexpr uint8_t DIG_H1 = 0xA1; constexpr uint8_t DIG_H2_L = 0xE1; constexpr uint8_t DIG_H2_M = 0xE2;
    constexpr uint8_t DIG_H3 = 0xE3; constexpr uint8_t DIG_H4_M = 0xE4; constexpr uint8_t DIG_H_SHARED = 0xE5;
    constexpr uint8_t DIG_H5_M = 0xE6; constexpr uint8_t DIG_H6 = 0xE7;
}

namespace BME280Config {
    constexpr uint8_t CTRL_HUM = 0x01;
    constexpr uint8_t CTRL_MEAS = 0x27;
}

namespace MockValues {
    constexpr uint64_t BME_P_BASE = 512000ULL;
    constexpr uint64_t BME_T_BASE = 512000ULL;
    constexpr uint64_t BME_H_BASE = 25000ULL;
}

struct alignas(4) BME280Calibration {
    bool is_loaded = false;
    uint16_t dig_T1; int16_t dig_T2; int16_t dig_T3;
    uint16_t dig_P1; int16_t dig_P2; int16_t dig_P3;
    int16_t dig_P4;  int16_t dig_P5; int16_t dig_P6;
    int16_t dig_P7;  int16_t dig_P8; int16_t dig_P9;
    uint8_t  dig_H1; int16_t dig_H2; uint8_t  dig_H3;
    int16_t  dig_H4; int16_t dig_H5; int8_t   dig_H6;
};

struct BME280Data {
    float temperature;
    float pressure;
    float humidity;
};

/* Pure math: no I/O, no RTOS dependency. */
namespace BME280Math {
    [[nodiscard]] BME280Data decode(uint64_t raw_data, const BME280Calibration& calib) noexcept;
}

/* Device-level operations (I2C access, calibration cache). */
namespace BME280Driver {
    /* Raw-value change that counts as "activity" for the power manager. */
    constexpr uint64_t ActivityThreshold = 0x1000;

    /* Writes CTRL_HUM and CTRL_MEAS. Both writes are always attempted. */
    [[nodiscard]] bool initDevice(I2CManager& i2c) noexcept;

    /* Loads ROM calibration on first use (mutex protected), then decodes.
     * Returns false if calibration could not be read. */
    [[nodiscard]] bool decodeRaw(I2CManager& i2c, uint64_t raw, BME280Data& out) noexcept;

    void resetForTest() noexcept;
}
