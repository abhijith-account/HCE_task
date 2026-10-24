#include "LPS22HB.h"

namespace LPS22HBConst {
    constexpr float TEMP_DIV  = 100.0F;
    constexpr float PRESS_DIV = 4096.0F;
}

float LPS22HBMath::decodeTemperature(uint64_t raw_data) noexcept {
    const uint8_t byte2 = (raw_data >> 8) & 0xFF;
    const uint8_t byte3 = raw_data & 0xFF;
    return static_cast<float>(static_cast<int16_t>((byte2 << 8) | byte3)) / LPS22HBConst::TEMP_DIV;
}

float LPS22HBMath::decodePressure(uint64_t raw_data) noexcept {
    const uint8_t byte1 = (raw_data >> 16) & 0xFF;
    const uint8_t byte2 = (raw_data >> 8) & 0xFF;
    const uint8_t byte3 = raw_data & 0xFF;
    int32_t press_raw = static_cast<int32_t>((byte3 << 16) | (byte2 << 8) | byte1);
    if (press_raw & 0x00800000) press_raw |= 0xFF000000;
    return static_cast<float>(press_raw) / LPS22HBConst::PRESS_DIV;
}

namespace LPS22HBDriver {

bool initDevice(I2CManager& i2c) noexcept {
    const uint16_t addr = static_cast<uint16_t>(SensorID::LPS22HB);
    auto res = i2c.writeRegister(addr, LPS22HBConfig::CTRL_REG1, LPS22HBConfig::CTRL_REG1_VALUE);
    return res.isOk();
}

bool accumulate(uint8_t reg, uint64_t raw, LPS22HBData& out) noexcept {
    static LPS22HBData lps_data{};
    static bool has_pressure = false;

    if (reg == SensorReg::LPS_P_DESC.reg) {
        lps_data.pressure = LPS22HBMath::decodePressure(raw);
        has_pressure = true;
    } else if (reg == SensorReg::LPS_T_DESC.reg) {
        lps_data.temperature = LPS22HBMath::decodeTemperature(raw);
        if (has_pressure) {
            out = lps_data;
            has_pressure = false;
            return true;
        }
    }
    return false;
}

} // namespace LPS22HBDriver
