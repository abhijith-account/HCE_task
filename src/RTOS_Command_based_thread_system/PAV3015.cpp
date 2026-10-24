#include "PAV3015.h"

namespace PAV3015Const {
    constexpr float OFFSET = 0.1F;
    constexpr float LINEAR = 0.0000506F;
    constexpr float QUADRATIC = 0.000001F;
}

float PAV3015Math::decodeAirflow(uint64_t raw_data) noexcept {
    const uint8_t byte2 = (raw_data >> 8) & 0xFF;
    const uint8_t byte3 = raw_data & 0xFF;
    const float x = static_cast<float>(static_cast<uint16_t>((byte2 << 8) | byte3));
    float flow = PAV3015Const::OFFSET + (PAV3015Const::LINEAR * x) + (PAV3015Const::QUADRATIC * x * x);
    return flow ;
}

namespace PAV3015Driver {

bool initDevice(I2CManager& i2c) noexcept {
    const uint16_t addr = static_cast<uint16_t>(SensorID::PAV3015);
    auto res = i2c.readWord(addr, SensorReg::PAV_DESC.reg);
    return res.isOk();
}

#ifdef CONFIG_BOARD_MPS2_AN386
uint64_t readMock() noexcept {
    static int32_t mock_pav = MockValues::PAV_BASE;
    static bool increasing = true;

    if (increasing) {
        mock_pav += 1;
        if (mock_pav > static_cast<int32_t>(MockValues::PAV_BASE) + 50) { 
            increasing = false;
        }
    } else {
        mock_pav -= 1;
        if (mock_pav < static_cast<int32_t>(MockValues::PAV_BASE) - 50) {
            increasing = true;
        }
    }
    return mock_pav;
}
#endif


} // namespace PAV3015Driver
