#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include "BME280.h"

LOG_MODULE_DECLARE(COMMANDS, LOG_LEVEL_INF);

namespace BMEConstants {
    constexpr int32_t T_FINE_OFFSET = 76800;
    constexpr int64_t P_OFFSET      = 128000;
    constexpr int64_t P_1048576     = 1048576;
    constexpr int64_t P_3125        = 3125;
    constexpr int32_t H_OFFSET      = 16384;
    constexpr int32_t H_32768       = 32768;
    constexpr int32_t H_2097152     = 2097152;
    constexpr int32_t H_8192        = 8192;
    constexpr int32_t H_MAX         = 419430400;

    constexpr float TEMP_DIV   = 100.0F;
    constexpr float PRESS_DIV1 = 256.0F;
    constexpr float PRESS_DIV2 = 100.0F;
    constexpr float HUM_DIV    = 1024.0F;
}

namespace {
    BME280Calibration g_bme280Calib;

    struct k_mutex calib_mutex;
    bool calib_mutex_initialized = false;

    void ensureCalibMutex() {
        if (!calib_mutex_initialized) {
            k_mutex_init(&calib_mutex);
            calib_mutex_initialized = true;
        }
    }

    [[nodiscard]] bool readTempCalibration(I2CManager& i2c, uint16_t addr) noexcept {
        auto t1 = i2c.readWord(addr, BME280CalibReg::DIG_T1); if (!t1.isOk()) { return false; }
        auto t2 = i2c.readWord(addr, BME280CalibReg::DIG_T2); if (!t2.isOk()) { return false; }
        auto t3 = i2c.readWord(addr, BME280CalibReg::DIG_T3); if (!t3.isOk()) { return false; }

        g_bme280Calib.dig_T1 = t1.unwrap();
        g_bme280Calib.dig_T2 = static_cast<int16_t>(t2.unwrap());
        g_bme280Calib.dig_T3 = static_cast<int16_t>(t3.unwrap());
        return true;
    }

    [[nodiscard]] bool readPressureCalibration(I2CManager& i2c, uint16_t addr) noexcept {
        auto p1 = i2c.readWord(addr, BME280CalibReg::DIG_P1); if (!p1.isOk()) { return false; }
        auto p2 = i2c.readWord(addr, BME280CalibReg::DIG_P2); if (!p2.isOk()) { return false; }
        auto p3 = i2c.readWord(addr, BME280CalibReg::DIG_P3); if (!p3.isOk()) { return false; }
        auto p4 = i2c.readWord(addr, BME280CalibReg::DIG_P4); if (!p4.isOk()) { return false; }
        auto p5 = i2c.readWord(addr, BME280CalibReg::DIG_P5); if (!p5.isOk()) { return false; }
        auto p6 = i2c.readWord(addr, BME280CalibReg::DIG_P6); if (!p6.isOk()) { return false; }
        auto p7 = i2c.readWord(addr, BME280CalibReg::DIG_P7); if (!p7.isOk()) { return false; }
        auto p8 = i2c.readWord(addr, BME280CalibReg::DIG_P8); if (!p8.isOk()) { return false; }
        auto p9 = i2c.readWord(addr, BME280CalibReg::DIG_P9); if (!p9.isOk()) { return false; }

        g_bme280Calib.dig_P1 = p1.unwrap();
        g_bme280Calib.dig_P2 = static_cast<int16_t>(p2.unwrap());
        g_bme280Calib.dig_P3 = static_cast<int16_t>(p3.unwrap());
        g_bme280Calib.dig_P4 = static_cast<int16_t>(p4.unwrap());
        g_bme280Calib.dig_P5 = static_cast<int16_t>(p5.unwrap());
        g_bme280Calib.dig_P6 = static_cast<int16_t>(p6.unwrap());
        g_bme280Calib.dig_P7 = static_cast<int16_t>(p7.unwrap());
        g_bme280Calib.dig_P8 = static_cast<int16_t>(p8.unwrap());
        g_bme280Calib.dig_P9 = static_cast<int16_t>(p9.unwrap());
        return true;
    }

    [[nodiscard]] bool readHumidityCalibration(I2CManager& i2c, uint16_t addr) noexcept {
        auto h1 = i2c.readRegister(addr, BME280CalibReg::DIG_H1); if (!h1.isOk()) { return false;}
        auto h2_l = i2c.readRegister(addr, BME280CalibReg::DIG_H2_L); if (!h2_l.isOk()) { return false;}
        auto h2_m = i2c.readRegister(addr, BME280CalibReg::DIG_H2_M); if (!h2_m.isOk()) { return false;}
        auto h3 = i2c.readRegister(addr, BME280CalibReg::DIG_H3); if (!h3.isOk()) { return false;}
        auto h4_m = i2c.readRegister(addr, BME280CalibReg::DIG_H4_M); if (!h4_m.isOk()) { return false;}
        auto h_shared = i2c.readRegister(addr, BME280CalibReg::DIG_H_SHARED); if (!h_shared.isOk()) { return false;}
        auto h5_m = i2c.readRegister(addr, BME280CalibReg::DIG_H5_M); if (!h5_m.isOk()) { return false;}
        auto h6 = i2c.readRegister(addr, BME280CalibReg::DIG_H6); if (!h6.isOk()) { return false;}

        const auto shared_val = h_shared.unwrap();
        g_bme280Calib.dig_H1 = h1.unwrap();
        g_bme280Calib.dig_H2 = static_cast<int16_t>((h2_m.unwrap() << 8) | h2_l.unwrap());
        g_bme280Calib.dig_H3 = h3.unwrap();
        g_bme280Calib.dig_H4 = static_cast<int16_t>((static_cast<int8_t>(h4_m.unwrap()) * 16) | (shared_val & 0x0F));
        g_bme280Calib.dig_H5 = static_cast<int16_t>((static_cast<int8_t>(h5_m.unwrap()) * 16) | (shared_val >> 4));
        g_bme280Calib.dig_H6 = static_cast<int8_t>(h6.unwrap());
        return true;
    }

    [[nodiscard]] bool loadBME280Calibration(I2CManager& i2c, uint16_t addr) noexcept {
        ensureCalibMutex();
        k_mutex_lock(&calib_mutex, K_FOREVER);
        if (g_bme280Calib.is_loaded) {
            k_mutex_unlock(&calib_mutex);
            return true;
        }

        if (!readTempCalibration(i2c, addr) || !readPressureCalibration(i2c, addr) || !readHumidityCalibration(i2c, addr)) {
            k_mutex_unlock(&calib_mutex);
            return false;
        }
        g_bme280Calib.is_loaded = true;
        k_mutex_unlock(&calib_mutex);
        LOG_INF("[%s] BME280 ROM Calibration Loaded Successfully.", LogTags::PRODUCER);
        return true;
    }
}

namespace BitExtractor {
    constexpr int32_t adcTemp(uint64_t raw) noexcept { return static_cast<int32_t>((raw >> 16) & 0xFFFFFF) >> 4; }
    constexpr int32_t adcPress(uint64_t raw) noexcept { return static_cast<int32_t>((raw >> 40) & 0xFFFFFF) >> 4; }
    constexpr int32_t adcHum(uint64_t raw) noexcept { return static_cast<int32_t>(raw & 0xFFFF); }
}

BME280Data BME280Math::decode(uint64_t raw_data, const BME280Calibration& c) noexcept {
    const int32_t adc_T = BitExtractor::adcTemp(raw_data);
    const int32_t adc_P = BitExtractor::adcPress(raw_data);
    const int32_t adc_H = BitExtractor::adcHum(raw_data);

    const int32_t var1 = ((((adc_T >> 3) - (static_cast<int32_t>(c.dig_T1) << 1))) * static_cast<int32_t>(c.dig_T2)) >> 11;
    const int32_t var2 = (((((adc_T >> 4) - static_cast<int32_t>(c.dig_T1)) * ((adc_T >> 4) - static_cast<int32_t>(c.dig_T1))) >> 12) * static_cast<int32_t>(c.dig_T3)) >> 14;
    const int32_t t_fine = var1 + var2;

    BME280Data result{};
    result.temperature = static_cast<float>((t_fine * 5 + 128) >> 8) / BMEConstants::TEMP_DIV;

    int64_t p_var1 = static_cast<int64_t>(t_fine) - BMEConstants::P_OFFSET;
    int64_t p_var2 = p_var1 * p_var1 * static_cast<int64_t>(c.dig_P6);
    p_var2 = p_var2 + ((p_var1 * static_cast<int64_t>(c.dig_P5)) << 17);
    p_var2 = p_var2 + (static_cast<int64_t>(c.dig_P4) << 35);
    p_var1 = ((p_var1 * p_var1 * static_cast<int64_t>(c.dig_P3)) >> 8) + ((p_var1 * static_cast<int64_t>(c.dig_P2)) << 12);
    p_var1 = (((static_cast<int64_t>(1) << 47) + p_var1)) * static_cast<int64_t>(c.dig_P1) >> 33;

    if (p_var1 != 0) {
        int64_t p = BMEConstants::P_1048576 - adc_P;
        p = (((p << 31) - p_var2) * BMEConstants::P_3125) / p_var1;
        p_var1 = (static_cast<int64_t>(c.dig_P9) * (p >> 13) * (p >> 13)) >> 25;
        p_var2 = (static_cast<int64_t>(c.dig_P8) * p) >> 19;
        p = ((p + p_var1 + p_var2) >> 8) + (static_cast<int64_t>(c.dig_P7) << 4);
        result.pressure = static_cast<float>(p) / BMEConstants::PRESS_DIV1 / BMEConstants::PRESS_DIV2;
    }

    int32_t v_x1_u32r = t_fine - BMEConstants::T_FINE_OFFSET;
    const int32_t hum_h4 = static_cast<int32_t>(c.dig_H4) << 20;
    const int32_t hum_h5 = static_cast<int32_t>(c.dig_H5) * v_x1_u32r;
    const int32_t h_term1 = (((adc_H << 14) - hum_h4 - hum_h5) + BMEConstants::H_OFFSET) >> 15;

    const int32_t hum_h6 = (v_x1_u32r * static_cast<int32_t>(c.dig_H6)) >> 10;
    const int32_t hum_h3 = (v_x1_u32r * static_cast<int32_t>(c.dig_H3)) >> 11;
    const int32_t h_term2 = (((hum_h6 * (hum_h3 + BMEConstants::H_32768)) >> 10) + BMEConstants::H_2097152);
    const int32_t h_term3 = (h_term2 * static_cast<int32_t>(c.dig_H2) + BMEConstants::H_8192) >> 14;

    v_x1_u32r = h_term1 * h_term3;
    const int32_t hum_h1 = static_cast<int32_t>(c.dig_H1);
    const int32_t h_term4 = ((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7;

    v_x1_u32r = v_x1_u32r - ((h_term4 * hum_h1) >> 4);
    v_x1_u32r = (v_x1_u32r < 0 ? 0 : v_x1_u32r);
    v_x1_u32r = (v_x1_u32r > BMEConstants::H_MAX ? BMEConstants::H_MAX : v_x1_u32r);
    result.humidity = static_cast<float>(v_x1_u32r >> 12) / BMEConstants::HUM_DIV;

    return result;
}

namespace BME280Driver {

bool initDevice(I2CManager& i2c) noexcept {
    const uint16_t addr = static_cast<uint16_t>(SensorID::BME280);
    auto res1 = i2c.writeRegister(addr, SensorReg::BME280_CTRL_HUM, BME280Config::CTRL_HUM);
    auto res2 = i2c.writeRegister(addr, SensorReg::BME280_CTRL_MEAS, BME280Config::CTRL_MEAS);
    return res1.isOk() && res2.isOk();
}

bool decodeRaw(I2CManager& i2c, uint64_t raw, BME280Data& out) noexcept {
    if (!loadBME280Calibration(i2c, static_cast<uint16_t>(SensorID::BME280))) {
        return false;
    }
    out = BME280Math::decode(raw, g_bme280Calib);
    return true;
}

void resetForTest() noexcept {
    g_bme280Calib = BME280Calibration{};
}

} // namespace BME280Driver
