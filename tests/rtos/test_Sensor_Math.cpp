// test_Sensor_Math.cpp
// Pure decode-math tests for all three sensors: BME280Math::decode (BME280.cpp), LPS22HBMath (LPS22HB.cpp) and PAV3015Math (PAV3015.cpp).

#include "test_rtos_common.h"

TEST(BME280MathTest, PressureBranchNonZeroDirect) {
    BME280Calibration c{};
    c.dig_P1 = 1;
    auto d = BME280Math::decode(0, c);
    EXPECT_NE(d.pressure, 0.0f);
}

TEST(BME280MathTest, ZeroPressureBranch) {
    BME280Calibration c{};
    c.dig_P1 = 0;
    auto d = BME280Math::decode(0, c);
    EXPECT_FLOAT_EQ(d.pressure, 0.0f);
}

TEST(BME280MathTest, HumidityClampLow) {
    BME280Calibration c{};
    c.dig_H1 = 1;
    auto d = BME280Math::decode(0, c);
    EXPECT_GE(d.humidity, 0);
}

TEST(BME280MathTest, HumidityClampHigh) {
    BME280Calibration c{};

    c.dig_H2 = 300;
    auto d = BME280Math::decode(0x000000000000FFFFULL, c);
    EXPECT_FLOAT_EQ(d.humidity, 100.0f);
}

TEST(MathCoverage, NegativeTemperature) {
    EXPECT_LT(LPS22HBMath::decodeTemperature(0xFFFF), 0);
}

TEST(MathCoverage, NegativePressure) {
    EXPECT_LT(LPS22HBMath::decodePressure(0xFFFFFF), 0);
}

TEST(MathCoverage, ZeroAirflow) {
    EXPECT_GE(PAV3015Math::decodeAirflow(0), 0);
}

TEST(BME280MathTest, PressureDivideByZero) {
    BME280Calibration c{};
    c.dig_P1 = 0;
    auto d = BME280Math::decode(0, c);
    EXPECT_FLOAT_EQ(d.pressure, 0.0f);
}

TEST(MathCoverage, LPS22HBAccumulateDegradedAndReliabilityBranches) {
    LPS22HBData out{};

    // Case 1: degraded pressure (60) + healthy temperature (100)
    // Covers lps_data.degraded == true and (60 < 100) == true
    EXPECT_FALSE(LPS22HBDriver::accumulate(SensorReg::LPS_P_DESC.reg, 0x3F0000ULL, out, true, 60U));
    ASSERT_TRUE(LPS22HBDriver::accumulate(SensorReg::LPS_T_DESC.reg, 0x09C4ULL, out, false, 100U));
    EXPECT_TRUE(out.degraded);
    EXPECT_EQ(out.reliability_score, 60U);

    // Case 2: healthy pressure (100) + degraded temperature (70)
    // Covers lps_data.degraded == false, degraded == true, and (100 < 70) == false
    EXPECT_FALSE(LPS22HBDriver::accumulate(SensorReg::LPS_P_DESC.reg, 0x3F0000ULL, out, false, 100U));
    ASSERT_TRUE(LPS22HBDriver::accumulate(SensorReg::LPS_T_DESC.reg, 0x09C4ULL, out, true, 70U));
    EXPECT_TRUE(out.degraded);
    EXPECT_EQ(out.reliability_score, 70U);
}
