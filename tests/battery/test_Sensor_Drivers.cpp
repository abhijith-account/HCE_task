// test_Sensor_Drivers.cpp
// Tests for the sensor-driver layer: INA226_Driver.cpp (Driver::init failures), Thermistor.cpp (ADC init/read failures,
// range checks) and the CurveFitting interpolation templates (OCV curve from Battery_Config.h, NTC curve from Thermistor.h).

#include "test_battery_common.h"

TEST_F(SmartBatteryTestSuite, CurveFitting_LUT_Edges) {
    EXPECT_EQ(battery.estimateSocFromVoltage(8700), 0);
    EXPECT_EQ(battery.estimateSocFromVoltage(8600), 0);
    EXPECT_EQ(battery.estimateSocFromVoltage(12600), 100);
    EXPECT_EQ(battery.estimateSocFromVoltage(13000), 100);
    EXPECT_EQ(battery.estimateSocFromVoltage(9000), 1);

    g_adc_mock_mv_val = 3300;
    EXPECT_EQ(Thermistor::readCelsius().error, Thermistor::Fault::OUT_OF_RANGE);
    g_adc_mock_mv_val = 3220;
    EXPECT_EQ(Thermistor::readCelsius().value, -400);
    g_adc_mock_mv_val = 3250;
    EXPECT_EQ(Thermistor::readCelsius().value, -400);
    g_adc_mock_mv_val = 114;
    EXPECT_EQ(Thermistor::readCelsius().value, 1250);
    g_adc_mock_mv_val = 100;
    EXPECT_EQ(Thermistor::readCelsius().value, 1250);
}

TEST_F(SmartBatteryTestSuite, Thermistor_InitAndReadFailures) {
    g_adc_ready_mock = false;
    EXPECT_FALSE(Thermistor::init());
    EXPECT_EQ(Thermistor::readCelsius().error, Thermistor::Fault::ADC_NOT_READY);

    g_adc_ready_mock = true;
    g_adc_raw_to_mv_errno = -EIO;
    EXPECT_EQ(Thermistor::readCelsius().error, Thermistor::Fault::ADC_READ_ERROR);

    g_adc_raw_to_mv_errno = 0;
    g_adc_mock_mv_val = 0;
    EXPECT_EQ(Thermistor::readCelsius().error, Thermistor::Fault::OUT_OF_RANGE);
}

TEST_F(SmartBatteryTestSuite, INA226_InitFailures) {
    INA226::Driver drv(nullptr);
    EXPECT_FALSE(drv.init());

    INA226::Driver drv2(&i2c_manager);
    g_i2c_call_counter = 0;
    g_i2c_fail_on_call_n = 2;
    g_i2c_fail_on_call_errno = -EIO;
    EXPECT_FALSE(drv2.init());
}

TEST_F(SmartBatteryTestSuite, CurveFitting_InteriorSegments) {
    static constexpr uint16_t ocv_probe_mv[] = {
        9100, 9900, 10500, 10950, 11250, 11550, 11850, 12150, 12450
    };
    for (uint16_t mv : ocv_probe_mv) {
        (void)battery.estimateSocFromVoltage(mv);
    }

    static constexpr int32_t ntc_probe_mv[] = {
        3200, 3160, 3110, 3050, 2970, 2870, 2750, 2610, 2460, 2290,
        2110, 1930, 1740, 1560, 1385, 1220, 1070,  935,  810,  700,
         605,  520,  450,  390,  340,  300,  260,  230,  200,  175,
         155,  135,  120
    };
    for (int32_t mv : ntc_probe_mv) {
        g_adc_mock_mv_val = mv;
        EXPECT_TRUE(Thermistor::readCelsius().success);
    }
}

TEST_F(SmartBatteryTestSuite, Thermistor_AdcReadFailures) {
    g_adc_ready_mock = true;

    g_adc_sequence_init_dt_errno = -EIO;
    EXPECT_EQ(Thermistor::readCelsius().error, Thermistor::Fault::ADC_READ_ERROR);
    g_adc_sequence_init_dt_errno = 0;

    g_adc_read_force_errno = -EIO;
    EXPECT_EQ(Thermistor::readCelsius().error, Thermistor::Fault::ADC_READ_ERROR);
    g_adc_read_force_errno = 0;
}
