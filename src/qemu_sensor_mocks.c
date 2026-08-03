#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/i2c_emul.h>
#include <zephyr/drivers/adc/adc_emul.h>
#include <zephyr/logging/log.h>
#include <math.h>
#include <stdbool.h>

LOG_MODULE_REGISTER(qemu_sensor_mocks, LOG_LEVEL_INF);

struct mock_sensor_data {
    uint8_t current_reg;
    uint8_t regs[256];
};

static int ina226_transfer(const struct emul *target, struct i2c_msg *msgs, int num_msgs, int addr) {
    struct mock_sensor_data *data = (struct mock_sensor_data *)target->data;

    for (int i = 0; i < num_msgs; i++) {
        if (msgs[i].flags & I2C_MSG_READ) {
            if (msgs[i].len == 2) {
                switch (data->current_reg) {
                    case 0x00: /* Configuration Register (POR default) */
                        msgs[i].buf[0] = 0x41; msgs[i].buf[1] = 0x27;
                        break;
                    case 0x01: /* Shunt Voltage: 4000 * 2.5uV/LSB = 10.00 mV */
                        msgs[i].buf[0] = 0x0F; msgs[i].buf[1] = 0xA0;
                        break;
                    case 0x02: /* Bus Voltage: pack voltage, not a logic rail.
                                * 3S Li-ion pack (8.7-12.6V valid range) -- 8880 * 1.25mV/LSB
                                * = 11100 mV = 11.1V nominal. */
                        msgs[i].buf[0] = 0x22; msgs[i].buf[1] = 0xB0;
                        break;
                    case 0x03: /* Power (Power_LSB = 25 * Current_LSB, calibration-dependent) */
                        msgs[i].buf[0] = 0x05; msgs[i].buf[1] = 0x14;
                        break;
                    case 0x04: /* Current: representative ~100mA for the mock calibration below */
                        msgs[i].buf[0] = 0x03; msgs[i].buf[1] = 0xE8;
                        break;
                    case 0x05: /* Calibration Register (mock value; real value = 0.00512/(Current_LSB*R_shunt)) */
                        msgs[i].buf[0] = 0x0A; msgs[i].buf[1] = 0x00;
                        break;
                    case 0xFE: /* Manufacturer ID: 'TI' */
                        msgs[i].buf[0] = 0x54; msgs[i].buf[1] = 0x49;
                        break;
                    case 0xFF: /* Die ID: 0x226 in bits[15:4], revision 0 in bits[3:0] */
                        msgs[i].buf[0] = 0x22; msgs[i].buf[1] = 0x60;
                        break;
                    default:
                        msgs[i].buf[0] = 0x00; msgs[i].buf[1] = 0x00;
                        break;
                }
            }
        } else {
            /* Write transaction: set the active register pointer */
            if (msgs[i].len >= 1) {
                data->current_reg = msgs[i].buf[0];
            }
        }
    }
    return 0;
}

static const struct i2c_emul_api api_ina226 = { .transfer = ina226_transfer };
static int init_ina226(const struct emul *target, const struct device *parent) { return 0; }

#define MOCK_INA226(n) \
    static struct mock_sensor_data data_ina226_##n; \
    EMUL_DT_DEFINE(n, init_ina226, &data_ina226_##n, NULL, &api_ina226, NULL)
DT_FOREACH_STATUS_OKAY(ti_ina226, MOCK_INA226);

/* Generic auto-increment memory-mapped register emulation, used by both
 * BME280 and LPS22HB: first byte of a write sets the pointer, subsequent
 * bytes are stored sequentially; reads stream out sequentially from the
 * pointer. This matches both parts' actual bus behavior. */
static int bme280_transfer(const struct emul *target, struct i2c_msg *msgs, int num_msgs, int addr) {
    struct mock_sensor_data *data = (struct mock_sensor_data *)target->data;

    for (int i = 0; i < num_msgs; i++) {
        if (msgs[i].flags & I2C_MSG_READ) {
            for (uint32_t j = 0; j < msgs[i].len; j++) {
                msgs[i].buf[j] = data->regs[(data->current_reg + j) & 0xFF];
            }
            data->current_reg = (data->current_reg + msgs[i].len) & 0xFF;
        } else {
            if (msgs[i].len >= 1) {
                data->current_reg = msgs[i].buf[0];
                for (uint32_t j = 1; j < msgs[i].len; j++) {
                    data->regs[(data->current_reg + j - 1) & 0xFF] = msgs[i].buf[j];
                }
            }
        }
    }
    return 0;
}

static const struct i2c_emul_api api_bme280 = { .transfer = bme280_transfer };

static int init_bme280(const struct emul *target, const struct device *parent) {
    struct mock_sensor_data *data = (struct mock_sensor_data *)target->data;

    /* Zero-init. We no longer need the old 0x01 filler hack: every register
     * the real Bosch compensation algorithm actually reads is explicitly
     * populated below, so there's nothing left that can divide by zero. */
    for (int i = 0; i < 256; i++) {
        data->regs[i] = 0x00;
    }

    data->regs[0xD0] = 0x60; /* CHIP_ID */

    /* STATUS (0xF3): bit3=measuring, bit0=im_update (NVM->image copy busy).
     * Leaving this at the old filler value of 0x01 told the driver the NVM
     * copy was permanently in progress; a driver that polls im_update before
     * trusting the calibration registers would spin/timeout forever. Idle
     * and calibration-ready is what the real part reports once powered up. */
    data->regs[0xF3] = 0x00;

    /* --- Compensation trim registers, 0x88-0xA1 and 0xE1-0xE7 --- *
     * These are the classic Bosch reference calibration coefficients,
     * verified here to decompensate (via the real Bosch integer formulas)
     * to ~25.08 C / ~1006.53 hPa / ~49.99 %RH against the raw ADC codes
     * programmed below -- i.e. this is a self-consistent, physically
     * plausible calibration+data pair, not placeholder bytes. */
    static const uint8_t calib_88_9F[24] = {
        0x70, 0x6B, /* 0x88/89 dig_T1 = 27504 (u16) */
        0x43, 0x67, /* 0x8A/8B dig_T2 = 26435 (s16) */
        0x18, 0xFC, /* 0x8C/8D dig_T3 = -1000 (s16) */
        0x7D, 0x8E, /* 0x8E/8F dig_P1 = 36477 (u16) */
        0x43, 0xD6, /* 0x90/91 dig_P2 = -10685 (s16) */
        0xD0, 0x0B, /* 0x92/93 dig_P3 = 3024   (s16) */
        0x27, 0x0B, /* 0x94/95 dig_P4 = 2855   (s16) */
        0x8C, 0x00, /* 0x96/97 dig_P5 = 140    (s16) */
        0xF9, 0xFF, /* 0x98/99 dig_P6 = -7     (s16) */
        0x8C, 0x3C, /* 0x9A/9B dig_P7 = 15500  (s16) */
        0xF8, 0xC6, /* 0x9C/9D dig_P8 = -14600 (s16) */
        0x70, 0x17, /* 0x9E/9F dig_P9 = 6000   (s16) */
    };
    for (int i = 0; i < 24; i++) {
        data->regs[0x88 + i] = calib_88_9F[i];
    }
    data->regs[0xA1] = 0x4B; /* dig_H1 = 75 (u8) */

    data->regs[0xE1] = 0x80; /* dig_H2 lo: dig_H2 = 384 (s16) */
    data->regs[0xE2] = 0x01; /* dig_H2 hi */
    data->regs[0xE3] = 0x00; /* dig_H3 = 0 (u8) */
    data->regs[0xE4] = 0x11; /* dig_H4[11:4]: dig_H4 = 280 (s12) */
    data->regs[0xE5] = 0x08; /* dig_H4[3:0] | dig_H5[3:0] (dig_H5 = 0) */
    data->regs[0xE6] = 0x00; /* dig_H5[11:4] */
    data->regs[0xE7] = 0x1E; /* dig_H6 = 30 (s8) */

    /* --- Raw ADC output, 0xF7-0xFE ---
     * adc_P = 415148 (20-bit), adc_T = 519888 (20-bit), adc_H = 26320 (16-bit).
     * Combined with the trim data above these decompensate to the values
     * noted in the block comment. */
    data->regs[0xF7] = 0x65; /* press_msb */
    data->regs[0xF8] = 0x5A; /* press_lsb */
    data->regs[0xF9] = 0xC0; /* press_xlsb (top nibble only, 20-bit mode) */
    data->regs[0xFA] = 0x7E; /* temp_msb */
    data->regs[0xFB] = 0xED; /* temp_lsb */
    data->regs[0xFC] = 0x00; /* temp_xlsb (top nibble only) */
    data->regs[0xFD] = 0x66; /* hum_msb */
    data->regs[0xFE] = 0xD0; /* hum_lsb */

    return 0;
}

#define MOCK_BME280(n) \
    static struct mock_sensor_data data_bme280_##n; \
    EMUL_DT_DEFINE(n, init_bme280, &data_bme280_##n, NULL, &api_bme280, NULL)
DT_FOREACH_STATUS_OKAY(bosch_bme280, MOCK_BME280);

static int lps22hb_transfer(const struct emul *target, struct i2c_msg *msgs, int num_msgs, int addr) {
    struct mock_sensor_data *data = (struct mock_sensor_data *)target->data;

    for (int i = 0; i < num_msgs; i++) {
        if (msgs[i].flags & I2C_MSG_READ) {
            bool auto_inc = (data->current_reg & 0x80) != 0;
            uint8_t base_reg = data->current_reg & 0x7F;

            for (uint32_t j = 0; j < msgs[i].len; j++) {
                msgs[i].buf[j] = data->regs[base_reg];
                if (auto_inc) {
                    base_reg = (base_reg + 1) & 0x7F;
                }
            }
        } else {
            if (msgs[i].len >= 1) {
                data->current_reg = msgs[i].buf[0];
                bool auto_inc = (data->current_reg & 0x80) != 0;
                uint8_t base_reg = data->current_reg & 0x7F;

                for (uint32_t j = 1; j < msgs[i].len; j++) {
                    data->regs[base_reg] = msgs[i].buf[j];
                    if (auto_inc) {
                        base_reg = (base_reg + 1) & 0x7F;
                    }
                }
            }
        }
    }
    return 0;
}

static const struct i2c_emul_api api_lps22hb = { .transfer = lps22hb_transfer };
static int init_lps22hb(const struct emul *target, const struct device *parent) {
    struct mock_sensor_data *data = (struct mock_sensor_data *)target->data;

    for (int i = 0; i < 256; i++) {
        data->regs[i] = 0x00;
    }

    data->regs[0x0F] = 0xB1; /* WHO_AM_I */

    /* Pressure: 1013.25 hPa * 4096 LSB/hPa = 4150272 = 0x3F5400 (24-bit, LE).
     * NOTE: the original bytes here (H=0x3E, L=0x80, XL=0x00) actually
     * decoded to exactly 1000.0 hPa, not the 1013.25 hPa the comment
     * claimed -- fixed below. */
    data->regs[0x28] = 0x00; /* PRESS_OUT_XL */
    data->regs[0x29] = 0x54; /* PRESS_OUT_L  */
    data->regs[0x2A] = 0x3F; /* PRESS_OUT_H  */

    /* Temperature: 24.00 C * 100 LSB/degC = 2400 = 0x0960 (16-bit, LE) */
    data->regs[0x2B] = 0x60; /* TEMP_OUT_L */
    data->regs[0x2C] = 0x09; /* TEMP_OUT_H */

    return 0;
}

#define MOCK_LPS22HB(n) \
    static struct mock_sensor_data data_lps22hb_##n; \
    EMUL_DT_DEFINE(n, init_lps22hb, &data_lps22hb_##n, NULL, &api_lps22hb, NULL)
DT_FOREACH_STATUS_OKAY(st_lps22hb_press, MOCK_LPS22HB);

/* 10K B3950 NTC thermistor, computed from the actual B-parameter equation
 * rather than a bare magic constant, so the emulated code is traceable to
 * physical parameters and can be re-derived for a different target
 * temperature or divider topology.
 *
 * Assumes the common topology: Vref -- R_fixed -- node -- R_ntc -- GND,
 * ADC sampling the divider node, with R_fixed == R0 (10k) -- a standard
 * pairing that centers the divider at the NTC's rated temperature. */
static uint16_t b3950_temp_to_adc_raw(float temp_c) {
    static const float R0 = 10000.0f;      /* NTC nominal resistance at T0 */
    static const float B = 3950.0f;        /* B-constant */
    static const float T0 = 298.15f;       /* 25 C in Kelvin */
    static const float R_FIXED = 10000.0f; /* divider fixed resistor */
    static const int ADC_MAX = 4095;       /* 12-bit ADC full scale */

    float temp_k = temp_c + 273.15f;
    float r_ntc = R0 * expf(B * (1.0f / temp_k - 1.0f / T0));
    float ratio = r_ntc / (r_ntc + R_FIXED);

    return (uint16_t)lroundf(ratio * ADC_MAX);
}

static int init_thermistor_mock(void) {
    const struct device *adc_dev = DEVICE_DT_GET(DT_NODELABEL(adc_emul0));

    if (!device_is_ready(adc_dev)) {
        LOG_ERR("ADC emulator not ready");
        return -ENODEV;
    }

    const float target_temp_c = 25.0f;
    uint16_t raw = b3950_temp_to_adc_raw(target_temp_c);

    int ret = adc_emul_const_value_set(adc_dev, 0, raw);
    if (ret != 0) {
        LOG_ERR("Failed to set emulated ADC value (err %d)", ret);
        return ret;
    }

    LOG_INF("Thermistor ADC emulator set for %d.%01uC (raw=%u)",
            (int)target_temp_c,
            (unsigned)((target_temp_c - (int)target_temp_c) * 10.0f + 0.5f),
            raw);
    return 0;
}

SYS_INIT(init_thermistor_mock, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);
