#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/i2c_emul.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(qemu_sensor_mocks, LOG_LEVEL_INF);

/* Shared data struct to track the last requested register address */
struct mock_sensor_data {
    uint8_t current_reg;
};

/* ============================================================================== */
/* 1. INA226 Emulator (ti,ina226)                                                 */
/* ============================================================================== */
static int ina226_transfer(const struct emul *target, struct i2c_msg *msgs, int num_msgs, int addr) {
    struct mock_sensor_data *data = (struct mock_sensor_data *)target->data;
    for (int i = 0; i < num_msgs; i++) {
        if (msgs[i].flags & I2C_MSG_READ) {
            if (msgs[i].len == 2) {
                if (data->current_reg == 0xFE) { 
                    /* Manufacturer ID Register -> 'TI' */
                    msgs[i].buf[0] = 0x54; msgs[i].buf[1] = 0x49; 
                } else if (data->current_reg == 0xFF) { 
                    /* Die ID Register -> 0x2260 */
                    msgs[i].buf[0] = 0x22; msgs[i].buf[1] = 0x60;
                } else {
                    /* Safe dummy data for voltage/current reads */
                    msgs[i].buf[0] = 0x01; msgs[i].buf[1] = 0x00; 
                }
            }
        } else {
            if (msgs[i].len >= 1) data->current_reg = msgs[i].buf[0];
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


/* ============================================================================== */
/* 2. BME280 Emulator (bosch,bme280)                                              */
/* ============================================================================== */
static int bme280_transfer(const struct emul *target, struct i2c_msg *msgs, int num_msgs, int addr) {
    struct mock_sensor_data *data = (struct mock_sensor_data *)target->data;
    for (int i = 0; i < num_msgs; i++) {
        if (msgs[i].flags & I2C_MSG_READ) {
            if (data->current_reg == 0xD0 && msgs[i].len >= 1) { 
                /* BME280 Chip ID */
                msgs[i].buf[0] = 0x60;
            } else {
                /* Fill calibration queries with 0x01 to prevent division-by-zero faults */
                for(uint32_t j = 0; j < msgs[i].len; j++) msgs[i].buf[j] = 0x01;
            }
        } else {
            if (msgs[i].len >= 1) data->current_reg = msgs[i].buf[0];
        }
    }
    return 0;
}

static const struct i2c_emul_api api_bme280 = { .transfer = bme280_transfer };
static int init_bme280(const struct emul *target, const struct device *parent) { return 0; }

#define MOCK_BME280(n) \
    static struct mock_sensor_data data_bme280_##n; \
    EMUL_DT_DEFINE(n, init_bme280, &data_bme280_##n, NULL, &api_bme280, NULL)
DT_FOREACH_STATUS_OKAY(bosch_bme280, MOCK_BME280);


/* ============================================================================== */
/* 3. LPS22HB Emulator (st,lps22hb-press)                                         */
/* ============================================================================== */
static int lps22hb_transfer(const struct emul *target, struct i2c_msg *msgs, int num_msgs, int addr) {
    struct mock_sensor_data *data = (struct mock_sensor_data *)target->data;
    for (int i = 0; i < num_msgs; i++) {
        if (msgs[i].flags & I2C_MSG_READ) {
            if (data->current_reg == 0x0F && msgs[i].len >= 1) { 
                /* LPS22HB WHO_AM_I ID */
                msgs[i].buf[0] = 0xB1;
            } else {
                for(uint32_t j = 0; j < msgs[i].len; j++) msgs[i].buf[j] = 0x01;
            }
        } else {
            /* ST sensors OR the register address with 0x80 for auto-increment. Mask it out. */
            if (msgs[i].len >= 1) data->current_reg = msgs[i].buf[0] & 0x7F;
        }
    }
    return 0;
}

static const struct i2c_emul_api api_lps22hb = { .transfer = lps22hb_transfer };
static int init_lps22hb(const struct emul *target, const struct device *parent) { return 0; }

#define MOCK_LPS22HB(n) \
    static struct mock_sensor_data data_lps22hb_##n; \
    EMUL_DT_DEFINE(n, init_lps22hb, &data_lps22hb_##n, NULL, &api_lps22hb, NULL)
DT_FOREACH_STATUS_OKAY(st_lps22hb_press, MOCK_LPS22HB);
