/*
 * Copyright (c) 2024 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause OR Apache-2.0
 */
#include <string.h>
#include <stdio.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include "main.h"
#include "i2c.h"
#include "dect_nr.h"

LOG_MODULE_REGISTER(i2c);

/* BME280 register addresses */
#define CTRLMEAS 0xF4
#define CALIB00  0x88
#define ID       0xD0
#define TEMPMSB  0xFA
#define CHIP_ID  0x60
#define SENSOR_CONFIG_VALUE 0x93
#define SENSOR_READ_INTERVAL_MS 2000

/* Global variables */
struct bme280_data bmedata;
const struct i2c_dt_spec dev_i2c = I2C_DT_SPEC_GET(DT_NODELABEL(mysensor));

/* Initialize BME280 */
int bme280_init(void)
{
    int err;

    if (!device_is_ready(dev_i2c.bus)) {
        LOG_ERR("I2C bus %s is not ready!", dev_i2c.bus->name);
        return -ENODEV;
    }
    LOG_INF("I2C device %s ready", dev_i2c.name);

    /* Check BME280 Chip ID */
    uint8_t chip_id_reg[] = {ID};
    uint8_t chip_id_val = 0;
    err = i2c_write_read_dt(&dev_i2c, chip_id_reg, sizeof(chip_id_reg), 
                           &chip_id_val, sizeof(chip_id_val));
    if (err) {
        LOG_ERR("Failed to read BME Chip ID, err: %d", err);
        return err; 
    } else if (chip_id_val != CHIP_ID) {
        LOG_WRN("BME Chip ID mismatch! Expected 0x%02X, Got 0x%02X", CHIP_ID, chip_id_val);
        return -ENODEV;
    } else {
        LOG_INF("BME Chip ID OK (0x%02X)", chip_id_val);
    }

    /* Get BME280 Calibration Data */
    bme_calibrationdata();

    /* Configure BME280 Sensor */
    uint8_t sensor_config[] = {CTRLMEAS, SENSOR_CONFIG_VALUE};
    err = i2c_write_dt(&dev_i2c, sensor_config, sizeof(sensor_config));
    if (err) {
        LOG_ERR("Failed to write BME config, err: %d", err);
        return err;
    } else {
        LOG_INF("BME configured with value 0x%02X to reg 0x%02X", 
               SENSOR_CONFIG_VALUE, CTRLMEAS);
    }

    return 0;
}

/* Get calibration data */
void bme_calibrationdata(void)
{
    uint8_t values[6];
    int ret = i2c_burst_read_dt(&dev_i2c, CALIB00, values, 6);

    if (ret != 0) {
        LOG_ERR("Failed to read BME calibration data, err %d", ret);
        return;
    }

    bmedata.dig_t1 = ((uint16_t)values[1]) << 8 | values[0];
    bmedata.dig_t2 = ((int16_t)values[3]) << 8 | values[2];
    bmedata.dig_t3 = ((int16_t)values[5]) << 8 | values[4];
    LOG_INF("BME T1: %u, T2: %d, T3: %d", 
           bmedata.dig_t1, bmedata.dig_t2, bmedata.dig_t3);
}

/* Compensate temperature */
int32_t bme280_compensate_temp(struct bme280_data *data, int32_t adc_temp)
{
    int32_t var1, var2;

    var1 = (((adc_temp >> 3) - ((int32_t)data->dig_t1 << 1)) * ((int32_t)data->dig_t2)) >> 11;
    var2 = (((((adc_temp >> 4) - ((int32_t)data->dig_t1)) *
              ((adc_temp >> 4) - ((int32_t)data->dig_t1))) >> 12) *
            ((int32_t)data->dig_t3)) >> 14;

    return ((var1 + var2) * 5 + 128) >> 8; // Returns temperature in DegC * 100
}

/* Sensor Thread Entry Function */
void sensor_thread_entry(void *p1, void *p2, void *p3)
{
    int ret;
    uint8_t temp_val[3] = {0};
    int32_t adc_temp;
    int32_t comp_temp;
    float temperature_c;

    LOG_INF("Sensor thread started");

    while (1) {
        /* Read raw temperature data */
        ret = i2c_burst_read_dt(&dev_i2c, TEMPMSB, temp_val, 3);
        if (ret != 0) {
            LOG_ERR("Failed to read temperature register, err %d", ret);
            k_msleep(SENSOR_READ_INTERVAL_MS);
            continue;
        }

        /* Combine raw data */
        adc_temp = (temp_val[0] << 12) | (temp_val[1] << 4) | ((temp_val[2] >> 4) & 0x0F);

        /* Compensate temperature */
        comp_temp = bme280_compensate_temp(&bmedata, adc_temp);
        temperature_c = (float)comp_temp / 100.0f;

        /* Update Shared Temperature Variable */
        ret = k_mutex_lock(&temp_mutex, K_MSEC(100));
        if (ret == 0) {
            g_current_temperature_c = temperature_c;
            k_mutex_unlock(&temp_mutex);
            LOG_INF("Sensor Read: %.2f C", temperature_c);
        } else {
            LOG_WRN("Could not lock temp mutex in sensor thread");
        }

        /* Wait before next reading */
        k_msleep(SENSOR_READ_INTERVAL_MS);
    }
}