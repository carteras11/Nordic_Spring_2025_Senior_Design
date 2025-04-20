/*
 * Copyright (c) 2024 Nordic Semiconductor ASA
 * Copyright (c) 2017 Linaro Limited // Added for multi-threading concepts
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause OR Apache-2.0
 */
#include <string.h>
#include <stdio.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <modem/nrf_modem_lib.h>
#include <zephyr/drivers/hwinfo.h>

#include "main.h"
#include "i2c.h"
#include "dect_nr.h"

LOG_MODULE_REGISTER(app);

BUILD_ASSERT(CONFIG_CARRIER, "Carrier must be configured according to local regulations");

/* Thread stack sizes and priorities */
#define SENSOR_STACKSIZE        1024
#define DECT_STACKSIZE          2048 
#define SENSOR_PRIORITY         7
#define DECT_PRIORITY           5

/* Shared Data */
float g_current_temperature_c = 0.0f;
K_MUTEX_DEFINE(temp_mutex); 
K_SEM_DEFINE(operation_sem, 0, 1);

/* Global Variables */
static bool exit_app;
uint16_t device_id;

int main(void)
{
    int err;

    LOG_INF("DECT NR+ PHY Multi-Threaded Sample Started");

    /* Initialize Modem Library */
    err = nrf_modem_lib_init();
    if (err) {
        LOG_ERR("Modem library init failed, err %d", err);
        return err;
    }

    /* Initialize I2C Sensor */
    err = bme280_init();
    if (err) {
        LOG_ERR("BME280 initialization failed, err %d", err);
        // Decide if this is fatal or if we continue
    }

    /* Initialize DECT PHY */
    err = dect_phy_init();
    if (err) {
        LOG_ERR("DECT PHY initialization failed, err %d", err);
        // nrf_modem_lib_shutdown(); // Attempt cleanup
        return err;
    }

    /* Get Device ID */
    uint8_t dev_id_buf[sizeof(device_id)];
    err = hwinfo_get_device_id(dev_id_buf, sizeof(dev_id_buf));
    if (err < 0) {
        LOG_ERR("Failed to get device ID, err %d", err);
        device_id = 0xFFFF; // Assign placeholder ID
    } else {
        memcpy(&device_id, dev_id_buf, sizeof(device_id));
    }

    LOG_INF("DECT NR+ PHY initialized, Device ID: %u (0x%04X)", device_id, device_id);
    LOG_INF("Starting worker threads...");
    
    return 0;
}

/* Thread Definitions */
K_THREAD_DEFINE(sensor_thread_id, SENSOR_STACKSIZE, sensor_thread_entry, NULL, NULL, NULL,
                SENSOR_PRIORITY, 0, 0);

K_THREAD_DEFINE(dect_thread_id, DECT_STACKSIZE, dect_thread_entry, NULL, NULL, NULL,
                DECT_PRIORITY, 0, 0);