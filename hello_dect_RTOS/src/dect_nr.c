/*
 * Copyright (c) 2024 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause OR Apache-2.0
 */
#include <string.h>
#include <stdio.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <nrf_modem_dect_phy.h>
#include "main.h"
#include "dect_nr.h"

LOG_MODULE_REGISTER(dect);

#define DATA_LEN_MAX 32

/* Dect PHY callbacks struct */
static struct nrf_modem_dect_phy_callbacks dect_phy_callbacks = {
    .init = init,
    .deinit = deinit,
    .op_complete = op_complete,
    .rx_stop = rx_stop,
    .pcc = pcc,
    .pcc_crc_err = pcc_crc_err,
    .pdc = pdc,
    .pdc_crc_err = pdc_crc_err,
    .rssi = rssi,
    .link_config = link_config,
    .time_get = time_get,
    .capability_get = capability_get,
    .stf_cover_seq_control = stf_cover_seq_control,
};

/* Dect PHY init parameters */
static struct nrf_modem_dect_phy_init_params dect_phy_init_params = {
    .harq_rx_expiry_time_us = 5000000,
    .harq_rx_process_count = 4,
};

/* Global Variables */
static bool exit_app; // Shared with main.c through callback

/* Initialize DECT PHY */
int dect_phy_init(void)
{
    int err;

    err = nrf_modem_dect_phy_callback_set(&dect_phy_callbacks);
    if (err) {
        LOG_ERR("nrf_modem_dect_phy_callback_set failed, err %d", err);
        return err;
    }

    err = nrf_modem_dect_phy_init(&dect_phy_init_params);
    if (err) {
        LOG_ERR("nrf_modem_dect_phy_init failed, err %d", err);
        return err;
    }

    /* Wait for the init callback to complete */
    err = k_sem_take(&operation_sem, K_SECONDS(10));
    if (err != 0) {
        LOG_ERR("Timeout waiting for DECT PHY init complete!");
        return -ETIMEDOUT;
    }
    if (exit_app) {
        LOG_ERR("DECT PHY init failed in callback.");
        return -EIO;
    }

    return 0;
}

/* DECT callbacks implementation */
void init(const uint64_t *time, int16_t temp, enum nrf_modem_dect_phy_err err,
         const struct nrf_modem_dect_phy_modem_cfg *cfg)
{
    if (err) {
        LOG_ERR("Init failed, err %d", err);
        exit_app = true;
    }
    k_sem_give(&operation_sem);
}

void deinit(const uint64_t *time, enum nrf_modem_dect_phy_err err)
{
    if (err) {
        LOG_ERR("Deinit failed, err %d", err);
    }
    k_sem_give(&operation_sem);
}

void op_complete(const uint64_t *time, int16_t temperature,
                enum nrf_modem_dect_phy_err err, uint32_t handle)
{
    LOG_DBG("op_complete cb time %"PRIu64" status %d handle %u", *time, err, handle);
    if (err) {
        LOG_WRN("Operation %u failed with err %d", handle, err);
    }
    k_sem_give(&operation_sem);
}

void rx_stop(const uint64_t *time, enum nrf_modem_dect_phy_err err, uint32_t handle)
{
    LOG_DBG("rx_stop cb time %"PRIu64" status %d handle %u", *time, err, handle);
    if (err && err != NRF_MODEM_DECT_PHY_ERR_RX_STOPPED) {
        LOG_WRN("rx_stop failed with err %d", err);
    }
}

void pcc(const uint64_t *time,
         const struct nrf_modem_dect_phy_rx_pcc_status *status,
         const union nrf_modem_dect_phy_hdr *hdr)
{
    struct phy_ctrl_field_common *header = (struct phy_ctrl_field_common *)hdr->type_1;
    LOG_INF("PCC from device ID %d",
            header->transmitter_id_hi << 8 | header->transmitter_id_lo);
}

void pcc_crc_err(const uint64_t *time,
                const struct nrf_modem_dect_phy_rx_pcc_crc_failure *crc_failure)
{
    LOG_WRN("PCC CRC Error cb time %"PRIu64"", *time);
}

void pdc(const uint64_t *time,
         const struct nrf_modem_dect_phy_rx_pdc_status *status,
         const void *data, uint32_t len)
{
    LOG_INF("Received data (RSSI: %d.%d): %.*s",
            (status->rssi_2 / 2), (status->rssi_2 & 0b1) * 5, len, (char *)data);
}

void pdc_crc_err(const uint64_t *time, 
                const struct nrf_modem_dect_phy_rx_pdc_crc_failure *crc_failure)
{
    LOG_WRN("PDC CRC Error cb time %"PRIu64"", *time);
}

void rssi(const uint64_t *time, const struct nrf_modem_dect_phy_rssi_meas *status)
{
    LOG_DBG("rssi cb time %"PRIu64" carrier %d", *time, status->carrier);
}

void link_config(const uint64_t *time, enum nrf_modem_dect_phy_err err)
{
    LOG_DBG("link_config cb time %"PRIu64" status %d", *time, err);
}

void time_get(const uint64_t *time, enum nrf_modem_dect_phy_err err)
{
    LOG_DBG("time_get cb time %"PRIu64" status %d", *time, err);
}

void capability_get(const uint64_t *time, enum nrf_modem_dect_phy_err err,
                   const struct nrf_modem_dect_phy_capability *capability)
{
    LOG_DBG("capability_get cb time %"PRIu64" status %d", *time, err);
}

void stf_cover_seq_control(const uint64_t *time, enum nrf_modem_dect_phy_err err)
{
    LOG_WRN("Unexpectedly in %s\n", (__func__));
}

/* DECT Transmit/Receive Functions */
int transmit(uint32_t handle, void *data, size_t data_len)
{
    int err;

    struct phy_ctrl_field_common header = {
        .header_format = 0x0,
        .packet_length_type = 0x0,
        .packet_length = 0x01,
        .short_network_id = (CONFIG_NETWORK_ID & 0xff),
        .transmitter_id_hi = (device_id >> 8),
        .transmitter_id_lo = (device_id & 0xff),
        .transmit_power = CONFIG_TX_POWER,
        .reserved = 0,
        .df_mcs = CONFIG_MCS,
    };

    struct nrf_modem_dect_phy_tx_params tx_op_params = {
        .start_time = 0,
        .handle = handle,
        .network_id = CONFIG_NETWORK_ID,
        .phy_type = 0,
        .lbt_rssi_threshold_max = 0,
        .carrier = CONFIG_CARRIER,
        .lbt_period = NRF_MODEM_DECT_LBT_PERIOD_MAX,
        .phy_header = (union nrf_modem_dect_phy_hdr *)&header,
        .data = data,
        .data_size = data_len,
    };

    err = nrf_modem_dect_phy_tx(&tx_op_params);
    if (err != 0) {
        LOG_ERR("nrf_modem_dect_phy_tx failed, err %d", err);
        return err;
    }
    LOG_DBG("TX submitted with handle %u", handle);
    return 0;
}

int receive(uint32_t handle)
{
    int err;

    struct nrf_modem_dect_phy_rx_params rx_op_params = {
        .start_time = 0,
        .handle = handle,
        .network_id = CONFIG_NETWORK_ID,
        .mode = NRF_MODEM_DECT_PHY_RX_MODE_CONTINUOUS,
        .rssi_interval = NRF_MODEM_DECT_PHY_RSSI_INTERVAL_OFF,
        .link_id = NRF_MODEM_DECT_PHY_LINK_UNSPECIFIED,
        .rssi_level = -60,
        .carrier = CONFIG_CARRIER,
        .duration = (uint64_t)CONFIG_RX_PERIOD_S * MSEC_PER_SEC *
                   NRF_MODEM_DECT_MODEM_TIME_TICK_RATE_KHZ,
        .filter.short_network_id = CONFIG_NETWORK_ID & 0xff,
        .filter.is_short_network_id_used = 1,
        .filter.receiver_identity = 0,
    };

    err = nrf_modem_dect_phy_rx(&rx_op_params);
    if (err != 0) {
        LOG_ERR("nrf_modem_dect_phy_rx failed, err %d", err);
        return err;
    }
    LOG_DBG("RX submitted with handle %u for %u s", handle, CONFIG_RX_PERIOD_S);
    return 0;
}

/* DECT Thread Entry Function */
void dect_thread_entry(void *p1, void *p2, void *p3)
{
    int err;
    uint32_t tx_handle = 0;
    uint32_t rx_handle = 1;
    uint8_t tx_buf[DATA_LEN_MAX];
    size_t tx_len;
    float local_temperature_c;

    LOG_INF("DECT thread started");

    while (1) {
        /* Read Shared Temperature Variable */
        err = k_mutex_lock(&temp_mutex, K_MSEC(100));
        if (err == 0) {
            local_temperature_c = g_current_temperature_c;
            k_mutex_unlock(&temp_mutex);
        } else {
            LOG_WRN("Could not lock temp mutex in DECT thread, using last known temp: %.2f", 
                   local_temperature_c);
        }

        /* Format message for transmission */
        tx_len = snprintf(tx_buf, sizeof(tx_buf), "T:%.2fC", local_temperature_c);
        if (tx_len >= sizeof(tx_buf)) {
            LOG_WRN("Temperature string truncated!");
            tx_len = sizeof(tx_buf) - 1;
        }

        LOG_INF("Transmitting: %s", tx_buf);

        /* Transmit message */
        err = transmit(tx_handle, tx_buf, tx_len);
        if (err) {
            LOG_ERR("Transmission failed, err %d", err);
            k_msleep(1000);
            continue;
        }

        /* Wait for TX operation to complete */
        err = k_sem_take(&operation_sem, K_SECONDS(5));
        if (err != 0) {
            LOG_ERR("Timeout waiting for TX complete (handle %u)", tx_handle);
        } else {
            LOG_DBG("TX complete confirmation received (handle %u)", tx_handle);
        }

        /* Receive messages for configured duration */
        err = receive(rx_handle);
        if (err) {
            LOG_ERR("Starting reception failed, err %d", err);
            k_msleep(1000);
            continue;
        }

        /* Wait for RX operation to complete */
        err = k_sem_take(&operation_sem, K_SECONDS(CONFIG_RX_PERIOD_S + 2));
        if (err != 0) {
            LOG_ERR("Timeout waiting for RX complete (handle %u)", rx_handle);
        } else {
            LOG_DBG("RX complete confirmation received (handle %u)", rx_handle);
        }
    }
}