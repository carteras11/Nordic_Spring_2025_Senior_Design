/* 
 * nr_rangetest_receiver
 * Written Mar. 31st, 2025
 * Based on Dect NR+ Phy Hello example from Nordic
 * 
 * Listens for a burst of NR+ packets. To be used in conjunction
 * with a device running nr_rangetest_transmitter.
 */
#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <nrf_modem_dect_phy.h>
#include <modem/nrf_modem_lib.h>
#include <zephyr/drivers/hwinfo.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/uart.h>

LOG_MODULE_REGISTER(app);

static const struct gpio_dt_spec led0 = GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);
const struct device *uart = DEVICE_DT_GET(DT_NODELABEL(uart0));

BUILD_ASSERT(CONFIG_CARRIER, "Carrier must be configured according to local regulations");

#define RX_BUF_LEN 32

static bool exit;
static uint16_t device_id;

#define MEASUREMENT_BUF_SIZE 100
static char measurement_buf[MEASUREMENT_BUF_SIZE];
static volatile uint16_t measurement_len;
static volatile uint8_t new_measurement = 0;


/* Header type 1, due to endianness the order is different than in the specification. */
struct phy_ctrl_field_common {
	uint32_t packet_length : 4;
	uint32_t packet_length_type : 1;
	uint32_t header_format : 3;
	uint32_t short_network_id : 8;
	uint32_t transmitter_id_hi : 8;
	uint32_t transmitter_id_lo : 8;
	uint32_t df_mcs : 3;
	uint32_t reserved : 1;
	uint32_t transmit_power : 4;
	uint32_t pad : 24;
};

/* Semaphore to synchronize modem calls. */
K_SEM_DEFINE(operation_sem, 0, 1);

/* Callback after init operation. */
static void init(const uint64_t *time, int16_t temp, enum nrf_modem_dect_phy_err err,
	  const struct nrf_modem_dect_phy_modem_cfg *cfg)
{
	if (err) {
		LOG_ERR("Init failed, err %d", err);
		exit = true;
		return;
	}

	k_sem_give(&operation_sem);
}

/* Callback after deinit operation. */
static void deinit(const uint64_t *time, enum nrf_modem_dect_phy_err err)
{
	if (err) {
		LOG_ERR("Deinit failed, err %d", err);
		return;
	}

	k_sem_give(&operation_sem);
}

/* Operation complete notification. */
static void op_complete(const uint64_t *time, int16_t temperature,
		 enum nrf_modem_dect_phy_err err, uint32_t handle)
{
	LOG_DBG("op_complete cb time %"PRIu64" status %d", *time, err);
	k_sem_give(&operation_sem);
}

/* Callback after receive stop operation. */
static void rx_stop(const uint64_t *time, enum nrf_modem_dect_phy_err err, uint32_t handle)
{
	LOG_DBG("rx_stop cb time %"PRIu64" status %d handle %d", *time, err, handle);
	k_sem_give(&operation_sem);
}

/* Physical Control Channel reception notification. */
static void pcc(
	const uint64_t *time,
	const struct nrf_modem_dect_phy_rx_pcc_status *status,
	const union nrf_modem_dect_phy_hdr *hdr)
{
	struct phy_ctrl_field_common *header = (struct phy_ctrl_field_common *)hdr->type_1;

	LOG_DBG("Received header from device ID %d",
		header->transmitter_id_hi << 8 |  header->transmitter_id_lo);
}

/* Physical Control Channel CRC error notification. */
static void pcc_crc_err(const uint64_t *time,
		 const struct nrf_modem_dect_phy_rx_pcc_crc_failure *crc_failure)
{
	LOG_DBG("pcc_crc_err cb time %"PRIu64"", *time);
}

/* Physical Data Channel reception notification. */
static void pdc(const uint64_t *time,
		const struct nrf_modem_dect_phy_rx_pdc_status *status,
		const void *data, uint32_t len)
{
	/* Received RSSI value is in fixed precision format Q14.1, SNR Q13.2*/
	LOG_DBG("Received data (RSSI: %d.%d), (SNR: %d.%d): %s",
		(status->rssi_2 / 2), (status->rssi_2 & 0b1) * 5,
        (status->snr >> 2), (status->snr & 0b11) * 5 / 2,
        (char *)data);

    // Store measurement in format: RSSI,SNR
    measurement_len = sprintf(measurement_buf, "%d.%d,%d.%d\r\n",
		(status->rssi_2 / 2), (status->rssi_2 & 0b1) * 5,
        (status->snr >> 2), (status->snr & 0b11) * 5 / 2);

    new_measurement = 1;
}

/* Physical Data Channel CRC error notification. */
static void pdc_crc_err(
	const uint64_t *time, const struct nrf_modem_dect_phy_rx_pdc_crc_failure *crc_failure)
{
	LOG_DBG("pdc_crc_err cb time %"PRIu64"", *time);
}

/* RSSI measurement result notification. */
static void rssi(const uint64_t *time, const struct nrf_modem_dect_phy_rssi_meas *status)
{
	LOG_DBG("rssi cb time %"PRIu64" carrier %d", *time, status->carrier);
}

/* Callback after link configuration operation. */
static void link_config(const uint64_t *time, enum nrf_modem_dect_phy_err err)
{
	LOG_DBG("link_config cb time %"PRIu64" status %d", *time, err);
}

/* Callback after time query operation. */
static void time_get(const uint64_t *time, enum nrf_modem_dect_phy_err err)
{
	LOG_DBG("time_get cb time %"PRIu64" status %d", *time, err);
}

/* Callback after capability get operation. */
static void capability_get(const uint64_t *time, enum nrf_modem_dect_phy_err err,
		    const struct nrf_modem_dect_phy_capability *capability)
{
	LOG_DBG("capability_get cb time %"PRIu64" status %d", *time, err);
}

static void stf_cover_seq_control(const uint64_t *time, enum nrf_modem_dect_phy_err err)
{
	LOG_WRN("Unexpectedly in %s\n", (__func__));
}

/* Dect PHY callbacks. */
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

/* Dect PHY init parameters. */
static struct nrf_modem_dect_phy_init_params dect_phy_init_params = {
	.harq_rx_expiry_time_us = 5000000,
	.harq_rx_process_count = 4,
};

/* Receive operation. */
static int receive_oneshot(uint32_t handle)
{
	int err;

	struct nrf_modem_dect_phy_rx_params rx_op_params = {
		.start_time = 0,
		.handle = handle,
		.network_id = CONFIG_NETWORK_ID,
		.mode = NRF_MODEM_DECT_PHY_RX_MODE_SINGLE_SHOT,
		.rssi_interval = NRF_MODEM_DECT_PHY_RSSI_INTERVAL_12_SLOTS,
		.link_id = NRF_MODEM_DECT_PHY_LINK_UNSPECIFIED,
		.rssi_level = -60,
		.carrier = CONFIG_CARRIER,
		.duration = CONFIG_RX_PERIOD_S * MSEC_PER_SEC *
			    NRF_MODEM_DECT_MODEM_TIME_TICK_RATE_KHZ,
		.filter.short_network_id = CONFIG_NETWORK_ID & 0xff,
		.filter.is_short_network_id_used = 1,
		/* listen for everything (broadcast mode used) */
		.filter.receiver_identity = 0,
	};

	err = nrf_modem_dect_phy_rx(&rx_op_params);
	if (err != 0) {
		return err;
	}

	return 0;
}

/* RSSI operation. */
/*
 Note: for testing purposes, it's easier to just use nrf_modem_dect_phy_rx() to measure rssi and SNR.
*/
static int measure_RSSI(uint32_t handle)
{
	int err;

    struct nrf_modem_dect_phy_rssi_params rssi_op_params = {
        .carrier = CONFIG_CARRIER,
		.duration = 4800,
        .handle = handle,
        .reporting_interval = NRF_MODEM_DECT_PHY_RSSI_INTERVAL_12_SLOTS,
        .start_time = 0
    };

	err = nrf_modem_dect_phy_rssi(&rssi_op_params);
	if (err != 0) {
		return err;
	}

	return 0;
}

int main(void)
{
	int err;
	uint32_t rx_handle = 1;

	LOG_INF("nr_rangetest_reciever started");

    if (!device_is_ready(led0.port)) {
        printk("GPIO device is not ready\n");
        return 1;
    }

    err = gpio_pin_configure_dt(&led0, GPIO_OUTPUT_ACTIVE);
    if (err < 0) {
        return err;
    }

    if (!device_is_ready(uart)) {
        printk("UART device not ready\n");
        return 1;
    }

	err = nrf_modem_lib_init();
	if (err) {
		LOG_ERR("modem init failed, err %d", err);
		return err;
	}

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

	k_sem_take(&operation_sem, K_FOREVER);
	if (exit) {
		return -EIO;
	}

	hwinfo_get_device_id((void *)&device_id, sizeof(device_id));

	LOG_INF("Dect NR+ PHY initialized, device ID: %d", device_id);

	err = nrf_modem_dect_phy_capability_get();
	if (err) {
		LOG_ERR("nrf_modem_dect_phy_capability_get failed, err %d", err);
	}

	while (1) {

        gpio_pin_set_dt(&led0, 1);

        err = receive_oneshot(rx_handle);
        if (err) {
			LOG_ERR("Reception failed, err %d", err);
			return err;
		}

		/* Wait for RX operation to complete. */
		k_sem_take(&operation_sem, K_FOREVER);

        if (new_measurement) {

            gpio_pin_set_dt(&led0, 0);
            err = uart_tx(uart, measurement_buf, measurement_len, SYS_FOREVER_MS);
            if (err) {
                return err;
            }

            new_measurement = 0;
        }
	}
}
