/*
 * Copyright (c) 2024 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */
#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <nrf_modem_dect_phy.h>
#include <modem/nrf_modem_lib.h>
#include <zephyr/drivers/hwinfo.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/mutex.h> //include for mutex

LOG_MODULE_REGISTER(app);

BUILD_ASSERT(CONFIG_CARRIER, "Carrier must be configured according to local regulations");

// --------------------------- Defines and Variables ----------------------------------

#define DATA_LEN_MAX 32

// ------ RTOS Thread Defines -------
#define SENSOR_STACKSIZE 2048
#define DECT_STACKSIZE 2048
#define SENSORY_PRIORITY 7
#define DECT_PRIORITY	5

#define SENSOR_READ_INTERVAL_MS  2000

// --- Data Sharing -----
static float g_current_temperature_c = 0.0f;
K_MUTEX_DEFINE(temp_mutex);

// ---- Global Variables (from initial dect NR ---
static bool exit;
static uint16_t device_id;
// GPT recommend v
//static const struct i2c_dt_spec dev_i2c = I2C_DT_SPEC_GET(DT_NODELABEL(mysensor)); // Make I2C spec global

// Semaphore to synchronize modem calls. --from initial hello_dect
K_SEM_DEFINE(operation_sem, 0, 1);

/* Header type 1, due to endianness the order is different than in the specification. -- from initial hello_dect*/
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


// --------------------------- DECT Threads/Callbacks -------------------------------

// Callback after init operation. (from initial hello_dect, so are the next 3)
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

	// CONCERNS FROM GPT
	// Check if this semaphore give is expected/needed. Often op_complete handles RX end.
    // If receive() duration ends naturally, op_complete should be called.
    // If nrf_modem_dect_phy_rx_stop() is called, this callback occurs.
    // Let's assume op_complete handles the semaphore give for now based on original code.
    // k_sem_give(&operation_sem); // Might be needed depending on exact flow
}

/* Physical Control Channel reception notification. */
static void pcc(
	const uint64_t *time,
	const struct nrf_modem_dect_phy_rx_pcc_status *status,
	const union nrf_modem_dect_phy_hdr *hdr)
{
	struct phy_ctrl_field_common *header = (struct phy_ctrl_field_common *)hdr->type_1;

	LOG_INF("Received header from device ID %d",
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
	
	/* Received RSSI value is in fixed precision format Q14.1 */
	LOG_INF("Received data (RSSI: %d.%d): %s",
		(status->rssi_2 / 2), (status->rssi_2 & 0b1) * 5, (char *)data);
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

/* Dect PHY callbacks. */ //(For the callbacks mentioned above)
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

// --------------------DECT NR+ Initialization and Functions ------------------

/* Dect PHY init parameters. */
static struct nrf_modem_dect_phy_init_params dect_phy_init_params = {
	.harq_rx_expiry_time_us = 5000000,
	.harq_rx_process_count = 4,
};

/* Send operation. */
static int transmit(uint32_t handle, void *data, size_t data_len)
{
	int err;

	struct phy_ctrl_field_common header = {
		.header_format = 0x0,
		.packet_length_type = 0x0,
		.packet_length = 0x01,   // NOTE: this could likely be adjusted for larger 
								//  Amounts of data
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
	return 0;
}

/* Receive operation. */
static int receive(uint32_t handle)
{
	int err;

	struct nrf_modem_dect_phy_rx_params rx_op_params = {
		.start_time = 0,
		.handle = handle,
		.network_id = CONFIG_NETWORK_ID,
		.mode = NRF_MODEM_DECT_PHY_RX_MODE_CONTINUOUS, // NOTE: could try 
				//NRF_MODEM_DECT_PHY_RX_MODE_SINGLE_SHOT 
		.rssi_interval = NRF_MODEM_DECT_PHY_RSSI_INTERVAL_OFF,
		.link_id = NRF_MODEM_DECT_PHY_LINK_UNSPECIFIED,
		.rssi_level = -60,  // NOTE: could likely be adjusted for range test
		.carrier = CONFIG_CARRIER,
		.duration = CONFIG_RX_PERIOD_S * MSEC_PER_SEC *
			    NRF_MODEM_DECT_MODEM_TIME_TICK_RATE_KHZ, // NOTE: may a 
														// bottleneck
		.filter.short_network_id = CONFIG_NETWORK_ID & 0xff,
		.filter.is_short_network_id_used = 1,
		/* listen for everything (broadcast mode used) */
		.filter.receiver_identity = 0,  //NOTE: where devices can be filtered
	};

	err = nrf_modem_dect_phy_rx(&rx_op_params);
	if (err != 0) {
		LOG_ERR("nrf_modem_dect_phy_rx failed, err %d", err);
		return err;
	}

	return 0;
}


// ----------------------- BME280 Functions/Defines ------------------------

// ---- BME Data Structure ----
struct bme280_data {
	// Compensation/callibration parameters 
	uint16_t dig_t1;
	int16_t dig_t2;
	int16_t dig_t3;
} bmedata; 

#define SLEEP_TIME_MS 1000
#define CTRLMEAS 0xF4
#define CALIB00	 0x88
#define ID	     0xD0
#define TEMPMSB	 0xFA
#define CHIP_ID  0x60
#define SENSOR_CONFIG_VALUE 0x93

#define I2C_NODE DT_NODELABEL(mysensor)

//used to receive the callibration data
void bme_calibrationdata(const struct i2c_dt_spec *spec, struct bme280_data *sensor_data_ptr)
{
	uint8_t values[6];
	int ret = i2c_burst_read_dt(spec, CALIB00, values, 6);

	if (ret != 0) {
		printk("Failed to read register %x \n", CALIB00);
		return;
	}

	sensor_data_ptr->dig_t1 = ((uint16_t)values[1]) << 8 | values[0];
	sensor_data_ptr->dig_t2 = ((uint16_t)values[3]) << 8 | values[2];
	sensor_data_ptr->dig_t3 = ((uint16_t)values[5]) << 8 | values[4];
	LOG_INF("BME T1: %u, T2: %d, T3: %d", sensor_data_ptr->dig_t1, sensor_data_ptr->dig_t2, sensor_data_ptr->dig_t3);

}

//used to compensate data with callibration data
static int32_t bme280_compensate_temp(struct bme280_data *data, int32_t adc_temp)
{
	int32_t var1, var2;

	var1 = (((adc_temp >> 3) - ((int32_t)data->dig_t1 << 1)) * ((int32_t)data->dig_t2)) >> 11;

	var2 = (((((adc_temp >> 4) - ((int32_t)data->dig_t1)) *
		  ((adc_temp >> 4) - ((int32_t)data->dig_t1))) >>
		 12) *
		((int32_t)data->dig_t3)) >>
	       14;

	return ((var1 + var2) * 5 + 128) >> 8;
}



// -------------------------- Sensor Thread ------------------------------/

// this is where the thread for the sensor enters. 
void sensor_thread_entry (void *p1, void *p2, void *p3) 
{
	// variables to be used in the thread
	
	int ret;
	uint8_t temp_val[3] = {0};
	int32_t adc_temp;
	int32_t comp_temp;
	float temperature_c;

	LOG_INF("Sensor thread started");

	while (1) {
        // Read raw temperature data
        ret = i2c_burst_read_dt(&dev_i2c, TEMPMSB, temp_val, 3);
        if (ret != 0) {
            LOG_ERR("Failed to read temperature register, err %d", ret);
            k_msleep(SENSOR_READ_INTERVAL_MS); // Wait before retrying
            continue;
        }

        // Combine raw data
        adc_temp = (temp_val[0] << 12) | (temp_val[1] << 4) | ((temp_val[2] >> 4) & 0x0F);

        // Compensate temperature
        comp_temp = bme280_compensate_temp(&bmedata, adc_temp);
        temperature_c = (float)comp_temp / 100.0f;

        // --- Update Shared Temperature Variable ---
        ret = k_mutex_lock(&temp_mutex, K_MSEC(100)); // Wait up to 100ms for mutex
        if (ret == 0) {
            g_current_temperature_c = temperature_c;
            k_mutex_unlock(&temp_mutex);
            LOG_INF("Sensor Read: %.2f C", temperature_c);
        } else {
             LOG_WRN("Could not lock temp mutex in sensor thread");
        }
        // -----------------------------------------

        // Wait before next reading
        k_msleep(SENSOR_READ_INTERVAL_MS);
    }



}





int main(void)
{
	
	// ------------------ Temperature Sensor Initialization---------------

	static const struct i2c_dt_spec dev_i2c = I2C_DT_SPEC_GET(I2C_NODE);
	uint8_t id = 0;
	uint8_t regs[] = {ID};
	int ret = i2c_write_read_dt(&dev_i2c, regs, 1, &id, 1);
	bme_calibrationdata(&dev_i2c, &bmedata);
	uint8_t sensor_config[] = {CTRLMEAS, SENSOR_CONFIG_VALUE};
	ret = i2c_write_dt(&dev_i2c, sensor_config, 2);
	
	/* bme part end
	*************************************************************************
	*/
	int err;
	uint32_t tx_handle = 0;
	uint32_t rx_handle = 1;
	uint32_t tx_counter_value = 0;
	uint8_t tx_buf[DATA_LEN_MAX];
	size_t tx_len;

	LOG_INF("Dect NR+ PHY Hello sample started");

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
	tx_counter_value = 0x7FFFFFFF;
	while (1) {
		/* BME PART BEGIN*/
		uint8_t temp_val[3] = {0};
		int ret = i2c_burst_read_dt(&dev_i2c, TEMPMSB, temp_val, 3);
		int32_t adc_temp =	(temp_val[0] << 12) 
							| (temp_val[1] << 4) 
							| ((temp_val[2] >> 4) & 0x0F);

		int32_t comp_temp = bme280_compensate_temp(&bmedata, adc_temp);

		float temperature = (float)comp_temp / 100.0f;
		double fTemp = (double)temperature * 1.8 + 32;

		printk("Temperature in Celsius : %8.2f C\n", (double)temperature);
		printk("Temperature in Fahrenheit : %.2f F\n", fTemp);
		printk("Test %d",nrf_modem_dect_phy_time_get());
		/* BME PART END*/


		/** Transmitting message */
		LOG_INF("Transmitting %8.2f", (double)temperature);
		tx_len = sprintf(tx_buf,"%8.2f", (double)temperature);

		err = transmit(tx_handle, tx_buf, tx_len);
		if (err) {
			LOG_ERR("Transmisstion failed, err %x", err);
			return err;
		}

		/**tx_counter_value++; */

		/**if ((tx_counter_value >= CONFIG_TX_TRANSMISSIONS) && CONFIG_TX_TRANSMISSIONS) {
			LOG_INF("Reached maximum number of transmissions (%d)",
				CONFIG_TX_TRANSMISSIONS);
			break;
		}*/

		/* Wait for TX operation to complete. */
		k_sem_take(&operation_sem, K_FOREVER);

		/** Receiving messages for CONFIG_RX_PERIOD_S seconds. */
		err = receive(rx_handle);
		if (err) {
			LOG_ERR("Reception failed, err %d", err);
			return err;
		}

		/* Wait for RX operation to complete. */
		k_sem_take(&operation_sem, K_FOREVER);
	}

	LOG_INF("Shutting down");

	err = nrf_modem_dect_phy_deinit();
	if (err) {
		LOG_ERR("nrf_modem_dect_phy_deinit() failed, err %d", err);
		return err;
	}

	k_sem_take(&operation_sem, K_FOREVER);

	err = nrf_modem_lib_shutdown();
	if (err) {
		LOG_ERR("nrf_modem_lib_shutdown() failed, err %d", err);
		return err;
	}

	LOG_INF("Bye!");

	return 0;
}
