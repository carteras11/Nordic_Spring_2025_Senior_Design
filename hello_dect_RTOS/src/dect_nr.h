#ifndef DECT_NR_H
#define DECT_NR_H

#include <nrf_modem_dect_phy.h>

/* DECT Function declarations */
int dect_phy_init(void);
int transmit(uint32_t handle, void *data, size_t data_len);
int receive(uint32_t handle);
void dect_thread_entry(void *p1, void *p2, void *p3);

/* Header type 1 */
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

/* Callback declarations */
void init(const uint64_t *time, int16_t temp, 
          enum nrf_modem_dect_phy_err err,
          const struct nrf_modem_dect_phy_modem_cfg *cfg);

void deinit(const uint64_t *time, enum nrf_modem_dect_phy_err err);

void op_complete(const uint64_t *time, int16_t temperature,
                enum nrf_modem_dect_phy_err err, uint32_t handle);

void rx_stop(const uint64_t *time, enum nrf_modem_dect_phy_err err, uint32_t handle);

void pcc(const uint64_t *time,
         const struct nrf_modem_dect_phy_rx_pcc_status *status,
         const union nrf_modem_dect_phy_hdr *hdr);

void pcc_crc_err(const uint64_t *time,
                const struct nrf_modem_dect_phy_rx_pcc_crc_failure *crc_failure);

void pdc(const uint64_t *time,
         const struct nrf_modem_dect_phy_rx_pdc_status *status,
         const void *data, uint32_t len);

void pdc_crc_err(const uint64_t *time, 
                const struct nrf_modem_dect_phy_rx_pdc_crc_failure *crc_failure);

void rssi(const uint64_t *time, const struct nrf_modem_dect_phy_rssi_meas *status);

void link_config(const uint64_t *time, enum nrf_modem_dect_phy_err err);

void time_get(const uint64_t *time, enum nrf_modem_dect_phy_err err);

void capability_get(const uint64_t *time, enum nrf_modem_dect_phy_err err,
                   const struct nrf_modem_dect_phy_capability *capability);

void stf_cover_seq_control(const uint64_t *time, enum nrf_modem_dect_phy_err err);

#endif /* DECT_NR_H */