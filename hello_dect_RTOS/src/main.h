#ifndef MAIN_H
#define MAIN_H

#include <zephyr/kernel.h>
#include <nrf_modem_dect_phy.h>

/* External declarations for global variables/resources used across files */
extern struct k_sem operation_sem;
extern struct k_mutex temp_mutex;
extern float g_current_temperature_c;
extern uint16_t device_id;

/* Thread entry point declarations */
void sensor_thread_entry(void *p1, void *p2, void *p3);
void dect_thread_entry(void *p1, void *p2, void *p3);

#endif /* MAIN_H */