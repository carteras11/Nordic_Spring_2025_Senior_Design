#ifndef I2C_H
#define I2C_H

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/i2c.h>

/* BME280 Data structure */
struct bme280_data {
    uint16_t dig_t1;
    int16_t dig_t2;
    int16_t dig_t3;
};

/* Function declarations */
int bme280_init(void);
void bme_calibrationdata(void);
int32_t bme280_compensate_temp(struct bme280_data *data, int32_t adc_temp);
void sensor_thread_entry(void *p1, void *p2, void *p3);

/* External declarations */
extern struct bme280_data bmedata;
extern const struct i2c_dt_spec dev_i2c;

#endif /* I2C_H */