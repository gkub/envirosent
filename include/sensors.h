#ifndef SENSORS_H
#define SENSORS_H

#include <stdbool.h>

bool bh1750_init(void);
bool bh1750_read_lux(float *lux_out);

bool bme280_init(void);
bool bme280_read_environment(float *temp_out, float *hum_out, float *press_out);

#endif