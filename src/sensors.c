#include "sensors.h"

bool bh1750_init(void) {
    return true;
}

bool bh1750_read_lux(float *lux_out) {
    *lux_out = 100;
    return true;
}

bool bme280_init(void) {
    return true;
}

bool bme280_read_environment(float *temp_out, float *hum_out, float *press_out) {
    *temp_out = 20;
    *hum_out = 50;
    *press_out = 1013;
    return true;
}