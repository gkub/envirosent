#ifndef SENSORS_H
#define SENSORS_H

#include <stdbool.h>

// Initialize the BH1750 light sensor.
// Returns true if the sensor acknowledged the init command.
bool bh1750_init(void);

// Read the current lux value from the BH1750.
// Writes the lux value into *lux_out on success.
// Returns true on success, false on failure.
bool bh1750_read_lux(float *lux_out);

// Stubbed for now. Later these will become real.
bool bme280_init(void);
bool bme280_read_environment(float *temp_out, float *hum_out, float *press_out);

#endif