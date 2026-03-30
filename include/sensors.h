#ifndef SENSORS_H
#define SENSORS_H

#include <stdbool.h>

// Initialize the BH1750 light sensor.
// Returns true if initialization succeeded.
bool bh1750_init(void);

// Read current lux value from BH1750.
// Writes result into *lux_out and returns true on success.
bool bh1750_read_lux(float *lux_out);

// Initialize the BME280 environmental sensor.
// This will:
//   - verify the chip ID
//   - read factory calibration coefficients
//   - configure oversampling / mode registers
bool bme280_init(void);

// Read temperature / humidity / pressure from the BME280.
// Writes results into the provided output pointers.
// Returns true on success, false on failure.
bool bme280_read_environment(float *temp_out, float *hum_out, float *press_out);

#endif