#include "sensors.h"          // our own public sensor API

#include <stddef.h>           // for NULL
#include <stdint.h>           // for uint8_t, uint16_t

#include "driver/i2c.h"       // ESP-IDF I2C driver API
#include "esp_err.h"          // ESP_OK and esp_err_t
#include "freertos/FreeRTOS.h"// pdMS_TO_TICKS()

/*
 * These constants are private implementation details of this sensor module.
 * They do not belong in main.c, because main.c should not care about BH1750
 * device addresses or command bytes.
 */

// The I2C peripheral instance we are using.
// In our i2cscanner.c we initialized I2C_NUM_0, so we use the same port here.
#define I2C_PORT I2C_NUM_0

// How long we allow an I2C transaction to take before timing out.
#define I2C_TIMEOUT_TICKS pdMS_TO_TICKS(100)

// BH1750 7-bit I2C address.
// Our scanner found the device at 0x23, which is the common default.
#define BH1750_ADDR 0x23

// BH1750 command: continuous high-resolution mode.
// This tells the sensor to continuously measure ambient light with good precision.
#define BH1750_CONT_HIRES_MODE 0x10

bool bh1750_init(void)
{
    /*
     * We create an I2C command link object.
     *
     * Think of this as a small "script" or "transaction builder" that we fill with:
     *   START
     *   address + write
     *   command byte
     *   STOP
     *
     * Then we ask the ESP32 I2C hardware to execute that script.
     */
    i2c_cmd_handle_t cmd = i2c_cmd_link_create(); // create a command link (a list of I2C commands which will be executed sequentially)

    // If allocation failed, bail out.
    if (cmd == NULL) {
        return false;
    }

    // START condition on the I2C bus.
    i2c_master_start(cmd);

    /*
     * Send the address byte in WRITE mode.
     *
     * Why (BH1750_ADDR << 1)?
     * Because the I2C bus transmits an 8-bit address byte where:
     *   bits 7..1 = 7-bit slave address
     *   bit 0     = read/write bit
     *
     * So we shift the 7-bit address left by 1 to make room for the R/W bit,
     * then OR in I2C_MASTER_WRITE (which is 0).
     *
     * The final 'true' means: we expect the slave to ACK this byte.
     */
    i2c_master_write_byte(cmd, (BH1750_ADDR << 1) | I2C_MASTER_WRITE, true);

    /*
     * Send the BH1750 measurement mode command byte.
     *
     * 0x10 = continuous high-resolution mode.
     *
     * Again, 'true' means we expect the sensor to acknowledge reception.
     */
    i2c_master_write_byte(cmd, BH1750_CONT_HIRES_MODE, true);

    // STOP condition on the I2C bus.
    i2c_master_stop(cmd);

    /*
     * Actually execute the queued transaction on the hardware.
     *
     * This is the point where the ESP32 I2C peripheral really drives the bus.
     */
    esp_err_t ret = i2c_master_cmd_begin(I2C_PORT, cmd, I2C_TIMEOUT_TICKS);

    // Always clean up the command link after use.
    i2c_cmd_link_delete(cmd);

    // Return true only if the transaction fully succeeded.
    return (ret == ESP_OK);
}

bool bh1750_read_lux(float *lux_out)
{
    /*
     * If the caller passed a null pointer, we cannot write the result anywhere.
     * Returning false is safer than crashing by dereferencing NULL.
     */
    if (lux_out == NULL) {
        return false;
    }

    /*
     * Buffer to hold the two bytes returned by the BH1750.
     *
     * The sensor returns a 16-bit raw measurement split across two bytes:
     *   data[0] = high byte
     *   data[1] = low byte
     */
    uint8_t data[2] = {0};

    // Create a new command link for this read transaction.
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    if (cmd == NULL) {
        return false;
    }

    // START condition.
    i2c_master_start(cmd);

    /*
     * Send the address byte in READ mode this time.
     *
     * Same address-shift idea as before, but now we OR in I2C_MASTER_READ (which is 1).
     */
    i2c_master_write_byte(cmd, (BH1750_ADDR << 1) | I2C_MASTER_READ, true);

    /*
     * Read the first byte and send ACK.
     *
     * Why ACK?
     * Because after receiving the first byte, we want to tell the sensor:
     *   "Yes, keep going, I want another byte."
     */
    i2c_master_read_byte(cmd, &data[0], I2C_MASTER_ACK);

    /*
     * Read the second (last) byte and send NACK.
     *
     * Why NACK?
     * Because on the last byte we tell the sensor:
     *   "I'm done reading now."
     */
    i2c_master_read_byte(cmd, &data[1], I2C_MASTER_NACK);

    // STOP condition.
    i2c_master_stop(cmd);

    // Execute the read transaction.
    esp_err_t ret = i2c_master_cmd_begin(I2C_PORT, cmd, I2C_TIMEOUT_TICKS);

    // Clean up the command link.
    i2c_cmd_link_delete(cmd);

    // If the bus transaction failed, return false.
    if (ret != ESP_OK) {
        return false;
    }

    /*
     * Reassemble the two bytes into one 16-bit raw value.
     *
     * data[0] is the high byte, so it needs to be shifted up by 8 bits.
     * data[1] is the low byte, so it stays as-is.
     *
     * Example:
     *   data[0] = 0x01
     *   data[1] = 0x20
     *   raw     = 0x0120
     */
    uint16_t raw = ((uint16_t)data[0] << 8) | data[1];

    /*
     * Convert raw reading to lux.
     *
     * For the standard BH1750 continuous high-resolution mode,
     * the common conversion is:
     *   lux = raw / 1.2
     *
     * 1.2f is a float literal, so we do floating-point division rather than integer division.
     */
    *lux_out = (float)raw / 1.2f;

    return true;
}

/*
 * These are still stubbed for now so the overall program continues to work
 * while we bring one sensor online at a time.
 */

bool bme280_init(void)
{
    return true;
}

bool bme280_read_environment(float *temp_out, float *hum_out, float *press_out)
{
    // Defensive checks in case caller passed bad pointers.
    if (temp_out == NULL || hum_out == NULL || press_out == NULL) {
        return false;
    }

    // Placeholder values until we implement the real BME280 driver.
    *temp_out = 20.0f;
    *hum_out = 50.0f;
    *press_out = 1013.0f;

    return true;
}