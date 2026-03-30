#include "sensors.h"

#include <stddef.h>             // for NULL
#include <stdint.h>             // for uint8_t, uint16_t, int32_t, etc.

#include "driver/i2c.h"         // ESP-IDF I2C driver API
#include "esp_err.h"            // esp_err_t and ESP_OK
#include "freertos/FreeRTOS.h"  // pdMS_TO_TICKS()
#include "freertos/task.h"      // vTaskDelay()

/*
 * ============================================================================
 * I2C bus / timeout configuration
 * ============================================================================
 *
 * These values match the I2C peripheral we already initialized in i2cscanner.c.
 * Our bus is using I2C_NUM_0 on GPIO 21/22, so we use I2C_NUM_0 here too.
 */

#define I2C_PORT          I2C_NUM_0
#define I2C_TIMEOUT_TICKS pdMS_TO_TICKS(100)

/*
 * ============================================================================
 * BH1750 constants
 * ============================================================================
 */

#define BH1750_ADDR            0x23
#define BH1750_CONT_HIRES_MODE 0x10

/*
 * ============================================================================
 * BME280 I2C address and register map
 * ============================================================================
 *
 * BME280 can appear at 0x76 or 0x77 depending on SDO wiring.
 * Our scan showed 0x76, so we use that.
 */

#define BME280_ADDR 0x76

// Identification / reset registers
#define BME280_REG_CHIP_ID 0xD0
#define BME280_REG_RESET   0xE0

// Control / config registers
#define BME280_REG_CTRL_HUM  0xF2
#define BME280_REG_STATUS    0xF3
#define BME280_REG_CTRL_MEAS 0xF4
#define BME280_REG_CONFIG    0xF5

// Data registers begin here
#define BME280_REG_PRESS_MSB 0xF7

// Expected chip ID value for BME280
#define BME280_CHIP_ID 0x60

// Soft reset command value
#define BME280_RESET_CMD 0xB6

/*
 * ============================================================================
 * BME280 calibration storage
 * ============================================================================
 *
 * Bosch stores calibration coefficients inside the sensor at the factory.
 * We read them once during init and keep them in this static struct.
 *
 * Why static?
 * Because calibration belongs to the sensor module, not to main.c.
 * main.c should not care about these details.
 */

typedef struct {
    // Temperature calibration coefficients
    uint16_t dig_T1;
    int16_t  dig_T2;
    int16_t  dig_T3;

    // Pressure calibration coefficients
    uint16_t dig_P1;
    int16_t  dig_P2;
    int16_t  dig_P3;
    int16_t  dig_P4;
    int16_t  dig_P5;
    int16_t  dig_P6;
    int16_t  dig_P7;
    int16_t  dig_P8;
    int16_t  dig_P9;

    // Humidity calibration coefficients
    uint8_t  dig_H1;
    int16_t  dig_H2;
    uint8_t  dig_H3;
    int16_t  dig_H4;
    int16_t  dig_H5;
    int8_t   dig_H6;
} bme280_calib_data_t;

// Static module-level calibration storage
static bme280_calib_data_t bme280_calib;

// Bosch’s compensation formulas use an intermediate variable called t_fine.
// It is derived from the temperature calculation and then reused for pressure/humidity.
static int32_t bme280_t_fine;

/*
 * ============================================================================
 * Private helper: write one byte to a sensor register
 * ============================================================================
 *
 * This function performs:
 *
 *   START
 *   address + write
 *   register address
 *   value byte
 *   STOP
 *
 * and returns true if the transaction succeeded.
 *
 * This helper is kept private to sensors.c because register-level access
 * is an implementation detail of the sensor module.
 */
static bool i2c_write_reg(uint8_t dev_addr, uint8_t reg_addr, uint8_t value)
{
    // Create a new command link ("transaction script")
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    if (cmd == NULL) {
        return false;
    }

    // START condition on the bus
    i2c_master_start(cmd);

    // Send device address with WRITE bit
    i2c_master_write_byte(cmd, (dev_addr << 1) | I2C_MASTER_WRITE, true);

    // Send the register address we want to write to
    i2c_master_write_byte(cmd, reg_addr, true);

    // Send the byte value to store into that register
    i2c_master_write_byte(cmd, value, true);

    // STOP condition
    i2c_master_stop(cmd);

    // Execute the full transaction on the I2C peripheral
    esp_err_t ret = i2c_master_cmd_begin(I2C_PORT, cmd, I2C_TIMEOUT_TICKS);

    // Free the command link structure
    i2c_cmd_link_delete(cmd);

    return (ret == ESP_OK);
}

/*
 * ============================================================================
 * Private helper: read one or more bytes starting from a register
 * ============================================================================
 *
 * This function performs a common I2C register-read sequence:
 *
 *   START
 *   address + write
 *   register address
 *   REPEATED START
 *   address + read
 *   read N bytes
 *   STOP
 *
 * Why repeated START?
 * Because many I2C devices expect:
 *   first, tell me which register we want
 *   then, without releasing the bus, switch into read mode
 *
 * 'data' is the destination buffer
 * 'len' is the number of bytes to read
 */
static bool i2c_read_regs(uint8_t dev_addr, uint8_t start_reg, uint8_t *data, size_t len)
{
    // Defensive checks
    if (data == NULL || len == 0) {
        return false;
    }

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    if (cmd == NULL) {
        return false;
    }

    // START
    i2c_master_start(cmd);

    // Send device address with WRITE bit so we can specify the register address
    i2c_master_write_byte(cmd, (dev_addr << 1) | I2C_MASTER_WRITE, true);

    // Send the starting register address
    i2c_master_write_byte(cmd, start_reg, true);

    // REPEATED START: switch from "write register address" to "read data"
    i2c_master_start(cmd);

    // Send device address with READ bit
    i2c_master_write_byte(cmd, (dev_addr << 1) | I2C_MASTER_READ, true);

    /*
     * Read bytes.
     *
     * For every byte except the last, send ACK to say "I want more".
     * On the final byte, send NACK to say "I'm done".
     */
    for (size_t i = 0; i < len; i++) {
        i2c_ack_type_t ack = (i == (len - 1)) ? I2C_MASTER_NACK : I2C_MASTER_ACK;
        i2c_master_read_byte(cmd, &data[i], ack);
    }

    // STOP
    i2c_master_stop(cmd);

    // Execute the transaction
    esp_err_t ret = i2c_master_cmd_begin(I2C_PORT, cmd, I2C_TIMEOUT_TICKS);

    // Free the command link
    i2c_cmd_link_delete(cmd);

    return (ret == ESP_OK);
}

/*
 * ============================================================================
 * BH1750 implementation
 * ============================================================================
 */

 bool bh1750_init(void)
 {
     // Create command link
     i2c_cmd_handle_t cmd = i2c_cmd_link_create();
     if (cmd == NULL) {
         return false;
     }
 
     // START
     i2c_master_start(cmd);
 
     // Device address + WRITE
     i2c_master_write_byte(cmd, (BH1750_ADDR << 1) | I2C_MASTER_WRITE, true);
 
     // Command byte: continuous high-resolution mode
     i2c_master_write_byte(cmd, BH1750_CONT_HIRES_MODE, true);
 
     // STOP
     i2c_master_stop(cmd);
 
     // Execute transaction
     esp_err_t ret = i2c_master_cmd_begin(I2C_PORT, cmd, I2C_TIMEOUT_TICKS);
 
     // Clean up
     i2c_cmd_link_delete(cmd);
 
     return (ret == ESP_OK);
 }
 
 bool bh1750_read_lux(float *lux_out)
 {
     if (lux_out == NULL) {
         return false;
     }
 
     // Two-byte read buffer
     uint8_t data[2] = {0};
 
     // Create command link
     i2c_cmd_handle_t cmd = i2c_cmd_link_create();
     if (cmd == NULL) {
         return false;
     }
 
     // START
     i2c_master_start(cmd);
 
     // Device address + READ
     i2c_master_write_byte(cmd, (BH1750_ADDR << 1) | I2C_MASTER_READ, true);
 
     // Read first byte, ACK because we want a second byte
     i2c_master_read_byte(cmd, &data[0], I2C_MASTER_ACK);
 
     // Read second byte, NACK because we are done
     i2c_master_read_byte(cmd, &data[1], I2C_MASTER_NACK);
 
     // STOP
     i2c_master_stop(cmd);
 
     // Execute transaction
     esp_err_t ret = i2c_master_cmd_begin(I2C_PORT, cmd, I2C_TIMEOUT_TICKS);
 
     // Free command link
     i2c_cmd_link_delete(cmd);
 
     if (ret != ESP_OK) {
         return false;
     }
 
     // Reconstruct 16-bit raw value from two bytes
     uint16_t raw = ((uint16_t)data[0] << 8) | data[1];
 
     // Convert to lux
     *lux_out = (float)raw / 1.2f;
 
     return true;
 }

 /*
 * ============================================================================
 * Private helper: little-endian register decoding helpers
 * ============================================================================
 *
 * BME280 stores many calibration values in little-endian form:
 *   low byte first, then high byte.
 *
 * These helpers let us reconstruct unsigned/signed 16-bit values from raw bytes.
 */
static uint16_t u16_le(const uint8_t *buf)
{
    return (uint16_t)buf[0] | ((uint16_t)buf[1] << 8);
}

static int16_t s16_le(const uint8_t *buf)
{
    return (int16_t)u16_le(buf);
}

/*
 * ============================================================================
 * Private helper: read BME280 calibration registers
 * ============================================================================
 *
 * Calibration registers are split across two regions:
 *   0x88..0xA1
 *   0xE1..0xE7
 *
 * We read both blocks, then unpack the coefficients.
 */
static bool bme280_read_calibration(void)
{
    // First block contains T, P, and H1 coefficients
    uint8_t calib1[26] = {0};

    // Second block contains remaining humidity coefficients
    uint8_t calib2[7] = {0};

    if (!i2c_read_regs(BME280_ADDR, 0x88, calib1, sizeof(calib1))) {
        return false;
    }

    if (!i2c_read_regs(BME280_ADDR, 0xE1, calib2, sizeof(calib2))) {
        return false;
    }

    // Temperature calibration
    bme280_calib.dig_T1 = u16_le(&calib1[0]);
    bme280_calib.dig_T2 = s16_le(&calib1[2]);
    bme280_calib.dig_T3 = s16_le(&calib1[4]);

    // Pressure calibration
    bme280_calib.dig_P1 = u16_le(&calib1[6]);
    bme280_calib.dig_P2 = s16_le(&calib1[8]);
    bme280_calib.dig_P3 = s16_le(&calib1[10]);
    bme280_calib.dig_P4 = s16_le(&calib1[12]);
    bme280_calib.dig_P5 = s16_le(&calib1[14]);
    bme280_calib.dig_P6 = s16_le(&calib1[16]);
    bme280_calib.dig_P7 = s16_le(&calib1[18]);
    bme280_calib.dig_P8 = s16_le(&calib1[20]);
    bme280_calib.dig_P9 = s16_le(&calib1[22]);

    // Humidity calibration
    bme280_calib.dig_H1 = calib1[25];
    bme280_calib.dig_H2 = s16_le(&calib2[0]);
    bme280_calib.dig_H3 = calib2[2];

    /*
     * H4 and H5 are packed awkwardly across byte boundaries:
     *
     * H4 = (E4 << 4) | (E5 & 0x0F)
     * H5 = (E6 << 4) | (E5 >> 4)
     *
     * This is one of the annoying parts of BME280.
     */
    bme280_calib.dig_H4 = (int16_t)((calib2[3] << 4) | (calib2[4] & 0x0F));
    bme280_calib.dig_H5 = (int16_t)((calib2[5] << 4) | (calib2[4] >> 4));
    bme280_calib.dig_H6 = (int8_t)calib2[6];

    return true;
}

/*
 * ============================================================================
 * BME280 compensation functions
 * ============================================================================
 *
 * These are based on Bosch’s standard compensation formulas.
 *
 * Why are these needed?
 * Because the raw ADC values from the sensor are not directly meaningful in
 * human units; they must be adjusted using the factory calibration coefficients.
 *
 * The formulas use 't_fine', which comes from the temperature compensation and
 * is then reused for pressure and humidity compensation.
 */

// Compensate raw temperature and return Celsius
static float bme280_compensate_temperature(int32_t adc_T)
{
    int32_t var1, var2;

    var1 = ((((adc_T >> 3) - ((int32_t)bme280_calib.dig_T1 << 1))) *
            ((int32_t)bme280_calib.dig_T2)) >> 11;

    var2 = (((((adc_T >> 4) - ((int32_t)bme280_calib.dig_T1)) *
              ((adc_T >> 4) - ((int32_t)bme280_calib.dig_T1))) >> 12) *
            ((int32_t)bme280_calib.dig_T3)) >> 14;

    bme280_t_fine = var1 + var2;

    int32_t T = (bme280_t_fine * 5 + 128) >> 8;

    // Datasheet formula gives temperature in 0.01 deg C, so divide by 100
    return (float)T / 100.0f;
}

// Compensate raw pressure and return hPa
static float bme280_compensate_pressure(int32_t adc_P)
{
    int64_t var1, var2, p;

    var1 = ((int64_t)bme280_t_fine) - 128000;
    var2 = var1 * var1 * (int64_t)bme280_calib.dig_P6;
    var2 = var2 + ((var1 * (int64_t)bme280_calib.dig_P5) << 17);
    var2 = var2 + (((int64_t)bme280_calib.dig_P4) << 35);
    var1 = ((var1 * var1 * (int64_t)bme280_calib.dig_P3) >> 8) +
           ((var1 * (int64_t)bme280_calib.dig_P2) << 12);
    var1 = (((((int64_t)1) << 47) + var1)) * ((int64_t)bme280_calib.dig_P1) >> 33;

    // Avoid divide-by-zero if calibration data is invalid
    if (var1 == 0) {
        return 0.0f;
    }

    p = 1048576 - adc_P;
    p = (((p << 31) - var2) * 3125) / var1;
    var1 = (((int64_t)bme280_calib.dig_P9) * (p >> 13) * (p >> 13)) >> 25;
    var2 = (((int64_t)bme280_calib.dig_P8) * p) >> 19;

    p = ((p + var1 + var2) >> 8) + (((int64_t)bme280_calib.dig_P7) << 4);

    // Result here is in Pa * 256, so divide appropriately to get hPa
    return (float)p / 25600.0f;
}

// Compensate raw humidity and return %RH
static float bme280_compensate_humidity(int32_t adc_H)
{
    int32_t v_x1_u32r;

    v_x1_u32r = bme280_t_fine - ((int32_t)76800);

    v_x1_u32r = (((((adc_H << 14) - (((int32_t)bme280_calib.dig_H4) << 20) -
                    (((int32_t)bme280_calib.dig_H5) * v_x1_u32r)) + ((int32_t)16384)) >> 15) *
                  (((((((v_x1_u32r * ((int32_t)bme280_calib.dig_H6)) >> 10) *
                       (((v_x1_u32r * ((int32_t)bme280_calib.dig_H3)) >> 11) + ((int32_t)32768))) >> 10) +
                     ((int32_t)2097152)) * ((int32_t)bme280_calib.dig_H2) + 8192) >> 14));

    v_x1_u32r = v_x1_u32r -
                (((((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7) *
                  ((int32_t)bme280_calib.dig_H1)) >> 4);

    if (v_x1_u32r < 0) {
        v_x1_u32r = 0;
    }

    if (v_x1_u32r > 419430400) {
        v_x1_u32r = 419430400;
    }

    // Datasheet formula gives humidity in Q22.10 format
    return (float)(v_x1_u32r >> 12) / 1024.0f;
}

/*
 * ============================================================================
 * BME280 init
 * ============================================================================
 */
bool bme280_init(void)
{
    uint8_t chip_id = 0;

    // Read chip ID register and verify device identity
    if (!i2c_read_regs(BME280_ADDR, BME280_REG_CHIP_ID, &chip_id, 1)) {
        return false;
    }

    if (chip_id != BME280_CHIP_ID) {
        return false;
    }

    // Soft reset the sensor
    if (!i2c_write_reg(BME280_ADDR, BME280_REG_RESET, BME280_RESET_CMD)) {
        return false;
    }

    // Give sensor a little time to reset internally
    vTaskDelay(pdMS_TO_TICKS(10));

    // Read factory calibration coefficients
    if (!bme280_read_calibration()) {
        return false;
    }

    /*
     * Configure humidity oversampling first.
     * This must be written before ctrl_meas for the setting to take effect.
     *
     * 0x01 = oversampling x1
     */
    if (!i2c_write_reg(BME280_ADDR, BME280_REG_CTRL_HUM, 0x01)) {
        return false;
    }

    /*
     * Configure temperature oversampling x1, pressure oversampling x1,
     * and normal mode.
     *
     * ctrl_meas format:
     *   osrs_t[7:5] | osrs_p[4:2] | mode[1:0]
     *
     * Here:
     *   osrs_t = 001 (x1)
     *   osrs_p = 001 (x1)
     *   mode   = 11  (normal mode)
     *
     * That gives:
     *   001 001 11 = 0x27
     */
    if (!i2c_write_reg(BME280_ADDR, BME280_REG_CTRL_MEAS, 0x27)) {
        return false;
    }

    /*
     * Optional config register:
     * standby/filter settings.
     * 0x00 is a simple default that keeps things straightforward.
     */
    if (!i2c_write_reg(BME280_ADDR, BME280_REG_CONFIG, 0x00)) {
        return false;
    }

    return true;
}

/*
 * ============================================================================
 * BME280 read environment
 * ============================================================================
 */
bool bme280_read_environment(float *temp_out, float *hum_out, float *press_out)
{
    // Validate caller pointers before dereferencing them
    if (temp_out == NULL || hum_out == NULL || press_out == NULL) {
        return false;
    }

    /*
     * Read 8 bytes starting at pressure MSB register (0xF7):
     *
     *   0xF7 pressure msb
     *   0xF8 pressure lsb
     *   0xF9 pressure xlsb
     *   0xFA temp msb
     *   0xFB temp lsb
     *   0xFC temp xlsb
     *   0xFD hum msb
     *   0xFE hum lsb
     */
    uint8_t data[8] = {0};

    if (!i2c_read_regs(BME280_ADDR, BME280_REG_PRESS_MSB, data, sizeof(data))) {
        return false;
    }

    /*
     * Reconstruct 20-bit raw pressure and temperature values.
     *
     * The xlsb registers contain only the upper 4 bits of the final nibble.
     */
    int32_t adc_P = ((int32_t)data[0] << 12) |
                    ((int32_t)data[1] << 4)  |
                    ((int32_t)data[2] >> 4);

    int32_t adc_T = ((int32_t)data[3] << 12) |
                    ((int32_t)data[4] << 4)  |
                    ((int32_t)data[5] >> 4);

    // Humidity is a 16-bit value
    int32_t adc_H = ((int32_t)data[6] << 8) | data[7];

    /*
     * Compensation order matters:
     * temperature must be compensated first because it computes t_fine,
     * which pressure and humidity compensation both rely on.
     */
    *temp_out  = bme280_compensate_temperature(adc_T);
    *press_out = bme280_compensate_pressure(adc_P);
    *hum_out   = bme280_compensate_humidity(adc_H);

    return true;
}