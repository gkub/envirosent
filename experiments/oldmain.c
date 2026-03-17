
// Environment sensor + OLED demo using ESP-IDF on ESP32.
// -------------------------------------------------------
// Hardware:
//   - ESP32
//   - 0.96" 128x64 I2C SSD1306 OLED (IZOKEE module)
//   - BH1750 I2C light sensor module (ambient light, lux)
//   - SCD41 I2C sensor module (CO2, temperature, humidity)
//
// I2C wiring (shared bus):
//   GPIO 21 -> SDA  -> OLED SDA, BH1750 SDA, SCD41 SDA
//   GPIO 22 -> SCL  -> OLED SCL, BH1750 SCL, SCD41 SCL
//   3.3 V   -> VCC  on all modules (SCD41 VDD)
//   GND     -> GND  on all modules
//   BH1750 ADDR -> GND (address 0x23; some boards use 0x5C instead)
//   SCD41: fixed I2C address 0x62
//
// What this program does:
//   - Initializes I2C, SSD1306 OLED, and BH1750
//   - Periodically reads the light level in lux from BH1750
//   - Prints the lux value to the serial console
//   - Shows the current lux value on the OLED display
//
// Structure (good for learning FreeRTOS):
//   - One place configures hardware (I2C + devices)
//   - One task reads the sensor and updates a shared variable
//   - One task uses that value (serial + OLED)

#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/i2c.h"
#include "esp_err.h"
#include "esp_system.h"

// ---------- I2C bus configuration ----------
// Same two pins drive both the OLED and the BH1750 on the shared I2C bus.

#define I2C_MASTER_SCL_IO 22
#define I2C_MASTER_SDA_IO 21
#define I2C_MASTER_NUM    I2C_NUM_0
#define I2C_MASTER_FREQ_HZ 400000   // 400 kHz; fine for OLED and sensor

// ---------- OLED (SSD1306) configuration ----------

#define SSD1306_ADDR    0x3C       // Common I2C address for 0.96" modules
#define SSD1306_WIDTH   128
#define SSD1306_HEIGHT  64
#define SSD1306_PAGES   (SSD1306_HEIGHT / 8)   // 8 rows per "page"

// Framebuffer: one byte per column per page (bitmap for the whole screen).
static uint8_t s_framebuffer[SSD1306_WIDTH * SSD1306_PAGES];

// ---------- BH1750 light sensor configuration ----------
// err=263 = ESP_ERR_TIMEOUT (no ACK). We detect address at startup via I2C scan.
// ADDR to GND -> 0x23; ADDR to VCC -> 0x5C.
#define BH1750_CMD_CONTINUOUS_H_RES  0x10
// Runtime address: set by I2C scan (0 = not found).
static uint8_t s_bh1750_addr = 0;

// ---------- SCD41 (CO2) sensor configuration ----------
// SCD41 uses a fixed 7‑bit I2C address 0x62. It measures:
//   - CO2 concentration in ppm
//   - Temperature in °C
//   - Relative humidity in %
#define SCD41_ADDR 0x62
// We track whether the SCD41 answered on the I2C scan.
static bool s_scd41_present = false;

// ---------- Shared data between tasks ----------
// One task writes, the other reads. "volatile" so the compiler doesn't optimize away reads.
static volatile float g_latest_lux      = -1.0f;
static volatile float g_latest_co2_ppm  = -1.0f;

// ---------- Helpers: error logging and I2C init ----------

static void log_if_error(const char *label, esp_err_t err)
{
    if (err != ESP_OK)
        printf("%s failed: err=%d\n", label, err);
}

static void i2c_master_init(void)
{
    i2c_config_t conf = {
        .mode             = I2C_MODE_MASTER,
        .sda_io_num       = I2C_MASTER_SDA_IO,
        .scl_io_num       = I2C_MASTER_SCL_IO,
        .sda_pullup_en    = GPIO_PULLUP_ENABLE,
        .scl_pullup_en    = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };

    log_if_error("i2c_param_config", i2c_param_config(I2C_MASTER_NUM, &conf));
    log_if_error("i2c_driver_install",
                 i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0));
}

// Scan I2C bus 0x08..0x77; print every address that ACKs.
// Also:
//   - sets s_bh1750_addr to 0x23 or 0x5C if present
//   - flags s_scd41_present if 0x62 is seen
static void i2c_scan_and_find_bh1750(void)
{
    printf("\n=== I2C bus scan (GPIO21=SDA, GPIO22=SCL) ===\n");
    for (uint8_t addr = 0x08; addr <= 0x77; addr++) {
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
        i2c_master_stop(cmd);
        esp_err_t e = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(50));
        i2c_cmd_link_delete(cmd);
        if (e == ESP_OK)
        {
            printf("  device at 0x%02X\n", addr);

            if (addr == 0x23)
            {
                s_bh1750_addr = 0x23;
            }
            else if (addr == 0x5C && s_bh1750_addr == 0)
            {
                s_bh1750_addr = 0x5C;
            }
            else if (addr == SCD41_ADDR)
            {
                s_scd41_present = true;
            }
        }
    }
    if (s_bh1750_addr != 0)
        printf("BH1750 -> using 0x%02X (lux readings will run)\n", s_bh1750_addr);
    else
        printf("BH1750 -> NOT FOUND. Only 0x3C? That is the OLED. Check sensor wiring/power.\n");
    printf("==========================================\n\n");
}

// ---------- SSD1306 OLED: low-level write and init ----------
// I2C protocol: after the device address, first byte is "control".
// 0x00 = next byte(s) are commands; 0x40 = next byte(s) are display data.

static void ssd1306_write_command(uint8_t cmd)
{
    uint8_t payload[2] = { 0x00, cmd };
    log_if_error("ssd1306_write_command",
                 i2c_master_write_to_device(I2C_MASTER_NUM, SSD1306_ADDR,
                                           payload, 2, pdMS_TO_TICKS(100)));
}

static void ssd1306_write_data(const uint8_t *data, size_t len)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (SSD1306_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, 0x40, true);   // data mode
    i2c_master_write(cmd, (uint8_t *)data, len, true);
    i2c_master_stop(cmd);
    log_if_error("ssd1306_write_data",
                 i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(100)));
    i2c_cmd_link_delete(cmd);
}

static void ssd1306_init(void)
{
    vTaskDelay(pdMS_TO_TICKS(100));   // power-up settling

    ssd1306_write_command(0xAE);      // DISPLAYOFF
    ssd1306_write_command(0x20);
    ssd1306_write_command(0x00);       // horizontal addressing
    ssd1306_write_command(0xC8);      // COMSCANDEC
    ssd1306_write_command(0x00);
    ssd1306_write_command(0x10);      // column range
    ssd1306_write_command(0x40);      // start line 0
    ssd1306_write_command(0x81);
    ssd1306_write_command(0x7F);      // contrast
    ssd1306_write_command(0xA1);      // SEGREMAP
    ssd1306_write_command(0xA6);      // normal (non-inverted)
    ssd1306_write_command(0xA8);
    ssd1306_write_command(SSD1306_HEIGHT - 1);  // multiplex
    ssd1306_write_command(0xA4);      // DISPLAYALLON_RESUME
    ssd1306_write_command(0xD3);
    ssd1306_write_command(0x00);      // display offset
    ssd1306_write_command(0xD5);
    ssd1306_write_command(0x80);      // clock div
    ssd1306_write_command(0xD9);
    ssd1306_write_command(0xF1);     // precharge
    ssd1306_write_command(0xDA);
    ssd1306_write_command(0x12);      // comp pins
    ssd1306_write_command(0xDB);
    ssd1306_write_command(0x40);     // Vcomh
    ssd1306_write_command(0x8D);
    ssd1306_write_command(0x14);      // charge pump on
    ssd1306_write_command(0xAF);     // DISPLAYON
}

static void ssd1306_clear_buffer(void)
{
    memset(s_framebuffer, 0, sizeof(s_framebuffer));
}

static void ssd1306_flush(void)
{
    ssd1306_write_command(0x21);      // COLUMNADDR
    ssd1306_write_command(0);
    ssd1306_write_command(SSD1306_WIDTH - 1);
    ssd1306_write_command(0x22);      // PAGEADDR
    ssd1306_write_command(0);
    ssd1306_write_command(SSD1306_PAGES - 1);
    ssd1306_write_data(s_framebuffer, sizeof(s_framebuffer));
}

// Set or clear one pixel in the framebuffer. (x,y) in pixel coordinates.
static void ssd1306_draw_pixel(int x, int y, bool on)
{
    if (x < 0 || x >= SSD1306_WIDTH || y < 0 || y >= SSD1306_HEIGHT)
        return;
    int page = y / 8;
    int bit  = y % 8;
    uint8_t *byte = &s_framebuffer[page * SSD1306_WIDTH + x];
    if (on)
        *byte |= (1u << bit);
    else
        *byte &= ~(1u << bit);
}

// ---------- Tiny font: digits, '.', 'L', 'u', 'x' (5 cols x 7 rows) ----------
typedef struct { char ch; uint8_t cols[5]; } small_glyph_t;

static const small_glyph_t s_font[] = {
    {'0', {0x3E, 0x51, 0x49, 0x45, 0x3E}}, {'1', {0x00, 0x42, 0x7F, 0x40, 0x00}},
    {'2', {0x42, 0x61, 0x51, 0x49, 0x46}}, {'3', {0x21, 0x41, 0x45, 0x4B, 0x31}},
    {'4', {0x18, 0x14, 0x12, 0x7F, 0x10}}, {'5', {0x27, 0x45, 0x45, 0x45, 0x39}},
    {'6', {0x3C, 0x4A, 0x49, 0x49, 0x30}}, {'7', {0x01, 0x71, 0x09, 0x05, 0x03}},
    {'8', {0x36, 0x49, 0x49, 0x49, 0x36}}, {'9', {0x06, 0x49, 0x49, 0x29, 0x1E}},
    {'.', {0x00, 0x60, 0x60, 0x00, 0x00}}, {'L', {0x7F, 0x40, 0x40, 0x40, 0x40}},
    {'u', {0x38, 0x40, 0x40, 0x20, 0x78}}, {'x', {0x44, 0x28, 0x10, 0x28, 0x44}},
};

static const small_glyph_t *find_glyph(char c)
{
    for (size_t i = 0; i < sizeof(s_font) / sizeof(s_font[0]); i++)
        if (s_font[i].ch == c)
            return &s_font[i];
    return NULL;
}

static void ssd1306_draw_char(int x, int y_top, char c)
{
    const small_glyph_t *g = find_glyph(c);
    if (!g) return;
    for (int col = 0; col < 5; col++)
        for (int row = 0; row < 7; row++)
            ssd1306_draw_pixel(x + col, y_top + row, (g->cols[col] >> row) & 1);
}

static void ssd1306_draw_string(int x, int y_top, const char *text)
{
    int cx = x;
    while (*text) {
        ssd1306_draw_char(cx, y_top, *text++);
        cx += 6;   // 5 px glyph + 1 px space
    }
}

// ---------- BH1750: init and read lux ----------
// Use s_bh1750_addr (set by I2C scan). If 0, sensor wasn't found.

static void bh1750_init(void)
{
    if (s_bh1750_addr == 0)
        return;
    uint8_t cmd = BH1750_CMD_CONTINUOUS_H_RES;
    log_if_error("bh1750_init",
                 i2c_master_write_to_device(I2C_MASTER_NUM, s_bh1750_addr,
                                           &cmd, 1, pdMS_TO_TICKS(100)));
    vTaskDelay(pdMS_TO_TICKS(200));   // time for first measurement
}

// Read 2 bytes (high byte, low byte), combine, convert to lux. Returns -1 on error.
static float bh1750_read_lux(void)
{
    if (s_bh1750_addr == 0)
        return -1.0f;
    uint8_t data[2];
    esp_err_t err = i2c_master_read_from_device(I2C_MASTER_NUM, s_bh1750_addr,
                                                data, 2, pdMS_TO_TICKS(100));
    if (err != ESP_OK) {
        log_if_error("bh1750_read_lux", err);
        return -1.0f;
    }
    uint16_t raw = (uint16_t)(data[0] << 8) | data[1];
    return raw / 1.2f;   // datasheet: lux = raw / 1.2
}

// ---------- SCD41: CRC, init and read CO2 ----------
// SCD41 uses a CRC‑8 over each 2‑byte word. Polynomial 0x31, init 0xFF.

static uint8_t scd41_crc8(const uint8_t *data, int len)
{
    uint8_t crc = 0xFF;
    for (int i = 0; i < len; ++i)
    {
        crc ^= data[i];
        for (int bit = 0; bit < 8; ++bit)
        {
            if (crc & 0x80)
                crc = (crc << 1) ^ 0x31;
            else
                crc <<= 1;
        }
    }
    return crc;
}

// Send a 16‑bit command (no arguments) to SCD41.
static void scd41_send_command(uint16_t cmd)
{
    uint8_t buf[2] = { (uint8_t)(cmd >> 8), (uint8_t)(cmd & 0xFF) };
    log_if_error("scd41_send_command",
                 i2c_master_write_to_device(I2C_MASTER_NUM, SCD41_ADDR,
                                            buf, sizeof(buf), pdMS_TO_TICKS(100)));
}

// Start periodic measurement on SCD41 (CO2/temperature/humidity).
// According to the datasheet the first reading will be ready about 5 seconds later.
static void scd41_start_periodic_measurement(void)
{
    if (!s_scd41_present)
        return;

    // 0x21B1 = Start periodic measurement
    scd41_send_command(0x21B1);
}

// Read one measurement from SCD41. Returns CO2 ppm, or -1 on error.
static float scd41_read_co2_ppm(void)
{
    if (!s_scd41_present)
        return -1.0f;

    // 0xEC05 = Read measurement
    scd41_send_command(0xEC05);
    // Max measurement rate is 5 s, but we are calling this much slower,
    // so a short wait here is enough.
    vTaskDelay(pdMS_TO_TICKS(10));

    uint8_t buf[9] = {0};
    esp_err_t err = i2c_master_read_from_device(
        I2C_MASTER_NUM, SCD41_ADDR, buf, sizeof(buf), pdMS_TO_TICKS(100));
    if (err != ESP_OK)
    {
        log_if_error("scd41_read", err);
        return -1.0f;
    }

    // Each value is 2 bytes data + 1 byte CRC.
    uint8_t co2_raw[2] = { buf[0], buf[1] };
    uint8_t co2_crc    = buf[2];
    if (scd41_crc8(co2_raw, 2) != co2_crc)
    {
        printf("scd41_read: CO2 CRC mismatch\n");
        return -1.0f;
    }

    uint16_t co2_ticks = (uint16_t)co2_raw[0] << 8 | co2_raw[1];
    // For CO2, the raw ticks already correspond to ppm.
    return (float)co2_ticks;
}

// ---------- FreeRTOS tasks ----------
// Task 1: read sensor every 500 ms and store result in g_latest_lux (producer).
// Task 2: every 500 ms read g_latest_lux, print to serial, draw on OLED (consumer).

static void sensor_task(void *pvParameters)
{
    (void)pvParameters;
    if (s_bh1750_addr == 0)
        printf("[sensor_task] No BH1750 on bus -> lux will stay -1.00 (see I2C scan above)\n");
    else
        printf("[sensor_task] BH1750 at 0x%02X -> reading lux every 500 ms\n", s_bh1750_addr);

    if (s_scd41_present)
    {
        printf("[sensor_task] SCD41 detected at 0x%02X -> starting periodic CO2 measurement\n", SCD41_ADDR);
        scd41_start_periodic_measurement();
    }
    else
    {
        printf("[sensor_task] No SCD41 on bus -> CO2 will stay -1.00\n");
    }

    // We poll BH1750 every 500 ms, but SCD41 only updates every ~5 seconds.
    // We use a simple counter to read SCD41 less often.
    int scd41_counter = 0;

    for (;;) {
        float lux = bh1750_read_lux();
        if (lux >= 0.0f)
            g_latest_lux = lux;

        if (s_scd41_present)
        {
            scd41_counter++;
            // Every 10 loops * 500 ms = ~5 seconds.
            if (scd41_counter >= 10)
            {
                scd41_counter = 0;
                float co2 = scd41_read_co2_ppm();
                if (co2 >= 0.0f)
                    g_latest_co2_ppm = co2;
            }
        }

        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

static void display_task(void *pvParameters)
{
    (void)pvParameters;
    char line_lux[24];
    char line_co2[24];
    int loop_count = 0;
    for (;;) {
        float lux = g_latest_lux;   // single read of volatile
        float co2 = g_latest_co2_ppm;

        // When no sensor, only print to serial every ~5 s to avoid flooding the console.
        if (s_bh1750_addr != 0 || (++loop_count % 10) == 1)
        {
            printf("[display] Lux: %.2f, CO2: %.0f ppm%s\n",
                   lux,
                   co2,
                   (s_bh1750_addr == 0 ? " (no BH1750 - Type R+Enter to reset, check wiring)" : ""));
        }

        // First line: light level in lux (e.g. "123.4 Lx").
        snprintf(line_lux, sizeof(line_lux), "%.1f Lx", lux);

        // Second line: CO2 in ppm. Our tiny font only has digits and '.', so
        // we draw just the number (no units) and describe it in comments/logs.
        if (co2 >= 0.0f)
            snprintf(line_co2, sizeof(line_co2), "%.0f", co2);
        else
            snprintf(line_co2, sizeof(line_co2), "----");

        ssd1306_clear_buffer();
        // Place lux on the upper half and CO2 value below it.
        ssd1306_draw_string(8, 16, line_lux);
        ssd1306_draw_string(8, 32, line_co2);
        ssd1306_flush();
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

// Listens on serial: type 'r' or 'R' + Enter to reset the board (no unplug needed; handy on WSL).
static void reset_listener_task(void *pvParameters)
{
    (void)pvParameters;
    for (;;) {
        int c = getchar();
        if (c == 'r' || c == 'R')
            esp_restart();
        if (c == EOF)
            vTaskDelay(pdMS_TO_TICKS(100));
    }
}

// ---------- app_main: init hardware, create tasks, return ----------
// After app_main returns, the FreeRTOS scheduler keeps running the tasks.
// We never come back here; the two tasks run until power off.

void app_main(void)
{
    printf("Environment sensor + OLED demo\n");
    printf("Type R + Enter to reset board (no unplug needed)\n");

    i2c_master_init();
    ssd1306_init();
    ssd1306_clear_buffer();
    ssd1306_flush();

    i2c_scan_and_find_bh1750();
    bh1750_init();

    xTaskCreate(sensor_task,         "sensor_task",         4096, NULL, 1, NULL);
    xTaskCreate(display_task,       "display_task",       4096, NULL, 1, NULL);
    xTaskCreate(reset_listener_task, "reset_listener_task", 2048, NULL, 0, NULL);
}
