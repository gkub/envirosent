#include "oled.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "driver/i2c.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"

/*
 * ============================================================================
 * SSD1306 I2C configuration
 * ============================================================================
 */

// Same I2C peripheral your project already uses.
#define OLED_I2C_PORT I2C_NUM_0

// Same style of timeout as your other sensors.
#define OLED_I2C_TIMEOUT_TICKS pdMS_TO_TICKS(100)

// Common I2C address for 128x64 SSD1306 modules.
#define OLED_ADDR 0x3C

// Display dimensions.
#define OLED_WIDTH 128
#define OLED_HEIGHT 64

// SSD1306 memory is organized into 8 "pages" of 8 rows each.
#define OLED_PAGES (OLED_HEIGHT / 8)

// SSD1306 control bytes:
// 0x00 means following bytes are commands.
// 0x40 means following bytes are display RAM data.
#define OLED_CONTROL_CMD  0x00
#define OLED_CONTROL_DATA 0x40

/*
 * ============================================================================
 * Local framebuffer
 * ============================================================================
 *
 * This is our software-side copy of the display memory.
 *
 * Why keep a framebuffer in RAM?
 * Because then drawing becomes simple:
 * - modify memory locally
 * - flush the whole image to the display when ready
 *
 * 128x64 monochrome OLED:
 *   128 columns * 8 pages = 1024 bytes
 */
static uint8_t g_oled_buffer[OLED_WIDTH * OLED_PAGES];

/*
 * ============================================================================
 * Low-level private helpers
 * ============================================================================
 */

/*
 * Send one command byte to the SSD1306.
 *
 * I2C transaction shape:
 *   START
 *   address + write
 *   control byte = 0x00 (command mode)
 *   command byte
 *   STOP
 */
static bool oled_write_command(uint8_t cmd_byte)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    if (cmd == NULL) {
        return false;
    }

    i2c_master_start(cmd);

    // Send OLED address in WRITE mode.
    i2c_master_write_byte(cmd, (OLED_ADDR << 1) | I2C_MASTER_WRITE, true);

    // Tell SSD1306 that the next byte is a command.
    i2c_master_write_byte(cmd, OLED_CONTROL_CMD, true);

    // Send the command byte itself.
    i2c_master_write_byte(cmd, cmd_byte, true);

    i2c_master_stop(cmd);

    esp_err_t ret = i2c_master_cmd_begin(OLED_I2C_PORT, cmd, OLED_I2C_TIMEOUT_TICKS);
    i2c_cmd_link_delete(cmd);

    return (ret == ESP_OK);
}

/*
 * Send a block of display data bytes to the SSD1306.
 *
 * Transaction shape:
 *   START
 *   address + write
 *   control byte = 0x40 (data mode)
 *   N bytes of framebuffer data
 *   STOP
 *
 * We use this when flushing each display page.
 */
static bool oled_write_data(const uint8_t *data, size_t len)
{
    if (data == NULL || len == 0) {
        return false;
    }

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    if (cmd == NULL) {
        return false;
    }

    i2c_master_start(cmd);

    // Send OLED address in WRITE mode.
    i2c_master_write_byte(cmd, (OLED_ADDR << 1) | I2C_MASTER_WRITE, true);

    // Tell SSD1306 that following bytes are display data, not commands.
    i2c_master_write_byte(cmd, OLED_CONTROL_DATA, true);

    // Send each framebuffer byte.
    for (size_t i = 0; i < len; i++) {
        i2c_master_write_byte(cmd, data[i], true);
    }

    i2c_master_stop(cmd);

    esp_err_t ret = i2c_master_cmd_begin(OLED_I2C_PORT, cmd, OLED_I2C_TIMEOUT_TICKS);
    i2c_cmd_link_delete(cmd);

    return (ret == ESP_OK);
}

/*
 * ============================================================================
 * Tiny font support
 * ============================================================================
 *
 * To keep this self-contained and understandable, we are NOT adding a giant
 * full ASCII font table. Instead, we support only the characters we actually
 * need for this project:
 *
 *   digits: 0-9
 *   punctuation: space, colon, dot, percent, minus
 *   letters: L u x T H P C
 *
 * Each character is 5 columns wide and 7 pixels tall.
 * Each byte represents one vertical column of 8 bits.
 *
 * This is a very common minimal embedded font format.
 *
 * Example:
 *   0x3E means bits 1..5 are on in that column.
 */
static const uint8_t *oled_glyph(char c)
{
    // Static storage for glyph patterns.
    // Each glyph is 5 bytes wide.
    static const uint8_t GLYPH_SPACE[5] = {0x00, 0x00, 0x00, 0x00, 0x00};
    static const uint8_t GLYPH_COLON[5] = {0x00, 0x36, 0x36, 0x00, 0x00};
    static const uint8_t GLYPH_DOT[5]   = {0x00, 0x00, 0x60, 0x60, 0x00};
    static const uint8_t GLYPH_MINUS[5] = {0x08, 0x08, 0x08, 0x08, 0x08};
    static const uint8_t GLYPH_PERCENT[5]= {0x62, 0x64, 0x08, 0x13, 0x23};

    static const uint8_t GLYPH_0[5] = {0x3E, 0x51, 0x49, 0x45, 0x3E};
    static const uint8_t GLYPH_1[5] = {0x00, 0x42, 0x7F, 0x40, 0x00};
    static const uint8_t GLYPH_2[5] = {0x42, 0x61, 0x51, 0x49, 0x46};
    static const uint8_t GLYPH_3[5] = {0x21, 0x41, 0x45, 0x4B, 0x31};
    static const uint8_t GLYPH_4[5] = {0x18, 0x14, 0x12, 0x7F, 0x10};
    static const uint8_t GLYPH_5[5] = {0x27, 0x45, 0x45, 0x45, 0x39};
    static const uint8_t GLYPH_6[5] = {0x3C, 0x4A, 0x49, 0x49, 0x30};
    static const uint8_t GLYPH_7[5] = {0x01, 0x71, 0x09, 0x05, 0x03};
    static const uint8_t GLYPH_8[5] = {0x36, 0x49, 0x49, 0x49, 0x36};
    static const uint8_t GLYPH_9[5] = {0x06, 0x49, 0x49, 0x29, 0x1E};

    static const uint8_t GLYPH_L[5] = {0x7F, 0x40, 0x40, 0x40, 0x40};
    static const uint8_t GLYPH_T[5] = {0x01, 0x01, 0x7F, 0x01, 0x01};
    static const uint8_t GLYPH_H[5] = {0x7F, 0x08, 0x08, 0x08, 0x7F};
    static const uint8_t GLYPH_P[5] = {0x7F, 0x09, 0x09, 0x09, 0x06};
    static const uint8_t GLYPH_C[5] = {0x3E, 0x41, 0x41, 0x41, 0x22};

    static const uint8_t GLYPH_u[5] = {0x38, 0x40, 0x40, 0x20, 0x78};
    static const uint8_t GLYPH_x[5] = {0x44, 0x28, 0x10, 0x28, 0x44};

    switch (c) {
        case ' ': return GLYPH_SPACE;
        case ':': return GLYPH_COLON;
        case '.': return GLYPH_DOT;
        case '-': return GLYPH_MINUS;
        case '%': return GLYPH_PERCENT;

        case '0': return GLYPH_0;
        case '1': return GLYPH_1;
        case '2': return GLYPH_2;
        case '3': return GLYPH_3;
        case '4': return GLYPH_4;
        case '5': return GLYPH_5;
        case '6': return GLYPH_6;
        case '7': return GLYPH_7;
        case '8': return GLYPH_8;
        case '9': return GLYPH_9;

        case 'L': return GLYPH_L;
        case 'T': return GLYPH_T;
        case 'H': return GLYPH_H;
        case 'P': return GLYPH_P;
        case 'C': return GLYPH_C;

        case 'u': return GLYPH_u;
        case 'x': return GLYPH_x;

        default:  return GLYPH_SPACE;
    }
}

/*
 * ============================================================================
 * Public OLED API
 * ============================================================================
 */

bool oled_init(void)
{
    /*
     * Initialization sequence for a common 128x64 SSD1306.
     *
     * This is the standard style of SSD1306 bring-up:
     * - display off
     * - addressing / mux / scan settings
     * - charge pump
     * - contrast / precharge / vcomh
     * - display on
     *
     * You do NOT need to memorize all of these.
     * The important thing is understanding:
     *   these are configuration commands sent over I2C.
     */

    if (!oled_write_command(0xAE)) return false; // Display OFF
    if (!oled_write_command(0xD5)) return false; // Set display clock divide ratio / oscillator freq
    if (!oled_write_command(0x80)) return false; // Suggested default clock setting

    if (!oled_write_command(0xA8)) return false; // Set multiplex ratio
    if (!oled_write_command(0x3F)) return false; // 0x3F = 63 -> 64 rows total

    if (!oled_write_command(0xD3)) return false; // Set display offset
    if (!oled_write_command(0x00)) return false; // No vertical offset

    if (!oled_write_command(0x40)) return false; // Set display start line = 0

    if (!oled_write_command(0x8D)) return false; // Charge pump setting
    if (!oled_write_command(0x14)) return false; // Enable charge pump (common for I2C modules)

    if (!oled_write_command(0x20)) return false; // Set memory addressing mode
    if (!oled_write_command(0x00)) return false; // Horizontal addressing mode

    if (!oled_write_command(0xA1)) return false; // Segment remap (mirror horizontally)
    if (!oled_write_command(0xC8)) return false; // COM output scan direction remap

    if (!oled_write_command(0xDA)) return false; // Set COM pins hardware configuration
    if (!oled_write_command(0x12)) return false; // Standard value for 128x64 modules

    if (!oled_write_command(0x81)) return false; // Set contrast control
    if (!oled_write_command(0xCF)) return false; // Reasonable default contrast

    if (!oled_write_command(0xD9)) return false; // Set pre-charge period
    if (!oled_write_command(0xF1)) return false; // Common value with internal charge pump

    if (!oled_write_command(0xDB)) return false; // Set VCOMH deselect level
    if (!oled_write_command(0x40)) return false; // Common default

    if (!oled_write_command(0xA4)) return false; // Entire display ON follows RAM content
    if (!oled_write_command(0xA6)) return false; // Normal display (not inverted)

    if (!oled_write_command(0x2E)) return false; // Deactivate scrolling
    if (!oled_write_command(0xAF)) return false; // Display ON

    // Start with a clean local framebuffer and push it to the panel.
    oled_clear();
    return oled_update();
}

void oled_clear(void)
{
    // Fill the whole local framebuffer with 0 = all pixels off.
    for (size_t i = 0; i < sizeof(g_oled_buffer); i++) {
        g_oled_buffer[i] = 0x00;
    }
}

bool oled_update(void)
{
    /*
     * Flush the local framebuffer to the display.
     *
     * SSD1306 RAM is page-oriented, so we send one page at a time.
     */
    for (int page = 0; page < OLED_PAGES; page++) {
        // Set current page address
        if (!oled_write_command((uint8_t)(0xB0 + page))) {
            return false;
        }

        // Set lower column address nibble = 0
        if (!oled_write_command(0x00)) {
            return false;
        }

        // Set upper column address nibble = 0
        if (!oled_write_command(0x10)) {
            return false;
        }

        // Write 128 bytes for this page
        const uint8_t *page_ptr = &g_oled_buffer[page * OLED_WIDTH];
        if (!oled_write_data(page_ptr, OLED_WIDTH)) {
            return false;
        }
    }

    return true;
}

void oled_draw_pixel(int x, int y, bool color)
{
    // Ignore out-of-bounds pixels instead of crashing or corrupting memory.
    if (x < 0 || x >= OLED_WIDTH || y < 0 || y >= OLED_HEIGHT) {
        return;
    }

    /*
     * Page = y / 8 because each page is 8 pixels tall.
     * Bit  = y % 8 because within that page byte, one bit represents one row.
     */
    int page = y / 8;
    int bit = y % 8;

    // Compute framebuffer index for (column x, page page)
    int index = page * OLED_WIDTH + x;

    if (color) {
        // Turn pixel on by setting that bit
        g_oled_buffer[index] |= (1U << bit);
    } else {
        // Turn pixel off by clearing that bit
        g_oled_buffer[index] &= (uint8_t)~(1U << bit);
    }
}

void oled_draw_char(int x, int y, char c)
{
    /*
     * Get the 5-column bitmap for the requested character.
     * Each byte is one vertical 8-pixel column.
     */
    const uint8_t *glyph = oled_glyph(c);

    // Draw 5 glyph columns
    for (int col = 0; col < 5; col++) {
        uint8_t bits = glyph[col];

        // For each bit in that column, decide whether to light the pixel
        for (int row = 0; row < 7; row++) {
            bool pixel_on = ((bits >> row) & 0x01U) != 0;
            oled_draw_pixel(x + col, y + row, pixel_on);
        }
    }

    // Add one blank spacer column between characters
    for (int row = 0; row < 7; row++) {
        oled_draw_pixel(x + 5, y + row, false);
    }
}

void oled_draw_string(int x, int y, const char *str)
{
    if (str == NULL) {
        return;
    }

    /*
     * Draw characters left-to-right.
     * Each character takes 6 pixels total:
     *   5 glyph columns + 1 spacer column
     */
    while (*str != '\0') {
        oled_draw_char(x, y, *str);
        x += 6;
        str++;
    }
}