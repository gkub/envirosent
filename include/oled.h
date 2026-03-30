#ifndef OLED_H
#define OLED_H

#include <stdbool.h>
#include <stdint.h>

// Initialize the SSD1306 OLED.
// Returns true on success.
bool oled_init(void);

// Clear the local framebuffer (RAM copy) to all-black pixels.
void oled_clear(void);

// Push the local framebuffer to the physical OLED over I2C.
// Returns true on success.
bool oled_update(void);

// Draw one ASCII character into the framebuffer at pixel position (x, y).
// This driver supports only a small subset of characters needed for this project.
void oled_draw_char(int x, int y, char c);

// Draw a null-terminated string into the framebuffer starting at (x, y).
void oled_draw_string(int x, int y, const char *str);

// Draw one pixel into the framebuffer.
// color = true  -> pixel on
// color = false -> pixel off
void oled_draw_pixel(int x, int y, bool color);

#endif