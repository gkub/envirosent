#include "display.h"

#include <stdio.h>

#include "oled.h"

/*
 * This file is a small abstraction layer between the app and the raw OLED driver.
 *
 * Why keep display.c if oled.c already exists?
 *
 * Because oled.c is the low-level device driver.
 * display.c is the higher-level "how do we present our application data?" layer.
 *
 * That separation is good architecture:
 *   oled.c    = hardware details
 *   display.c = app presentation logic
 */

bool display_init(void)
{
    // Right now, display init is just OLED init.
    return oled_init();
}

bool display_show_sample(const sensor_sample_t *sample)
{
    if (sample == NULL) {
        return false;
    }

    /*
     * Small text buffers for each display line.
     * snprintf prevents overflow by respecting buffer size.
     */
    char line0[32];
    char line1[32];
    char line2[32];
    char line3[32];
    char line4[32];

    /*
     * Keep strings short and use only characters supported by our tiny font.
     *
     * We intentionally avoid unsupported lowercase text except 'u' and 'x'.
     */
    snprintf(line0, sizeof(line0), "TS:%lu", sample->timestamp);
    snprintf(line1, sizeof(line1), "Lux: %.1f", sample->lux);
    snprintf(line2, sizeof(line2), "T: %.2fC", sample->temperature);
    snprintf(line3, sizeof(line3), "H: %.1f%%", sample->humidity);
    snprintf(line4, sizeof(line4), "P: %.1f", sample->pressure);

    // Clear old framebuffer contents
    oled_clear();

    // Draw five text lines
    oled_draw_string(0, 0,  line0);
    oled_draw_string(0, 16, line1);
    oled_draw_string(0, 28, line2);
    oled_draw_string(0, 40, line3);
    oled_draw_string(0, 52, line4);
    // Push framebuffer to the actual display
    return oled_update();
}