#ifndef DISPLAY_H
#define DISPLAY_H

#include <stdbool.h>
#include "app.h"

// Initialize display subsystem.
bool display_init(void);

// Render one sample onto the OLED.
bool display_show_sample(const sensor_sample_t *sample);

#endif