#ifndef APP_H
#define APP_H

#include <stdint.h>

typedef struct {
    float lux;
    float temperature;
    float humidity;
    float pressure;
    uint32_t timestamp;
} sensor_sample_t;

#endif