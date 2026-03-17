#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include <stdio.h>

#include "app.h"
#include "i2cscanner.h"

QueueHandle_t sensor_queue;
SemaphoreHandle_t i2c_mutex;

void sensor_task(void *pvParameters) {
    while (1) {
        printf("sensor task alive\n");
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
};
void display_task(void *pvParameters) {
    while (1) {
        printf("display task alive\n");
        vTaskDelay(pdMS_TO_TICKS(1500));
    }
}

void app_main(void)
{
    printf("Room Sentinel booting...\n");
    i2c_master_init();
    i2c_scanner();

    i2c_mutex = xSemaphoreCreateMutex();
    if (i2c_mutex == NULL) {
        printf("ERROR: Failed to create I2C mutex\n");
        while (1);  // halt
    }
    sensor_queue = xQueueCreate(16, sizeof(sensor_sample_t));
    if (sensor_queue == NULL) {
        printf("ERROR: Failed to create sensor queue\n");
        while (1);  // halt
    }
    xTaskCreate(sensor_task, "sensor_task", 4096, NULL, 2, NULL); // priority 2
    xTaskCreate(display_task, "display_task", 4096, NULL, 1, NULL); // priority 1
}