#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include <stdio.h>

#include "app.h"
#include "i2cscanner.h"
#include "sensors.h"

SemaphoreHandle_t i2c_mutex;
QueueHandle_t sensor_queue;
static uint32_t t = 0;

void sensor_task(void *pvParameters) {
    while (1) {
        if (xSemaphoreTake(i2c_mutex, pdMS_TO_TICKS(100)) == pdTRUE) { // take the mutex
            sensor_sample_t sample; // create a sample
            bool ok_lux = bh1750_read_lux(&sample.lux); // read the lux
            bool ok_bme = bme280_read_environment( // read the environment
                &sample.temperature, 
                &sample.humidity, 
                &sample.pressure); 
            xSemaphoreGive(i2c_mutex); // release the mutex

            if (ok_lux && ok_bme) { // if the lux and environment readings are successful
                sample.timestamp = t; // set the timestamp
                if (xQueueSend(sensor_queue, &sample, 0) != pdTRUE) { // if the queue is full, drop the sample
                    printf("Queue full, dropping sample\n");
                }
                else {
                    printf("Sample sent to queue\n");
                    t += 1000;
                }
            } else {
                printf("Error reading sensors\n");
            }
        } else {
            printf("Error taking mutex\n");
        }
        vTaskDelay(pdMS_TO_TICKS(1000)); // wait 1 second
    }
}

void display_task(void *pvParameters) {
    while (1) {
        sensor_sample_t sample;
        if (xQueueReceive(sensor_queue, &sample, portMAX_DELAY) == pdTRUE) {
            printf("display task received sample: Lux: %f, Temp: %f, Hum: %f, Press: %f, Timestamp: %lu\n", sample.lux, sample.temperature, sample.humidity, sample.pressure, (unsigned long)sample.timestamp);
        }
        else {
            printf("Queue empty, waiting for sample\n");
        }
        vTaskDelay(pdMS_TO_TICKS(1000)); // wait 1 second
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