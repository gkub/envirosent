#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include <stdio.h>

#include "app.h"
#include "i2cscanner.h"
#include "sensors.h"   // <-- needed so main.c knows the sensor function prototypes

SemaphoreHandle_t i2c_mutex;
QueueHandle_t sensor_queue;

// Simple fake timestamp counter for now.
// We can later replace this with xTaskGetTickCount() or a real time base.
static uint32_t t = 0;

void sensor_task(void *pvParameters)
{
    // Silence "unused parameter" warnings since we are not using pvParameters right now.
    (void) pvParameters;

    while (1) {
        // One sample struct per loop iteration.
        sensor_sample_t sample;

        /*
         * Acquire exclusive ownership of the I2C bus.
         *
         * This is important because eventually multiple tasks will want to use:
         *   - BH1750
         *   - BME280
         *   - OLED
         *
         * Without a mutex, their bus transactions could overlap and corrupt each other.
         */
        if (xSemaphoreTake(i2c_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {

            /*
             * Call into the sensor module to perform actual sensor reads.
             *
             * Notice:
             * - main.c decides WHEN to lock and read
             * - sensors.c knows HOW to talk to the hardware
             *
             * This is good separation of concerns.
             */
            bool ok_lux = bh1750_read_lux(&sample.lux);
            bool ok_bme = bme280_read_environment(
                &sample.temperature,
                &sample.humidity,
                &sample.pressure
            );

            // Release the bus as soon as we are done with I2C operations.
            xSemaphoreGive(i2c_mutex);

            if (ok_lux && ok_bme) {
                // Fill the timestamp after successful reads.
                sample.timestamp = t;

                /*
                 * Send the sample to the queue.
                 *
                 * Why '&sample'?
                 * Because xQueueSend expects a POINTER to the data to copy.
                 *
                 * The queue copies sizeof(sensor_sample_t) bytes into its own storage.
                 * It does not keep a pointer to this stack variable.
                 */
                if (xQueueSend(sensor_queue, &sample, 0) != pdTRUE) {
                    printf("Queue full, dropping sample\n");
                } else {
                    printf("Sample sent to queue\n");

                    // Advance our fake timestamp by 1000 each 1-second cycle.
                    t += 1000;
                }
            } else {
                printf("Sensor read failed\n");
            }
        } else {
            printf("Failed to acquire I2C mutex\n");
        }

        // Sample once per second.
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void display_task(void *pvParameters)
{
    (void) pvParameters;

    while (1) {
        sensor_sample_t sample;

        /*
         * Block until a sample arrives.
         *
         * This is the producer/consumer RTOS pattern we wanted to implement:
         * - sensor_task produces data periodically
         * - display_task sleeps until work exists
         *
         * portMAX_DELAY means:
         *   "Wait indefinitely until a queue item becomes available."
         */
        if (xQueueReceive(sensor_queue, &sample, portMAX_DELAY) == pdTRUE) {
            printf(
                "display task received sample: "
                "Lux: %f, Temp: %f, Hum: %f, Press: %f, Timestamp: %lu\n",
                sample.lux,
                sample.temperature,
                sample.humidity,
                sample.pressure,
                (unsigned long) sample.timestamp
            );
        }
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
        while (1) {
        }
    }

    sensor_queue = xQueueCreate(16, sizeof(sensor_sample_t));
    if (sensor_queue == NULL) {
        printf("ERROR: Failed to create sensor queue\n");
        while (1) {
        }
    }

    if (xSemaphoreTake(i2c_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        if (bh1750_init()) {
            printf("BH1750 initialized successfully\n");
        } else {
            printf("ERROR: Failed to initialize BH1750\n");
        }

        if (bme280_init()) {
            printf("BME280 initialized successfully\n");
        } else {
            printf("ERROR: Failed to initialize BME280\n");
        }

        xSemaphoreGive(i2c_mutex);
    } else {
        printf("ERROR: Failed to acquire I2C mutex for sensor init\n");
    }

    xTaskCreate(sensor_task, "sensor_task", 4096, NULL, 2, NULL);
    xTaskCreate(display_task, "display_task", 4096, NULL, 1, NULL);
}