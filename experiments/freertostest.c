#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

void heartbeat_task(void *pvParameters)
{
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
        printf("heartbeat\n");
    }
}

void blink_logic_task(void *pvParameters)
{
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(300));
        printf("blink\n");
    }
}

void app_main(void)
{
    xTaskCreate
}