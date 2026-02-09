#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

void app_main(void)
{
    while (1) {
        printf("HELLO ESP32-P4\n");
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}

