#include "driver/uart.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdio.h>
#include <string.h>

#define UART_PORT UART_NUM_0 // GPIO37 = TX UART0 sur ESP32-P4
#define BAUD_RATE 115200

void app_main(void)
{
    // Coupe les logs applicatifs pour ne pas polluer la ligne UART vers le STM32.
    esp_log_level_set("*", ESP_LOG_NONE);

    // Si le driver UART0 existe deja (console), cette installation peut echouer sans gravite.
    uart_driver_install(UART_PORT, 1024, 0, 0, NULL, 0);
    uart_set_baudrate(UART_PORT, BAUD_RATE);

    // Laisse finir le boot/log initial avant d'envoyer nos trames applicatives.
    vTaskDelay(pdMS_TO_TICKS(1500));

    const char* start = "\r\n[APP] UART TX ready on GPIO37\r\n";
    uart_write_bytes(UART_PORT, start, strlen(start));
    uart_wait_tx_done(UART_PORT, pdMS_TO_TICKS(50));

    int seq = 0;
    while (1)
    {
        char msg[64];
        int len = snprintf(msg, sizeof(msg), "HELLO STM32 #%d\r\n", seq++);
        if (len > 0)
        {
            uart_write_bytes(UART_PORT, msg, len);
            uart_wait_tx_done(UART_PORT, pdMS_TO_TICKS(50));
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
