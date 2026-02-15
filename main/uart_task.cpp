/**
 * @file uart_task.cpp
 * @brief Implémentation de la tâche de communication UART
 */

#include "uart_task.h"
#include "event_types.h"
#include "protocol_uart.h"

#include <cstring>

#include "driver/uart.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char* TAG = "uart_task";

/**
 * @brief Mapper le type d'événement count_event_type_t vers le type de trame UART
 * 
 * @param type Type d'événement interne
 * @return uint8_t Type d'événement UART
 */
static uint8_t map_event_type(count_event_type_t type)
{
    switch (type)
    {
        case COUNT_EVENT_HEARTBEAT:
            return EVENT_PERSON_COUNT;
        case COUNT_EVENT_CHANGE:
            return EVENT_PERSON_COUNT_CHANGE;
        case COUNT_EVENT_ALARM:
            return EVENT_ALARM;
        case COUNT_EVENT_SYSTEM_STATUS:
            return EVENT_SYSTEM_STATUS;
        default:
            return EVENT_PERSON_COUNT;
    }
}

/**
 * @brief Envoyer une trame UART
 * 
 * @param uart_port Port UART
 * @param event Événement de comptage
 */
static void send_uart_frame(uart_port_t uart_port, const count_event_t* event)
{
    uart_event_frame_t frame = {};

    // Construire la trame selon le protocole
    build_event_frame(&frame,
                      event->node_id,
                      event->timestamp,
                      map_event_type(event->type),
                      event->count,
                      event->confidence,
                      event->flags,
                      event->counter);

    // Envoyer la trame binaire (16 octets)
    int len = uart_write_bytes(uart_port, (const char*)&frame, sizeof(frame));
    if (len != sizeof(frame))
    {
        ESP_LOGE(TAG, "Failed to send complete frame (sent %d/%u bytes)",
                 len, (unsigned)sizeof(frame));
    }
    else
    {
        ESP_LOGI(TAG, "Sent UART frame: type=0x%02X, count=%d, conf=%d%%, flags=0x%02X, ctr=%d",
                 map_event_type(event->type),
                 event->count,
                 event->confidence,
                 event->flags,
                 event->counter);
    }

    // Attendre que la transmission soit terminée
    uart_wait_tx_done(uart_port, pdMS_TO_TICKS(100));
}

/**
 * @brief Point d'entrée de la tâche UART
 */
void uart_task(void* pvParameters)
{
    system_config_t* config = (system_config_t*)pvParameters;
    ESP_LOGI(TAG, "UART task started");

    // Initialiser l'UART
    uart_config_t uart_config = {
        .baud_rate = config->uart_baudrate,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 122,
        .source_clk = UART_SCLK_DEFAULT,
    };

    ESP_ERROR_CHECK(uart_driver_install(config->uart_port, 1024, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(config->uart_port, &uart_config));
    ESP_LOGI(TAG, "UART%d configured: %d baud, 8N1", config->uart_port, config->uart_baudrate);

    // Envoyer un message de démarrage
    vTaskDelay(pdMS_TO_TICKS(500));
    const char* banner = "\r\n[ESP32-P4-EYE] Event-driven architecture started\r\n";
    uart_write_bytes(config->uart_port, banner, strlen(banner));
    uart_wait_tx_done(config->uart_port, pdMS_TO_TICKS(100));

    count_event_t event;

    // Boucle principale
    while (true)
    {
        // Recevoir un événement depuis la queue de comptage (bloquant)
        if (xQueueReceive(config->counting_to_uart_queue, &event, portMAX_DELAY) == pdTRUE)
        {
            // Envoyer la trame UART
            send_uart_frame(config->uart_port, &event);
        }
    }

    // Cleanup (jamais atteint)
    uart_driver_delete(config->uart_port);
    vTaskDelete(NULL);
}

/**
 * @brief Créer et démarrer la tâche UART
 */
TaskHandle_t uart_task_start(system_config_t* config)
{
    TaskHandle_t task_handle = NULL;
    
    BaseType_t ret = xTaskCreatePinnedToCore(
        uart_task,
        "uart_task",
        4096,               // Stack size
        config,             // Parameters
        2,                  // Priority (la plus basse)
        &task_handle,
        0                   // Core 0
    );

    if (ret != pdPASS)
    {
        ESP_LOGE(TAG, "Failed to create UART task");
        return NULL;
    }

    ESP_LOGI(TAG, "UART task created");
    return task_handle;
}
