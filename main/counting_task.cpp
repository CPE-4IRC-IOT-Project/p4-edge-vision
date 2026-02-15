/**
 * @file counting_task.cpp
 * @brief Implémentation de la tâche de comptage et logique événementielle
 */

#include "counting_task.h"
#include "event_types.h"
#include "protocol_uart.h"

#include <sys/time.h>
#include <cstring>

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char* TAG = "counting_task";

/**
 * @brief État interne de la tâche de comptage
 */
typedef struct {
    uint8_t last_count;              // Dernier comptage envoyé
    uint8_t current_count;           // Comptage actuel
    uint32_t last_heartbeat_time;    // Timestamp du dernier heartbeat (s)
    uint16_t event_counter;          // Compteur anti-rejeu
    bool alarm_active;               // État de l'alarme
} counting_state_t;

/**
 * @brief Obtenir le timestamp Unix actuel (secondes)
 */
static uint32_t get_unix_timestamp()
{
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return (uint32_t)tv.tv_sec;
}

/**
 * @brief Détecter si la lumière du jour est présente (placeholder)
 * 
 * @return true si c'est le jour, false si c'est la nuit
 * 
 * @note Pour l'instant, retourne toujours true. Pourrait être amélioré
 *       avec un capteur de luminosité ou analyse de l'image.
 */
static bool is_daylight()
{
    // TODO: implémenter détection jour/nuit
    // Option 1: capteur de luminosité
    // Option 2: analyse de la luminosité moyenne de l'image
    // Option 3: heure du système
    return true;
}

/**
 * @brief Créer un événement de comptage
 * 
 * @param state État de comptage actuel
 * @param config Configuration système
 * @param type Type d'événement
 * @param count Nombre de personnes
 * @param confidence Confiance (0-100)
 * @return count_event_t Événement créé
 */
static count_event_t create_count_event(counting_state_t* state,
                                        system_config_t* config,
                                        count_event_type_t type,
                                        uint8_t count,
                                        uint8_t confidence)
{
    count_event_t event = {};
    event.type = type;
    event.count = count;
    event.confidence = confidence;
    event.timestamp = get_unix_timestamp();
    event.node_id = config->node_id;
    event.counter = state->event_counter++;

    // Construire les flags
    event.flags = 0;
    if (is_daylight())
        event.flags |= FLAG_DAYLIGHT;
    if (state->alarm_active)
        event.flags |= FLAG_ALARM;
    // TODO: ajouter FLAG_MOTION si mouvement détecté

    return event;
}

/**
 * @brief Point d'entrée de la tâche de comptage
 */
void counting_task(void* pvParameters)
{
    system_config_t* config = (system_config_t*)pvParameters;
    ESP_LOGI(TAG, "Counting task started");

    // Initialiser l'état
    counting_state_t state = {};
    state.last_count = 0;
    state.current_count = 0;
    state.last_heartbeat_time = get_unix_timestamp();
    state.event_counter = 0;
    state.alarm_active = false;

    detection_result_event_t* result = NULL;

    // Boucle principale
    while (true)
    {
        // Recevoir un résultat de détection depuis la queue (bloquant avec timeout)
        if (xQueueReceive(config->detection_to_counting_queue, &result, 
                          pdMS_TO_TICKS(100)) == pdTRUE)
        {
            // Mettre à jour le comptage actuel
            state.current_count = (uint8_t)result->people_count;
            uint8_t confidence_pct = (uint8_t)(result->avg_confidence * 100.0f);

            ESP_LOGI(TAG, "Received detection result: count=%d, confidence=%d%%",
                     state.current_count, confidence_pct);

            // 1. Détecter un changement de comptage
            int diff = abs((int)state.current_count - (int)state.last_count);
            if (diff >= 1)
            {
                ESP_LOGI(TAG, "Count changed: %d -> %d (diff=%d)",
                         state.last_count, state.current_count, diff);

                count_event_t event = create_count_event(&state, config,
                                                         COUNT_EVENT_CHANGE,
                                                         state.current_count,
                                                         confidence_pct);

                // Envoyer vers la queue UART
                if (xQueueSend(config->counting_to_uart_queue, &event, 0) != pdTRUE)
                {
                    ESP_LOGW(TAG, "UART queue full, dropping event");
                }

                state.last_count = state.current_count;
                state.last_heartbeat_time = get_unix_timestamp(); // Reset heartbeat
            }

            // 2. Détecter une alarme (seuil dépassé)
            if (state.current_count >= config->alarm_threshold)
            {
                if (!state.alarm_active)
                {
                    ESP_LOGW(TAG, "ALARM: threshold exceeded (%d >= %d)",
                             state.current_count, config->alarm_threshold);
                    state.alarm_active = true;

                    count_event_t event = create_count_event(&state, config,
                                                             COUNT_EVENT_ALARM,
                                                             state.current_count,
                                                             confidence_pct);

                    if (xQueueSend(config->counting_to_uart_queue, &event, 0) != pdTRUE)
                    {
                        ESP_LOGW(TAG, "UART queue full, dropping alarm event");
                    }
                }
            }
            else
            {
                if (state.alarm_active)
                {
                    ESP_LOGI(TAG, "ALARM cleared");
                    state.alarm_active = false;
                }
            }

            // Libérer le résultat de détection
            free_detection_result(result);
            result = NULL;
        }

        // 3. Heartbeat périodique (toutes les N secondes)
        uint32_t current_time = get_unix_timestamp();
        uint32_t elapsed = current_time - state.last_heartbeat_time;

        if (elapsed >= config->heartbeat_period_s)
        {
            ESP_LOGI(TAG, "Heartbeat: count=%d", state.current_count);

            count_event_t event = create_count_event(&state, config,
                                                     COUNT_EVENT_HEARTBEAT,
                                                     state.current_count,
                                                     0); // Confidence non applicable

            if (xQueueSend(config->counting_to_uart_queue, &event, 0) != pdTRUE)
            {
                ESP_LOGW(TAG, "UART queue full, dropping heartbeat event");
            }

            state.last_heartbeat_time = current_time;
        }

        // Laisser du temps aux autres tâches
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    // Cleanup (jamais atteint)
    vTaskDelete(NULL);
}

/**
 * @brief Créer et démarrer la tâche de comptage
 */
TaskHandle_t counting_task_start(system_config_t* config)
{
    TaskHandle_t task_handle = NULL;
    
    BaseType_t ret = xTaskCreatePinnedToCore(
        counting_task,
        "counting_task",
        4096,               // Stack size
        config,             // Parameters
        3,                  // Priority (inférieure à détection)
        &task_handle,
        0                   // Core 0
    );

    if (ret != pdPASS)
    {
        ESP_LOGE(TAG, "Failed to create counting task");
        return NULL;
    }

    ESP_LOGI(TAG, "Counting task created");
    return task_handle;
}
