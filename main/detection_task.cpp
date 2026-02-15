/**
 * @file detection_task.cpp
 * @brief Implémentation de la tâche de détection humaine
 */

#include "detection_task.h"
#include "event_types.h"

#include <cstring>

#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "dl_detect_define.hpp"
#include "dl_image_define.hpp"
#include "pedestrian_detect.hpp"

static const char* TAG = "detection_task";

#define MAX_DETECTED_PERSONS 20  // Nombre max de personnes détectables par frame

/**
 * @brief Convertit un pixel RGB565 (little-endian) en RGB888
 */
static inline void rgb565_to_rgb888(uint16_t px, uint8_t* r, uint8_t* g, uint8_t* b)
{
    *r = (px >> 8) & 0xF8;
    *g = (px >> 3) & 0xFC;
    *b = (px << 3) & 0xF8;
}

/**
 * @brief Redimensionne un buffer RGB565 vers RGB888 (nearest-neighbor)
 * 
 * @param src Buffer source RGB565
 * @param src_w Largeur source
 * @param src_h Hauteur source
 * @param dst Buffer destination RGB888 (pré-alloué)
 * @param dst_w Largeur destination
 * @param dst_h Hauteur destination
 */
static void resize_rgb565_to_rgb888(const uint16_t* src, int src_w, int src_h, 
                                     uint8_t* dst, int dst_w, int dst_h)
{
    for (int y = 0; y < dst_h; y++)
    {
        int sy = y * src_h / dst_h;
        for (int x = 0; x < dst_w; x++)
        {
            int sx = x * src_w / dst_w;
            uint16_t px = src[sy * src_w + sx];
            int idx = (y * dst_w + x) * 3;
            rgb565_to_rgb888(px, &dst[idx], &dst[idx + 1], &dst[idx + 2]);
        }
    }
}

/**
 * @brief Point d'entrée de la tâche de détection
 */
void detection_task(void* pvParameters)
{
    system_config_t* config = (system_config_t*)pvParameters;
    ESP_LOGI(TAG, "Detection task started");

    // Allouer le buffer de redimensionnement (PSRAM)
    size_t rgb_buf_size = config->model_width * config->model_height * 3;
    uint8_t* rgb_buf = (uint8_t*)heap_caps_malloc(rgb_buf_size, MALLOC_CAP_SPIRAM);
    if (!rgb_buf)
    {
        ESP_LOGE(TAG, "Failed to allocate RGB buffer (%u bytes)", (unsigned)rgb_buf_size);
        vTaskDelete(NULL);
        return;
    }
    ESP_LOGI(TAG, "Allocated RGB buffer: %u bytes in PSRAM", (unsigned)rgb_buf_size);

    // Charger le modèle PedestrianDetect (esp-dl)
    ESP_LOGI(TAG, "Loading PedestrianDetect model...");
    PedestrianDetect* detector = new PedestrianDetect();
    ESP_LOGI(TAG, "Model loaded successfully");

    camera_frame_event_t frame_event;

    // Boucle principale
    while (true)
    {
        // Recevoir une frame depuis la queue caméra (bloquant avec timeout)
        if (xQueueReceive(config->camera_to_detection_queue, &frame_event, 
                          pdMS_TO_TICKS(1000)) != pdTRUE)
        {
            continue; // Timeout, réessayer
        }

        int64_t t_start = esp_timer_get_time();

        // Redimensionner l'image pour le modèle (1920x1080 RGB565 -> 224x224 RGB888)
        resize_rgb565_to_rgb888(frame_event.frame_data, 
                                frame_event.width, frame_event.height,
                                rgb_buf, 
                                config->model_width, config->model_height);

        int64_t t_resize = esp_timer_get_time();

        // Préparer l'image pour le modèle
        dl::image::img_t img = {};
        img.data = rgb_buf;
        img.width = config->model_width;
        img.height = config->model_height;
        img.pix_type = dl::image::DL_IMAGE_PIX_TYPE_RGB888;

        // Exécuter l'inférence
        auto& results = detector->run(img);
        int64_t t_infer = esp_timer_get_time();

        int people_count = (int)results.size();

        // Allouer l'événement de résultat
        detection_result_event_t* result = alloc_detection_result(MAX_DETECTED_PERSONS);
        if (!result)
        {
            ESP_LOGE(TAG, "Failed to allocate detection result");
            continue;
        }

        result->frame_id = frame_event.frame_id;
        result->timestamp_us = frame_event.timestamp_us;
        result->people_count = people_count;
        result->inference_time_ms = (int)((t_infer - t_resize) / 1000);

        // Copier les résultats de détection
        float sum_confidence = 0.0f;
        int idx = 0;
        for (const auto& r : results)
        {
            if (idx >= MAX_DETECTED_PERSONS)
                break;

            if (r.box.size() >= 4)
            {
                result->persons[idx].x0 = (int)r.box[0];
                result->persons[idx].y0 = (int)r.box[1];
                result->persons[idx].x1 = (int)r.box[2];
                result->persons[idx].y1 = (int)r.box[3];
                result->persons[idx].confidence = r.score;
                result->persons[idx].person_id = idx;
                
                sum_confidence += r.score;
                idx++;
            }
        }

        result->people_count = idx;
        result->avg_confidence = (idx > 0) ? (sum_confidence / idx) : 0.0f;

        // Envoyer vers la queue de comptage (non-bloquant)
        if (xQueueSend(config->detection_to_counting_queue, &result, 0) != pdTRUE)
        {
            ESP_LOGW(TAG, "Counting queue full, dropping result for frame %lu", 
                     (unsigned long)frame_event.frame_id);
            free_detection_result(result);
        }

        int resize_ms = (int)((t_resize - t_start) / 1000);
        int infer_ms = result->inference_time_ms;
        int total_ms = (int)((esp_timer_get_time() - t_start) / 1000);

        ESP_LOGI(TAG, "Frame %lu: detected %d person(s), conf=%.2f, "
                      "resize=%dms, infer=%dms, total=%dms",
                 (unsigned long)frame_event.frame_id,
                 result->people_count,
                 result->avg_confidence,
                 resize_ms, infer_ms, total_ms);

        // Laisser du temps aux autres tâches
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    // Cleanup (jamais atteint)
    delete detector;
    heap_caps_free(rgb_buf);
    vTaskDelete(NULL);
}

/**
 * @brief Créer et démarrer la tâche de détection
 */
TaskHandle_t detection_task_start(system_config_t* config)
{
    TaskHandle_t task_handle = NULL;
    
    BaseType_t ret = xTaskCreatePinnedToCore(
        detection_task,
        "detection_task",
        16384,              // Stack size (plus grand pour l'inférence)
        config,             // Parameters
        4,                  // Priority (inférieure à caméra)
        &task_handle,
        1                   // Core 1 (séparé de la caméra)
    );

    if (ret != pdPASS)
    {
        ESP_LOGE(TAG, "Failed to create detection task");
        return NULL;
    }

    ESP_LOGI(TAG, "Detection task created");
    return task_handle;
}
