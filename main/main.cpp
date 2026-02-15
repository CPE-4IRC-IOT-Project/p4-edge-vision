/**
 * @file main.cpp
 * @brief ESP32-P4-EYE - Event-driven architecture with FreeRTOS queues
 * 
 * Architecture:
 *   CameraTask -> Queue -> DetectionTask -> Queue -> CountingTask -> Queue -> UARTTask
 * 
 * Inspirée de: "Developing IoT Projects with ESP32" (Packt Publishing)
 * 
 * Pipeline:
 *   1. CameraTask: Capture frames from MIPI-CSI camera
 *   2. DetectionTask: Run pedestrian detection inference (esp-dl)
 *   3. CountingTask: Count people, detect changes, manage heartbeat & alarms
 *   4. UARTTask: Send binary frames to STM32 via UART protocol
 * 
 * Queues:
 *   - camera_to_detection: frame buffers (camera_frame_event_t)
 *   - detection_to_counting: detection results (detection_result_event_t*)
 *   - counting_to_uart: count events (count_event_t)
 */

#include <cstdio>
#include <cstring>

#include "esp_log.h"
#include "esp_heap_caps.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/uart.h"

// BSP
#include "bsp/display.h"
#include "bsp/esp32_p4_eye.h"

// V4L2
#include "linux/videodev2.h"

// Event types & tasks
#include "event_types.h"
#include "camera_task.h"
#include "detection_task.h"
#include "counting_task.h"
#include "uart_task.h"

// UART protocol
#include "protocol_uart.h"

static const char* TAG = "main";

// ============================================================================
// Configuration par défaut
// ============================================================================

// Paramètres caméra
#define CAM_WIDTH               1920
#define CAM_HEIGHT              1080
#define CAM_PIX_FMT             V4L2_PIX_FMT_RGB565
#define CAM_CAPTURE_PERIOD_MS   200     // Capture toutes les 200ms (5 FPS)

// Paramètres modèle
#define MODEL_WIDTH             224
#define MODEL_HEIGHT            224

// Paramètres LCD
#define LCD_WIDTH               BSP_LCD_H_RES  // 240
#define LCD_HEIGHT              BSP_LCD_V_RES  // 240
#define LCD_ENABLE_PREVIEW      true

// Paramètres UART
#define UART_PORT_NUM           UART_NUM_0
#define UART_BAUD_RATE          115200
#define NODE_ID                 0x01          // ID de ce nœud caméra

// Paramètres détection
#define DETECTION_CONFIDENCE    0.5f          // Seuil de confiance (0.0-1.0)
#define HEARTBEAT_PERIOD_S      120           // Heartbeat toutes les 2 minutes
#define ALARM_THRESHOLD         10            // Alarme si >= 10 personnes

// Tailles des queues
#define QUEUE_CAMERA_SIZE       2             // Petite queue (frames lourdes)
#define QUEUE_DETECTION_SIZE    5             // Queue moyenne
#define QUEUE_COUNTING_SIZE     10            // Plus grande queue (events légers)

// ============================================================================
// Configuration globale
// ============================================================================

static system_config_t g_system_config = {
    // Caméra
    .camera_width = CAM_WIDTH,
    .camera_height = CAM_HEIGHT,
    .camera_pixel_format = CAM_PIX_FMT,
    .camera_capture_period_ms = CAM_CAPTURE_PERIOD_MS,
    
    // Modèle
    .model_width = MODEL_WIDTH,
    .model_height = MODEL_HEIGHT,
    
    // LCD
    .lcd_width = LCD_WIDTH,
    .lcd_height = LCD_HEIGHT,
    .lcd_enable_preview = LCD_ENABLE_PREVIEW,
    
    // UART
    .uart_port = UART_PORT_NUM,
    .uart_baudrate = UART_BAUD_RATE,
    .node_id = NODE_ID,
    
    // Détection
    .detection_confidence_threshold = DETECTION_CONFIDENCE,
    .heartbeat_period_s = HEARTBEAT_PERIOD_S,
    .alarm_threshold = ALARM_THRESHOLD,
    
    // Queues (initialisées dans app_main)
    .camera_to_detection_queue = NULL,
    .detection_to_counting_queue = NULL,
    .counting_to_uart_queue = NULL,
};

// ============================================================================
// Main
// ============================================================================

extern "C" void app_main(void)
{
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "ESP32-P4-EYE - Event-Driven Architecture");
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "Camera: %dx%d @ %dms", CAM_WIDTH, CAM_HEIGHT, CAM_CAPTURE_PERIOD_MS);
    ESP_LOGI(TAG, "Model: %dx%d", MODEL_WIDTH, MODEL_HEIGHT);
    ESP_LOGI(TAG, "LCD: %dx%d", LCD_WIDTH, LCD_HEIGHT);
    ESP_LOGI(TAG, "UART: UART%d @ %d baud", UART_PORT_NUM, UART_BAUD_RATE);
    ESP_LOGI(TAG, "Node ID: 0x%02X", NODE_ID);
    ESP_LOGI(TAG, "Heartbeat: %ds, Alarm threshold: %d persons",
             HEARTBEAT_PERIOD_S, ALARM_THRESHOLD);
    ESP_LOGI(TAG, "========================================");

    // ========================================================================
    // 1. Créer les queues FreeRTOS
    // ========================================================================
    
    ESP_LOGI(TAG, "Creating FreeRTOS queues...");
    
    g_system_config.camera_to_detection_queue = xQueueCreate(
        QUEUE_CAMERA_SIZE,
        sizeof(camera_frame_event_t)
    );
    
    g_system_config.detection_to_counting_queue = xQueueCreate(
        QUEUE_DETECTION_SIZE,
        sizeof(detection_result_event_t*)  // Pointeur
    );
    
    g_system_config.counting_to_uart_queue = xQueueCreate(
        QUEUE_COUNTING_SIZE,
        sizeof(count_event_t)
    );

    if (!g_system_config.camera_to_detection_queue ||
        !g_system_config.detection_to_counting_queue ||
        !g_system_config.counting_to_uart_queue)
    {
        ESP_LOGE(TAG, "Failed to create queues!");
        return;
    }

    ESP_LOGI(TAG, "Queues created successfully");
    ESP_LOGI(TAG, "  - camera_to_detection: %d slots", QUEUE_CAMERA_SIZE);
    ESP_LOGI(TAG, "  - detection_to_counting: %d slots", QUEUE_DETECTION_SIZE);
    ESP_LOGI(TAG, "  - counting_to_uart: %d slots", QUEUE_COUNTING_SIZE);

    // ========================================================================
    // 2. Initialiser le display LCD (optionnel pour preview)
    // ========================================================================
    
    if (g_system_config.lcd_enable_preview)
    {
        ESP_LOGI(TAG, "Initializing LCD display...");
        lv_display_t* lvgl_disp = bsp_display_start();
        if (lvgl_disp != NULL)
        {
            bsp_display_brightness_set(80);
            ESP_LOGI(TAG, "LCD display ready (%dx%d)", LCD_WIDTH, LCD_HEIGHT);
            
            // Créer un label simple pour indiquer le mode
            bsp_display_lock(0);
            {
                lv_obj_t* lbl = lv_label_create(lv_screen_active());
                lv_obj_center(lbl);
                lv_obj_set_style_text_color(lbl, lv_color_white(), 0);
                lv_label_set_text(lbl, "Event-Driven Mode\nSee UART for logs");
            }
            bsp_display_unlock();
        }
        else
        {
            ESP_LOGW(TAG, "Failed to initialize LCD, continuing without display");
        }
    }

    // ========================================================================
    // 3. Démarrer les tâches dans l'ordre
    // ========================================================================
    
    ESP_LOGI(TAG, "Starting tasks...");

    // Démarrer la tâche UART (priorité la plus basse)
    TaskHandle_t uart_task_handle = uart_task_start(&g_system_config);
    if (!uart_task_handle)
    {
        ESP_LOGE(TAG, "Failed to start UART task");
        return;
    }
    vTaskDelay(pdMS_TO_TICKS(500));  // Laisser l'UART s'initialiser

    // Démarrer la tâche de comptage
    TaskHandle_t counting_task_handle = counting_task_start(&g_system_config);
    if (!counting_task_handle)
    {
        ESP_LOGE(TAG, "Failed to start counting task");
        return;
    }

    // Démarrer la tâche de détection
    TaskHandle_t detection_task_handle = detection_task_start(&g_system_config);
    if (!detection_task_handle)
    {
        ESP_LOGE(TAG, "Failed to start detection task");
        return;
    }

    // Démarrer la tâche caméra (priorité la plus haute)
    TaskHandle_t camera_task_handle = camera_task_start(&g_system_config);
    if (!camera_task_handle)
    {
        ESP_LOGE(TAG, "Failed to start camera task");
        return;
    }

    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "All tasks started successfully!");
    ESP_LOGI(TAG, "System is running in event-driven mode");
    ESP_LOGI(TAG, "========================================");

    // ========================================================================
    // 4. Tâche principale: monitoring (optionnel)
    // ========================================================================
    
    while (true)
    {
        // Afficher les statistiques des queues toutes les 30 secondes
        vTaskDelay(pdMS_TO_TICKS(30000));
        
        int cam_waiting = uxQueueMessagesWaiting(g_system_config.camera_to_detection_queue);
        int det_waiting = uxQueueMessagesWaiting(g_system_config.detection_to_counting_queue);
        int cnt_waiting = uxQueueMessagesWaiting(g_system_config.counting_to_uart_queue);
        
        ESP_LOGI(TAG, "Queue status: Camera=%d/%d, Detection=%d/%d, Counting=%d/%d",
                 cam_waiting, QUEUE_CAMERA_SIZE,
                 det_waiting, QUEUE_DETECTION_SIZE,
                 cnt_waiting, QUEUE_COUNTING_SIZE);

        // Afficher la mémoire disponible
        ESP_LOGI(TAG, "Free heap: %lu bytes, Free PSRAM: %lu bytes",
                 (unsigned long)esp_get_free_heap_size(),
                 (unsigned long)heap_caps_get_free_size(MALLOC_CAP_SPIRAM));
    }
}
