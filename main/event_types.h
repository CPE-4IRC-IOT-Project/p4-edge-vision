#ifndef EVENT_TYPES_H
#define EVENT_TYPES_H

#include <stdint.h>
#include <vector>
#include "driver/uart.h"
#include "dl_detect_define.hpp"

/**
 * @file event_types.h
 * @brief Définition des structures d'événements pour la communication inter-tâches
 * 
 * Architecture basée sur des queues FreeRTOS pour gérer le pipeline:
 * Camera -> Detection -> Counting -> UART
 */

// ============================================================================
// EVENT 1: Frame capturée (Camera -> Detection)
// ============================================================================

/**
 * @brief Structure contenant une frame RGB565 capturée par la caméra
 * 
 * Cette structure est envoyée de CameraTask vers DetectionTask via une queue.
 */
typedef struct {
    uint16_t* frame_data;      // Pointeur vers les données RGB565 (1920x1080)
    int width;                  // Largeur de l'image
    int height;                 // Hauteur de l'image
    int64_t timestamp_us;       // Timestamp de capture (microseconds)
    uint32_t frame_id;          // ID unique de la frame
} camera_frame_event_t;

// ============================================================================
// EVENT 2: Résultat de détection (Detection -> Counting)
// ============================================================================

/**
 * @brief Structure représentant une personne détectée
 */
typedef struct {
    int x0, y0, x1, y1;        // Bounding box (coordonnées)
    float confidence;          // Confiance de détection (0.0 - 1.0)
    int person_id;             // ID temporaire (pour tracking futur)
} detected_person_t;

/**
 * @brief Structure contenant les résultats de détection
 * 
 * Cette structure est envoyée de DetectionTask vers CountingTask.
 */
typedef struct {
    uint32_t frame_id;                          // ID de la frame source
    int64_t timestamp_us;                       // Timestamp
    int people_count;                           // Nombre total de personnes
    detected_person_t* persons;                 // Tableau des personnes détectées
    int max_persons;                            // Capacité max du tableau
    float avg_confidence;                       // Confiance moyenne
    int inference_time_ms;                      // Temps d'inférence (ms)
} detection_result_event_t;

// ============================================================================
// EVENT 3: Event de comptage (Counting -> UART)
// ============================================================================

/**
 * @brief Type d'événement à transmettre
 */
typedef enum {
    COUNT_EVENT_HEARTBEAT,      // Envoi périodique (toutes les 2 minutes)
    COUNT_EVENT_CHANGE,         // Changement détecté (±1 personne)
    COUNT_EVENT_ALARM,          // Alarme (seuil dépassé)
    COUNT_EVENT_SYSTEM_STATUS   // Status système
} count_event_type_t;

/**
 * @brief Structure contenant un événement de comptage
 * 
 * Cette structure est envoyée de CountingTask vers UARTTask.
 */
typedef struct {
    count_event_type_t type;    // Type d'événement
    uint8_t count;              // Nombre de personnes
    uint8_t confidence;         // Confiance (0-100%)
    uint8_t flags;              // Flags (jour/nuit, mouvement, alarme)
    uint32_t timestamp;         // Timestamp Unix (secondes)
    uint8_t node_id;            // ID du nœud (caméra)
    uint16_t counter;           // Compteur anti-rejeu
} count_event_t;

// ============================================================================
// Structures de configuration des tâches
// ============================================================================

/**
 * @brief Configuration globale du système
 */
typedef struct {
    // Paramètres caméra
    int camera_width;
    int camera_height;
    int camera_pixel_format;
    int camera_capture_period_ms;
    
    // Paramètres modèle
    int model_width;
    int model_height;
    
    // Paramètres LCD
    int lcd_width;
    int lcd_height;
    bool lcd_enable_preview;
    
    // Paramètres UART
    uart_port_t uart_port;
    int uart_baudrate;
    uint8_t node_id;
    
    // Paramètres détection
    float detection_confidence_threshold;
    int heartbeat_period_s;      // Période heartbeat (secondes)
    int alarm_threshold;          // Seuil d'alarme (nombre de personnes)
    
    // Queues
    QueueHandle_t camera_to_detection_queue;
    QueueHandle_t detection_to_counting_queue;
    QueueHandle_t counting_to_uart_queue;
    
} system_config_t;

// ============================================================================
// Helpers pour allocation/libération
// ============================================================================

/**
 * @brief Alloue une structure detection_result_event_t avec tableau de personnes
 * 
 * @param max_persons Nombre maximum de personnes à détecter
 * @return Pointeur vers la structure allouée, ou NULL en cas d'erreur
 */
static inline detection_result_event_t* alloc_detection_result(int max_persons)
{
    detection_result_event_t* result = (detection_result_event_t*)malloc(sizeof(detection_result_event_t));
    if (result)
    {
        result->persons = (detected_person_t*)malloc(max_persons * sizeof(detected_person_t));
        result->max_persons = max_persons;
        result->people_count = 0;
        if (!result->persons)
        {
            free(result);
            return NULL;
        }
    }
    return result;
}

/**
 * @brief Libère une structure detection_result_event_t
 */
static inline void free_detection_result(detection_result_event_t* result)
{
    if (result)
    {
        if (result->persons)
        {
            free(result->persons);
        }
        free(result);
    }
}

#endif // EVENT_TYPES_H
