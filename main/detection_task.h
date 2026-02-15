#ifndef DETECTION_TASK_H
#define DETECTION_TASK_H

#include "event_types.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/**
 * @file detection_task.h
 * @brief Tâche de détection humaine avec esp-dl
 * 
 * Cette tâche est responsable de :
 * - Recevoir les frames de la queue caméra
 * - Redimensionner les images pour le modèle
 * - Exécuter l'inférence (détection de piétons)
 * - Envoyer les résultats vers la queue de comptage
 */

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Point d'entrée de la tâche de détection
 * 
 * @param pvParameters Pointeur vers system_config_t
 */
void detection_task(void* pvParameters);

/**
 * @brief Créer et démarrer la tâche de détection
 * 
 * @param config Configuration système
 * @return TaskHandle_t Handle de la tâche créée
 */
TaskHandle_t detection_task_start(system_config_t* config);

#ifdef __cplusplus
}
#endif

#endif // DETECTION_TASK_H
