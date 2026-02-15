#ifndef CAMERA_TASK_H
#define CAMERA_TASK_H

#include "event_types.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/**
 * @file camera_task.h
 * @brief Tâche de capture caméra
 * 
 * Cette tâche est responsable de :
 * - Initialiser la caméra via BSP
 * - Capturer les frames en continu
 * - Envoyer les frames vers la queue de détection
 */

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Point d'entrée de la tâche caméra
 * 
 * @param pvParameters Pointeur vers system_config_t
 */
void camera_task(void* pvParameters);

/**
 * @brief Créer et démarrer la tâche caméra
 * 
 * @param config Configuration système
 * @return TaskHandle_t Handle de la tâche créée
 */
TaskHandle_t camera_task_start(system_config_t* config);

#ifdef __cplusplus
}
#endif

#endif // CAMERA_TASK_H
