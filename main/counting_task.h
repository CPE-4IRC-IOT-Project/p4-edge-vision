#ifndef COUNTING_TASK_H
#define COUNTING_TASK_H

#include "event_types.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/**
 * @file counting_task.h
 * @brief Tâche de comptage et logique événementielle
 * 
 * Cette tâche est responsable de :
 * - Recevoir les résultats de détection
 * - Compter le nombre de personnes
 * - Détecter les changements de comptage
 * - Gérer le heartbeat périodique
 * - Gérer les alarmes (seuil dépassé)
 * - Envoyer les événements vers la queue UART
 */

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Point d'entrée de la tâche de comptage
 * 
 * @param pvParameters Pointeur vers system_config_t
 */
void counting_task(void* pvParameters);

/**
 * @brief Créer et démarrer la tâche de comptage
 * 
 * @param config Configuration système
 * @return TaskHandle_t Handle de la tâche créée
 */
TaskHandle_t counting_task_start(system_config_t* config);

#ifdef __cplusplus
}
#endif

#endif // COUNTING_TASK_H
