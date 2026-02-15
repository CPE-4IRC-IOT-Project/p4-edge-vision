#ifndef UART_TASK_H
#define UART_TASK_H

#include "event_types.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/**
 * @file uart_task.h
 * @brief Tâche de communication UART vers STM32
 * 
 * Cette tâche est responsable de :
 * - Recevoir les événements de comptage
 * - Construire les trames selon le protocole UART
 * - Envoyer les trames binaires vers le STM32
 */

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Point d'entrée de la tâche UART
 * 
 * @param pvParameters Pointeur vers system_config_t
 */
void uart_task(void* pvParameters);

/**
 * @brief Créer et démarrer la tâche UART
 * 
 * @param config Configuration système
 * @return TaskHandle_t Handle de la tâche créée
 */
TaskHandle_t uart_task_start(system_config_t* config);

#ifdef __cplusplus
}
#endif

#endif // UART_TASK_H
