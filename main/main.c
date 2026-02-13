#include "driver/uart.h"
#include "esp_log.h"
#include "esp_random.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "protocol_uart.h"
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <sys/time.h>

#define UART_PORT UART_NUM_0 // GPIO37 = TX UART0 sur ESP32-P4
#define BAUD_RATE 115200
#define NODE_ID   0x01       // ID de ce noeud (caméra salle 1)

// Configuration de l'envoi événementiel
#define CHECK_INTERVAL_MS    1000    // Vérification toutes les 1s
#define HEARTBEAT_INTERVAL_S 120     // Heartbeat toutes les 2 minutes
#define MIN_COUNT_CHANGE     1       // Envoyer si changement >= 1 personne

// Simule un comptage de personnes (à remplacer par votre vrai algo de vision)
static uint8_t simulate_person_count(void) {
    static uint8_t count = 0;
    static int direction = 1;
    
    // Simulation simple: monte de 0 à 10 puis redescend
    count += direction;
    if (count >= 10) direction = -1;
    if (count <= 0) direction = 1;
    
    return count;
}

// Fonction pour envoyer une trame
static void send_event_frame(uint8_t count, uint8_t confidence, uint8_t flags, uint8_t event_id, uint16_t *counter) {
    uart_event_frame_t frame;
    
    // Récupérer le timestamp Unix
    struct timeval tv;
    gettimeofday(&tv, NULL);
    uint32_t timestamp = (uint32_t)tv.tv_sec;
    
    // Incrémenter le compteur anti-rejeu
    (*counter)++;
    
    // Construire la trame
    build_event_frame(&frame, NODE_ID, timestamp, event_id,
                     count, confidence, flags, *counter);
    
    // Envoyer sur UART
    uart_write_bytes(UART_PORT, (const char*)&frame, sizeof(frame));
    uart_wait_tx_done(UART_PORT, pdMS_TO_TICKS(50));
}

void app_main(void)
{
    // Coupe les logs applicatifs pour ne pas polluer la ligne UART vers le STM32.
    esp_log_level_set("*", ESP_LOG_NONE);

    // Si le driver UART0 existe deja (console), cette installation peut echouer sans gravite.
    uart_driver_install(UART_PORT, 1024, 0, 0, NULL, 0);
    uart_set_baudrate(UART_PORT, BAUD_RATE);

    // Laisse finir le boot/log initial avant d'envoyer nos trames applicatives.
    vTaskDelay(pdMS_TO_TICKS(1500));

    // Variables pour la logique événementielle
    uint16_t counter = 0;           // Compteur anti-rejeu
    uint8_t last_count = 0;         // Dernier comptage envoyé
    uint32_t last_send_time = 0;    // Timestamp du dernier envoi (en secondes)
    
    // Initialiser le timestamp
    struct timeval tv;
    gettimeofday(&tv, NULL);
    last_send_time = (uint32_t)tv.tv_sec;
    
    // Envoyer une trame initiale
    uint8_t current_count = simulate_person_count();
    uint8_t confidence = 85;  // Confiance initiale 85%
    uint8_t flags = FLAG_DAYLIGHT;  // Mode jour par défaut
    
    send_event_frame(current_count, confidence, flags, EVENT_PERSON_COUNT, &counter);
    last_count = current_count;
    
    // Boucle principale: envoi événementiel
    while (1)
    {
        vTaskDelay(pdMS_TO_TICKS(CHECK_INTERVAL_MS));
        
        // Simuler le comptage (à remplacer par votre détection réelle)
        current_count = simulate_person_count();
        confidence = 85 + (esp_random() % 15);  // Confiance 85-100%
        
        // Obtenir le temps actuel
        gettimeofday(&tv, NULL);
        uint32_t current_time = (uint32_t)tv.tv_sec;
        
        // Déterminer les flags (exemple: basculer jour/nuit toutes les 5 minutes)
        flags = ((current_time / 300) % 2 == 0) ? FLAG_DAYLIGHT : FLAG_NIGHT;
        
        // Conditions d'envoi:
        // 1. Changement significatif du comptage (>= MIN_COUNT_CHANGE)
        // 2. OU heartbeat timeout (pas d'envoi depuis HEARTBEAT_INTERVAL_S)
        
        int count_change = (int)current_count - (int)last_count;
        if (count_change < 0) count_change = -count_change;  // Valeur absolue
        
        bool should_send = false;
        uint8_t event_id = EVENT_PERSON_COUNT;
        
        if (count_change >= MIN_COUNT_CHANGE) {
            // Changement détecté
            should_send = true;
            event_id = EVENT_PERSON_COUNT_CHANGE;
        } else if ((current_time - last_send_time) >= HEARTBEAT_INTERVAL_S) {
            // Heartbeat timeout
            should_send = true;
            event_id = EVENT_PERSON_COUNT;
        }
        
        if (should_send) {
            send_event_frame(current_count, confidence, flags, event_id, &counter);
            last_count = current_count;
            last_send_time = current_time;
        }
    }
}
