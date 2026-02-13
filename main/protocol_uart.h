#ifndef PROTOCOL_UART_H
#define PROTOCOL_UART_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

// ============================================================================
// PROTOCOLE UART ESP32-P4 -> STM32 -> LoRaWAN
// Format: 16 octets binaires avec CRC-16
// ============================================================================

// Constantes du protocole
#define SOF_BYTE            0xAA        // Start of Frame
#define FRAME_LENGTH        16          // Longueur totale de la trame (octets)
#define PROTOCOL_VERSION    0x01        // Version 1.0

// Types d'événements
#define EVENT_PERSON_COUNT         0x10  // Comptage périodique (heartbeat)
#define EVENT_PERSON_COUNT_CHANGE  0x11  // Changement détecté
#define EVENT_ALARM                0x20  // Alarme (trop de monde, etc.)
#define EVENT_SYSTEM_STATUS        0x30  // Status système

// Flags (bits)
#define FLAG_DAYLIGHT       0x01        // Bit 0: Jour=1, Nuit=0
#define FLAG_MOTION         0x02        // Bit 1: Mouvement détecté
#define FLAG_ALARM          0x04        // Bit 2: Alarme active
#define FLAG_LOW_BATTERY    0x08        // Bit 3: Batterie faible (future)

// ============================================================================
// STRUCTURE DE LA TRAME (16 octets)
// ============================================================================
typedef struct __attribute__((packed)) {
    uint8_t  sof;           // 0: Start of Frame (0xAA)
    uint8_t  len;           // 1: Longueur totale (16)
    uint8_t  type;          // 2: Type d'événement (EVENT_xxx)
    uint8_t  node_id;       // 3: ID du noeud (caméra)
    uint32_t timestamp;     // 4-7: Timestamp Unix (big-endian)
    uint8_t  event_id;      // 8: ID événement spécifique
    uint8_t  count;         // 9: Nombre de personnes
    uint8_t  confidence;    // 10: Confiance (0-100%)
    uint8_t  flags;         // 11: Flags (jour/nuit, mouvement, etc.)
    uint16_t counter;       // 12-13: Compteur anti-rejeu (big-endian)
    uint16_t crc;           // 14-15: CRC-16-CCITT (big-endian)
} uart_event_frame_t;

// ============================================================================
// CRC-16-CCITT (polynôme 0x1021, init 0xFFFF)
// ============================================================================
static inline uint16_t crc16_ccitt(const uint8_t *data, size_t len) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (int j = 0; j < 8; j++) {
            if (crc & 0x8000) {
                crc = (crc << 1) ^ 0x1021;
            } else {
                crc <<= 1;
            }
        }
    }
    return crc;
}

// ============================================================================
// Construction d'une trame
// ============================================================================
static inline void build_event_frame(uart_event_frame_t *frame,
                                     uint8_t node_id,
                                     uint32_t timestamp,
                                     uint8_t event_id,
                                     uint8_t count,
                                     uint8_t confidence,
                                     uint8_t flags,
                                     uint16_t counter)
{
    frame->sof = SOF_BYTE;
    frame->len = FRAME_LENGTH;
    frame->type = EVENT_PERSON_COUNT;  // Type principal
    frame->node_id = node_id;
    
    // Timestamp en big-endian
    frame->timestamp = __builtin_bswap32(timestamp);
    
    frame->event_id = event_id;
    frame->count = count;
    frame->confidence = confidence;
    frame->flags = flags;
    
    // Counter en big-endian
    frame->counter = __builtin_bswap16(counter);
    
    // CRC sur tous les octets sauf le CRC lui-même
    uint16_t crc = crc16_ccitt((const uint8_t*)frame, FRAME_LENGTH - 2);
    frame->crc = __builtin_bswap16(crc);
}

// ============================================================================
// Validation d'une trame
// ============================================================================
static inline bool validate_frame(const uart_event_frame_t *frame, uint16_t last_counter) {
    // Vérifier SOF
    if (frame->sof != SOF_BYTE) {
        return false;
    }
    
    // Vérifier longueur
    if (frame->len != FRAME_LENGTH) {
        return false;
    }
    
    // Vérifier CRC
    uint16_t calc_crc = crc16_ccitt((const uint8_t*)frame, FRAME_LENGTH - 2);
    uint16_t recv_crc = __builtin_bswap16(frame->crc);
    if (calc_crc != recv_crc) {
        return false;
    }
    
    // Vérifier compteur anti-rejeu (doit être > last_counter)
    uint16_t current_counter = __builtin_bswap16(frame->counter);
    if (current_counter <= last_counter && last_counter != 0) {
        return false;  // Rejeu détecté
    }
    
    return true;
}

// ============================================================================
// Extraction des données (conversion big-endian -> host)
// ============================================================================
static inline uint32_t get_timestamp(const uart_event_frame_t *frame) {
    return __builtin_bswap32(frame->timestamp);
}

static inline uint16_t get_counter(const uart_event_frame_t *frame) {
    return __builtin_bswap16(frame->counter);
}

#endif // PROTOCOL_UART_H
