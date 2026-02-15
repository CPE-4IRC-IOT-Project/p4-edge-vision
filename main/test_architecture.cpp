/**
 * @file test_architecture.cpp
 * @brief Fichier de test pour valider les structures et l'intégration
 * 
 * Ce fichier permet de tester individuellement les composants sans
 * avoir besoin de tout le matériel.
 */

#include "event_types.h"
#include "protocol_uart.h"
#include <stdio.h>
#include <assert.h>

static const char* TAG = "test";

/**
 * @brief Test de la structure camera_frame_event
 */
void test_camera_frame_event()
{
    printf("[TEST] camera_frame_event_t... ");
    
    camera_frame_event_t evt = {
        .frame_data = nullptr,
        .width = 1920,
        .height = 1080,
        .timestamp_us = 1234567890,
        .frame_id = 42
    };
    
    assert(evt.width == 1920);
    assert(evt.height == 1080);
    assert(evt.frame_id == 42);
    
    printf("OK\n");
}

/**
 * @brief Test de la structure detection_result_event
 */
void test_detection_result_event()
{
    printf("[TEST] detection_result_event_t... ");
    
    detection_result_event_t* result = alloc_detection_result(10);
    assert(result != nullptr);
    assert(result->max_persons == 10);
    assert(result->persons != nullptr);
    
    result->people_count = 3;
    result->avg_confidence = 0.85f;
    
    // Remplir quelques détections
    result->persons[0] = {10, 20, 50, 80, 0.9f, 0};
    result->persons[1] = {60, 30, 100, 90, 0.85f, 1};
    result->persons[2] = {120, 40, 160, 100, 0.80f, 2};
    
    assert(result->people_count == 3);
    assert(result->persons[0].confidence == 0.9f);
    
    free_detection_result(result);
    
    printf("OK\n");
}

/**
 * @brief Test de la structure count_event
 */
void test_count_event()
{
    printf("[TEST] count_event_t... ");
    
    count_event_t evt = {
        .type = COUNT_EVENT_CHANGE,
        .count = 5,
        .confidence = 87,
        .flags = FLAG_DAYLIGHT | FLAG_MOTION,
        .timestamp = 1705185040,
        .node_id = 0x01,
        .counter = 42
    };
    
    assert(evt.count == 5);
    assert(evt.confidence == 87);
    assert(evt.flags & FLAG_DAYLIGHT);
    assert(evt.flags & FLAG_MOTION);
    
    printf("OK\n");
}

/**
 * @brief Test du protocole UART (construction + validation)
 */
void test_uart_protocol()
{
    printf("[TEST] uart_event_frame_t... ");
    
    uart_event_frame_t frame;
    
    build_event_frame(&frame,
                      0x01,           // node_id
                      1705185040,     // timestamp
                      0x11,           // event_id (CHANGE)
                      5,              // count
                      87,             // confidence
                      FLAG_DAYLIGHT,  // flags
                      42);            // counter
    
    // Vérifications de base
    assert(frame.sof == SOF_BYTE);
    assert(frame.len == FRAME_LENGTH);
    assert(frame.node_id == 0x01);
    assert(frame.count == 5);
    assert(frame.confidence == 87);
    
    // Vérification du CRC
    uint16_t crc_received = __builtin_bswap16(frame.crc);
    uint16_t crc_calculated = crc16_ccitt((const uint8_t*)&frame, FRAME_LENGTH - 2);
    assert(crc_received == crc_calculated);
    
    printf("OK\n");
    
    // Afficher la trame en hexadécimal
    printf("[TEST] Frame hex: ");
    uint8_t* bytes = (uint8_t*)&frame;
    for (int i = 0; i < FRAME_LENGTH; i++) {
        printf("%02X ", bytes[i]);
    }
    printf("\n");
}

/**
 * @brief Test de validation de trame UART
 */
void test_uart_validation()
{
    printf("[TEST] UART frame validation... ");
    
    uart_event_frame_t frame;
    build_event_frame(&frame, 0x01, 1234567890, 0x10, 3, 75, 0x01, 10);
    
    // Validation SOF
    assert(frame.sof == SOF_BYTE);
    
    // Validation longueur
    assert(frame.len == FRAME_LENGTH);
    
    // Validation CRC
    uint16_t crc_received = __builtin_bswap16(frame.crc);
    uint16_t crc_calculated = crc16_ccitt((const uint8_t*)&frame, FRAME_LENGTH - 2);
    assert(crc_received == crc_calculated);
    
    printf("OK\n");
}

/**
 * @brief Test de corruption de trame (CRC doit échouer)
 */
void test_uart_corruption()
{
    printf("[TEST] UART corruption detection... ");
    
    uart_event_frame_t frame;
    build_event_frame(&frame, 0x01, 1234567890, 0x10, 3, 75, 0x01, 10);
    
    // Sauvegarder le CRC original
    uint16_t original_crc = frame.crc;
    
    // Corrompre un octet
    frame.count = 99;  // Changer le comptage
    
    // Vérifier que le CRC ne correspond plus
    uint16_t crc_received = __builtin_bswap16(original_crc);
    uint16_t crc_calculated = crc16_ccitt((const uint8_t*)&frame, FRAME_LENGTH - 2);
    assert(crc_received != crc_calculated);
    
    printf("OK\n");
}

/**
 * @brief Point d'entrée des tests
 */
int main(void)
{
    printf("\n");
    printf("========================================\n");
    printf("  ESP32-P4-EYE Architecture Tests\n");
    printf("========================================\n");
    printf("\n");
    
    test_camera_frame_event();
    test_detection_result_event();
    test_count_event();
    test_uart_protocol();
    test_uart_validation();
    test_uart_corruption();
    
    printf("\n");
    printf("========================================\n");
    printf("  All tests passed! ✓\n");
    printf("========================================\n");
    printf("\n");
    
    return 0;
}
