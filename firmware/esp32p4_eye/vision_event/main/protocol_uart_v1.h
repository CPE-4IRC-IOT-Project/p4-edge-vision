#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define UART_V1_SOF1              0x55
#define UART_V1_SOF2              0xAA
#define UART_V1_VERSION           0x01
#define UART_V1_PAYLOAD_LEN       16
#define UART_V1_FRAME_LEN         (2 + 1 + UART_V1_PAYLOAD_LEN + 2)

#define UART_V1_MSG_HEARTBEAT         0x01
#define UART_V1_MSG_OCCUPANCY_CHANGED 0x02

#define UART_V1_FLAG_LOW_LIGHT    (1U << 0)

typedef struct __attribute__((packed)) {
    uint8_t ver;
    uint8_t msg_type;
    uint8_t node_id;
    uint8_t flags;
    uint8_t luma;
    uint8_t occupied;
    uint8_t stable_count;
    uint8_t raw_count;
    uint32_t counter;
    uint32_t uptime_s;
} vision_uart_payload_v1_t;

static inline uint16_t uart_v1_crc16_ccitt(const uint8_t *data, size_t len)
{
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; ++i) {
        crc ^= (uint16_t)data[i] << 8;
        for (int b = 0; b < 8; ++b) {
            if ((crc & 0x8000U) != 0U) {
                crc = (uint16_t)((crc << 1) ^ 0x1021U);
            } else {
                crc = (uint16_t)(crc << 1);
            }
        }
    }
    return crc;
}

static inline void uart_v1_write_be32(uint8_t *dst, uint32_t value)
{
    dst[0] = (uint8_t)(value >> 24);
    dst[1] = (uint8_t)(value >> 16);
    dst[2] = (uint8_t)(value >> 8);
    dst[3] = (uint8_t)value;
}

static inline uint32_t uart_v1_read_be32(const uint8_t *src)
{
    return ((uint32_t)src[0] << 24)
        | ((uint32_t)src[1] << 16)
        | ((uint32_t)src[2] << 8)
        | (uint32_t)src[3];
}

static inline void serialize_payload_v1(const vision_uart_payload_v1_t *payload, uint8_t out[UART_V1_PAYLOAD_LEN])
{
    out[0] = payload->ver;
    out[1] = payload->msg_type;
    out[2] = payload->node_id;
    out[3] = payload->flags;
    out[4] = payload->luma;
    out[5] = payload->occupied;
    out[6] = payload->stable_count;
    out[7] = payload->raw_count;
    uart_v1_write_be32(&out[8], payload->counter);
    uart_v1_write_be32(&out[12], payload->uptime_s);
}

static inline void deserialize_payload_v1(vision_uart_payload_v1_t *payload, const uint8_t in[UART_V1_PAYLOAD_LEN])
{
    payload->ver = in[0];
    payload->msg_type = in[1];
    payload->node_id = in[2];
    payload->flags = in[3];
    payload->luma = in[4];
    payload->occupied = in[5];
    payload->stable_count = in[6];
    payload->raw_count = in[7];
    payload->counter = uart_v1_read_be32(&in[8]);
    payload->uptime_s = uart_v1_read_be32(&in[12]);
}

static inline size_t build_uart_frame_v1(const vision_uart_payload_v1_t *payload, uint8_t out[UART_V1_FRAME_LEN])
{
    out[0] = UART_V1_SOF1;
    out[1] = UART_V1_SOF2;
    out[2] = UART_V1_PAYLOAD_LEN;
    serialize_payload_v1(payload, &out[3]);

    uint16_t crc = uart_v1_crc16_ccitt(&out[2], 1U + UART_V1_PAYLOAD_LEN);
    out[3 + UART_V1_PAYLOAD_LEN] = (uint8_t)(crc >> 8);
    out[4 + UART_V1_PAYLOAD_LEN] = (uint8_t)crc;
    return UART_V1_FRAME_LEN;
}

#ifdef __cplusplus
}
#endif
