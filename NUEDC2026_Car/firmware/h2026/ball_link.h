/**
 * ball_link.h - 01Studio CanMV K230 -> MSPM0 ball-state protocol.
 *
 * Fixed 17-byte little-endian frame:
 * AA 55 | 10 | seq | t_ms:u32 | x_mm:i16 | vx_mm_s:i16 |
 * confidence:u8 | flags:u8 | crc16:u16 | 0D
 *
 * CRC-16/CCITT-FALSE covers bytes [2..13].
 */
#ifndef H2026_BALL_LINK_H
#define H2026_BALL_LINK_H

#include <stdbool.h>
#include <stdint.h>

#define BALL_LINK_FRAME_LEN       17u
#define BALL_LINK_TYPE_STATE      0x10u
#define BALL_LINK_FLAG_VALID      0x01u
#define BALL_LINK_FLAG_CALIBRATED 0x02u

typedef struct {
    int16_t x_mm;
    int16_t vx_mm_s;
    uint32_t camera_ms;
    uint32_t received_ms;
    uint32_t frames_ok;
    uint32_t crc_errors;
    uint32_t framing_errors;
    uint32_t sequence_gaps;
    uint8_t confidence;
    uint8_t flags;
    uint8_t sequence;
    bool has_state;

    uint8_t buffer[BALL_LINK_FRAME_LEN];
    uint8_t index;
} ball_link_t;

void ball_link_init(ball_link_t *link);
void ball_link_rx_byte(ball_link_t *link, uint8_t byte, uint32_t now_ms);
bool ball_link_alive(const ball_link_t *link, uint32_t now_ms,
                     uint32_t timeout_ms, uint8_t min_confidence);
uint16_t ball_link_crc16(const uint8_t *data, uint16_t length);

#endif
