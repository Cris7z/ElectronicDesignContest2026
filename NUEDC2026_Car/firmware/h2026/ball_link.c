#include "ball_link.h"

static uint16_t read_u16_le(const uint8_t *p)
{
    return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}
static uint32_t read_u32_le(const uint8_t *p)
{
    return (uint32_t)p[0] |
           ((uint32_t)p[1] << 8) |
           ((uint32_t)p[2] << 16) |
           ((uint32_t)p[3] << 24);
}

uint16_t ball_link_crc16(const uint8_t *data, uint16_t length)
{
    uint16_t crc = 0xFFFFu;
    for (uint16_t i = 0; i < length; ++i) {
        crc ^= (uint16_t)data[i] << 8;
        for (uint8_t bit = 0; bit < 8u; ++bit) {
            crc = (crc & 0x8000u)
                ? (uint16_t)((crc << 1) ^ 0x1021u)
                : (uint16_t)(crc << 1);
        }
    }
    return crc;
}

void ball_link_init(ball_link_t *link)
{
    *link = (ball_link_t){0};
}

static void consume_frame(ball_link_t *link, uint32_t now_ms)
{
    const uint8_t *b = link->buffer;
    if (b[0] != 0xAAu || b[1] != 0x55u ||
        b[2] != BALL_LINK_TYPE_STATE || b[16] != 0x0Du) {
        ++link->framing_errors;
        return;
    }

    const uint16_t expected = read_u16_le(&b[14]);
    const uint16_t actual = ball_link_crc16(&b[2], 12u);
    if (expected != actual) {
        ++link->crc_errors;
        return;
    }

    const uint8_t seq = b[3];
    if (link->has_state) {
        const uint8_t delta = (uint8_t)(seq - link->sequence);
        if (delta == 0u) {
            return;
        }
        if (delta > 1u) {
            link->sequence_gaps += (uint32_t)(delta - 1u);
        }
    }

    link->sequence = seq;
    link->camera_ms = read_u32_le(&b[4]);
    link->x_mm = (int16_t)read_u16_le(&b[8]);
    link->vx_mm_s = (int16_t)read_u16_le(&b[10]);
    link->confidence = b[12];
    link->flags = b[13];
    link->received_ms = now_ms;
    link->has_state = true;
    ++link->frames_ok;
}

void ball_link_rx_byte(ball_link_t *link, uint8_t byte, uint32_t now_ms)
{
    if (link->index == 0u) {
        if (byte != 0xAAu) {
            return;
        }
        link->buffer[link->index++] = byte;
        return;
    }

    if (link->index == 1u && byte != 0x55u) {
        link->index = (byte == 0xAAu) ? 1u : 0u;
        link->buffer[0] = 0xAAu;
        return;
    }

    link->buffer[link->index++] = byte;
    if (link->index == BALL_LINK_FRAME_LEN) {
        link->index = 0u;
        consume_frame(link, now_ms);
    }
}

bool ball_link_alive(const ball_link_t *link, uint32_t now_ms,
                     uint32_t timeout_ms, uint8_t min_confidence)
{
    return link->has_state &&
           (link->flags & BALL_LINK_FLAG_VALID) != 0u &&
           (link->flags & BALL_LINK_FLAG_CALIBRATED) != 0u &&
           link->confidence >= min_confidence &&
           (uint32_t)(now_ms - link->received_ms) <= timeout_ms;
}
