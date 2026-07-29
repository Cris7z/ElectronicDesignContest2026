#include "oled_ssd1306.h"
#include "oled_font8x16.h"
#include "../bsp/bsp.h"
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

static const uint8_t init_seq[] = {
    0xAE, 0x20, 0x00, 0xB0, 0xC8, 0x00, 0x10, 0x40, 0x81, 0x7F,
    0xA1, 0xA6, 0xA8, 0x3F, 0xA4, 0xD3, 0x00, 0xD5, 0xF0, 0xD9,
    0x22, 0xDA, 0x12, 0xDB, 0x20, 0x8D, 0x14, 0xAF
};

static void set_pos(uint8_t page, uint8_t x)
{
    bsp_oled_write_cmd(0xB0 | page);
    bsp_oled_write_cmd(0x00 | (x & 0x0F));
    bsp_oled_write_cmd(0x10 | (x >> 4));
}

void oled_init(void)
{
    for (unsigned i = 0; i < sizeof(init_seq); i++)
        bsp_oled_write_cmd(init_seq[i]);
    oled_clear();
}

void oled_clear(void)
{
    uint8_t zero[16];
    memset(zero, 0, sizeof(zero));
    for (uint8_t p = 0; p < 8; p++) {
        set_pos(p, 0);
        for (uint8_t x = 0; x < 128; x += 16)
            bsp_oled_write_data(zero, 16);
    }
}

void oled_str(uint8_t row, uint8_t col, const char *s)
{
    uint8_t page = row * 2, x = col * 8;
    for (const char *c = s; *c && x <= 120; c++, x += 8) {
        uint8_t idx = (*c < 32 || *c > 126) ? 0 : (uint8_t)(*c - 32);
        set_pos(page, x);
        bsp_oled_write_data(&g_font8x16[idx][0], 8);
        set_pos(page + 1, x);
        bsp_oled_write_data(&g_font8x16[idx][8], 8);
    }
}

void oled_printf(uint8_t row, uint8_t col, const char *fmt, ...)
{
    char buf[24];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    oled_str(row, col, buf);
}
