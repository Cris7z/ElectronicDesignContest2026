/** oled_ssd1306.h — 0.96" 128x64 I2C OLED, 8x16 ASCII */
#ifndef OLED_SSD1306_H
#define OLED_SSD1306_H
#include <stdint.h>
void oled_init(void);
void oled_clear(void);
/* row: 0..3 (每行16像素高), col: 0..15 字符列 */
void oled_str(uint8_t row, uint8_t col, const char *s);
void oled_printf(uint8_t row, uint8_t col, const char *fmt, ...);
#endif
