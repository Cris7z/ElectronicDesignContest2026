/**
 * bsp.h — 硬件抽象层。移植到你的板子只需要实现这一个头文件里的函数。
 * MSPM0G3507 参考实现见 bsp_mspm0.c (配合 SysConfig 生成的 ti_msp_dl_config)。
 * PC 仿真实现见 ../../sim/sim_main.c。
 */
#ifndef BSP_H
#define BSP_H

#include <stdint.h>
#include <stdbool.h>

/* ---- 时基 ---- */
uint32_t bsp_millis(void);

/* ---- 电机: duty [-1,1], 正=前进 (方向不对就在实现里翻转) ---- */
void bsp_motor_set(float duty_l, float duty_r);

/* ---- 编码器累计计数(有符号, 前进增加) ---- */
int32_t bsp_encoder_left(void);
int32_t bsp_encoder_right(void);

/* ---- 灰度: 二选一, 用哪种在 bsp_config.h 里选 ---- */
void    bsp_gray_read_analog(uint16_t raw[8]);
uint8_t bsp_gray_read_digital(void);

/* ---- 人机 ---- */
void bsp_buzzer(bool on);
void bsp_led(int idx, bool on);
uint8_t bsp_keys(void);              /* bit0..3 = KEY1..4, 1=按下 */

/* ---- OLED (I2C) ---- */
void bsp_oled_write_cmd(uint8_t cmd);
void bsp_oled_write_data(const uint8_t *data, uint16_t len);

/* ---- 串口发送(阻塞或DMA均可) ---- */
void bsp_uart_debug_tx(const uint8_t *data, uint16_t len);  /* 蓝牙/VOFA */
void bsp_uart_radio_tx(const uint8_t *data, uint16_t len);  /* 双车/图传 */

/* ---- flash 参数区(掉电保存 PID 参数) ---- */
bool bsp_flash_save(const void *data, uint16_t len);
bool bsp_flash_load(void *data, uint16_t len);

/* ---- 初始化(时钟/外设全部就绪后返回) ---- */
void bsp_init(void);

/* 中断回调: BSP 收到串口字节时调用这些(在 main.c 里实现) */
void app_on_imu_byte(uint8_t b);
void app_on_openmv_byte(uint8_t b);
void app_on_debug_byte(uint8_t b);
void app_on_radio_byte(uint8_t b);
/* 5ms 定时器中断回调(速度环) */
void app_on_ctrl_tick(void);

#endif
