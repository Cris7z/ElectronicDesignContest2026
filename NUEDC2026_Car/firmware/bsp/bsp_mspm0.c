/**
 * bsp_mspm0.c — MSPM0G3507 参考实现(已对照 TI SDK 2.11.00.07 真实 driverlib 头文件校验语法)
 *
 * !!! 使用方法(重要) !!!
 * 1. CCS 导入 SDK 的 empty(LP_MSPM0G3507)例程, 把 firmware/ 整体拷进工程;
 * 2. 用同目录的 car_mspm0g3507.syscfg 替换工程里的 empty.syscfg
 *    (在 SysConfig GUI 里打开一次, 解决它标红的引脚冲突后保存生成代码);
 * 3. 工程属性定义编译宏 TARGET_MSPM0;
 * 4. 编译。若你在 SysConfig 里改了实例名, 对照下表改本文件顶部宏即可。
 *
 * 本文件期望的 SysConfig 实例名($name)→ 生成宏对照表:
 *   PWM_MOTOR  (TIMG0, 2路20kHz)   → PWM_MOTOR_INST
 *   TIMER_CTRL (TIMG6, 5ms 周期)   → TIMER_CTRL_INST / _INT_IRQN / _IRQHandler
 *   UART_DEBUG (UART0 115200)      → UART_DEBUG_INST / ...
 *   UART_IMU   (UART1 9600 JY61P)  → UART_IMU_INST / ...
 *   UART_OPENMV(UART2 115200)      → UART_OPENMV_INST / ...
 *   UART_RADIO (UART3 115200)      → UART_RADIO_INST / ...
 *   ADC_GRAY   (ADC0 8通道序列)    → ADC_GRAY_INST / ...
 *   I2C_OLED   (I2C1 400kHz)       → I2C_OLED_INST
 *   GPIO组: GPIO_ENC_L{LA,LB} GPIO_ENC_R{RA,RB}(pin 名全局唯一, 别改成重名)
 *           GPIO_MOTOR{AIN1,AIN2,BIN1,BIN2} GPIO_KEY{K1..K4} GPIO_MISC{BUZZER,LED1,LED2}
 *
 * !!! 链接脚本必改一处 !!!
 * 参数区占用主 flash 最后 1KB(0x1FC00), 必须把工程链接文件里 FLASH 的
 * length 从 0x00020000 改成 0x0001FC00(ticlang: device_linker.cmd,
 * gcc: device_linker.lds), 否则固件长大后会和参数区静默重叠,
 * 保存参数 = 擦掉自己的代码。
 *
 * 架构说明(与最初设想的差异, 原因见 reference/NOTES-MSPM0-driverlib.md):
 * - G3507 只有 TIMG8 一个硬件 QEI, 双编码器无法都用 QEI。本实现采用 2025 年
 *   电赛真车验证过的方案: 两路编码器都走 GPIO 上升沿中断软件解码(2倍频),
 *   GPIOA/GPIOB 共用 GROUP1_IRQHandler。编码器每转计数因此是 13线*2*减速比,
 *   实测填入 bsp_config.h 的 CFG_COUNTS_PER_REV。
 * - PWM 采用 SysConfig 默认输出极性(OCTL 初值低), 占空比映射为
 *   cc = (1-duty)*period。若上板发现占空比反了(灯常亮/电机满转), 把
 *   motor_duty_to_cc() 里的一行改回 duty*period 即可。
 */
#ifdef TARGET_MSPM0

#include <string.h>
#include "bsp.h"
#include "bsp_config.h"
#include "ti_msp_dl_config.h"   /* SysConfig 生成 */

static volatile uint32_t s_ms = 0;
static volatile int32_t  s_enc_l = 0, s_enc_r = 0;
static uint32_t s_pwm_period = 1600;    /* bsp_init 里从定时器 LOAD 读真值 */
static volatile bool s_adc_done = false;
static volatile uint32_t s_adc_timeouts = 0;  /* 调试器观察: 非零=ADC 采样异常 */

/* 自旋等待上限(防外设失联卡死主循环; 32MHz 下约几个 ms) */
#define BSP_SPIN_LIMIT  200000u

/* ---------------- 时基 ---------------- */
void SysTick_Handler(void) { s_ms++; }
uint32_t bsp_millis(void) { return s_ms; }

/* ---------------- 电机 ---------------- */
/* TB6612: PWM_MOTOR CC0=左 CC1=右, 方向脚 AIN/BIN。
 * SysConfig 默认 PWM 极性(EDGE_ALIGN 下计数+OCTL初值低)下 cc 越大占空比
 * 越小, 故用 (1-duty)。注意: 2025 真车 motor.c 直写 cc=speed 是因为它在
 * syscfg 里显式选了 EDGE_ALIGN_UP —— 别照它"改回去", 会得到反相输出。 */
static uint32_t motor_duty_to_cc(float d)
{
    if (d > 1.0f) d = 1.0f;
    if (d < 0.0f) d = 0.0f;
    return (uint32_t)((1.0f - d) * (float)s_pwm_period);
}

void bsp_motor_set(float duty_l, float duty_r)
{
    duty_l *= (float)CFG_MOTOR_L_SIGN;
    duty_r *= (float)CFG_MOTOR_R_SIGN;

    if (duty_l >= 0) {
        DL_GPIO_setPins(GPIO_MOTOR_PORT, GPIO_MOTOR_AIN1_PIN);
        DL_GPIO_clearPins(GPIO_MOTOR_PORT, GPIO_MOTOR_AIN2_PIN);
    } else {
        DL_GPIO_clearPins(GPIO_MOTOR_PORT, GPIO_MOTOR_AIN1_PIN);
        DL_GPIO_setPins(GPIO_MOTOR_PORT, GPIO_MOTOR_AIN2_PIN);
        duty_l = -duty_l;
    }
    if (duty_r >= 0) {
        DL_GPIO_setPins(GPIO_MOTOR_PORT, GPIO_MOTOR_BIN1_PIN);
        DL_GPIO_clearPins(GPIO_MOTOR_PORT, GPIO_MOTOR_BIN2_PIN);
    } else {
        DL_GPIO_clearPins(GPIO_MOTOR_PORT, GPIO_MOTOR_BIN1_PIN);
        DL_GPIO_setPins(GPIO_MOTOR_PORT, GPIO_MOTOR_BIN2_PIN);
        duty_r = -duty_r;
    }

    DL_TimerG_setCaptureCompareValue(PWM_MOTOR_INST,
        motor_duty_to_cc(duty_l), DL_TIMER_CC_0_INDEX);
    DL_TimerG_setCaptureCompareValue(PWM_MOTOR_INST,
        motor_duty_to_cc(duty_r), DL_TIMER_CC_1_INDEX);
}

/* ---------------- 编码器(GPIO 中断软件解码, 2倍频) ----------------
 * A/B 相都配上升沿中断; A 沿到时 B 的电平给方向, B 沿到时 A 的电平给方向。
 * GPIOA 和 GPIOB 的中断在 G3507 上同属 GROUP1, 只有一个共享向量。
 * 方向不对时改 bsp_config.h 的 CFG_ENC_L_SIGN / CFG_ENC_R_SIGN, 别改这里。 */
void GROUP1_IRQHandler(void)
{
    uint32_t a = DL_GPIO_getEnabledInterruptStatus(GPIO_ENC_L_PORT,
                     GPIO_ENC_L_LA_PIN | GPIO_ENC_L_LB_PIN);
    uint32_t b = DL_GPIO_getEnabledInterruptStatus(GPIO_ENC_R_PORT,
                     GPIO_ENC_R_RA_PIN | GPIO_ENC_R_RB_PIN);

    if (a & GPIO_ENC_L_LA_PIN) {
        s_enc_l += (DL_GPIO_readPins(GPIO_ENC_L_PORT, GPIO_ENC_L_LB_PIN) == 0)
                   ? CFG_ENC_L_SIGN : -CFG_ENC_L_SIGN;
        DL_GPIO_clearInterruptStatus(GPIO_ENC_L_PORT, GPIO_ENC_L_LA_PIN);
    }
    if (a & GPIO_ENC_L_LB_PIN) {
        s_enc_l += (DL_GPIO_readPins(GPIO_ENC_L_PORT, GPIO_ENC_L_LA_PIN) != 0)
                   ? CFG_ENC_L_SIGN : -CFG_ENC_L_SIGN;
        DL_GPIO_clearInterruptStatus(GPIO_ENC_L_PORT, GPIO_ENC_L_LB_PIN);
    }
    if (b & GPIO_ENC_R_RA_PIN) {
        s_enc_r += (DL_GPIO_readPins(GPIO_ENC_R_PORT, GPIO_ENC_R_RB_PIN) == 0)
                   ? CFG_ENC_R_SIGN : -CFG_ENC_R_SIGN;
        DL_GPIO_clearInterruptStatus(GPIO_ENC_R_PORT, GPIO_ENC_R_RA_PIN);
    }
    if (b & GPIO_ENC_R_RB_PIN) {
        s_enc_r += (DL_GPIO_readPins(GPIO_ENC_R_PORT, GPIO_ENC_R_RA_PIN) != 0)
                   ? CFG_ENC_R_SIGN : -CFG_ENC_R_SIGN;
        DL_GPIO_clearInterruptStatus(GPIO_ENC_R_PORT, GPIO_ENC_R_RB_PIN);
    }
}

/* 32 位对齐读在 M0+ 上是单条指令, 无需关中断 */
int32_t bsp_encoder_left(void)  { return s_enc_l; }
int32_t bsp_encoder_right(void) { return s_enc_r; }

/* ---------------- 灰度 ---------------- */
#if CFG_GRAY_ANALOG
/* ADC0 软件触发 8 通道序列(MEM0..7), 末通道装载中断置完成标志。
 * raw[0]=最左 .. raw[7]=最右, 12bit 右对齐 — 接线时按此顺序接 CH0..CH7。 */
void ADC_GRAY_INST_IRQHandler(void)
{
    switch (DL_ADC12_getPendingInterrupt(ADC_GRAY_INST)) {
        case DL_ADC12_IIDX_MEM7_RESULT_LOADED:
            s_adc_done = true;
            break;
        default:
            break;
    }
}

void bsp_gray_read_analog(uint16_t raw[8])
{
    static const DL_ADC12_MEM_IDX mem[8] = {
        DL_ADC12_MEM_IDX_0, DL_ADC12_MEM_IDX_1, DL_ADC12_MEM_IDX_2,
        DL_ADC12_MEM_IDX_3, DL_ADC12_MEM_IDX_4, DL_ADC12_MEM_IDX_5,
        DL_ADC12_MEM_IDX_6, DL_ADC12_MEM_IDX_7
    };
    uint32_t guard = BSP_SPIN_LIMIT;

    s_adc_done = false;
    DL_ADC12_startConversion(ADC_GRAY_INST);
    while (!s_adc_done && --guard) {}
    if (!guard) s_adc_timeouts++;         /* 超时时 raw 里是上一轮旧值 */
    for (int i = 0; i < 8; i++)
        raw[i] = DL_ADC12_getMemResult(ADC_GRAY_INST, mem[i]);
    DL_ADC12_enableConversions(ADC_GRAY_INST);   /* 重新武装下一轮 */
}

uint8_t bsp_gray_read_digital(void) { return 0; }   /* 模拟方案下不用 */

#else /* 数字灰度: 在 SysConfig 加 GPIO_GRAY 组, 8 个输入 D0..D7 后启用 */

void bsp_gray_read_analog(uint16_t raw[8]) { for (int i = 0; i < 8; i++) raw[i] = 0; }

uint8_t bsp_gray_read_digital(void)
{
    static const uint32_t pin[8] = {
        GPIO_GRAY_D0_PIN, GPIO_GRAY_D1_PIN, GPIO_GRAY_D2_PIN, GPIO_GRAY_D3_PIN,
        GPIO_GRAY_D4_PIN, GPIO_GRAY_D5_PIN, GPIO_GRAY_D6_PIN, GPIO_GRAY_D7_PIN
    };
    uint8_t v = 0;
    for (int i = 0; i < 8; i++)
        if (DL_GPIO_readPins(GPIO_GRAY_PORT, pin[i]) != 0) v |= (uint8_t)(1u << i);
    return v;
}
#endif

/* ---------------- 人机 ---------------- */
void bsp_buzzer(bool on)
{
    if (on) DL_GPIO_setPins(GPIO_MISC_PORT, GPIO_MISC_BUZZER_PIN);
    else    DL_GPIO_clearPins(GPIO_MISC_PORT, GPIO_MISC_BUZZER_PIN);
}

void bsp_led(int idx, bool on)
{
    uint32_t pin = (idx == 0) ? GPIO_MISC_LED1_PIN : GPIO_MISC_LED2_PIN;
    if (on) DL_GPIO_setPins(GPIO_MISC_PORT, pin);
    else    DL_GPIO_clearPins(GPIO_MISC_PORT, pin);
}

uint8_t bsp_keys(void)
{
    uint32_t v = DL_GPIO_readPins(GPIO_KEY_PORT,
        GPIO_KEY_K1_PIN | GPIO_KEY_K2_PIN | GPIO_KEY_K3_PIN | GPIO_KEY_K4_PIN);
    uint8_t k = 0;
    if (!(v & GPIO_KEY_K1_PIN)) k |= 1u;   /* 上拉输入, 低电平=按下 */
    if (!(v & GPIO_KEY_K2_PIN)) k |= 2u;
    if (!(v & GPIO_KEY_K3_PIN)) k |= 4u;
    if (!(v & GPIO_KEY_K4_PIN)) k |= 8u;
    return k;
}

/* ---------------- OLED (SSD1306 @ I2C 0x3C) ----------------
 * I2C 控制器 TX FIFO 只有 8 字节, >8 字节必须用 fillControllerTXFIFO 的
 * 返回值循环补投喂(SDK i2c_controller_rw_multibyte_fifo_poll 例程写法)。
 * OLED 掉线保护: 出错/超时冲刷 FIFO 防残留字节错位, 并熔断降频重试,
 * 避免每次调用都烧满自旋上限把 10ms 主循环饿死(行驶中掉屏不能失控)。 */
#define I2C_SPIN_LIMIT   50000u          /* 400kHz 下 17 字节约 0.5ms, 余量充足 */
static uint16_t s_i2c_skip = 0;          /* 熔断: >0 时跳过本次尝试 */

static void i2c_write(uint8_t ctrl, const uint8_t *data, uint16_t len)
{
    uint8_t tx[17];                       /* 1 控制字节 + 最多 16 数据字节 */
    uint16_t total, sent;
    uint32_t guard;
    bool ok = false;

    if (s_i2c_skip) { s_i2c_skip--; return; }

    if (len > 16) len = 16;               /* 调用方最大 16(oled_clear) */
    tx[0] = ctrl;
    memcpy(&tx[1], data, len);
    total = (uint16_t)(len + 1);

    guard = I2C_SPIN_LIMIT;               /* 等空闲 */
    while (!(DL_I2C_getControllerStatus(I2C_OLED_INST) &
             DL_I2C_CONTROLLER_STATUS_IDLE) && --guard) {}

    if (guard) {
        sent = DL_I2C_fillControllerTXFIFO(I2C_OLED_INST, tx, total);
        DL_I2C_startControllerTransfer(I2C_OLED_INST, 0x3C,
            DL_I2C_CONTROLLER_DIRECTION_TX, total);

        guard = I2C_SPIN_LIMIT;           /* FIFO 边发边补 */
        while (sent < total && --guard)
            sent += DL_I2C_fillControllerTXFIFO(I2C_OLED_INST, &tx[sent],
                                                (uint16_t)(total - sent));

        delay_cycles(24);                 /* 勘误 I2C_ERR_13: BUSY 置位有延迟 */
        guard = I2C_SPIN_LIMIT;
        while ((DL_I2C_getControllerStatus(I2C_OLED_INST) &
                DL_I2C_CONTROLLER_STATUS_BUSY) && --guard) {}

        ok = guard && sent == total &&
             !(DL_I2C_getControllerStatus(I2C_OLED_INST) &
               DL_I2C_CONTROLLER_STATUS_ERROR);
    }

    if (!ok) {
        DL_I2C_flushControllerTXFIFO(I2C_OLED_INST);  /* 防残留字节错位 */
        s_i2c_skip = 256;                 /* 熔断约 3 帧菜单刷新后再试 */
    }
}

void bsp_oled_write_cmd(uint8_t cmd)  { i2c_write(0x00, &cmd, 1); }
void bsp_oled_write_data(const uint8_t *d, uint16_t n) { i2c_write(0x40, d, n); }

/* ---------------- UART ---------------- */
static void uart_tx(UART_Regs *inst, const uint8_t *d, uint16_t n)
{
    for (uint16_t i = 0; i < n; i++)
        DL_UART_Main_transmitDataBlocking(inst, d[i]);
}
void bsp_uart_debug_tx(const uint8_t *d, uint16_t n) { uart_tx(UART_DEBUG_INST, d, n); }
void bsp_uart_radio_tx(const uint8_t *d, uint16_t n) { uart_tx(UART_RADIO_INST, d, n); }

void UART_DEBUG_INST_IRQHandler(void)
{
    switch (DL_UART_Main_getPendingInterrupt(UART_DEBUG_INST)) {
        case DL_UART_MAIN_IIDX_RX:
            app_on_debug_byte(DL_UART_Main_receiveData(UART_DEBUG_INST));
            break;
        default:
            break;
    }
}
void UART_IMU_INST_IRQHandler(void)
{
    switch (DL_UART_Main_getPendingInterrupt(UART_IMU_INST)) {
        case DL_UART_MAIN_IIDX_RX:
            app_on_imu_byte(DL_UART_Main_receiveData(UART_IMU_INST));
            break;
        default:
            break;
    }
}
void UART_OPENMV_INST_IRQHandler(void)
{
    switch (DL_UART_Main_getPendingInterrupt(UART_OPENMV_INST)) {
        case DL_UART_MAIN_IIDX_RX:
            app_on_openmv_byte(DL_UART_Main_receiveData(UART_OPENMV_INST));
            break;
        default:
            break;
    }
}
void UART_RADIO_INST_IRQHandler(void)
{
    switch (DL_UART_Main_getPendingInterrupt(UART_RADIO_INST)) {
        case DL_UART_MAIN_IIDX_RX:
            app_on_radio_byte(DL_UART_Main_receiveData(UART_RADIO_INST));
            break;
        default:
            break;
    }
}

/* ---------------- 5ms 控制中断(速度环) ---------------- */
void TIMER_CTRL_INST_IRQHandler(void)
{
    switch (DL_TimerG_getPendingInterrupt(TIMER_CTRL_INST)) {
        case DL_TIMER_IIDX_ZERO:
            app_on_ctrl_tick();
            break;
        default:
            break;
    }
}

/* ---------------- flash 参数区(主 flash 最后 1KB 扇区) ----------------
 * G3507 扇区 1KB; 每条 erase/program 命令后写保护自动重新武装,
 * 所以 program 前必须再次 unprotect(手册行为, 不是多余调用)。 */
#define PARAM_FLASH_ADDR  (0x00020000u - 1024u)   /* 128KB flash 尾部 0x1FC00 */

bool bsp_flash_save(const void *data, uint16_t len)
{
    static uint64_t buf64[16];            /* 128B, 8 字节对齐, 64bit 编程要求 */
    uint32_t words = (((uint32_t)len + 7u) / 8u) * 2u;   /* 32bit 字数, 偶数 */
    uint32_t pm;
    bool ok = false;

    if (len == 0 || len > sizeof(buf64)) return false;
    memset(buf64, 0xFF, sizeof(buf64));
    memcpy(buf64, data, len);

    /* 擦写期间取指/中断都会被 flash 命令 stall, 干脆关中断做完:
     * 数 ms 内丢 5ms 节拍和编码器沿 — 所以只在停车状态(菜单里)调用本函数 */
    pm = __get_PRIMASK();
    __disable_irq();

    DL_FlashCTL_unprotectSector(FLASHCTL, PARAM_FLASH_ADDR,
                                DL_FLASHCTL_REGION_SELECT_MAIN);
    if (DL_FlashCTL_eraseMemoryFromRAM(FLASHCTL, PARAM_FLASH_ADDR,
            DL_FLASHCTL_COMMAND_SIZE_SECTOR) != DL_FLASHCTL_COMMAND_STATUS_FAILED) {
        DL_FlashCTL_unprotectSector(FLASHCTL, PARAM_FLASH_ADDR,
                                    DL_FLASHCTL_REGION_SELECT_MAIN);
        ok = DL_FlashCTL_programMemoryBlockingFromRAM64WithECCGenerated(
                 FLASHCTL, PARAM_FLASH_ADDR, (uint32_t *)buf64, words,
                 DL_FLASHCTL_REGION_SELECT_MAIN) != DL_FLASHCTL_COMMAND_STATUS_FAILED;
    }

    __set_PRIMASK(pm);
    return ok;
}

bool bsp_flash_load(void *data, uint16_t len)
{
    memcpy(data, (const void *)PARAM_FLASH_ADDR, len);
    return true;    /* 有效性由调用方 magic 判断(擦除态读出 0xFF) */
}

/* ---------------- 初始化 ---------------- */
void bsp_init(void)
{
    SYSCFG_DL_init();                     /* SysConfig 生成: 时钟+引脚+外设 */

    /* LOAD 寄存器 = timerCount-1, 而 SysConfig 的 duty→cc 约定以 timerCount
     * 为基数(duty=0 → cc=timerCount), 所以 +1 对齐, 避开 cc==LOAD 的
     * 未文档化硬件角落 */
    s_pwm_period = DL_TimerG_getLoadValue(PWM_MOTOR_INST) + 1u;

    SysTick_Config(CPUCLK_FREQ / 1000u);  /* 1ms, 跟随 SysConfig 实际主频 */

    /* 中断优先级(G3507 只有 0..3): 编码器沿(最急) > 串口字节 > 5ms 控制环/ADC。
     * ENC_L/ENC_R 两个宏其实解析到同一个 GROUP1 IRQn, 两行是幂等冗余 */
    NVIC_SetPriority(GPIO_ENC_L_INT_IRQN, 0);
    NVIC_SetPriority(GPIO_ENC_R_INT_IRQN, 0);
    NVIC_SetPriority(UART_DEBUG_INST_INT_IRQN, 1);
    NVIC_SetPriority(UART_IMU_INST_INT_IRQN, 1);
    NVIC_SetPriority(UART_OPENMV_INST_INT_IRQN, 1);
    NVIC_SetPriority(UART_RADIO_INST_INT_IRQN, 1);
    NVIC_SetPriority(TIMER_CTRL_INST_INT_IRQN, 2);
#if CFG_GRAY_ANALOG
    NVIC_SetPriority(ADC_GRAY_INST_INT_IRQN, 2);
#endif

    NVIC_ClearPendingIRQ(GPIO_ENC_L_INT_IRQN);
    NVIC_EnableIRQ(GPIO_ENC_L_INT_IRQN);
    NVIC_ClearPendingIRQ(GPIO_ENC_R_INT_IRQN);
    NVIC_EnableIRQ(GPIO_ENC_R_INT_IRQN);
    NVIC_ClearPendingIRQ(UART_DEBUG_INST_INT_IRQN);
    NVIC_EnableIRQ(UART_DEBUG_INST_INT_IRQN);
    NVIC_ClearPendingIRQ(UART_IMU_INST_INT_IRQN);
    NVIC_EnableIRQ(UART_IMU_INST_INT_IRQN);
    NVIC_ClearPendingIRQ(UART_OPENMV_INST_INT_IRQN);
    NVIC_EnableIRQ(UART_OPENMV_INST_INT_IRQN);
    NVIC_ClearPendingIRQ(UART_RADIO_INST_INT_IRQN);
    NVIC_EnableIRQ(UART_RADIO_INST_INT_IRQN);
    NVIC_ClearPendingIRQ(TIMER_CTRL_INST_INT_IRQN);
    NVIC_EnableIRQ(TIMER_CTRL_INST_INT_IRQN);
#if CFG_GRAY_ANALOG
    NVIC_ClearPendingIRQ(ADC_GRAY_INST_INT_IRQN);
    NVIC_EnableIRQ(ADC_GRAY_INST_INT_IRQN);
#endif

    /* syscfg 里 timerStartTimer=true 已启动; 再调一次幂等, 防配置被改 */
    DL_TimerG_startCounter(PWM_MOTOR_INST);
    DL_TimerG_startCounter(TIMER_CTRL_INST);
    /* 注意: 5ms 中断从这里开始触发, chassis 尚未 init —
     * chassis_update 在 enabled=0 时输出 0, 无害(见 main.c 初始化顺序)。 */
}

#endif /* TARGET_MSPM0 */
