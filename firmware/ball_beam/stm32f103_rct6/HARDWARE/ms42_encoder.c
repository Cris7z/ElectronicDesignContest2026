#include "ms42_encoder.h"

/*
 * Encoder wiring:
 * PA0 = A (TIM2_CH1), PA1 = B (TIM2_CH2), PA6 = PWM (TIM3_CH1), PA12 = Z.
 */
static volatile s32 s_count;
static volatile s32 s_zero_count;
static volatile u16 s_last_timer_count;
static volatile s16 s_last_delta;
static volatile u16 s_sample_period_ms;
static volatile s32 s_speed_rpm_x100;
static volatile u32 s_z_count;
static volatile u32 s_last_z_ms;
static volatile u32 s_now_ms;

static volatile u16 s_pwm_last_rise;
static volatile u16 s_pwm_period;
static volatile u16 s_pwm_high;
static volatile u8 s_pwm_wait_fall;
static volatile u8 s_pwm_have_rise;
static volatile u8 s_pwm_valid;
static volatile u32 s_pwm_last_edge_ms;

void MS42_Encoder_Init(void)
{
    GPIO_InitTypeDef gpio;
    TIM_TimeBaseInitTypeDef timer;
    TIM_ICInitTypeDef capture;
    EXTI_InitTypeDef exti;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO | RCC_APB2Periph_GPIOA, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2 | RCC_APB1Periph_TIM3, ENABLE);

    /* PA0/PA1: hardware quadrature counter. */
    gpio.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
    gpio.GPIO_Mode = GPIO_Mode_IPU;
    gpio.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init(GPIOA, &gpio);

    TIM_TimeBaseStructInit(&timer);
    timer.TIM_Period = 0xFFFF;
    timer.TIM_Prescaler = 0;
    timer.TIM_CounterMode = TIM_CounterMode_Up;
    timer.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInit(TIM2, &timer);
    TIM_EncoderInterfaceConfig(TIM2, TIM_EncoderMode_TI12,
                               TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);
    TIM_SetCounter(TIM2, 0U);
    TIM_Cmd(TIM2, ENABLE);

    /* PA6: PWM absolute-angle waveform, TIM3 capture timer at 1 MHz. */
    gpio.GPIO_Pin = GPIO_Pin_6;
    gpio.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &gpio);

    TIM_TimeBaseStructInit(&timer);
    timer.TIM_Period = 0xFFFF;
    timer.TIM_Prescaler = 71;
    timer.TIM_CounterMode = TIM_CounterMode_Up;
    timer.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInit(TIM3, &timer);

    TIM_ICStructInit(&capture);
    capture.TIM_Channel = TIM_Channel_1;
    capture.TIM_ICPolarity = TIM_ICPolarity_Rising;
    capture.TIM_ICSelection = TIM_ICSelection_DirectTI;
    capture.TIM_ICPrescaler = TIM_ICPSC_DIV1;
    capture.TIM_ICFilter = 6;
    TIM_ICInit(TIM3, &capture);
    TIM_ClearITPendingBit(TIM3, TIM_IT_CC1);
    TIM_ITConfig(TIM3, TIM_IT_CC1, ENABLE);
    NVIC_SetPriority(TIM3_IRQn, 2U);
    NVIC_EnableIRQ(TIM3_IRQn);
    TIM_Cmd(TIM3, ENABLE);

    /* PA12: rising Z pulse once per mechanical revolution. */
    gpio.GPIO_Pin = GPIO_Pin_12;
    gpio.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(GPIOA, &gpio);
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource12);
    EXTI_StructInit(&exti);
    exti.EXTI_Line = EXTI_Line12;
    exti.EXTI_Mode = EXTI_Mode_Interrupt;
    exti.EXTI_Trigger = EXTI_Trigger_Rising;
    exti.EXTI_LineCmd = ENABLE;
    EXTI_Init(&exti);
    EXTI_ClearITPendingBit(EXTI_Line12);
    NVIC_SetPriority(EXTI15_10_IRQn, 1U);
    NVIC_EnableIRQ(EXTI15_10_IRQn);

    s_count = 0;
    s_zero_count = 0;
    s_last_timer_count = 0;
    s_last_delta = 0;
    s_sample_period_ms = 0;
    s_speed_rpm_x100 = 0;
    s_z_count = 0;
    s_last_z_ms = 0;
    s_now_ms = 0;
    s_pwm_last_rise = 0;
    s_pwm_period = 0;
    s_pwm_high = 0;
    s_pwm_wait_fall = 0;
    s_pwm_have_rise = 0;
    s_pwm_valid = 0;
    s_pwm_last_edge_ms = 0;
}
void MS42_Encoder_Service(u32 now_ms)
{
    u16 now;
    s16 delta;
    u32 period_ms;

    period_ms = now_ms - s_now_ms;
    s_now_ms = now_ms;
    s_sample_period_ms = (period_ms > 65535UL) ? 65535U : (u16)period_ms;

    now = (u16)TIM_GetCounter(TIM2);
    delta = (s16)(now - s_last_timer_count);
    s_last_timer_count = now;
    s_count += (s32)delta;
    s_last_delta = delta;

    /* RPM x 100 = delta * 6000000 / (counts/rev * milliseconds). */
    if(period_ms != 0U)
        s_speed_rpm_x100 = ((s32)delta * 6000000L)
                         / (MS42_ENCODER_COUNTS_PER_REV * (s32)period_ms);

    if(s_pwm_valid != 0U && (now_ms - s_pwm_last_edge_ms) > MS42_ENCODER_PWM_TIMEOUT_MS)
        s_pwm_valid = 0U;
}
void MS42_Encoder_SetZero(void)
{
    s_zero_count = s_count;
}

void MS42_Encoder_Update100ms(void)
{
    static u32 compatibility_ms;
    compatibility_ms += 100U;
    MS42_Encoder_Service(compatibility_ms);
}

void MS42_Encoder_GetData(MS42_EncoderData_t *data)
{
    u32 value;

    if(data == 0) return;

    data->raw_count = s_count;
    data->count = s_count - s_zero_count;
    data->delta_count = s_last_delta;
    data->sample_period_ms = s_sample_period_ms;
    data->speed_rpm_x100 = s_speed_rpm_x100;
    data->angle_x10 = (s32)((data->count * 3600L) / MS42_ENCODER_COUNTS_PER_REV);
    data->z_count = s_z_count;
    data->last_z_ms = s_last_z_ms;
    data->pwm_valid = s_pwm_valid;
    data->pwm_period_us = s_pwm_period;
    data->pwm_high_us = s_pwm_high;

    if(s_pwm_valid != 0U && s_pwm_period > s_pwm_high)
    {
        /* MT6816 PWM decoding used in the supplied MS42CG reference project. */
        value = ((u32)s_pwm_high * 4115UL - (u32)s_pwm_period) * 3600UL;
        value /= (u32)s_pwm_period * 4115UL;
        if(value > 3599UL) value = 3599UL;
        data->pwm_angle_x10 = value;
    }
    else
    {
        data->pwm_angle_x10 = 0U;
    }
}

void TIM3_IRQHandler(void)
{
    u16 capture;

    if(TIM_GetITStatus(TIM3, TIM_IT_CC1) != RESET)
    {
        capture = TIM_GetCapture1(TIM3);
        if(s_pwm_wait_fall == 0U)
        {
            if(s_pwm_have_rise != 0U)
                s_pwm_period = (u16)(capture - s_pwm_last_rise);
            s_pwm_last_rise = capture;
            s_pwm_last_edge_ms = s_now_ms;
            s_pwm_have_rise = 1U;
            s_pwm_wait_fall = 1U;
            TIM_CCxCmd(TIM3, TIM_Channel_1, TIM_CCx_Disable);
            TIM3->CCER |= TIM_CCER_CC1P;
            TIM_CCxCmd(TIM3, TIM_Channel_1, TIM_CCx_Enable);
        }
        else
        {
            s_pwm_high = (u16)(capture - s_pwm_last_rise);
            s_pwm_last_edge_ms = s_now_ms;
            s_pwm_wait_fall = 0U;
            TIM_CCxCmd(TIM3, TIM_Channel_1, TIM_CCx_Disable);
            TIM3->CCER &= ~TIM_CCER_CC1P;
            TIM_CCxCmd(TIM3, TIM_Channel_1, TIM_CCx_Enable);
            if(s_pwm_high != 0U && s_pwm_period > s_pwm_high)
                s_pwm_valid = 1U;
        }
        TIM_ClearITPendingBit(TIM3, TIM_IT_CC1);
    }
}

void EXTI15_10_IRQHandler(void)
{
    if(EXTI_GetITStatus(EXTI_Line12) != RESET)
    {
        s_z_count++;
        s_last_z_ms = s_now_ms;
        EXTI_ClearITPendingBit(EXTI_Line12);
    }
}
