#include "d36a_motor.h"

/*
 * Wiring:
 * PB6 -> D36A ST1 (TIM4_CH1)
 * PB8 -> D36A EN1
 * PB9 -> D36A DIR1
 * GND -> D36A GND
 */
void D36A_Motor_Init(void)
{
    GPIO_InitTypeDef gpio;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

    gpio.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;
    gpio.GPIO_Mode = GPIO_Mode_Out_PP;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &gpio);

    D36A_EN1 = 0;
    D36A_DIR1 = 0;
}

void D36A_StepperTimer_Init(void)
{
    GPIO_InitTypeDef gpio;
    TIM_TimeBaseInitTypeDef timer;
    TIM_OCInitTypeDef compare;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);

    gpio.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
    gpio.GPIO_Mode = GPIO_Mode_AF_PP;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &gpio);

    /* 72 MHz / (71 + 1) = 1 MHz timer tick. */
    timer.TIM_Prescaler = 71;
    timer.TIM_Period = 1874;       /* Initial 533.33 Hz, 10 RPM at 1/16 step. */
    timer.TIM_CounterMode = TIM_CounterMode_Up;
    timer.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInit(TIM4, &timer);

    compare.TIM_OCMode = TIM_OCMode_PWM1;
    compare.TIM_OutputState = TIM_OutputState_Enable;
    compare.TIM_OCPolarity = TIM_OCPolarity_High;
    compare.TIM_Pulse = 937;
    TIM_OC1Init(TIM4, &compare);
    TIM_OC2Init(TIM4, &compare);
    TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable);
    TIM_OC2PreloadConfig(TIM4, TIM_OCPreload_Enable);
    TIM_ARRPreloadConfig(TIM4, ENABLE);
    TIM_Cmd(TIM4, ENABLE);
}

void D36A_Motor1_Enable(u8 enable)
{
    D36A_EN1 = (enable != 0U) ? 1 : 0; /* D36A: low=sleep, high=enable */
}

void D36A_Motor1_SetDirection(u8 clockwise)
{
    D36A_DIR1 = (clockwise != 0U) ? 1 : 0;
}

void D36A_Motor1_SetRpm(u16 rpm, u8 microstep)
{
    u32 frequency_hz;
    u32 period_ticks;

    if(rpm == 0U || microstep == 0U)
    {
        D36A_Motor1_Stop();
        return;
    }

    /* MS42CG is 1.8 degrees: 200 full steps per revolution. */
    frequency_hz = ((u32)rpm * 200UL * (u32)microstep + 30UL) / 60UL;
    period_ticks = (1000000UL + frequency_hz / 2U) / frequency_hz;

    /* TIM4 is 16-bit: accepted range is roughly 16 Hz to 250 kHz. */
    if(period_ticks < 4U || period_ticks > 65536U)
        return;

    TIM_Cmd(TIM4, DISABLE);
    TIM_SetAutoreload(TIM4, (u16)(period_ticks - 1U));
    TIM_SetCompare1(TIM4, (u16)(period_ticks / 2U));
    TIM_SetCounter(TIM4, 0U);
    TIM_GenerateEvent(TIM4, TIM_EventSource_Update);
    TIM_Cmd(TIM4, ENABLE);
}

void D36A_Motor1_Stop(void)
{
    TIM_SetCompare1(TIM4, 0U);
}
