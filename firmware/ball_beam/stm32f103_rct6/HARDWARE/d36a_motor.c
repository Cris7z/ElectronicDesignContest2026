#include "d36a_motor.h"

static u8 s_motor1_pulse_running;

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
    compare.TIM_Pulse = 0;
    TIM_OC1Init(TIM4, &compare);
    TIM_OC2Init(TIM4, &compare);
    TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable);
    TIM_OC2PreloadConfig(TIM4, TIM_OCPreload_Enable);
    TIM_ARRPreloadConfig(TIM4, ENABLE);
    TIM_Cmd(TIM4, ENABLE);
    s_motor1_pulse_running = 0U;
}

void D36A_Motor1_Enable(u8 enable)
{
    D36A_EN1 = (enable != 0U) ? 1 : 0; /* D36A: low=sleep, high=enable */
}

void D36A_Motor1_SetDirection(u8 clockwise)
{
    D36A_DIR1 = (clockwise != 0U) ? 1 : 0;
}

void D36A_Motor1_SetRpmX100(u16 rpm_x100, u8 microstep)
{
    u32 frequency_millihz;
    u32 period_ticks;

    if(rpm_x100 == 0U || microstep == 0U)
    {
        D36A_Motor1_Stop();
        return;
    }
    if(rpm_x100 > 6000U) rpm_x100 = 6000U;

    /* MS42CG is 1.8 degrees: 200 full steps per revolution.
     * rpm_x100 keeps useful sub-RPM resolution near the position target.
     * STEP millihertz = rpm_x100 * 200 * microstep / 6. */
    frequency_millihz = ((u32)rpm_x100 * 200UL * (u32)microstep + 3UL) / 6UL;
    if(frequency_millihz == 0UL)
    {
        D36A_Motor1_Stop();
        return;
    }
    period_ticks = (1000000000UL + frequency_millihz / 2UL) / frequency_millihz;

    /* TIM4 is 16-bit: accepted range is roughly 16 Hz to 250 kHz. */
    if(period_ticks < 4U || period_ticks > 65536U)
    {
        D36A_Motor1_Stop();
        return;
    }

    if(s_motor1_pulse_running == 0U)
    {
        TIM_Cmd(TIM4, DISABLE);
        TIM_SetAutoreload(TIM4, (u16)(period_ticks - 1U));
        TIM_SetCompare1(TIM4, (u16)(period_ticks / 2U));
        TIM_SetCounter(TIM4, 0U);
        TIM_GenerateEvent(TIM4, TIM_EventSource_Update);
        TIM_Cmd(TIM4, ENABLE);
        s_motor1_pulse_running = 1U;
    }
    else
    {
        /* ARR/CCR1 preload transfers on the next natural update event.  Do
         * not reset CNT here: frequent low-speed updates would otherwise
         * postpone every STEP edge and starve the motor of pulses. */
        TIM_SetAutoreload(TIM4, (u16)(period_ticks - 1U));
        TIM_SetCompare1(TIM4, (u16)(period_ticks / 2U));
    }
}

void D36A_Motor1_SetRpm(u16 rpm, u8 microstep)
{
    u32 rpm_x100 = (u32)rpm * 100UL;

    if(rpm_x100 > 65535UL) rpm_x100 = 65535UL;
    D36A_Motor1_SetRpmX100((u16)rpm_x100, microstep);
}

void D36A_Motor1_Stop(void)
{
    TIM_SetCompare1(TIM4, 0U);
    TIM_GenerateEvent(TIM4, TIM_EventSource_Update);
    s_motor1_pulse_running = 0U;
}
