#include "ATD5984.h"

void ATD5984_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_8;	
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_2;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	GPIO_SetBits(GPIOB, GPIO_Pin_9);
	GPIO_SetBits(GPIOB, GPIO_Pin_8);
	GPIO_SetBits(GPIOA, GPIO_Pin_1);
	GPIO_SetBits(GPIOA, GPIO_Pin_2);
}

/**************************************************************************
函数功能：PWM初始化
入口参数：arr:自动重装载值     psc:预分频系数
返回  值：无
**************************************************************************/
/* 
 * 车轮一圈是210.4867mm
 * 看做是210mm吧，好计算一些，21cm，0.21m
 * 默认16细分的话，要实现0.1m/s，0.25m/s，0.5m/s
 * 360/（1.8/16）=3200，一圈有3200步，也就是3200Hz，
 * 即步进电机在16细分的情况下，1s可以收到3200个脉冲，进而实现转动一圈，实现0.21m/s = 一圈/s
 * 这样算下来速度对应的PWM频率分别为
 * （0.25/0.21）* 3200 = 3809.52380952381 ≈ 3810（四舍五入）					STEP12_PWM_Init(71, 261);
 * 假如没有车轮，要求电机转动 n圈/s，在16细分的前提下需要的脉冲数则为 n*3200，获得脉冲数之后再去配置定时器频率即可
 */
void STEP12_PWM_Init(u16 arr, u16 psc)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	TIM_OCInitTypeDef TIM_OCInitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	TIM_TimeBaseInitStructure.TIM_Period = arr;
	TIM_TimeBaseInitStructure.TIM_Prescaler = psc;
	TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseInitStructure);
	
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = 50;
	
	TIM_OC1Init(TIM4, &TIM_OCInitStructure);
	TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable);
	
	TIM_OC2Init(TIM4, &TIM_OCInitStructure);
	TIM_OC2PreloadConfig(TIM4, TIM_OCPreload_Enable);
	
	TIM_ARRPreloadConfig(TIM4, ENABLE);
	
	TIM_Cmd(TIM4, ENABLE);
	
}
















