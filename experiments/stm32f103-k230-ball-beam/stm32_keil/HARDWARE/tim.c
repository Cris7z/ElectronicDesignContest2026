#include "tim.h"

void TIM2_Init(u16 arr, u16 psc)
{
	TIM_TimeBaseInitTypeDef TIM_BaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
	
	TIM_BaseStructure.TIM_Period = arr;
	TIM_BaseStructure.TIM_Prescaler = psc;
	TIM_BaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_BaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseInit(TIM2, &TIM_BaseStructure);
	
	TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
	
	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
	TIM_Cmd(TIM2, ENABLE);
}

int TIM2_IRQHandler(void)
{
	static int Dir_Count;
	if(TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)
	{
		TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
		Dir_Count++;
		if(Dir_Count>=1000)					//每10s切换一些电机转动的方向
		{
			DIR1 = ~DIR1;
			DIR2 = ~DIR2;
			Dir_Count=0;
		}
	}
	return 0;
}




