/***********************************************
公司：轮趣科技（东莞）有限公司
品牌：WHEELTEC
官网：wheeltec.net
淘宝店铺：shop114407458.taobao.com 
版本V1.0
修改时间：2023.12.18
All rights reserved
***********************************************/
#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "ATD5984.h"
#include "adc.h"
#include "tim.h"

u16 adc_val;
float voltage;

int main(void)
{
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	delay_init();
	ADC_Power_Init();							//读取电源电压的ADC初始化
	uart_init(115200);						//串口初始化
	ATD5984_Init();								//初始化ATD5984的EN跟DIR引脚
	STEP12_PWM_Init(7199, 6);			//PWM输出，控制步进电机转动的速度，函数定义中有对输出的PWM频率做注释
	TIM2_Init(7199, 99);					//每10ms进一次中断
	
	while(1)
	{
		adc_val = Get_adc_Average(3, 5);
		
		voltage = (float)(adc_val*3.3*11/4096);
		
		printf("当前电池电压为: %f\r\n", voltage);
	}
}


