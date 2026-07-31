#ifndef __ADC_H
#define __ADC_H

#include "sys.h"
#include "delay.h"

void ADC_Power_Init(void);
u16 Get_Adc1(u8 ch);
u16 Get_adc_Average(u8 ch, u8 count);





#endif
