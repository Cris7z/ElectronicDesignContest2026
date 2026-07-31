#ifndef __D36A_MOTOR_H
#define __D36A_MOTOR_H

#include "sys.h"

/* D36A channel 1 control pins for STM32F103C8T6. */
#define D36A_EN1   PBout(8)
#define D36A_DIR1  PBout(9)

void D36A_Motor_Init(void);
void D36A_StepperTimer_Init(void);
void D36A_Motor1_Enable(u8 enable);
void D36A_Motor1_SetDirection(u8 clockwise);
void D36A_Motor1_SetRpm(u16 rpm, u8 microstep);
void D36A_Motor1_Stop(void);

#endif
