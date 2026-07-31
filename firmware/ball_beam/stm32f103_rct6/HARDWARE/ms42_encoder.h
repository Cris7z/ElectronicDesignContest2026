#ifndef __MS42_ENCODER_H
#define __MS42_ENCODER_H

#include "sys.h"

/* MS42CG: 1024 A/B lines; software reports fourfold count units. */
#define MS42_ENCODER_COUNTS_PER_REV  4096L

typedef struct
{
    s32 count;
    s32 speed_rpm_x100;
    s32 angle_x10;
    u32 pwm_angle_x10;
    u32 z_count;
    u8 pwm_valid;
} MS42_EncoderData_t;

void MS42_Encoder_Init(void);
void MS42_Encoder_Update100ms(void);
void MS42_Encoder_GetData(MS42_EncoderData_t *data);

#endif
