#ifndef __MS42_ENCODER_H
#define __MS42_ENCODER_H

#include "sys.h"

/* MS42CG: 1024 A/B lines; software reports fourfold count units. */
#define MS42_ENCODER_COUNTS_PER_REV  4096L
#define MS42_ENCODER_PWM_TIMEOUT_MS   120UL

typedef struct
{
    s32 count;              /* A/B count relative to the software zero. */
    s32 raw_count;          /* A/B count before applying the software zero. */
    s16 delta_count;        /* Signed A/B count change in the latest service period. */
    u16 sample_period_ms;   /* Actual elapsed time for delta_count; it can exceed the nominal task period while UART is printing. */
    s32 speed_rpm_x100;
    s32 angle_x10;
    u32 pwm_angle_x10;
    u32 z_count;
    u32 last_z_ms;
    u16 pwm_period_us;
    u16 pwm_high_us;
    u8 pwm_valid;
} MS42_EncoderData_t;

void MS42_Encoder_Init(void);
/* Call at a fixed, short interval (<= 100 ms) with a monotonic millisecond tick. */
void MS42_Encoder_Service(u32 now_ms);
void MS42_Encoder_SetZero(void);
void MS42_Encoder_Update100ms(void);
void MS42_Encoder_GetData(MS42_EncoderData_t *data);

#endif
