#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "d36a_motor.h"
#include "ms42_encoder.h"

/*
 * Ball/beam control for STM32F103RCT6 + D36A + MS42CG + CanMV K230.
 *
 * K230 UART1 TX (GPIO3, physical pin 8) -> STM32 PA10 (USART1_RX)
 * STM32 PA9 (USART1_TX) -> USB-TTL RX for diagnostics, common GND.
 * D36A channel 1: ST1/EN1/DIR1 = PB6/PB8/PB9.
 * MS42CG: A/B/PWM/Z = PA0/PA1/PA6/PA12.
 *
 * K230 frame (the current D:\K230\gz (1)\main.py):
 *   $B,SEQ,POS_MM,CONF*XOR\r\n
 * POS_MM is signed position relative to O; CONF is 0..100.
 *
 * IMPORTANT: level the beam at power-up.  The boot angle becomes software
 * zero.  No motor is enabled until a valid K230 frame is received.
 */

#define CONTROL_FIRMWARE_VERSION       "BALL_BEAM_F103_V4_HR04_PA10_RCT6"
#define CONTROL_TICK_MS                10U
#define OUTER_PERIOD_MS                20U
#define MOTOR_MICROSTEP                16U

#define K230_MIN_CONFIDENCE            40U
#define K230_STALE_MS                 180UL
#define K230_POSITION_LIMIT_MM       150L
/* Camera positions arrive in 1 mm units.  Filter them before they can move
 * the motor target, otherwise frame-to-frame box jitter becomes motor jitter. */
#define K230_POSITION_FILTER_ALPHA    0.45f

/* Result of the MS42 direction-probe bench: D36A DIR1=1 makes count rise.
 * If a positive ball-position error moves the ball farther from O, change
 * BALL_TO_MOTOR_SIGN from 1 to -1 after a low-speed direction check. */
#define ENCODER_POSITIVE_DIR_PIN        1U
#define BALL_TO_MOTOR_SIGN              1L

#define MOTOR_SOFT_LIMIT_COUNTS       350L
#define MOTOR_TARGET_LIMIT_COUNTS     180L
#define MOTOR_POSITION_DEADBAND         5L
#define MOTOR_MIN_RPM                   2U
#define MOTOR_MAX_RPM                  10U
#define MOTOR_RPM_ERROR_DIVISOR        18L
#define MOTOR_REVERSE_CONFIRM_TICKS     3U
#define MOTOR_STALL_ERROR_COUNTS        8L
#define MOTOR_STALL_TIMEOUT_MS       1000UL

/* Strong O-point outer loop, in motor encoder counts / millimetre. */
#define CENTER_DEADBAND_MM              2L
#define NEAR_ZONE_MM                   18L
#define RECOVERY_ENTER_MM              45L
#define RECOVERY_EXIT_MM               25L
#define KP_NEAR_COUNTS_PER_MM        1.15f
#define KP_FAR_COUNTS_PER_MM         0.90f
#define KI_COUNTS_PER_MM_S           0.000f
#define KD_COUNTS_S_PER_MM           0.20f
#define RECOVERY_KP_COUNTS_PER_MM    1.10f
#define RECOVERY_KD_COUNTS_S_PER_MM  0.30f
#define INTEGRAL_ZONE_MM              25L
#define INTEGRAL_LIMIT_MM_S         100.0f
#define NEAR_OUTPUT_LIMIT_COUNTS     28.0f
#define FAR_OUTPUT_LIMIT_COUNTS      80.0f
#define TARGET_SLEW_COUNTS_S        420.0f
#define VELOCITY_FILTER_ALPHA        0.25f
#define VELOCITY_LIMIT_MM_S         250.0f

/* H-R04 launch-disturbance rejection.  This is deliberately event-based,
 * because RCT6 has no start signal from C07A.  It adds tilt only while the
 * ball is moving away from O (or crossing O quickly), then releases as soon
 * as the ball starts returning. */
#define ENABLE_DYNAMIC_MOTION_BRAKE    1U
#define MOTION_BRAKE_POSITION_MM        3L
#define MOTION_BRAKE_START_SPEED_MM_S 16.0f
#define MOTION_BRAKE_CENTER_SPEED_MM_S 40.0f
#define MOTION_BRAKE_BASE_COUNTS        6.0f
#define MOTION_BRAKE_GAIN_COUNTS_S_MM   0.16f
#define MOTION_BRAKE_MAX_COUNTS        22.0f
#define MOTION_BRAKE_MAX_FRAME_AGE_MS  80UL

/* Breakaway compensation: it is removed while the ball is already moving. */
#define STICTION_ENTER_MM              5L
#define STICTION_EXIT_MM               2L
#define STICTION_SPEED_MM_S          4.0f
#define STICTION_MIN_COUNTS         10.0f
#define STICTION_MAX_COUNTS         26.0f
#define STICTION_RAMP_COUNTS_S      60.0f
#define STICTION_RELEASE_COUNTS_S  120.0f
#define ENABLE_STICTION_COMPENSATION  0U

/* Reference project's car-motion trapezoid feed-forward, ported to counts.
 * It arms only after O is stable, and starts after an external ball motion is
 * detected.  Set to 0 while tuning the basic O-point loop. */
#define ENABLE_TRAPEZOID_FEEDFORWARD  0U
#define FF_READY_POSITION_MM          12L
#define FF_READY_SPEED_MM_S         15.0f
#define FF_READY_HOLD_MS            250UL
#define FF_TRIGGER_POSITION_MM        8L
#define FF_TRIGGER_SPEED_MM_S      18.0f
#define FF_TRIGGER_SAMPLES            2U
#define FF_RAMP_MS                  250UL
#define FF_ACCEL_END_MS            2000UL
#define FF_DECEL_START_MS          5300UL
#define FF_END_MS                  7300UL
#define FF_COUNTS                  22.0f

typedef enum
{
    CONTROL_WAIT_CAMERA = 0,
    CONTROL_ACTIVE,
    CONTROL_CAMERA_LOST,
    CONTROL_FAULT
} ControlState_t;

typedef struct
{
    s32 sequence;
    s32 position_mm;
    u8 confidence;
    u32 received_ms;
    u8 valid;
} K230Sample_t;

static volatile u32 g_ms;
static volatile u8 g_control_due;
static ControlState_t s_state;
static K230Sample_t s_camera;
static MS42_EncoderData_t s_encoder;
static float s_filtered_position_mm;
static float s_previous_position_mm;
static u32 s_previous_camera_ms;
static float s_velocity_mm_s;
static float s_integral_mm_s;
static float s_target_count_float;
static s32 s_target_count;
static u32 s_last_outer_ms;
static u32 s_last_encoder_move_ms;
static u8 s_motor_running;
static u8 s_motor_dir;
static u16 s_motor_rpm;
static u8 s_reverse_dir;
static u8 s_reverse_confirm_ticks;
static u8 s_recovery_active;
static u8 s_stiction_active;
static float s_stiction_count;
static u8 s_ff_armed;
static u8 s_ff_started;
/* Kept runtime-readable so the optional profile is compiled and can be
 * enabled after basic O-point settling is proven. */
static volatile u8 s_feedforward_enabled = ENABLE_TRAPEZOID_FEEDFORWARD;
static u8 s_ff_trigger_samples;
static s32 s_ff_reference_mm;
static s8 s_ff_sign;
static u32 s_ff_ready_ms;
static u32 s_ff_start_ms;
static u32 s_last_log_ms;
static volatile u32 s_valid_frame_count;
static volatile u32 s_invalid_frame_count;
static volatile u32 s_parse_error_count;
static volatile s32 s_peak_abs_position_mm;
static volatile s32 s_peak_inner_error_count;
static volatile u8 s_motion_brake_active;
static volatile s32 s_motion_brake_x10;

static s32 Abs32(s32 value)
{
    return (value < 0L) ? -value : value;
}
static float AbsFloat(float value)
{
    return (value < 0.0f) ? -value : value;
}

static float ClampFloat(float value, float minimum, float maximum)
{
    if(value < minimum) return minimum;
    if(value > maximum) return maximum;
    return value;
}

static float SlewTo(float current, float target, float maximum_change)
{
    return current + ClampFloat(target - current,
                                -maximum_change, maximum_change);
}

static void ControlClock_Init(void)
{
    TIM_TimeBaseInitTypeDef timer;
    NVIC_InitTypeDef nvic;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
    TIM_TimeBaseStructInit(&timer);
    timer.TIM_Prescaler = 71U;
    timer.TIM_Period = 999U;
    timer.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM1, &timer);
    TIM_ClearITPendingBit(TIM1, TIM_IT_Update);
    TIM_ITConfig(TIM1, TIM_IT_Update, ENABLE);

    nvic.NVIC_IRQChannel = TIM1_UP_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 1U;
    nvic.NVIC_IRQChannelSubPriority = 0U;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);
    TIM_Cmd(TIM1, ENABLE);
}

void TIM1_UP_IRQHandler(void)
{
    if(TIM_GetITStatus(TIM1, TIM_IT_Update) != RESET)
    {
        TIM_ClearITPendingBit(TIM1, TIM_IT_Update);
        g_ms++;
        if((g_ms % CONTROL_TICK_MS) == 0U)
            g_control_due = 1U;
    }
}

static void MotorStop(u8 disable)
{
    D36A_Motor1_Stop();
    if(disable != 0U)
        D36A_Motor1_Enable(0U);
    s_motor_running = 0U;
    s_motor_rpm = 0U;
}

static void MotorRunCountDirection(s32 count_sign, u16 rpm)
{
    u8 dir_pin;

    dir_pin = (count_sign > 0L) ? ENCODER_POSITIVE_DIR_PIN
                                 : (u8)(ENCODER_POSITIVE_DIR_PIN ^ 1U);
    if((s_motor_running == 0U) || (s_motor_dir != dir_pin) ||
       (s_motor_rpm != rpm))
    {
        D36A_Motor1_Enable(1U);
        D36A_Motor1_SetDirection(dir_pin);
        D36A_Motor1_SetRpm(rpm, MOTOR_MICROSTEP);
        s_last_encoder_move_ms = g_ms;
        s_motor_dir = dir_pin;
        s_motor_rpm = rpm;
        s_motor_running = 1U;
    }
}

static u8 HexValue(char c)
{
    if((c >= '0') && (c <= '9')) return (u8)(c - '0');
    if((c >= 'A') && (c <= 'F')) return (u8)(c - 'A' + 10);
    if((c >= 'a') && (c <= 'f')) return (u8)(c - 'a' + 10);
    return 0xFFU;
}

static u32 ReadPrimask(void)
{
#if defined(__GNUC__)
    u32 primask;
    __asm volatile ("MRS %0, primask" : "=r" (primask));
    return primask;
#else
    return __get_PRIMASK();
#endif
}

static u8 ParseNumber(const char *text, u16 length, u16 *index, s32 *value)
{
    s32 result = 0L;
    s32 sign = 1L;
    u8 digit_seen = 0U;

    if(*index >= length) return 0U;
    if(text[*index] == '-')
    {
        sign = -1L;
        (*index)++;
    }
    while(*index < length && text[*index] >= '0' && text[*index] <= '9')
    {
        result = result * 10L + (s32)(text[*index] - '0');
        (*index)++;
        digit_seen = 1U;
    }
    *value = result * sign;
    return digit_seen;
}

static u8 ParseK230Frame(const char *frame, u16 length, K230Sample_t *sample)
{
    u16 index;
    u16 star;
    u8 checksum = 0U;
    u8 expected;
    u8 high;
    u8 low;
    s32 sequence;
    s32 position;
    s32 confidence;

    if((length < 12U) || frame[0] != '$' || frame[1] != 'B' || frame[2] != ',')
        return 0U;
    star = 0U;
    /* K230 XORs the complete body, including the initial "B,". */
    for(index = 1U; index < length; index++)
    {
        if(frame[index] == '*')
        {
            star = index;
            break;
        }
        checksum ^= (u8)frame[index];
    }
    if((star == 0U) || ((u16)(star + 2U) >= length)) return 0U;
    high = HexValue(frame[star + 1U]);
    low = HexValue(frame[star + 2U]);
    if((high == 0xFFU) || (low == 0xFFU)) return 0U;
    expected = (u8)((high << 4) | low);
    if(checksum != expected) return 0U;

    index = 3U;
    if(ParseNumber(frame, star, &index, &sequence) == 0U ||
       index >= star || frame[index++] != ',') return 0U;
    if(ParseNumber(frame, star, &index, &position) == 0U ||
       index >= star || frame[index++] != ',') return 0U;
    if(ParseNumber(frame, star, &index, &confidence) == 0U || index != star)
        return 0U;
    if((position > K230_POSITION_LIMIT_MM) ||
       (position < -K230_POSITION_LIMIT_MM) ||
       (confidence < 0L) || (confidence > 100L)) return 0U;

    sample->sequence = sequence;
    sample->position_mm = position;
    sample->confidence = (u8)confidence;
    sample->received_ms = g_ms;
    sample->valid = (sample->confidence >= K230_MIN_CONFIDENCE) ? 1U : 0U;
    return 1U;
}

static void PollK230Uart(void)
{
    char frame[USART_REC_LEN + 1U];
    K230Sample_t parsed;
    u16 length;
    u16 index;
    u32 primask;

    if((USART_RX_STA & 0x8000U) == 0U) return;
    primask = ReadPrimask();
    __disable_irq();
    length = USART_RX_STA & 0x3FFFU;
    if(length > USART_REC_LEN) length = USART_REC_LEN;
    for(index = 0U; index < length; index++) frame[index] = (char)USART_RX_BUF[index];
    frame[length] = '\0';
    USART_RX_STA = 0U;
    if(primask == 0U) __enable_irq();

    if(ParseK230Frame(frame, length, &parsed) != 0U)
    {
        if(parsed.valid != 0U)
        {
            s32 abs_position = Abs32(parsed.position_mm);
            s_camera = parsed;
            s_valid_frame_count++;
            if(abs_position > s_peak_abs_position_mm)
                s_peak_abs_position_mm = abs_position;
        }
        else
        {
            /* Keep the last valid position.  A single low-confidence frame
             * must not force CAMERA_LOST; K230_STALE_MS remains the timeout. */
            s_invalid_frame_count++;
        }
    }
    else s_parse_error_count++;
}

static float DynamicMotionBrake(s32 position_mm, float velocity_mm_s,
                                u32 frame_age_ms)
{
    float speed;
    float magnitude;
    u8 moving_outward;
    u8 crossing_center_fast;

    s_motion_brake_active = 0U;
    s_motion_brake_x10 = 0L;
    if((ENABLE_DYNAMIC_MOTION_BRAKE == 0U) ||
       (frame_age_ms > MOTION_BRAKE_MAX_FRAME_AGE_MS)) return 0.0f;

    speed = AbsFloat(velocity_mm_s);
    if(speed < MOTION_BRAKE_START_SPEED_MM_S) return 0.0f;
    moving_outward = (((position_mm >= MOTION_BRAKE_POSITION_MM) &&
                       (velocity_mm_s > 0.0f)) ||
                      ((position_mm <= -MOTION_BRAKE_POSITION_MM) &&
                       (velocity_mm_s < 0.0f))) ? 1U : 0U;
    crossing_center_fast = ((Abs32(position_mm) < MOTION_BRAKE_POSITION_MM) &&
                            (speed >= MOTION_BRAKE_CENTER_SPEED_MM_S)) ? 1U : 0U;
    if((moving_outward == 0U) && (crossing_center_fast == 0U)) return 0.0f;

    magnitude = MOTION_BRAKE_BASE_COUNTS +
                (speed - MOTION_BRAKE_START_SPEED_MM_S) *
                MOTION_BRAKE_GAIN_COUNTS_S_MM;
    magnitude = ClampFloat(magnitude, 0.0f, MOTION_BRAKE_MAX_COUNTS);
    if(velocity_mm_s > 0.0f) magnitude = -magnitude;
    s_motion_brake_active = 1U;
    s_motion_brake_x10 = (s32)(magnitude * 10.0f);
    return magnitude;
}

static s32 TrapezoidFeedforward(s32 position_mm, float velocity_mm_s)
{
    u32 elapsed;
    u32 phase;
    float command = 0.0f;
    float scale;
    s32 error = -position_mm;

    if(s_feedforward_enabled == 0U) return 0L;
    if(s_ff_started == 0U)
    {
        if(s_ff_armed == 0U)
        {
            if((Abs32(error) <= FF_READY_POSITION_MM) &&
               (AbsFloat(velocity_mm_s) <= FF_READY_SPEED_MM_S))
            {
                if(s_ff_ready_ms == 0U) s_ff_ready_ms = g_ms;
                else if((g_ms - s_ff_ready_ms) >= FF_READY_HOLD_MS)
                {
                    s_ff_armed = 1U;
                    s_ff_reference_mm = position_mm;
                }
            }
            else s_ff_ready_ms = 0U;
        }
        if(s_ff_armed != 0U &&
           ((Abs32(position_mm - s_ff_reference_mm) >= FF_TRIGGER_POSITION_MM) ||
            (AbsFloat(velocity_mm_s) >= FF_TRIGGER_SPEED_MM_S)))
        {
            if(s_ff_trigger_samples < FF_TRIGGER_SAMPLES) s_ff_trigger_samples++;
            if(s_ff_trigger_samples >= FF_TRIGGER_SAMPLES)
            {
                s_ff_sign = (velocity_mm_s > 0.0f) ? -1 : 1;
                if(AbsFloat(velocity_mm_s) < FF_TRIGGER_SPEED_MM_S)
                    s_ff_sign = (error >= 0L) ? 1 : -1;
                s_ff_started = 1U;
                s_ff_start_ms = g_ms;
            }
        }
        else if(s_ff_armed != 0U) s_ff_trigger_samples = 0U;
    }
    if(s_ff_started == 0U) return 0L;

    elapsed = g_ms - s_ff_start_ms;
    if(elapsed < FF_RAMP_MS)
    {
        scale = (float)elapsed / (float)FF_RAMP_MS;
        command = FF_COUNTS * scale;
    }
    else if(elapsed < FF_ACCEL_END_MS) command = FF_COUNTS;
    else if(elapsed < (FF_ACCEL_END_MS + FF_RAMP_MS))
    {
        phase = elapsed - FF_ACCEL_END_MS;
        command = FF_COUNTS * (1.0f - (float)phase / (float)FF_RAMP_MS);
    }
    else if(elapsed < FF_DECEL_START_MS) command = 0.0f;
    else if(elapsed < (FF_DECEL_START_MS + FF_RAMP_MS))
    {
        phase = elapsed - FF_DECEL_START_MS;
        command = -FF_COUNTS * (float)phase / (float)FF_RAMP_MS;
    }
    else if(elapsed < FF_END_MS) command = -FF_COUNTS;
    else if(elapsed < (FF_END_MS + FF_RAMP_MS))
    {
        phase = elapsed - FF_END_MS;
        command = -FF_COUNTS * (1.0f - (float)phase / (float)FF_RAMP_MS);
    }
    else
    {
        s_ff_armed = 0U;
        s_ff_started = 0U;
        s_ff_trigger_samples = 0U;
        s_ff_ready_ms = 0U;
        return 0L;
    }
    return (s32)(command * (float)s_ff_sign + ((command >= 0.0f) ? 0.5f : -0.5f));
}

static void RunOuterLoop(void)
{
    float raw_velocity;
    float output;
    float output_limit;
    float kp;
    s32 error_mm;
    s32 command;
    s32 feedforward;
    s32 filtered_position_mm;
    float motion_brake;
    u32 dt_ms;

    if((g_ms - s_last_outer_ms) < OUTER_PERIOD_MS) return;
    dt_ms = g_ms - s_last_outer_ms;
    s_last_outer_ms = g_ms;
    if(dt_ms > 100U) dt_ms = OUTER_PERIOD_MS;

    if((s_camera.valid == 0U) || ((g_ms - s_camera.received_ms) > K230_STALE_MS))
    {
        s_state = CONTROL_CAMERA_LOST;
        s_integral_mm_s = 0.0f;
        s_stiction_active = 0U;
        s_motion_brake_active = 0U;
        s_motion_brake_x10 = 0L;
        s_target_count_float = SlewTo(s_target_count_float, 0.0f,
                                      TARGET_SLEW_COUNTS_S * (float)dt_ms * 0.001f);
        s_target_count = (s32)((s_target_count_float >= 0.0f) ?
                                s_target_count_float + 0.5f : s_target_count_float - 0.5f);
        return;
    }

    if(s_state == CONTROL_WAIT_CAMERA || s_state == CONTROL_CAMERA_LOST)
    {
        s_filtered_position_mm = (float)s_camera.position_mm;
        s_previous_position_mm = s_filtered_position_mm;
        s_previous_camera_ms = s_camera.received_ms;
        s_velocity_mm_s = 0.0f;
        s_integral_mm_s = 0.0f;
        s_state = CONTROL_ACTIVE;
    }
    else if(s_camera.received_ms != s_previous_camera_ms)
    {
        dt_ms = s_camera.received_ms - s_previous_camera_ms;
        if(dt_ms == 0U || dt_ms > 150U) dt_ms = OUTER_PERIOD_MS;
        s_filtered_position_mm += K230_POSITION_FILTER_ALPHA
                                * ((float)s_camera.position_mm - s_filtered_position_mm);
        raw_velocity = (s_filtered_position_mm - s_previous_position_mm)
                     * 1000.0f / (float)dt_ms;
        raw_velocity = ClampFloat(raw_velocity, -VELOCITY_LIMIT_MM_S, VELOCITY_LIMIT_MM_S);
        s_velocity_mm_s += VELOCITY_FILTER_ALPHA * (raw_velocity - s_velocity_mm_s);
        s_previous_position_mm = s_filtered_position_mm;
        s_previous_camera_ms = s_camera.received_ms;
    }

    filtered_position_mm = (s32)((s_filtered_position_mm >= 0.0f) ?
                                 s_filtered_position_mm + 0.5f :
                                 s_filtered_position_mm - 0.5f);
    error_mm = -filtered_position_mm;
    if(Abs32(error_mm) <= CENTER_DEADBAND_MM) error_mm = 0L;
    if(Abs32(error_mm) >= RECOVERY_ENTER_MM) s_recovery_active = 1U;
    else if(Abs32(error_mm) <= RECOVERY_EXIT_MM) s_recovery_active = 0U;

    if(s_recovery_active != 0U)
    {
        s_integral_mm_s *= 0.85f;
        output = RECOVERY_KP_COUNTS_PER_MM * (float)error_mm
               - RECOVERY_KD_COUNTS_S_PER_MM * s_velocity_mm_s;
        output = ClampFloat(output, -FAR_OUTPUT_LIMIT_COUNTS, FAR_OUTPUT_LIMIT_COUNTS);
    }
    else
    {
        if(Abs32(error_mm) <= INTEGRAL_ZONE_MM)
            s_integral_mm_s += (float)error_mm * ((float)OUTER_PERIOD_MS * 0.001f);
        else
            s_integral_mm_s *= 0.92f;
        s_integral_mm_s = ClampFloat(s_integral_mm_s,
                                     -INTEGRAL_LIMIT_MM_S, INTEGRAL_LIMIT_MM_S);
        kp = (Abs32(error_mm) <= NEAR_ZONE_MM) ? KP_NEAR_COUNTS_PER_MM
                                                : KP_FAR_COUNTS_PER_MM;
        output_limit = (Abs32(error_mm) <= NEAR_ZONE_MM) ? NEAR_OUTPUT_LIMIT_COUNTS
                                                          : FAR_OUTPUT_LIMIT_COUNTS;
        output = kp * (float)error_mm + KI_COUNTS_PER_MM_S * s_integral_mm_s
               - KD_COUNTS_S_PER_MM * s_velocity_mm_s;
        output = ClampFloat(output, -output_limit, output_limit);

        if(Abs32(error_mm) >= STICTION_ENTER_MM) s_stiction_active = 1U;
        else if(Abs32(error_mm) <= STICTION_EXIT_MM)
        {
            s_stiction_active = 0U;
            s_stiction_count = 0.0f;
        }
        if((ENABLE_STICTION_COMPENSATION != 0U) && (s_stiction_active != 0U))
        {
            if((float)error_mm * s_velocity_mm_s > STICTION_SPEED_MM_S)
                s_stiction_count = ClampFloat(s_stiction_count - STICTION_RELEASE_COUNTS_S * 0.02f,
                                               0.0f, STICTION_MAX_COUNTS);
            else
                s_stiction_count = ClampFloat(s_stiction_count + STICTION_RAMP_COUNTS_S * 0.02f,
                                               STICTION_MIN_COUNTS, STICTION_MAX_COUNTS);
            if((output * (float)((error_mm >= 0L) ? 1 : -1)) < s_stiction_count)
                output = s_stiction_count * (float)((error_mm >= 0L) ? 1 : -1);
        }
    }

    motion_brake = DynamicMotionBrake(filtered_position_mm, s_velocity_mm_s,
                                      g_ms - s_camera.received_ms);
    output = ClampFloat(output + motion_brake,
                        -FAR_OUTPUT_LIMIT_COUNTS, FAR_OUTPUT_LIMIT_COUNTS);
    feedforward = TrapezoidFeedforward(filtered_position_mm, s_velocity_mm_s);
    command = (s32)(output + ((output >= 0.0f) ? 0.5f : -0.5f)) + feedforward;
    command *= BALL_TO_MOTOR_SIGN;
    if(command > MOTOR_TARGET_LIMIT_COUNTS) command = MOTOR_TARGET_LIMIT_COUNTS;
    if(command < -MOTOR_TARGET_LIMIT_COUNTS) command = -MOTOR_TARGET_LIMIT_COUNTS;
    s_target_count_float = SlewTo(s_target_count_float, (float)command,
                                  TARGET_SLEW_COUNTS_S * ((float)OUTER_PERIOD_MS * 0.001f));
    s_target_count = (s32)((s_target_count_float >= 0.0f) ?
                            s_target_count_float + 0.5f : s_target_count_float - 0.5f);
}

static void RunMotorPositionLoop(void)
{
    s32 error;
    u16 rpm;
    u8 requested_dir;

    if(Abs32(s_encoder.count) > MOTOR_SOFT_LIMIT_COUNTS)
    {
        MotorStop(1U);
        s_state = CONTROL_FAULT;
        printf("FAULT,SOFT_LIMIT,count=%ld\r\n", s_encoder.count);
        return;
    }
    if(s_state == CONTROL_FAULT) return;

    error = s_target_count - s_encoder.count;
    if(Abs32(error) > s_peak_inner_error_count)
        s_peak_inner_error_count = Abs32(error);
    if(Abs32(error) <= MOTOR_POSITION_DEADBAND)
    {
        MotorStop(0U); /* Hold torque remains enabled while balancing. */
        s_reverse_confirm_ticks = 0U;
        return;
    }
    requested_dir = (error > 0L) ? ENCODER_POSITIVE_DIR_PIN
                                  : (u8)(ENCODER_POSITIVE_DIR_PIN ^ 1U);

    /* Do not reverse on a single noisy encoder/camera update.  The STEP
     * output is muted while the requested direction proves stable. */
    if((s_motor_running != 0U) && (requested_dir != s_motor_dir))
    {
        if(s_reverse_dir != requested_dir)
        {
            s_reverse_dir = requested_dir;
            s_reverse_confirm_ticks = 1U;
        }
        else if(s_reverse_confirm_ticks < MOTOR_REVERSE_CONFIRM_TICKS)
        {
            s_reverse_confirm_ticks++;
        }
        if(s_reverse_confirm_ticks < MOTOR_REVERSE_CONFIRM_TICKS)
        {
            D36A_Motor1_Stop();
            s_last_encoder_move_ms = g_ms;
            return;
        }
    }
    s_reverse_confirm_ticks = 0U;

    rpm = (u16)(MOTOR_MIN_RPM + (u16)(Abs32(error) / MOTOR_RPM_ERROR_DIVISOR));
    if(rpm > MOTOR_MAX_RPM) rpm = MOTOR_MAX_RPM;
    MotorRunCountDirection((error > 0L) ? 1L : -1L, rpm);

    if(Abs32(s_encoder.delta_count) > 0L) s_last_encoder_move_ms = g_ms;
    if(Abs32(error) >= MOTOR_STALL_ERROR_COUNTS &&
       (g_ms - s_last_encoder_move_ms) > MOTOR_STALL_TIMEOUT_MS)
    {
        MotorStop(1U);
        s_state = CONTROL_FAULT;
        printf("FAULT,ENCODER_NO_MOTION,target=%ld,count=%ld\r\n",
               s_target_count, s_encoder.count);
    }
}

static void PrintStatus(void)
{
    if((g_ms - s_last_log_ms) < 500U) return;
    s_last_log_ms = g_ms;
    printf("BALL,t=%lu,state=%u,raw_mm=%ld,filt_x10=%ld,conf=%u,vel_x10=%ld,target=%ld,count=%ld,brake_x10=%ld,age=%lu,valid=%lu,invalid=%lu,parse=%lu,peak_mm=%ld,peak_inner=%ld\r\n",
           g_ms, (u16)s_state, s_camera.position_mm,
           (s32)(s_filtered_position_mm * 10.0f), s_camera.confidence,
           (s32)(s_velocity_mm_s * 10.0f), s_target_count, s_encoder.count,
           s_motion_brake_x10, g_ms - s_camera.received_ms,
           s_valid_frame_count, s_invalid_frame_count, s_parse_error_count,
           s_peak_abs_position_mm, s_peak_inner_error_count);
}

int main(void)
{
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    delay_init();
    uart_init(115200U);
    D36A_Motor_Init();
    D36A_StepperTimer_Init();
    D36A_Motor1_Stop();
    D36A_Motor1_Enable(0U);
    MS42_Encoder_Init();
    MS42_Encoder_SetZero();
    ControlClock_Init();

    s_state = CONTROL_WAIT_CAMERA;
    s_last_encoder_move_ms = 0U;
    printf("BALL_BEAM_READY,%s\r\n", CONTROL_FIRMWARE_VERSION);
    printf("LEVEL_BEAM_THEN_START: K230 $B frames, conf >= %u; FF=%u\r\n",
           K230_MIN_CONFIDENCE, ENABLE_TRAPEZOID_FEEDFORWARD);

    while(1)
    {
        PollK230Uart();
        if(g_control_due != 0U)
        {
            g_control_due = 0U;
            MS42_Encoder_Service(g_ms);
            MS42_Encoder_GetData(&s_encoder);
            RunOuterLoop();
            RunMotorPositionLoop();
            PrintStatus();
        }
    }
}

