#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "d36a_motor.h"
#include "ms42_encoder.h"

/*
 * Ball/beam control for STM32F103RCT6 + D36A + MS42CG + CanMV K230.
 *
 * K230 UART1 TX (GPIO3, physical pin 8) -> STM32 PA10 (USART1_RX)
 * STM32 PA9 (USART1_TX) -> K230 GPIO4 (UART1_RX) and onboard CH340 RX.
 * Board key K3 -> PC8 (backup RTSP), K4 -> PC9 (H-R03 sequence).
 * Both keys are active low with MCU internal pull-ups enabled.
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

#define CONTROL_FIRMWARE_VERSION       "BALL_BEAM_F103_V9_Q3_PREDICTIVE_STOP_PA10_RCT6"
#define CONTROL_TICK_MS                10U
#define OUTER_PERIOD_MS                20U
#define MOTOR_MICROSTEP                16U

#define VIDEO_MODE_COMMAND_PERIOD_MS 1000UL
#define VIDEO_MODE_REPEAT_GAP_MS       60UL
#define BUTTON_DEBOUNCE_TICKS           3U

#define Q3_POS_TARGET_MM               50L
#define Q3_NEG_TARGET_MM              -50L
#define Q3_NEG_BRAKE_POINT_MM         -55L
#define Q3_START_POSITION_MM           10L
#define Q3_START_SPEED_MM_S          15.0f
#define Q3_READY_HOLD_MS              300UL
#define Q3_TARGET_TOLERANCE_MM          6L
#define Q3_FINAL_TOLERANCE_MM            9L
#define Q3_FINAL_SPEED_MM_S          18.0f
#define Q3_FINAL_STABLE_FRAMES          2U
#define Q3_POS_TIMEOUT_MS            2500UL
#define Q3_TOTAL_TIMEOUT_MS          5000UL
/* Off-centre targets need a small sustained beam tilt to overcome ball/rail
 * stiction.  Hysteresis prevents chatter near the +/-6 mm scoring window.
 * This assist is disabled whenever the target is O, so H-R04 is unchanged. */
#define Q3_ASSIST_ENTER_MM              12L
#define Q3_ASSIST_EXIT_MM                6L
#define Q3_NEG_ASSIST_ENTER_MM           8L
#define Q3_NEG_ASSIST_EXIT_MM            4L
#define Q3_POS_ASSIST_MIN_COUNTS       60.0f
#define Q3_NEG_FAR_ASSIST_MIN_COUNTS   60.0f
#define Q3_NEG_NEAR_ASSIST_MIN_COUNTS  90.0f
#define Q3_NEG_OUTPUT_LIMIT_COUNTS     70.0f
#define Q3_NEG_NEAR_ZONE_MM             50L
#define Q3_NEG_NEAR_EXTRA_KD           1.35f
#define Q3_ASSIST_RELEASE_SPEED_MM_S    3.5f
#define Q3_NEG_BRAKE_TARGET_SLEW_COUNTS_S 1400.0f
#define Q3_SETTLE_KP_COUNTS_PER_MM      0.30f
#define Q3_SETTLE_KD_COUNTS_S_PER_MM    1.80f
#define Q3_SETTLE_OUTPUT_LIMIT_COUNTS  55.0f
#define Q3_SETTLE_TARGET_SLEW_COUNTS_S 1800.0f
#define Q3_HOLD_KP_COUNTS_PER_MM        1.00f
#define Q3_HOLD_KD_COUNTS_S_PER_MM      1.20f
#define Q3_HOLD_OUTPUT_LIMIT_COUNTS    60.0f
#define Q3_HOLD_KICK_COUNTS             90.0f
#define Q3_HOLD_KICK_ENTER_MM            15L
#define Q3_HOLD_KICK_STOP_SPEED_MM_S    10.0f
#define Q3_HOLD_KICK_MAX_MS             180UL
#define Q3_HOLD_KICK_REARM_MS           500UL
#define Q3_HOLD_KICK_TARGET_SLEW_COUNTS_S 1200.0f
#define Q3_REVERSAL_OUTPUT_COUNTS      90.0f
#define Q3_REVERSAL_TARGET_SLEW_COUNTS_S 1400.0f
#define Q3_PREDICT_DECEL_MM_S2        720.0f
#define Q3_PREDICT_MIN_SPEED_MM_S      20.0f
#define Q3_PREDICT_FALLBACK_MM          -49L
#define Q3_PREDICT_MAX_DISTANCE_MM      20.0f
#define Q3_BRAKE_RELEASE_SPEED_MM_S     -8.0f
#define ENABLE_Q3_HOLD_KICK             0U

#define Q3_FAIL_NONE                 0x00U
#define Q3_FAIL_POS_TIMEOUT          0x01U
#define Q3_FAIL_TOTAL_TIMEOUT        0x02U
#define Q3_FAIL_CAMERA               0x04U
#define Q3_FAIL_CONTROL              0x08U

#define K230_MIN_CONFIDENCE            40U
#define K230_STALE_MS                 180UL
#define K230_POSITION_LIMIT_MM       150L
/* Camera positions arrive in 1 mm units.  Filter them before they can move
 * the motor target, otherwise frame-to-frame box jitter becomes motor jitter. */
#define K230_POSITION_FILTER_ALPHA    0.55f

/* Result of the MS42 direction-probe bench: D36A DIR1=1 makes count rise.
 * If a positive ball-position error moves the ball farther from O, change
 * BALL_TO_MOTOR_SIGN from 1 to -1 after a low-speed direction check. */
#define ENCODER_POSITIVE_DIR_PIN        1U
#define BALL_TO_MOTOR_SIGN              1L

#define MOTOR_SOFT_LIMIT_COUNTS       350L
#define MOTOR_TARGET_LIMIT_COUNTS     180L
#define MOTOR_POSITION_STOP_COUNTS      2L
#define MOTOR_POSITION_RESTART_COUNTS   4L
#define MOTOR_MIN_RPM_X100             30U
#define MOTOR_MAX_RPM_X100           1400U
#define MOTOR_SPEED_KP_X100_PER_COUNT  12L
#define MOTOR_ACCEL_RPM_X100_S       5000L
#define MOTOR_DECEL_RPM_X100_S       9000L
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
#define RECOVERY_KP_COUNTS_PER_MM    1.15f
#define RECOVERY_KD_COUNTS_S_PER_MM  0.30f
#define INTEGRAL_ZONE_MM              25L
#define INTEGRAL_LIMIT_MM_S         100.0f
#define NEAR_OUTPUT_LIMIT_COUNTS     28.0f
#define FAR_OUTPUT_LIMIT_COUNTS      90.0f
#define TARGET_SLEW_COUNTS_S        450.0f
#define VELOCITY_FILTER_ALPHA        0.35f
#define VELOCITY_LIMIT_MM_S         250.0f

/* H-R04 launch-disturbance rejection.  This is deliberately event-based,
 * because RCT6 has no start signal from C07A.  It adds tilt only while the
 * ball is moving away from O (or crossing O quickly), then releases as soon
 * as the ball starts returning. */
#define ENABLE_DYNAMIC_MOTION_BRAKE    1U
#define MOTION_BRAKE_POSITION_MM        2L
#define MOTION_BRAKE_START_SPEED_MM_S 12.0f
#define MOTION_BRAKE_CENTER_SPEED_MM_S 30.0f
#define MOTION_BRAKE_BASE_COUNTS        8.0f
#define MOTION_BRAKE_GAIN_COUNTS_S_MM   0.22f
#define MOTION_BRAKE_MAX_COUNTS        32.0f
#define MOTION_BRAKE_MAX_FRAME_AGE_MS  80UL
#define MOTION_TARGET_SLEW_COUNTS_S   900.0f

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

typedef enum
{
    Q3_IDLE_O = 0,
    Q3_GO_POS_50,
    Q3_GO_NEG_50,
    Q3_BRAKE_RETURN_NEG,
    Q3_SETTLE_NEG,
    Q3_HOLD_NEG_DONE,
    Q3_HOLD_NEG_FAILED,
    Q3_ABORT_CENTER
} Q3State_t;

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
static u16 s_motor_rpm_x100;
static s32 s_motor_command_rpm_x100;
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
static volatile u32 s_duplicate_frame_count;
static volatile s32 s_peak_abs_position_mm;
static volatile s32 s_peak_abs_velocity_x10;
static volatile s32 s_peak_inner_error_count;
static volatile u8 s_motion_brake_active;
static volatile s32 s_motion_brake_x10;
static volatile u32 s_motion_event_count;
static u8 s_have_camera_sequence;
static s32 s_last_camera_sequence;
static Q3State_t s_q3_state;
static s32 s_ball_target_mm;
static u32 s_q3_ready_start_ms;
static u8 s_q3_ready;
static u32 s_q3_start_ms;
static u32 s_q3_elapsed_ms;
static volatile u32 s_q3_pos_hit_elapsed_ms;
static volatile u32 s_q3_neg_hit_elapsed_ms;
static volatile s32 s_q3_neg_hit_velocity_x10;
static volatile s32 s_q3_predicted_stop_x10;
static volatile s32 s_q3_predicted_distance_x10;
static volatile u32 s_q3_brake_return_elapsed_ms;
static volatile s32 s_q3_brake_return_position_mm;
static volatile s32 s_q3_brake_return_velocity_x10;
static volatile s32 s_q3_neg_hit_motor_target;
static volatile s32 s_q3_neg_hit_encoder_count;
static volatile s32 s_q3_hold_motor_count;
static volatile u8 s_q3_hold_kick_active;
static volatile s8 s_q3_hold_kick_sign;
static volatile u32 s_q3_hold_kick_until_ms;
static volatile u32 s_q3_hold_kick_next_ms;
static u8 s_q3_final_stable_frames;
static u8 s_q3_failure_flags;
static u32 s_q3_seen_invalid_frame_count;
static u8 s_q3_drive_assist_active;
static s32 s_q3_peak_positive_mm;
static s32 s_q3_peak_negative_mm;
static u8 s_k3_raw_released;
static u8 s_k3_stable_released;
static u8 s_k3_debounce_ticks;
static u8 s_k4_raw_released;
static u8 s_k4_stable_released;
static u8 s_k4_debounce_ticks;
static u8 s_video_mode_enabled;
static u8 s_video_command_repeats;
static u32 s_last_video_command_ms;

static void Q3ButtonPressed(void);
static void Q3AbortToCenter(u8 failure_flag, const char *reason);

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

static s32 SlewInt32(s32 current, s32 target, s32 maximum_change)
{
    s32 difference = target - current;

    if(difference > maximum_change) return current + maximum_change;
    if(difference < -maximum_change) return current - maximum_change;
    return target;
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

static void VideoModeButton_Init(void)
{
    GPIO_InitTypeDef gpio;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
    GPIO_StructInit(&gpio);
    gpio.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;
    gpio.GPIO_Speed = GPIO_Speed_2MHz;
    gpio.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(GPIOC, &gpio);

    s_k3_raw_released = (GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_8) != Bit_RESET) ? 1U : 0U;
    s_k3_stable_released = s_k3_raw_released;
    s_k3_debounce_ticks = 0U;
    s_k4_raw_released = (GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_9) != Bit_RESET) ? 1U : 0U;
    s_k4_stable_released = s_k4_raw_released;
    s_k4_debounce_ticks = 0U;
    /* Control-rate mode is the deterministic power-on default. */
    s_video_mode_enabled = 0U;
    s_video_command_repeats = 3U;
    s_last_video_command_ms = 0U;
}

static void SendVideoModeCommand(void)
{
    /* Strict XOR-framed records let K230 ignore the diagnostic text that
     * shares PA9. XOR("V,0")=0x4A and XOR("V,1")=0x4B. */
    if(s_video_mode_enabled != 0U)
        printf("$V,1*4B\r\n");
    else
        printf("$V,0*4A\r\n");
    s_last_video_command_ms = g_ms;
}

static void VideoModeButton_Service(void)
{
    u8 k3_raw_released;
    u8 k4_raw_released;

    k3_raw_released = (GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_8) != Bit_RESET) ? 1U : 0U;
    if(k3_raw_released != s_k3_raw_released)
    {
        s_k3_raw_released = k3_raw_released;
        s_k3_debounce_ticks = 0U;
    }
    else if(s_k3_debounce_ticks < BUTTON_DEBOUNCE_TICKS)
    {
        s_k3_debounce_ticks++;
        if((s_k3_debounce_ticks >= BUTTON_DEBOUNCE_TICKS) &&
           (s_k3_stable_released != s_k3_raw_released))
        {
            s_k3_stable_released = s_k3_raw_released;
            if(s_k3_stable_released == 0U)
            {
                if((s_q3_state == Q3_GO_POS_50) ||
                   (s_q3_state == Q3_GO_NEG_50) ||
                   (s_q3_state == Q3_BRAKE_RETURN_NEG) ||
                   (s_q3_state == Q3_SETTLE_NEG))
                    printf("VIDEO_MODE_IGNORED,Q3_ACTIVE,K3_PC8\r\n");
                else
                {
                    s_video_mode_enabled ^= 1U;
                    s_video_command_repeats = 3U;
                    printf("VIDEO_MODE_REQUEST,%u,K3_PC8\r\n",
                           (u16)s_video_mode_enabled);
                }
            }
        }
    }

    k4_raw_released = (GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_9) != Bit_RESET) ? 1U : 0U;
    if(k4_raw_released != s_k4_raw_released)
    {
        s_k4_raw_released = k4_raw_released;
        s_k4_debounce_ticks = 0U;
    }
    else if(s_k4_debounce_ticks < BUTTON_DEBOUNCE_TICKS)
    {
        s_k4_debounce_ticks++;
        if((s_k4_debounce_ticks >= BUTTON_DEBOUNCE_TICKS) &&
           (s_k4_stable_released != s_k4_raw_released))
        {
            s_k4_stable_released = s_k4_raw_released;
            if(s_k4_stable_released == 0U) Q3ButtonPressed();
        }
    }

    if(((s_video_command_repeats > 0U) &&
        ((s_last_video_command_ms == 0U) ||
         ((g_ms - s_last_video_command_ms) >= VIDEO_MODE_REPEAT_GAP_MS))) ||
       ((g_ms - s_last_video_command_ms) >= VIDEO_MODE_COMMAND_PERIOD_MS))
    {
        SendVideoModeCommand();
        if(s_video_command_repeats > 0U) s_video_command_repeats--;
    }
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
    s_motor_rpm_x100 = 0U;
    s_motor_command_rpm_x100 = 0L;
}

static void MotorRunCountDirection(s32 count_sign, u16 rpm_x100)
{
    u8 dir_pin;
    u8 was_running = s_motor_running;

    dir_pin = (count_sign > 0L) ? ENCODER_POSITIVE_DIR_PIN
                                 : (u8)(ENCODER_POSITIVE_DIR_PIN ^ 1U);
    if((s_motor_running == 0U) || (s_motor_dir != dir_pin) ||
       (s_motor_rpm_x100 != rpm_x100))
    {
        D36A_Motor1_Enable(1U);
        D36A_Motor1_SetDirection(dir_pin);
        D36A_Motor1_SetRpmX100(rpm_x100, MOTOR_MICROSTEP);
        if((was_running == 0U) || (s_motor_dir != dir_pin))
            s_last_encoder_move_ms = g_ms;
        s_motor_dir = dir_pin;
        s_motor_rpm_x100 = rpm_x100;
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
            /* The K230 UART worker may repeat its latest record.  A repeated
             * sequence is not a fresh camera observation and must not keep
             * the visual watchdog alive or distort the velocity estimate. */
            if((s_have_camera_sequence != 0U) &&
               (parsed.sequence == s_last_camera_sequence))
            {
                s_duplicate_frame_count++;
                return;
            }
            s_have_camera_sequence = 1U;
            s_last_camera_sequence = parsed.sequence;
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

static u8 Q3IsRunning(void)
{
    return ((s_q3_state == Q3_GO_POS_50) ||
            (s_q3_state == Q3_GO_NEG_50) ||
            (s_q3_state == Q3_BRAKE_RETURN_NEG) ||
            (s_q3_state == Q3_SETTLE_NEG)) ? 1U : 0U;
}

static void Q3ResetReady(void)
{
    s_q3_ready = 0U;
    s_q3_ready_start_ms = 0U;
}

static void Q3AbortToCenter(u8 failure_flag, const char *reason)
{
    s_q3_failure_flags |= failure_flag;
    s_q3_state = Q3_ABORT_CENTER;
    s_ball_target_mm = 0L;
    s_q3_drive_assist_active = 0U;
    s_q3_hold_kick_active = 0U;
    s_q3_final_stable_frames = 0U;
    Q3ResetReady();
    printf("Q3_ABORT,%s,flags=%u,elapsed=%lu\r\n",
           reason, (u16)s_q3_failure_flags, s_q3_elapsed_ms);
}

static void Q3ButtonPressed(void)
{
    if(Q3IsRunning() != 0U)
    {
        Q3AbortToCenter(Q3_FAIL_NONE, "K4_USER");
        return;
    }

    if((s_q3_state == Q3_HOLD_NEG_DONE) ||
       (s_q3_state == Q3_HOLD_NEG_FAILED))
    {
        Q3AbortToCenter(Q3_FAIL_NONE, "K4_RETURN_O");
        return;
    }

    if(s_q3_state == Q3_ABORT_CENTER)
    {
        printf("Q3_RETURNING_O,K4_IGNORED\r\n");
        return;
    }

    if((s_q3_ready == 0U) || (s_state != CONTROL_ACTIVE) ||
       (s_camera.valid == 0U) ||
       ((g_ms - s_camera.received_ms) > K230_STALE_MS))
    {
        printf("Q3_NOT_READY,pos_x10=%ld,vel_x10=%ld,age=%lu\r\n",
               (s32)(s_filtered_position_mm * 10.0f),
               (s32)(s_velocity_mm_s * 10.0f),
               g_ms - s_camera.received_ms);
        return;
    }

    s_q3_state = Q3_GO_POS_50;
    s_ball_target_mm = Q3_POS_TARGET_MM;
    s_q3_start_ms = g_ms;
    s_q3_elapsed_ms = 0U;
    s_q3_pos_hit_elapsed_ms = 0U;
    s_q3_neg_hit_elapsed_ms = 0U;
    s_q3_neg_hit_velocity_x10 = 0L;
    s_q3_predicted_stop_x10 = 0L;
    s_q3_predicted_distance_x10 = 0L;
    s_q3_brake_return_elapsed_ms = 0U;
    s_q3_brake_return_position_mm = 0L;
    s_q3_brake_return_velocity_x10 = 0L;
    s_q3_neg_hit_motor_target = 0L;
    s_q3_neg_hit_encoder_count = 0L;
    s_q3_hold_motor_count = 0L;
    s_q3_hold_kick_active = 0U;
    s_q3_hold_kick_sign = 0;
    s_q3_hold_kick_until_ms = 0U;
    s_q3_hold_kick_next_ms = 0U;
    s_q3_final_stable_frames = 0U;
    s_q3_failure_flags = Q3_FAIL_NONE;
    s_q3_seen_invalid_frame_count = s_invalid_frame_count;
    s_q3_drive_assist_active = 1U;
    s_q3_peak_positive_mm = s_camera.position_mm;
    s_q3_peak_negative_mm = s_camera.position_mm;
    Q3ResetReady();
    /* The control UART gets all K230 resources during the measured run. */
    s_video_mode_enabled = 0U;
    s_video_command_repeats = 3U;
    SendVideoModeCommand();
    s_video_command_repeats--;
    printf("Q3_START,target_mm=%ld,t=%lu\r\n",
           s_ball_target_mm, s_q3_start_ms);
}

static void Q3UpdateReady(s32 position_mm)
{
    u8 centered_and_slow;

    if((s_q3_state != Q3_IDLE_O) &&
       (s_q3_state != Q3_ABORT_CENTER)) return;

    centered_and_slow = ((s_state == CONTROL_ACTIVE) &&
                         (s_camera.valid != 0U) &&
                         ((g_ms - s_camera.received_ms) <= K230_STALE_MS) &&
                         (Abs32(position_mm) <= Q3_START_POSITION_MM) &&
                         (AbsFloat(s_velocity_mm_s) <= Q3_START_SPEED_MM_S)) ? 1U : 0U;
    if(centered_and_slow == 0U)
    {
        Q3ResetReady();
        return;
    }

    if(s_q3_ready_start_ms == 0U) s_q3_ready_start_ms = g_ms;
    if((g_ms - s_q3_ready_start_ms) < Q3_READY_HOLD_MS) return;

    s_q3_ready = 1U;
    if(s_q3_state == Q3_ABORT_CENTER)
    {
        s_q3_state = Q3_IDLE_O;
        s_q3_failure_flags = Q3_FAIL_NONE;
        s_q3_elapsed_ms = 0U;
        printf("Q3_RETURNED_O,READY\r\n");
    }
}

static void Q3TaskUpdate(s32 position_mm, u8 new_camera_sample)
{
    if((s_q3_state == Q3_IDLE_O) || (s_q3_state == Q3_ABORT_CENTER))
    {
        Q3UpdateReady(position_mm);
        return;
    }
    if(Q3IsRunning() == 0U) return;

    s_q3_elapsed_ms = g_ms - s_q3_start_ms;
    if(new_camera_sample != 0U)
    {
        if(position_mm > s_q3_peak_positive_mm)
            s_q3_peak_positive_mm = position_mm;
        if(position_mm < s_q3_peak_negative_mm)
            s_q3_peak_negative_mm = position_mm;
    }
    if(s_invalid_frame_count != s_q3_seen_invalid_frame_count)
    {
        s_q3_seen_invalid_frame_count = s_invalid_frame_count;
        s_q3_final_stable_frames = 0U;
    }
    if((s_camera.valid == 0U) ||
       ((g_ms - s_camera.received_ms) > K230_STALE_MS))
    {
        Q3AbortToCenter(Q3_FAIL_CAMERA, "CAMERA_LOST");
        return;
    }

    if(s_q3_state == Q3_GO_POS_50)
    {
        if((new_camera_sample != 0U) &&
           (Abs32(position_mm - Q3_POS_TARGET_MM) <= Q3_TARGET_TOLERANCE_MM))
        {
            s_q3_state = Q3_GO_NEG_50;
            s_ball_target_mm = Q3_NEG_BRAKE_POINT_MM;
            s_q3_pos_hit_elapsed_ms = s_q3_elapsed_ms;
            s_q3_final_stable_frames = 0U;
            printf("Q3_POS_HIT,pos_mm=%ld,elapsed=%lu,next_mm=%ld\r\n",
                   position_mm, s_q3_elapsed_ms, s_ball_target_mm);
        }
        else if(s_q3_elapsed_ms >= Q3_POS_TIMEOUT_MS)
        {
            s_q3_failure_flags |= Q3_FAIL_POS_TIMEOUT;
            s_q3_state = Q3_GO_NEG_50;
            s_ball_target_mm = Q3_NEG_BRAKE_POINT_MM;
            s_q3_pos_hit_elapsed_ms = s_q3_elapsed_ms;
            s_q3_final_stable_frames = 0U;
            printf("Q3_POS_TIMEOUT,pos_mm=%ld,elapsed=%lu,next_mm=%ld\r\n",
                   position_mm, s_q3_elapsed_ms, s_ball_target_mm);
        }
    }

    if((s_q3_state == Q3_GO_NEG_50) && (new_camera_sample != 0U))
    {
        float speed_mm_s = -s_velocity_mm_s;
        float predicted_distance_mm = 0.0f;
        float predicted_stop_mm = (float)position_mm;
        u8 should_brake = 0U;

        /* Calibrated from the previous physical run: starting the +90-count
         * reverse command at -55 mm and -66.2 mm/s stopped the ball near
         * -58 mm, giving an effective deceleration of about 730 mm/s^2.
         * Use the conservative 720 value to switch before -5 cm so inertia,
         * rather than a later return stroke, completes the travel. */
        if(speed_mm_s >= Q3_PREDICT_MIN_SPEED_MM_S)
        {
            predicted_distance_mm = speed_mm_s * speed_mm_s
                                  / (2.0f * Q3_PREDICT_DECEL_MM_S2);
            predicted_distance_mm = ClampFloat(predicted_distance_mm, 0.0f,
                                                Q3_PREDICT_MAX_DISTANCE_MM);
            predicted_stop_mm -= predicted_distance_mm;
            if(predicted_stop_mm <= (float)Q3_NEG_TARGET_MM)
                should_brake = 1U;
        }
        else if(position_mm <= Q3_PREDICT_FALLBACK_MM)
            should_brake = 1U;

        if(should_brake != 0U)
        {
            s_q3_neg_hit_elapsed_ms = s_q3_elapsed_ms;
            s_q3_neg_hit_velocity_x10 = (s32)(s_velocity_mm_s * 10.0f);
            s_q3_predicted_stop_x10 = (s32)(predicted_stop_mm * 10.0f);
            s_q3_predicted_distance_x10 = (s32)(predicted_distance_mm * 10.0f);
            s_q3_neg_hit_motor_target = s_target_count;
            s_q3_neg_hit_encoder_count = s_encoder.count;
            s_q3_state = Q3_BRAKE_RETURN_NEG;
            s_ball_target_mm = Q3_POS_TARGET_MM;
            s_q3_final_stable_frames = 0U;
            s_q3_drive_assist_active = 0U;
            printf("Q3_PREDICT_BRAKE,pos_mm=%ld,vel_x10=%ld,pred_x10=%ld,dist_x10=%ld,elapsed=%lu\r\n",
                   position_mm, s_q3_neg_hit_velocity_x10,
                   s_q3_predicted_stop_x10, s_q3_predicted_distance_x10,
                   s_q3_neg_hit_elapsed_ms);
        }
    }

    if((s_q3_state == Q3_BRAKE_RETURN_NEG) && (new_camera_sample != 0U) &&
       (s_velocity_mm_s >= Q3_BRAKE_RELEASE_SPEED_MM_S))
    {
        s_q3_brake_return_elapsed_ms = s_q3_elapsed_ms;
        s_q3_brake_return_position_mm = position_mm;
        s_q3_brake_return_velocity_x10 = (s32)(s_velocity_mm_s * 10.0f);
        s_q3_state = Q3_SETTLE_NEG;
        s_ball_target_mm = Q3_NEG_TARGET_MM;
        s_q3_final_stable_frames = 0U;
        printf("Q3_BRAKE_RETURN,pos_mm=%ld,vel_x10=%ld,elapsed=%lu,settle_mm=%ld\r\n",
               s_q3_brake_return_position_mm,
               s_q3_brake_return_velocity_x10,
               s_q3_brake_return_elapsed_ms, s_ball_target_mm);
    }

    if((s_q3_state == Q3_SETTLE_NEG) && (new_camera_sample != 0U))
    {
        if((Abs32(position_mm - Q3_NEG_TARGET_MM) <= Q3_FINAL_TOLERANCE_MM) &&
           (AbsFloat(s_velocity_mm_s) <= Q3_FINAL_SPEED_MM_S))
        {
            if(s_q3_final_stable_frames < Q3_FINAL_STABLE_FRAMES)
                s_q3_final_stable_frames++;
        }
        else s_q3_final_stable_frames = 0U;

        if(s_q3_final_stable_frames >= Q3_FINAL_STABLE_FRAMES)
        {
            s_q3_state = (s_q3_failure_flags == Q3_FAIL_NONE)
                       ? Q3_HOLD_NEG_DONE : Q3_HOLD_NEG_FAILED;
            if(s_q3_state == Q3_HOLD_NEG_DONE)
                s_q3_hold_motor_count = s_encoder.count;
            s_q3_hold_kick_next_ms = g_ms + Q3_HOLD_KICK_REARM_MS;
            printf("Q3_%s,pos_mm=%ld,vel_x10=%ld,elapsed=%lu,flags=%u\r\n",
                   (s_q3_state == Q3_HOLD_NEG_DONE) ? "DONE" : "FAILED",
                   position_mm, (s32)(s_velocity_mm_s * 10.0f),
                   s_q3_elapsed_ms, (u16)s_q3_failure_flags);
            return;
        }
    }

    if((Q3IsRunning() != 0U) && (s_q3_elapsed_ms >= Q3_TOTAL_TIMEOUT_MS))
    {
        s_q3_failure_flags |= Q3_FAIL_TOTAL_TIMEOUT;
        s_q3_state = Q3_HOLD_NEG_FAILED;
        s_ball_target_mm = Q3_NEG_TARGET_MM;
        s_q3_hold_kick_next_ms = g_ms + Q3_HOLD_KICK_REARM_MS;
        printf("Q3_TOTAL_TIMEOUT,pos_mm=%ld,elapsed=%lu,flags=%u\r\n",
               position_mm, s_q3_elapsed_ms, (u16)s_q3_failure_flags);
    }
}

static float DynamicMotionBrake(s32 position_mm, float velocity_mm_s,
                                u32 frame_age_ms)
{
    float speed;
    float magnitude;
    u8 moving_outward;
    u8 crossing_center_fast;
    u8 was_active = s_motion_brake_active;

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
    if(was_active == 0U) s_motion_event_count++;
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
    float target_slew_counts_s;
    float q3_assist_min_counts;
    float q3_output_limit_counts;
    s32 q3_assist_enter_mm;
    s32 q3_assist_exit_mm;
    u32 dt_ms;
    u8 new_camera_sample = 0U;
    u8 q3_allow_min_assist;
    u8 q3_suppress_motion_brake;

    if((g_ms - s_last_outer_ms) < OUTER_PERIOD_MS) return;
    dt_ms = g_ms - s_last_outer_ms;
    s_last_outer_ms = g_ms;
    if(dt_ms > 100U) dt_ms = OUTER_PERIOD_MS;

    if((s_camera.valid == 0U) || ((g_ms - s_camera.received_ms) > K230_STALE_MS))
    {
        if(Q3IsRunning() != 0U)
            Q3AbortToCenter(Q3_FAIL_CAMERA, "CAMERA_LOST");
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
        new_camera_sample = 1U;
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
        if((s32)(AbsFloat(s_velocity_mm_s) * 10.0f) > s_peak_abs_velocity_x10)
            s_peak_abs_velocity_x10 = (s32)(AbsFloat(s_velocity_mm_s) * 10.0f);
        s_previous_position_mm = s_filtered_position_mm;
        s_previous_camera_ms = s_camera.received_ms;
        new_camera_sample = 1U;
    }

    filtered_position_mm = (s32)((s_filtered_position_mm >= 0.0f) ?
                                 s_filtered_position_mm + 0.5f :
                                 s_filtered_position_mm - 0.5f);
    q3_output_limit_counts = FAR_OUTPUT_LIMIT_COUNTS;
    q3_suppress_motion_brake = 0U;
    /* Q3 target gates use the unfiltered coordinates of distinct camera
     * samples.  The closed-loop command below remains filtered. */
    Q3TaskUpdate(s_camera.position_mm, new_camera_sample);
    error_mm = s_ball_target_mm - filtered_position_mm;
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

    if(s_ball_target_mm == 0L)
        s_q3_drive_assist_active = 0U;
    else if(s_q3_state == Q3_BRAKE_RETURN_NEG)
    {
        /* +5 cm is a virtual target here: apply a single strong reverse
         * braking command, then leave this state as the ball returns to
         * -5 cm.  The ball is never intended to travel back to +5 cm. */
        s_q3_drive_assist_active = 0U;
        s_integral_mm_s = 0.0f;
        s_stiction_active = 0U;
        q3_output_limit_counts = Q3_REVERSAL_OUTPUT_COUNTS;
        output = Q3_REVERSAL_OUTPUT_COUNTS;
        q3_suppress_motion_brake = 1U;
    }
    else if(s_q3_state == Q3_SETTLE_NEG)
    {
        /* Once -5 cm is first reached, stop kicking the rail.  A dedicated
         * velocity-dominant landing controller brakes the ball and then
         * levels the beam as speed approaches zero. */
        s_q3_drive_assist_active = 0U;
        s_integral_mm_s = 0.0f;
        s_stiction_active = 0U;
        q3_output_limit_counts = Q3_SETTLE_OUTPUT_LIMIT_COUNTS;
        output = Q3_SETTLE_KP_COUNTS_PER_MM * (float)error_mm
               - Q3_SETTLE_KD_COUNTS_S_PER_MM * s_velocity_mm_s;
        output = ClampFloat(output, -q3_output_limit_counts,
                            q3_output_limit_counts);
        q3_suppress_motion_brake = 1U;
    }
    else if((s_q3_state == Q3_HOLD_NEG_DONE) ||
            (s_q3_state == Q3_HOLD_NEG_FAILED))
    {
        /* The continuous controller stays soft.  If static friction leaves
         * the ball stopped far from -5 cm, a short re-armed kick nudges it;
         * the kick ends as soon as useful motion is detected. */
        s_q3_drive_assist_active = 0U;
        s_integral_mm_s = 0.0f;
        s_stiction_active = 0U;
        q3_output_limit_counts = Q3_HOLD_OUTPUT_LIMIT_COUNTS;
        output = Q3_HOLD_KP_COUNTS_PER_MM * (float)error_mm
               - Q3_HOLD_KD_COUNTS_S_PER_MM * s_velocity_mm_s;
        output = ClampFloat(output, -q3_output_limit_counts,
                            q3_output_limit_counts);

        if(s_q3_hold_kick_active != 0U)
        {
            if((g_ms >= s_q3_hold_kick_until_ms) ||
               (Abs32(error_mm) <= Q3_FINAL_TOLERANCE_MM) ||
               (((float)error_mm * s_velocity_mm_s > 0.0f) &&
                (AbsFloat(s_velocity_mm_s) >= Q3_HOLD_KICK_STOP_SPEED_MM_S)))
            {
                s_q3_hold_kick_active = 0U;
                s_q3_hold_kick_next_ms = g_ms + Q3_HOLD_KICK_REARM_MS;
            }
        }
        if((ENABLE_Q3_HOLD_KICK != 0U) &&
           (s_q3_hold_kick_active == 0U) &&
           (g_ms >= s_q3_hold_kick_next_ms) &&
           (Abs32(error_mm) >= Q3_HOLD_KICK_ENTER_MM) &&
           (AbsFloat(s_velocity_mm_s) <= Q3_ASSIST_RELEASE_SPEED_MM_S))
        {
            s_q3_hold_kick_active = 1U;
            s_q3_hold_kick_sign = (error_mm >= 0L) ? 1 : -1;
            s_q3_hold_kick_until_ms = g_ms + Q3_HOLD_KICK_MAX_MS;
        }
        if(s_q3_hold_kick_active != 0U)
        {
            q3_output_limit_counts = Q3_HOLD_KICK_COUNTS;
            output = Q3_HOLD_KICK_COUNTS * (float)s_q3_hold_kick_sign;
        }
        q3_suppress_motion_brake = 1U;
    }
    else
    {
        q3_assist_min_counts = Q3_POS_ASSIST_MIN_COUNTS;
        q3_assist_enter_mm = Q3_ASSIST_ENTER_MM;
        q3_assist_exit_mm = Q3_ASSIST_EXIT_MM;
        q3_allow_min_assist = 1U;
        if(s_q3_state == Q3_GO_NEG_50)
        {
            q3_output_limit_counts = Q3_NEG_OUTPUT_LIMIT_COUNTS;
            q3_assist_enter_mm = Q3_NEG_ASSIST_ENTER_MM;
            q3_assist_exit_mm = Q3_NEG_ASSIST_EXIT_MM;
            q3_assist_min_counts = Q3_NEG_FAR_ASSIST_MIN_COUNTS;
            if(Abs32(error_mm) <= Q3_NEG_NEAR_ZONE_MM)
            {
                q3_assist_min_counts = Q3_NEG_NEAR_ASSIST_MIN_COUNTS;
                output -= Q3_NEG_NEAR_EXTRA_KD * s_velocity_mm_s;
                output = ClampFloat(output, -q3_output_limit_counts,
                                    q3_output_limit_counts);
                if(((float)error_mm * s_velocity_mm_s > 0.0f) &&
                   (AbsFloat(s_velocity_mm_s) > Q3_ASSIST_RELEASE_SPEED_MM_S))
                    q3_allow_min_assist = 0U;
            }
        }
        if(Abs32(error_mm) >= q3_assist_enter_mm)
            s_q3_drive_assist_active = 1U;
        else if(Abs32(error_mm) <= q3_assist_exit_mm)
            s_q3_drive_assist_active = 0U;

        /* Preserve derivative braking when it intentionally opposes the
         * position error; only lift a same-direction command that is too
         * weak to break static friction. */
        if((s_q3_drive_assist_active != 0U) &&
           (q3_allow_min_assist != 0U) &&
           ((float)error_mm * output > 0.0f) &&
           (AbsFloat(output) < q3_assist_min_counts))
            output = (error_mm > 0L) ? q3_assist_min_counts
                                     : -q3_assist_min_counts;
    }

    if(q3_suppress_motion_brake != 0U)
    {
        motion_brake = 0.0f;
        s_motion_brake_active = 0U;
        s_motion_brake_x10 = 0L;
    }
    else
        motion_brake = DynamicMotionBrake(filtered_position_mm - s_ball_target_mm,
                                          s_velocity_mm_s,
                                          g_ms - s_camera.received_ms);
    output = ClampFloat(output + motion_brake,
                        -q3_output_limit_counts, q3_output_limit_counts);
    feedforward = TrapezoidFeedforward(filtered_position_mm - s_ball_target_mm,
                                       s_velocity_mm_s);
    command = (s32)(output + ((output >= 0.0f) ? 0.5f : -0.5f)) + feedforward;
    command *= BALL_TO_MOTOR_SIGN;
    if(command > MOTOR_TARGET_LIMIT_COUNTS) command = MOTOR_TARGET_LIMIT_COUNTS;
    if(command < -MOTOR_TARGET_LIMIT_COUNTS) command = -MOTOR_TARGET_LIMIT_COUNTS;
    if(s_q3_state == Q3_BRAKE_RETURN_NEG)
        target_slew_counts_s = Q3_REVERSAL_TARGET_SLEW_COUNTS_S;
    else if(s_q3_state == Q3_SETTLE_NEG)
        target_slew_counts_s = Q3_SETTLE_TARGET_SLEW_COUNTS_S;
    else if(s_q3_hold_kick_active != 0U)
        target_slew_counts_s = Q3_HOLD_KICK_TARGET_SLEW_COUNTS_S;
    else if((s_q3_state == Q3_GO_NEG_50) &&
            (Abs32(error_mm) <= Q3_NEG_NEAR_ZONE_MM))
        target_slew_counts_s = Q3_NEG_BRAKE_TARGET_SLEW_COUNTS_S;
    else
        target_slew_counts_s = (s_motion_brake_active != 0U)
                             ? MOTION_TARGET_SLEW_COUNTS_S
                             : TARGET_SLEW_COUNTS_S;
    s_target_count_float = SlewTo(s_target_count_float, (float)command,
                                  target_slew_counts_s * ((float)OUTER_PERIOD_MS * 0.001f));
    s_target_count = (s32)((s_target_count_float >= 0.0f) ?
                            s_target_count_float + 0.5f : s_target_count_float - 0.5f);
}

static void RunMotorPositionLoop(void)
{
    s32 error;
    s32 desired_rpm_x100;
    s32 slew_step;
    s32 abs_command;

    if(Abs32(s_encoder.count) > MOTOR_SOFT_LIMIT_COUNTS)
    {
        MotorStop(1U);
        if(Q3IsRunning() != 0U)
            Q3AbortToCenter(Q3_FAIL_CONTROL, "SOFT_LIMIT");
        s_state = CONTROL_FAULT;
        printf("FAULT,SOFT_LIMIT,count=%ld\r\n", s_encoder.count);
        return;
    }
    if(s_state == CONTROL_FAULT) return;

    error = s_target_count - s_encoder.count;
    if(Abs32(error) > s_peak_inner_error_count)
        s_peak_inner_error_count = Abs32(error);

    if(Abs32(error) <= MOTOR_POSITION_STOP_COUNTS ||
       ((s_motor_command_rpm_x100 == 0L) &&
        (Abs32(error) <= MOTOR_POSITION_RESTART_COUNTS)))
    {
        desired_rpm_x100 = 0L;
    }
    else
    {
        abs_command = MOTOR_SPEED_KP_X100_PER_COUNT * Abs32(error);
        if(abs_command < (s32)MOTOR_MIN_RPM_X100)
            abs_command = (s32)MOTOR_MIN_RPM_X100;
        if(abs_command > (s32)MOTOR_MAX_RPM_X100)
            abs_command = (s32)MOTOR_MAX_RPM_X100;
        desired_rpm_x100 = (error > 0L) ? abs_command : -abs_command;
    }

    /* Never reverse STEP/DIR while pulses are active.  First ramp the
     * signed speed to zero; the following 10 ms control tick starts the
     * opposite direction from a low speed. */
    if((s_motor_command_rpm_x100 > 0L && desired_rpm_x100 < 0L) ||
       (s_motor_command_rpm_x100 < 0L && desired_rpm_x100 > 0L))
    {
        desired_rpm_x100 = 0L;
    }

    if(desired_rpm_x100 == 0L ||
       Abs32(desired_rpm_x100) < Abs32(s_motor_command_rpm_x100))
        slew_step = (MOTOR_DECEL_RPM_X100_S * (s32)CONTROL_TICK_MS) / 1000L;
    else
        slew_step = (MOTOR_ACCEL_RPM_X100_S * (s32)CONTROL_TICK_MS) / 1000L;
    if(slew_step < 1L) slew_step = 1L;
    s_motor_command_rpm_x100 = SlewInt32(s_motor_command_rpm_x100,
                                         desired_rpm_x100, slew_step);

    if(s_motor_command_rpm_x100 == 0L)
    {
        MotorStop(0U); /* Hold torque remains enabled while balancing. */
        return;
    }

    abs_command = Abs32(s_motor_command_rpm_x100);
    MotorRunCountDirection((s_motor_command_rpm_x100 > 0L) ? 1L : -1L,
                           (u16)abs_command);

    if(Abs32(s_encoder.delta_count) > 0L) s_last_encoder_move_ms = g_ms;
    if(Abs32(error) >= MOTOR_STALL_ERROR_COUNTS &&
       (g_ms - s_last_encoder_move_ms) > MOTOR_STALL_TIMEOUT_MS)
    {
        MotorStop(1U);
        if(Q3IsRunning() != 0U)
            Q3AbortToCenter(Q3_FAIL_CONTROL, "ENCODER_NO_MOTION");
        s_state = CONTROL_FAULT;
        printf("FAULT,ENCODER_NO_MOTION,target=%ld,count=%ld\r\n",
               s_target_count, s_encoder.count);
    }
}

static void PrintStatus(void)
{
    if((g_ms - s_last_log_ms) < 500U) return;
    s_last_log_ms = g_ms;
    printf("BALL,t=%lu,state=%u,q3=%u,q3_ready=%u,q3_ms=%lu,q3_fail=%u,q3_assist=%u,q3_peak_pos=%ld,q3_peak_neg=%ld,ball_target_mm=%ld,raw_mm=%ld,filt_x10=%ld,conf=%u,vel_x10=%ld,motor_target=%ld,count=%ld,rpm_x100=%ld,brake_x10=%ld,age=%lu,valid=%lu,dup=%lu,invalid=%lu,parse=%lu,events=%lu,peak_mm=%ld,peak_vel_x10=%ld,peak_inner=%ld\r\n",
           g_ms, (u16)s_state, (u16)s_q3_state,
           (u16)s_q3_ready, s_q3_elapsed_ms,
           (u16)s_q3_failure_flags, (u16)s_q3_drive_assist_active,
           s_q3_peak_positive_mm, s_q3_peak_negative_mm, s_ball_target_mm,
           s_camera.position_mm, (s32)(s_filtered_position_mm * 10.0f),
           s_camera.confidence, (s32)(s_velocity_mm_s * 10.0f),
           s_target_count, s_encoder.count,
           s_motor_command_rpm_x100, s_motion_brake_x10,
           g_ms - s_camera.received_ms,
           s_valid_frame_count, s_duplicate_frame_count,
           s_invalid_frame_count, s_parse_error_count, s_motion_event_count,
           s_peak_abs_position_mm, s_peak_abs_velocity_x10,
           s_peak_inner_error_count);
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
    VideoModeButton_Init();

    s_state = CONTROL_WAIT_CAMERA;
    s_q3_state = Q3_IDLE_O;
    s_ball_target_mm = 0L;
    s_q3_failure_flags = Q3_FAIL_NONE;
    s_last_encoder_move_ms = 0U;
    printf("BALL_BEAM_READY,%s\r\n", CONTROL_FIRMWARE_VERSION);
    printf("LEVEL_BEAM_THEN_START: K230 $B frames, conf >= %u; FF=%u\r\n",
           K230_MIN_CONFIDENCE, ENABLE_TRAPEZOID_FEEDFORWARD);
    printf("KEYS,K3_PC8_RTSP,K4_PC9_Q3,ACTIVE_LOW,DEBOUNCE_MS=30\r\n");
    printf("VIDEO_MODE_DEFAULT,0,STM32_PA9_TO_K230_GPIO4\r\n");

    while(1)
    {
        PollK230Uart();
        if(g_control_due != 0U)
        {
            g_control_due = 0U;
            VideoModeButton_Service();
            MS42_Encoder_Service(g_ms);
            MS42_Encoder_GetData(&s_encoder);
            RunOuterLoop();
            RunMotorPositionLoop();
            PrintStatus();
        }
    }
}
