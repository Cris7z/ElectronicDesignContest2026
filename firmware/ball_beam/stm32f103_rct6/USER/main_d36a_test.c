#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "adc.h"
#include "d36a_motor.h"
#include "ms42_encoder.h"

extern volatile u16 K230_RAW_RX_COUNT;

/*
 * Ball balancing task: K230 measures the ball coordinate and sends
 * $B,SEQ,POS_MM,CONF*XOR\r\n through UART1 at 115200 bit/s.
 * STM32 USART2_RX = PA3 receives it.  This program drives D36A channel 1:
 * PB6=ST1, PB8=EN1, PB9=DIR1.
 */

#define MOTOR_MICROSTEP          16U
#define MOTOR_RPM_MAX            18U
#define MOTOR_RPM_MIN            3U
#define MOTOR_RAMP_INTERVAL_MS   20U
#define MOTOR_RAMP_STEP_RPM      1L

/* Temporary communication diagnostic.  With this set to 1, a valid K230
 * packet makes the motor turn at low speed for exactly one second, once.
 * Set it back to 0 after the link test to run the +5cm -> -5cm task. */
#define UART_LINK_TEST            0
#define D36A_BOOT_SELF_TEST       0

/* Calibration signs.  Keep both at 1 initially.  If the motor encoder count
 * moves opposite to a commanded CW motor move, change ENCODER_CW_SIGN.  If
 * the ball moves away from its requested correction, change BALL_TILT_SIGN. */
#define ENCODER_CW_SIGN           1
#define BALL_TILT_SIGN            1

#define CENTER_TARGET_MM          0L
#define CENTER_DEADBAND_MM        8L     /* Keep within +/-1 cm with 2 mm margin. */
#define CENTER_STOP_SPEED_MM_S    12L
#define VISION_TIMEOUT_MS         150U
#define MIN_VISION_CONFIDENCE      40U

/* V0 centre-hold gains.  Ki deliberately remains zero. */
#define KP_RPM_PER_MM_X100       28L     /* 0.28 rpm/mm */
#define KD_RPM_PER_MM_S_X100      7L     /* 0.07 rpm/(mm/s) */

/* Alpha-beta estimator coefficients, for 0.1 mm position state units. */
#define EST_ALPHA_X100           35L
#define EST_BETA_X100            12L

/* Predictive V0 outer loop: desired bounded motor-position offset. */
#define LOOKAHEAD_MS              70L
#define OUTER_KP_COUNT_MM_X100   550L    /* 5.50 encoder counts/mm */
#define OUTER_KV_COUNT_MMS_X100   90L    /* 0.90 count/(mm/s) */
#define OUTER_KA_COUNT_MMSS_X100   2L    /* 0.02 count/(mm/s^2) */
#define MAX_MOTOR_OFFSET_COUNTS  120L

/* MS42 encoder inner position servo. */
#define MOTOR_POSITION_DEADBAND_COUNTS  4L
#define INNER_KP_RPM_COUNT_X100        18L /* 0.18 rpm/count */

typedef enum
{
    TASK_WAIT_FOR_VISION = 0,
    TASK_HOLD_CENTER,
    TASK_FAULT
} TaskState_t;

static volatile u32 g_control_ms;
static volatile u8 g_control_due;

static TaskState_t s_state;
static s32 s_position_x10;
static s32 s_speed_mm_s;
static s32 s_acceleration_mm_s2;
static u32 s_last_estimator_ms;
static u32 s_last_measurement_ms;
static u32 s_last_sample_ms;
static u8 s_have_position;
static u8 s_good_start_samples;
static u8 s_last_sequence;
static u8 s_have_sequence;
static s32 s_motor_command_rpm;
static u32 s_motor_ramp_ms;
static s32 s_motor_neutral_count;
static s32 s_motor_target_count;
static MS42_EncoderData_t s_motor_encoder;
#if UART_LINK_TEST
static u8 s_link_test_started;
static u8 s_link_test_finished;
static u32 s_link_test_start_ms;
#endif

static s32 Abs32(s32 value)
{
    return (value < 0L) ? -value : value;
}

/* TIM1 is used only as an accurate 1 ms task clock.  TIM2/3 remain available
 * to the already-connected MS42 encoder and TIM4 generates STEP pulses. */
static void ControlTick_Init(void)
{
    TIM_TimeBaseInitTypeDef timer;
    NVIC_InitTypeDef nvic;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
    TIM_TimeBaseStructInit(&timer);
    timer.TIM_Prescaler = 71U;       /* 72 MHz / 72 = 1 MHz */
    timer.TIM_Period = 999U;         /* 1 ms */
    timer.TIM_CounterMode = TIM_CounterMode_Up;
    timer.TIM_ClockDivision = TIM_CKD_DIV1;
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
        g_control_ms++;
        if((g_control_ms % 5U) == 0U)
            g_control_due = 1U;      /* 200 Hz predict/control; ISR stays short. */
    }
}

static u8 HexToNibble(u8 value, u8 *nibble)
{
    if(value >= '0' && value <= '9')
        *nibble = (u8)(value - '0');
    else if(value >= 'A' && value <= 'F')
        *nibble = (u8)(value - 'A' + 10U);
    else if(value >= 'a' && value <= 'f')
        *nibble = (u8)(value - 'a' + 10U);
    else
        return 0U;
    return 1U;
}

static u8 ParseUnsigned(const u8 *text, u16 length, u16 *index,
                        s32 *value, u8 terminator)
{
    s32 result = 0L;
    u8 digits = 0U;

    while(*index < length && text[*index] != terminator)
    {
        if(text[*index] < '0' || text[*index] > '9')
            return 0U;
        result = result * 10L + (s32)(text[*index] - '0');
        (*index)++;
        digits++;
    }
    if(digits == 0U || *index >= length)
        return 0U;
    *value = result;
    return 1U;
}

static u8 ParseSigned(const u8 *text, u16 length, u16 *index,
                      s32 *value, u8 terminator)
{
    u8 negative = 0U;
    if(*index < length && text[*index] == '-')
    {
        negative = 1U;
        (*index)++;
    }
    else if(*index < length && text[*index] == '+')
        (*index)++;

    if(ParseUnsigned(text, length, index, value, terminator) == 0U)
        return 0U;
    if(negative != 0U)
        *value = -*value;
    return 1U;
}

/* Read and validate a complete K230 text frame collected by USART2 IRQ. */
static u8 K230_ReadBallSample(s16 *position_mm, u8 *confidence, u8 *sequence)
{
    u16 length;
    u16 index;
    u16 star_index;
    s32 seq_value;
    s32 position_value;
    s32 confidence_value;
    u8 checksum = 0U;
    u8 packet_checksum;
    u8 high_nibble;
    u8 low_nibble;

    if((USART_RX_STA & 0x8000U) == 0U)
        return 0U;

    length = (u16)(USART_RX_STA & 0x3FFFU);
    if(length < 10U || USART_RX_BUF[0] != '$' || USART_RX_BUF[1] != 'B'
       || USART_RX_BUF[2] != ',')
        goto invalid_frame;

    star_index = 0U;
    for(index = 3U; index < length; index++)
    {
        if(USART_RX_BUF[index] == '*')
        {
            star_index = index;
            break;
        }
    }
    if(star_index == 0U || (u16)(star_index + 3U) != length)
        goto invalid_frame;

    for(index = 1U; index < star_index; index++)
        checksum ^= USART_RX_BUF[index];
    if(HexToNibble(USART_RX_BUF[star_index + 1U], &high_nibble) == 0U
       || HexToNibble(USART_RX_BUF[star_index + 2U], &low_nibble) == 0U)
        goto invalid_frame;
    packet_checksum = (u8)((high_nibble << 4) | low_nibble);
    if(checksum != packet_checksum)
        goto invalid_frame;

    index = 3U;
    if(ParseUnsigned(USART_RX_BUF, star_index, &index, &seq_value, ',') == 0U)
        goto invalid_frame;
    index++;
    if(ParseSigned(USART_RX_BUF, star_index, &index, &position_value, ',') == 0U)
        goto invalid_frame;
    index++;
    /* Pass the full frame length here so ParseUnsigned can see the '*'
     * delimiter.  Passing star_index made every otherwise-good frame fail. */
    if(ParseUnsigned(USART_RX_BUF, length, &index, &confidence_value, '*') == 0U)
        goto invalid_frame;
    if(seq_value > 255L || position_value < -32768L || position_value > 32767L
       || confidence_value > 100L)
        goto invalid_frame;

    *sequence = (u8)seq_value;
    *position_mm = (s16)position_value;
    *confidence = (u8)confidence_value;
    USART_RX_STA = 0U;
    return 1U;

invalid_frame:
    USART_RX_STA = 0U;
    return 0U;
}

/* Predict at every 5 ms control tick; correct only once for each fresh K230
 * frame.  Position is 0.1 mm and velocity is mm/s. */
static void Estimator_Predict(u32 now_ms)
{
    u32 dt_ms;

    if(s_have_position == 0U)
        return;
    dt_ms = now_ms - s_last_estimator_ms;
    if(dt_ms == 0U)
        return;
    if(dt_ms > VISION_TIMEOUT_MS)
        dt_ms = VISION_TIMEOUT_MS;
    s_position_x10 += (s_speed_mm_s * (s32)dt_ms) / 100L;
    s_last_estimator_ms = now_ms;
}

static void Estimator_Correct(s16 raw_mm, u32 now_ms)
{
    s32 measurement_x10 = (s32)raw_mm * 10L;
    s32 residual_x10;
    s32 previous_speed_mm_s;
    s32 acceleration_sample;
    u32 measurement_dt_ms;

    if(s_have_position == 0U)
    {
        s_position_x10 = measurement_x10;
        s_speed_mm_s = 0L;
        s_acceleration_mm_s2 = 0L;
        s_have_position = 1U;
        s_last_estimator_ms = now_ms;
        s_last_measurement_ms = now_ms;
        return;
    }

    measurement_dt_ms = now_ms - s_last_measurement_ms;
    Estimator_Predict(now_ms);
    if(measurement_dt_ms == 0U || measurement_dt_ms > VISION_TIMEOUT_MS)
    {
        s_position_x10 = measurement_x10;
        s_speed_mm_s = 0L;
        s_acceleration_mm_s2 = 0L;
        s_last_measurement_ms = now_ms;
        return;
    }

    previous_speed_mm_s = s_speed_mm_s;
    residual_x10 = measurement_x10 - s_position_x10;
    s_position_x10 += (residual_x10 * EST_ALPHA_X100) / 100L;
    s_speed_mm_s += (residual_x10 * EST_BETA_X100) / (s32)measurement_dt_ms;
    acceleration_sample = ((s_speed_mm_s - previous_speed_mm_s) * 1000L)
                        / (s32)measurement_dt_ms;
    if(acceleration_sample > 500L)
        acceleration_sample = 500L;
    else if(acceleration_sample < -500L)
        acceleration_sample = -500L;
    /* Low-bandwidth acceleration observer: do not use raw second differences. */
    s_acceleration_mm_s2 = (s_acceleration_mm_s2 * 3L + acceleration_sample) / 4L;
    s_last_measurement_ms = now_ms;
}

static void Motor_StopImmediately(void)
{
    s_motor_command_rpm = 0L;
    D36A_Motor1_Stop();
}

/* A motor direction reversal always ramps through zero.  This keeps the
 * inner position loop from flipping DIR on consecutive vision frames. */
static void MotorPositionServo(s32 target_count, u32 now_ms)
{
    s32 position_error;
    s32 desired_rpm;
    s32 difference;
    u16 rpm;
    u8 clockwise;

    position_error = target_count - s_motor_encoder.count;
    if(Abs32(position_error) <= MOTOR_POSITION_DEADBAND_COUNTS)
        desired_rpm = 0L;
    else
        desired_rpm = (INNER_KP_RPM_COUNT_X100 * position_error) / 100L;

    if(desired_rpm > (s32)MOTOR_RPM_MAX)
        desired_rpm = MOTOR_RPM_MAX;
    else if(desired_rpm < -(s32)MOTOR_RPM_MAX)
        desired_rpm = -(s32)MOTOR_RPM_MAX;

    if((now_ms - s_motor_ramp_ms) < MOTOR_RAMP_INTERVAL_MS)
        return;
    s_motor_ramp_ms = now_ms;

    if((s_motor_command_rpm > 0L && desired_rpm < 0L)
       || (s_motor_command_rpm < 0L && desired_rpm > 0L))
        desired_rpm = 0L;

    difference = desired_rpm - s_motor_command_rpm;
    if(difference > MOTOR_RAMP_STEP_RPM)
        s_motor_command_rpm += MOTOR_RAMP_STEP_RPM;
    else if(difference < -MOTOR_RAMP_STEP_RPM)
        s_motor_command_rpm -= MOTOR_RAMP_STEP_RPM;
    else
        s_motor_command_rpm = desired_rpm;

    if(s_motor_command_rpm == 0L)
    {
        D36A_Motor1_Stop();
        return;
    }

    clockwise = (s_motor_command_rpm > 0L) ? 1U : 0U;
    if(ENCODER_CW_SIGN < 0)
        clockwise = (clockwise == 0U) ? 1U : 0U;
    rpm = (u16)Abs32(s_motor_command_rpm);
    if(rpm < MOTOR_RPM_MIN)
        rpm = MOTOR_RPM_MIN;
    D36A_Motor1_SetDirection(clockwise);
    D36A_Motor1_SetRpm(rpm, MOTOR_MICROSTEP);
}

static void PredictiveBallController(u32 now_ms)
{
    s32 predicted_position_x10;
    s32 predicted_error_mm;
    s32 requested_offset;

    /* Predict where the ball will be after camera/motor latency.  The final
     * term is an acceleration feedforward/braking correction from a strongly
     * filtered acceleration estimate, not a raw second difference. */
    predicted_position_x10 = s_position_x10
        + (s_speed_mm_s * LOOKAHEAD_MS) / 100L
        + (s_acceleration_mm_s2 * LOOKAHEAD_MS * LOOKAHEAD_MS) / 200000L;
    predicted_error_mm = CENTER_TARGET_MM - predicted_position_x10 / 10L;

    if(Abs32(predicted_error_mm) <= CENTER_DEADBAND_MM
       && Abs32(s_speed_mm_s) <= CENTER_STOP_SPEED_MM_S)
        requested_offset = 0L;
    else
        requested_offset = (OUTER_KP_COUNT_MM_X100 * predicted_error_mm
                          - OUTER_KV_COUNT_MMS_X100 * s_speed_mm_s
                          - OUTER_KA_COUNT_MMSS_X100 * s_acceleration_mm_s2) / 100L;

    if(requested_offset > MAX_MOTOR_OFFSET_COUNTS)
        requested_offset = MAX_MOTOR_OFFSET_COUNTS;
    else if(requested_offset < -MAX_MOTOR_OFFSET_COUNTS)
        requested_offset = -MAX_MOTOR_OFFSET_COUNTS;

    s_motor_target_count = s_motor_neutral_count + BALL_TILT_SIGN * requested_offset;
    MotorPositionServo(s_motor_target_count, now_ms);
}

static void BallController_Step(void)
{
    s16 raw_position_mm;
    u8 confidence;
    u8 sequence;
    u8 new_sample = 0U;
    u32 now_ms = g_control_ms;

    /* Count is valid at this rate; the legacy function's speed field is not
     * used by this controller. */
    MS42_Encoder_Update100ms();
    MS42_Encoder_GetData(&s_motor_encoder);
    Estimator_Predict(now_ms);

    if(K230_ReadBallSample(&raw_position_mm, &confidence, &sequence) != 0U)
    {
        /* A repeated sequence is harmless, but does not refresh the watchdog. */
        if(s_have_sequence == 0U || sequence != s_last_sequence)
        {
            s_last_sequence = sequence;
            s_have_sequence = 1U;
            if(confidence >= MIN_VISION_CONFIDENCE)
            {
                Estimator_Correct(raw_position_mm, now_ms);
                s_last_sample_ms = now_ms;
                new_sample = 1U;
            }
        }
    }

#if UART_LINK_TEST
    /* This is deliberately independent of the ball-control PID.  It proves
     * the complete K230 TX -> PA3 -> USART2 parser -> D36A path. */
    if(s_link_test_started == 0U && s_link_test_finished == 0U
       && K230_RAW_RX_COUNT != 0U)
    {
        s_link_test_started = 1U;
        s_link_test_start_ms = now_ms;
        D36A_Motor1_SetDirection(1U);
        D36A_Motor1_SetRpm(18U, MOTOR_MICROSTEP);
        printf("K230 UART OK: motor link test started\r\n");
    }
    if(s_link_test_started != 0U && (now_ms - s_link_test_start_ms) >= 1000U)
    {
        D36A_Motor1_Stop();
        s_link_test_started = 0U;
        s_link_test_finished = 1U;
        printf("K230 UART OK: motor link test passed\r\n");
    }
    return;
#endif

    if(s_state == TASK_WAIT_FOR_VISION)
    {
        Motor_StopImmediately();
        if(s_have_position != 0U && (now_ms - s_last_sample_ms) > VISION_TIMEOUT_MS)
            s_good_start_samples = 0U;
        if(new_sample != 0U)
        {
            if(s_good_start_samples < 5U)
                s_good_start_samples++;
            if(s_good_start_samples >= 5U)
            {
                s_motor_neutral_count = s_motor_encoder.count;
                s_motor_target_count = s_motor_neutral_count;
                s_state = TASK_HOLD_CENTER;
                s_motor_ramp_ms = now_ms;
                printf("PREDICTIVE CENTER HOLD ARMED: q0=%ld\r\n",
                       s_motor_neutral_count);
            }
        }
        return;
    }

    if(s_state == TASK_FAULT)
        return;

    if((now_ms - s_last_sample_ms) > VISION_TIMEOUT_MS)
    {
        Motor_StopImmediately();
        s_state = TASK_FAULT;
        printf("FAULT: K230 vision timeout\r\n");
        return;
    }

    PredictiveBallController(now_ms);
}

int main(void)
{
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    delay_init();
    uart_init(115200U);
    ADC_Power_Init();

    D36A_Motor_Init();
    D36A_StepperTimer_Init();
    D36A_Motor1_Enable(1U);
    D36A_Motor1_Stop();
    MS42_Encoder_Init();
    ControlTick_Init();

#if D36A_BOOT_SELF_TEST
    /* Independent actuator-path check: deliberately ignores UART and vision. */
    D36A_Motor1_SetDirection(1U);
    D36A_Motor1_SetRpm(18U, MOTOR_MICROSTEP);
    delay_ms(1000U);
    D36A_Motor1_Stop();
#endif

    s_state = TASK_WAIT_FOR_VISION;
    printf("Ball control ready: wait K230 UART1 on PA3/USART2\r\n");

    while(1)
    {
        if(g_control_due != 0U)
        {
            g_control_due = 0U;
            BallController_Step();
        }
    }
}
