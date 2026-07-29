#include "h2026_bsp.h"

#include "ti_msp_dl_config.h"

/*
 * The control timer runs at 125 kHz after SysConfig's /256 prescaler.
 * The legacy U3 interface uses PB16/PB17 as open-drain software I2C.  A complete
 * register-select + one-byte read normally takes about 1.1 to 1.5 ms at the
 * effective initial rate of about 33 kHz.  Two milliseconds is deliberately
 * strict while still leaving
 * margin for clock stretching.  The iteration guard is a second line of
 * defence if interrupts are accidentally disabled.
 */
#define H2026_I2C_TIMEOUT_TIMER_COUNTS 250u
#define H2026_I2C_SPIN_GUARD           200000u
#define H2026_SOFT_I2C_HALF_PERIOD_CYCLES (CPUCLK_FREQ / 100000u)
#define H2026_SOFT_I2C_SCL_RISE_SPIN_GUARD 1000u
#define H2026_OLED_HALF_PERIOD_CYCLES     (CPUCLK_FREQ / 2000000u)
#define H2026_OLED_RESET_LOW_CYCLES    (CPUCLK_FREQ / 500u)
#define H2026_OLED_RESET_WAIT_CYCLES   (CPUCLK_FREQ / 100u)
#define H2026_DISPLAY_PERIOD_TICKS \
    (H2026_BSP_DISPLAY_PERIOD_MS / H2026_BSP_CONTROL_PERIOD_MS)
/* Manual UART state traffic is one request byte and one response byte. */
#define H2026_LINE_UART_RESPONSE_TIMEOUT_TICKS 4u
#define H2026_LINE_UART_STALE_TICKS            6u

typedef struct {
    uint32_t control_ticks;
    uint32_t timer_count;
    uint32_t spin_guard;
} i2c_deadline_t;

typedef struct {
    int8_t applied_sign;
    int8_t pending_sign;
    uint32_t coast_started_tick;
} motor_direction_guard_t;

static volatile int64_t s_encoder_left;
static volatile int64_t s_encoder_right;
static volatile uint32_t s_encoder_left_invalid;
static volatile uint32_t s_encoder_right_invalid;
static volatile uint8_t s_encoder_left_phase;
static volatile uint8_t s_encoder_right_phase;

static volatile bool s_control_tick_pending;
static volatile uint32_t s_control_ticks;
static volatile uint32_t s_control_tick_overruns;
static volatile bool s_display_refresh_pending;
static uint8_t s_display_tick_divider;
static volatile uint32_t s_display_counter;

static volatile uint32_t s_i2c_transactions;
static volatile uint32_t s_i2c_timeouts;
static volatile uint32_t s_i2c_bus_errors;

static bool s_line_uart_request_pending;
static bool s_line_uart_have_state;
static uint8_t s_line_uart_state;
static uint32_t s_line_uart_request_tick;
static uint32_t s_line_uart_last_state_tick;
static volatile uint32_t s_line_uart_requests;
static volatile uint32_t s_line_uart_responses;
static volatile uint32_t s_line_uart_timeouts;

static uint32_t s_pwm_motor_period;
static bool s_motor_armed;
static motor_direction_guard_t s_left_direction;
static motor_direction_guard_t s_right_direction;

/*
 * Four-state quadrature transition table.  Its sign describes electrical
 * A/B phase order only; the Q2 controller independently configures which sign
 * means vehicle-forward for each wheel.
 */
static const int8_t k_quadrature_lut[16] = {
     0, -1,  1,  0,
     1,  0,  0, -1,
    -1,  0,  0,  1,
     0,  1, -1,  0
};

static uint8_t read_left_phase(void)
{
    uint32_t pins = DL_GPIO_readPins(
        GPIO_ENCODER_LEFT_PORT,
        GPIO_ENCODER_LEFT_LEFT_A_PIN | GPIO_ENCODER_LEFT_LEFT_B_PIN);
    uint8_t phase = 0u;
    if ((pins & GPIO_ENCODER_LEFT_LEFT_A_PIN) != 0u) {
        phase |= 1u;
    }
    if ((pins & GPIO_ENCODER_LEFT_LEFT_B_PIN) != 0u) {
        phase |= 2u;
    }
    return phase;
}

static uint8_t read_right_phase(void)
{
    uint32_t pins = DL_GPIO_readPins(
        GPIO_ENCODER_RIGHT_PORT,
        GPIO_ENCODER_RIGHT_RIGHT_A_PIN | GPIO_ENCODER_RIGHT_RIGHT_B_PIN);
    uint8_t phase = 0u;
    if ((pins & GPIO_ENCODER_RIGHT_RIGHT_A_PIN) != 0u) {
        phase |= 1u;
    }
    if ((pins & GPIO_ENCODER_RIGHT_RIGHT_B_PIN) != 0u) {
        phase |= 2u;
    }
    return phase;
}

static void update_left_encoder(void)
{
    uint8_t previous = s_encoder_left_phase;
    uint8_t current = read_left_phase();
    int8_t delta = k_quadrature_lut[(previous << 2u) | current];

    if ((current != previous) && (delta == 0)) {
        ++s_encoder_left_invalid;
    } else {
        s_encoder_left += delta;
    }
    s_encoder_left_phase = current;
}

static void update_right_encoder(void)
{
    uint8_t previous = s_encoder_right_phase;
    uint8_t current = read_right_phase();
    int8_t delta = k_quadrature_lut[(previous << 2u) | current];

    if ((current != previous) && (delta == 0)) {
        ++s_encoder_right_invalid;
    } else {
        s_encoder_right += delta;
    }
    s_encoder_right_phase = current;
}

static uint32_t duty_to_compare(float duty, uint32_t period)
{
    if (!(duty == duty) || duty <= 0.0f) {
        return period;
    }
    if (duty >= 1.0f) {
        return 0u;
    }
    return (uint32_t)((1.0f - duty) * (float)period + 0.5f);
}

static void set_left_pwm(float duty)
{
    /* TB6612 B is the physical left wheel, verified on the suspended chassis. */
    DL_TimerA_setCaptureCompareValue(
        PWM_MOTOR_INST,
        duty_to_compare(duty, s_pwm_motor_period),
        DL_TIMER_CC_1_INDEX);
}

static void set_right_pwm(float duty)
{
    /* TB6612 A is the physical right wheel, verified on the suspended chassis. */
    DL_TimerA_setCaptureCompareValue(
        PWM_MOTOR_INST,
        duty_to_compare(duty, s_pwm_motor_period),
        DL_TIMER_CC_0_INDEX);
}

static void set_left_direction(bool input1, bool input2)
{
    if (input1) {
        DL_GPIO_setPins(GPIO_MOTOR_DIR_PORT, GPIO_MOTOR_DIR_TB_B_IN1_PIN);
    } else {
        DL_GPIO_clearPins(GPIO_MOTOR_DIR_PORT, GPIO_MOTOR_DIR_TB_B_IN1_PIN);
    }
    if (input2) {
        DL_GPIO_setPins(GPIO_MOTOR_DIR_PORT, GPIO_MOTOR_DIR_TB_B_IN2_PIN);
    } else {
        DL_GPIO_clearPins(GPIO_MOTOR_DIR_PORT, GPIO_MOTOR_DIR_TB_B_IN2_PIN);
    }
}

static void set_right_direction(bool input1, bool input2)
{
    if (input1) {
        DL_GPIO_setPins(GPIO_MOTOR_DIR_PORT, GPIO_MOTOR_DIR_TB_A_IN1_PIN);
    } else {
        DL_GPIO_clearPins(GPIO_MOTOR_DIR_PORT, GPIO_MOTOR_DIR_TB_A_IN1_PIN);
    }
    if (input2) {
        DL_GPIO_setPins(GPIO_MOTOR_DIR_PORT, GPIO_MOTOR_DIR_TB_A_IN2_PIN);
    } else {
        DL_GPIO_clearPins(GPIO_MOTOR_DIR_PORT, GPIO_MOTOR_DIR_TB_A_IN2_PIN);
    }
}

static void reset_direction_guard(motor_direction_guard_t *guard)
{
    guard->applied_sign = 0;
    guard->pending_sign = 0;
    guard->coast_started_tick = s_control_ticks;
}

static void reset_both_direction_guards(void)
{
    reset_direction_guard(&s_left_direction);
    reset_direction_guard(&s_right_direction);
}

static void motor_coast_unconditionally(void)
{
    set_left_pwm(0.0f);
    set_right_pwm(0.0f);
    set_left_direction(false, false);
    set_right_direction(false, false);
    reset_both_direction_guards();
}

static int8_t duty_sign(float duty)
{
    if (!(duty == duty) || duty == 0.0f) {
        return 0;
    }
    return (duty > 0.0f) ? 1 : -1;
}

/*
 * A non-zero sign reversal is a two-call transaction:
 *   call N:   force this wheel to 00 and remember the requested sign;
 *   call N+1: apply PWM only if the same sign is still requested and at
 *             least one 5 ms control tick has elapsed.
 *
 * The tick check prevents a second call in the same foreground iteration
 * from accidentally bypassing the electrical coast interval.
 */
static bool direction_guard_allows_drive(
    motor_direction_guard_t *guard,
    int8_t requested_sign,
    uint32_t control_tick)
{
    if (requested_sign == 0) {
        reset_direction_guard(guard);
        return false;
    }
    if (guard->applied_sign == requested_sign) {
        guard->pending_sign = 0;
        return true;
    }
    if (guard->applied_sign != 0) {
        guard->applied_sign = 0;
        guard->pending_sign = requested_sign;
        guard->coast_started_tick = control_tick;
        return false;
    }
    if (guard->pending_sign == 0) {
        guard->applied_sign = requested_sign;
        return true;
    }
    if (guard->pending_sign != requested_sign) {
        guard->pending_sign = requested_sign;
        guard->coast_started_tick = control_tick;
        return false;
    }
    if (control_tick == guard->coast_started_tick) {
        return false;
    }

    guard->pending_sign = 0;
    guard->applied_sign = requested_sign;
    return true;
}

static void set_signed_one_left(float duty, uint32_t control_tick)
{
    int8_t requested_sign = duty_sign(duty);

    if (!direction_guard_allows_drive(
            &s_left_direction, requested_sign, control_tick)) {
        set_left_pwm(0.0f);
        set_left_direction(false, false);
    } else if (requested_sign > 0) {
        set_left_direction(true, false);
        set_left_pwm(duty);
    } else {
        set_left_direction(false, true);
        set_left_pwm(-duty);
    }
}

static void set_signed_one_right(float duty, uint32_t control_tick)
{
    int8_t requested_sign = duty_sign(duty);

    if (!direction_guard_allows_drive(
            &s_right_direction, requested_sign, control_tick)) {
        set_right_pwm(0.0f);
        set_right_direction(false, false);
    } else if (requested_sign > 0) {
        set_right_direction(true, false);
        set_right_pwm(duty);
    } else {
        set_right_direction(false, true);
        set_right_pwm(-duty);
    }
}

static i2c_deadline_t i2c_deadline_start(void)
{
    i2c_deadline_t deadline;
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    deadline.control_ticks = s_control_ticks;
    deadline.timer_count = DL_TimerG_getTimerCount(TIMER_CONTROL_INST);
    __set_PRIMASK(primask);
    deadline.spin_guard = H2026_I2C_SPIN_GUARD;
    return deadline;
}

static bool i2c_deadline_expired(i2c_deadline_t *deadline)
{
    uint32_t ticks;
    uint32_t timer_count;
    uint32_t periods;
    uint32_t elapsed;

    if (deadline->spin_guard == 0u) {
        return true;
    }
    --deadline->spin_guard;

    /*
     * Read tick generation before and after the counter.  If an overflow
     * lands between the reads, retry once so the pair describes one period.
     */
    do {
        ticks = s_control_ticks;
        timer_count = DL_TimerG_getTimerCount(TIMER_CONTROL_INST);
    } while (ticks != s_control_ticks);

    periods = ticks - deadline->control_ticks;
    elapsed = periods * (TIMER_CONTROL_INST_LOAD_VALUE + 1u) +
              deadline->timer_count;
    if (elapsed < timer_count) {
        /* Counter wrapped but its ISR has not yet updated the generation. */
        return true;
    }
    elapsed -= timer_count;
    return elapsed >= H2026_I2C_TIMEOUT_TIMER_COUNTS;
}

/*
 * H8's PB6/PB7 pads were proved electrically isolated from the MCU on the
 * assembled vehicle, so U3 PB16/PB17 are used instead. The sensor provides
 * the verified 3.3 V pull-ups, so drive-low means output=0 and a logic high
 * is always made by disabling the GPIO output. Never drive high.
 */
static void soft_i2c_delay(void)
{
    delay_cycles(H2026_SOFT_I2C_HALF_PERIOD_CYCLES);
}

static void soft_i2c_scl_low(void)
{
    DL_GPIO_clearPins(
        GPIO_LINE_SOFT_I2C_PORT, GPIO_LINE_SOFT_I2C_LINE_SCL_PIN);
    DL_GPIO_enableOutput(
        GPIO_LINE_SOFT_I2C_PORT, GPIO_LINE_SOFT_I2C_LINE_SCL_PIN);
}

static void soft_i2c_sda_low(void)
{
    DL_GPIO_clearPins(
        GPIO_LINE_SOFT_I2C_PORT, GPIO_LINE_SOFT_I2C_LINE_SDA_PIN);
    DL_GPIO_enableOutput(
        GPIO_LINE_SOFT_I2C_PORT, GPIO_LINE_SOFT_I2C_LINE_SDA_PIN);
}

static void soft_i2c_scl_release(void)
{
    DL_GPIO_disableOutput(
        GPIO_LINE_SOFT_I2C_PORT, GPIO_LINE_SOFT_I2C_LINE_SCL_PIN);
}

static void soft_i2c_sda_release(void)
{
    DL_GPIO_disableOutput(
        GPIO_LINE_SOFT_I2C_PORT, GPIO_LINE_SOFT_I2C_LINE_SDA_PIN);
}

static void soft_i2c_release_bus(void)
{
    soft_i2c_scl_release();
    soft_i2c_sda_release();
}

static bool soft_i2c_scl_high(void)
{
    return (DL_GPIO_readPins(
                GPIO_LINE_SOFT_I2C_PORT,
                GPIO_LINE_SOFT_I2C_LINE_SCL_PIN) != 0u);
}

static bool soft_i2c_sda_high(void)
{
    return (DL_GPIO_readPins(
                GPIO_LINE_SOFT_I2C_PORT,
                GPIO_LINE_SOFT_I2C_LINE_SDA_PIN) != 0u);
}

static bool soft_i2c_raise_scl(
    i2c_deadline_t *deadline, bool *timed_out)
{
    uint32_t rise_spins = H2026_SOFT_I2C_SCL_RISE_SPIN_GUARD;

    if (i2c_deadline_expired(deadline)) {
        *timed_out = true;
        return false;
    }
    soft_i2c_scl_release();
    while (!soft_i2c_scl_high()) {
        /*
         * A connected I2C slave may stretch SCL, but an unconnected U3 or a
         * shorted line must not consume the entire 5 ms control period.  A
         * normal external pull-up rises in a few microseconds; 1000 GPIO
         * polls is deliberately much longer than that while remaining fast.
         */
        if (rise_spins == 0u) {
            return false;
        }
        --rise_spins;
        if (i2c_deadline_expired(deadline)) {
            *timed_out = true;
            return false;
        }
    }
    soft_i2c_delay();
    return true;
}

static bool soft_i2c_start(i2c_deadline_t *deadline, bool *timed_out)
{
    /* Also works as a repeated START because SDA is released while SCL is low. */
    soft_i2c_sda_release();
    soft_i2c_scl_release();
    if (!soft_i2c_raise_scl(deadline, timed_out)) {
        return false;
    }
    if (!soft_i2c_sda_high()) {
        return false;
    }

    soft_i2c_sda_low();
    soft_i2c_delay();
    soft_i2c_scl_low();
    soft_i2c_delay();
    return true;
}

static bool soft_i2c_stop(i2c_deadline_t *deadline, bool *timed_out)
{
    soft_i2c_sda_low();
    soft_i2c_delay();
    if (!soft_i2c_raise_scl(deadline, timed_out)) {
        return false;
    }
    soft_i2c_sda_release();
    soft_i2c_delay();
    return soft_i2c_sda_high();
}

static bool soft_i2c_write_byte(
    uint8_t value, i2c_deadline_t *deadline, bool *timed_out)
{
    uint8_t bit_mask;

    for (bit_mask = 0x80u; bit_mask != 0u; bit_mask >>= 1u) {
        if ((value & bit_mask) != 0u) {
            soft_i2c_sda_release();
        } else {
            soft_i2c_sda_low();
        }
        soft_i2c_delay();
        if (!soft_i2c_raise_scl(deadline, timed_out)) {
            return false;
        }
        soft_i2c_scl_low();
        soft_i2c_delay();
    }

    /* Ninth clock: the slave ACKs by pulling SDA low. */
    soft_i2c_sda_release();
    soft_i2c_delay();
    if (!soft_i2c_raise_scl(deadline, timed_out)) {
        return false;
    }
    if (soft_i2c_sda_high()) {
        soft_i2c_scl_low();
        soft_i2c_delay();
        return false;
    }
    soft_i2c_scl_low();
    soft_i2c_delay();
    return true;
}

static bool soft_i2c_read_byte_nack(
    uint8_t *value, i2c_deadline_t *deadline, bool *timed_out)
{
    uint8_t bit_mask;
    uint8_t received = 0u;

    for (bit_mask = 0x80u; bit_mask != 0u; bit_mask >>= 1u) {
        soft_i2c_sda_release();
        soft_i2c_delay();
        if (!soft_i2c_raise_scl(deadline, timed_out)) {
            return false;
        }
        if (soft_i2c_sda_high()) {
            received |= bit_mask;
        }
        soft_i2c_scl_low();
        soft_i2c_delay();
    }

    /* This is the only byte, so leave SDA released to send NACK. */
    soft_i2c_sda_release();
    soft_i2c_delay();
    if (!soft_i2c_raise_scl(deadline, timed_out)) {
        return false;
    }
    soft_i2c_scl_low();
    soft_i2c_delay();
    *value = received;
    return true;
}

static void soft_i2c_recover_after_failure(void)
{
    i2c_deadline_t deadline = i2c_deadline_start();
    bool timed_out = false;

    /* Nine released clocks give a partially transferred slave a way to reset. */
    soft_i2c_sda_release();
    for (uint8_t pulse = 0u; pulse < 9u; ++pulse) {
        soft_i2c_scl_low();
        soft_i2c_delay();
        if (!soft_i2c_raise_scl(&deadline, &timed_out)) {
            break;
        }
    }
    if (!timed_out) {
        (void)soft_i2c_stop(&deadline, &timed_out);
    }
    /* Never leave an output enabled on either external sensor line. */
    soft_i2c_release_bus();
}

static void line_uart_drain_rx(void)
{
    while (!DL_UART_Main_isRXFIFOEmpty(UART_LINE_SENSOR_INST)) {
        (void)DL_UART_Main_receiveData(UART_LINE_SENSOR_INST);
    }
}

static bool line_uart_try_transmit(uint8_t byte)
{
    if (DL_UART_Main_isTXFIFOFull(UART_LINE_SENSOR_INST)) {
        return false;
    }
    DL_UART_Main_transmitData(UART_LINE_SENSOR_INST, byte);
    return true;
}

bool h2026_bsp_init(void)
{
    SYSCFG_DL_init();
    soft_i2c_release_bus();
    line_uart_drain_rx();

    s_pwm_motor_period = DL_TimerA_getLoadValue(PWM_MOTOR_INST) + 1u;
    s_motor_armed = false;
    motor_coast_unconditionally();

    s_encoder_left = 0;
    s_encoder_right = 0;
    s_encoder_left_invalid = 0u;
    s_encoder_right_invalid = 0u;
    s_encoder_left_phase = read_left_phase();
    s_encoder_right_phase = read_right_phase();

    s_control_tick_pending = false;
    s_control_ticks = 0u;
    s_control_tick_overruns = 0u;
    s_display_refresh_pending = false;
    s_display_tick_divider = 0u;
    s_display_counter = 0u;
    s_i2c_transactions = 0u;
    s_i2c_timeouts = 0u;
    s_i2c_bus_errors = 0u;
    s_line_uart_request_pending = false;
    s_line_uart_have_state = false;
    s_line_uart_state = 0u;
    s_line_uart_request_tick = 0u;
    s_line_uart_last_state_tick = 0u;
    s_line_uart_requests = 0u;
    s_line_uart_responses = 0u;
    s_line_uart_timeouts = 0u;
    /* HiWonder command 0 selects manual request/response state mode. */
    (void)line_uart_try_transmit(0u);

    DL_GPIO_clearInterruptStatus(
        GPIO_ENCODER_LEFT_PORT,
        GPIO_ENCODER_LEFT_LEFT_A_PIN | GPIO_ENCODER_LEFT_LEFT_B_PIN);
    DL_GPIO_clearInterruptStatus(
        GPIO_ENCODER_RIGHT_PORT,
        GPIO_ENCODER_RIGHT_RIGHT_A_PIN | GPIO_ENCODER_RIGHT_RIGHT_B_PIN);
    NVIC_ClearPendingIRQ(GPIO_ENCODER_LEFT_INT_IRQN);
    NVIC_ClearPendingIRQ(GPIO_ENCODER_RIGHT_INT_IRQN);
    NVIC_SetPriority(GPIO_ENCODER_LEFT_INT_IRQN, 1u);
    NVIC_SetPriority(GPIO_ENCODER_RIGHT_INT_IRQN, 1u);
    NVIC_EnableIRQ(GPIO_ENCODER_LEFT_INT_IRQN);
    NVIC_EnableIRQ(GPIO_ENCODER_RIGHT_INT_IRQN);

    NVIC_ClearPendingIRQ(TIMER_CONTROL_INST_INT_IRQN);
    NVIC_SetPriority(TIMER_CONTROL_INST_INT_IRQN, 0u);
    NVIC_EnableIRQ(TIMER_CONTROL_INST_INT_IRQN);

    DL_TimerA_startCounter(PWM_MOTOR_INST);
    DL_TimerG_startCounter(TIMER_CONTROL_INST);

    return (s_pwm_motor_period == 8000u) &&
           (TIMER_CONTROL_INST_LOAD_VALUE == 624u);
}

bool h2026_bsp_take_control_tick(uint32_t *overrun_count)
{
    bool pending;
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    pending = s_control_tick_pending;
    s_control_tick_pending = false;
    if (overrun_count != NULL) {
        *overrun_count = s_control_tick_overruns;
    }
    __set_PRIMASK(primask);
    return pending;
}

bool h2026_bsp_display_refresh_pending(void)
{
    bool pending;
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    pending = s_display_refresh_pending;
    __set_PRIMASK(primask);
    return pending;
}

void h2026_bsp_display_refresh_complete(void)
{
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    s_display_refresh_pending = false;
    __set_PRIMASK(primask);
}

uint32_t h2026_bsp_display_counter(void)
{
    uint32_t counter;
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    counter = s_display_counter;
    __set_PRIMASK(primask);
    return counter;
}

uint32_t h2026_bsp_millis(void)
{
    return s_control_ticks * H2026_BSP_CONTROL_PERIOD_MS;
}

void h2026_bsp_motor_arm(bool armed)
{
    if (!armed) {
        motor_coast_unconditionally();
    }
    s_motor_armed = armed;
}

bool h2026_bsp_motor_is_armed(void)
{
    return s_motor_armed;
}

void h2026_bsp_motor_set_signed(float left_duty, float right_duty)
{
    uint32_t control_tick;

    if (!s_motor_armed) {
        motor_coast_unconditionally();
        return;
    }
    /*
     * Use one scheduler generation for both wheels even if TIMG7 preempts
     * between their compare-register updates.
     */
    control_tick = s_control_ticks;
    set_signed_one_left(left_duty, control_tick);
    set_signed_one_right(right_duty, control_tick);
}

void h2026_bsp_motor_coast(void)
{
    motor_coast_unconditionally();
}

void h2026_bsp_motor_brake(void)
{
    if (!s_motor_armed) {
        motor_coast_unconditionally();
        return;
    }
    set_left_direction(true, true);
    set_right_direction(true, true);
    set_left_pwm(1.0f);
    set_right_pwm(1.0f);
    reset_both_direction_guards();
}

void h2026_bsp_encoder_snapshot(h2026_bsp_encoder_snapshot_t *snapshot)
{
    uint32_t primask;
    if (snapshot == NULL) {
        return;
    }

    primask = __get_PRIMASK();
    __disable_irq();
    snapshot->left_count = s_encoder_left;
    snapshot->right_count = s_encoder_right;
    snapshot->left_invalid_transitions = s_encoder_left_invalid;
    snapshot->right_invalid_transitions = s_encoder_right_invalid;
    __set_PRIMASK(primask);
}

bool h2026_bsp_line_read_reg5(uint8_t *state)
{
    i2c_deadline_t deadline;
    bool timed_out = false;

    if (state == NULL) {
        return false;
    }
    ++s_i2c_transactions;
    deadline = i2c_deadline_start();

    /*
     * HiWonder's reference implementation ends the register-select write
     * before issuing the one-byte read.  Although a repeated START is legal
     * I2C, this module's firmware is verified with STOP + new START; using
     * that exact sequence also prevents it from holding SCL after a transfer.
     *
     * START, 0xBA, register 5, STOP, START, 0xBB, one byte+NACK, STOP.
     */
    if (!soft_i2c_start(&deadline, &timed_out) ||
        !soft_i2c_write_byte(
            (uint8_t)(H2026_BSP_LINE_I2C_ADDRESS_7BIT << 1u),
            &deadline,
            &timed_out) ||
        !soft_i2c_write_byte(
            H2026_BSP_LINE_STATE_REGISTER, &deadline, &timed_out) ||
        !soft_i2c_stop(&deadline, &timed_out) ||
        !soft_i2c_start(&deadline, &timed_out) ||
        !soft_i2c_write_byte(
            (uint8_t)((H2026_BSP_LINE_I2C_ADDRESS_7BIT << 1u) | 1u),
            &deadline,
            &timed_out) ||
        !soft_i2c_read_byte_nack(state, &deadline, &timed_out) ||
        !soft_i2c_stop(&deadline, &timed_out)) {
        goto fail;
    }
    return true;

fail:
    if (timed_out) {
        ++s_i2c_timeouts;
    } else {
        ++s_i2c_bus_errors;
    }
    soft_i2c_recover_after_failure();
    return false;
}

bool h2026_bsp_line_uart_read_state(uint8_t *state)
{
    bool received = false;
    const uint32_t now = s_control_ticks;

    if (state == NULL) {
        return false;
    }

    /* Manual-state protocol returns precisely one byte after command 1. */
    while (!DL_UART_Main_isRXFIFOEmpty(UART_LINE_SENSOR_INST)) {
        s_line_uart_state =
            (uint8_t)DL_UART_Main_receiveData(UART_LINE_SENSOR_INST);
        received = true;
    }
    if (received) {
        s_line_uart_have_state = true;
        s_line_uart_last_state_tick = now;
        s_line_uart_request_pending = false;
        ++s_line_uart_responses;
    }

    if (s_line_uart_request_pending &&
        ((uint32_t)(now - s_line_uart_request_tick) >=
         H2026_LINE_UART_RESPONSE_TIMEOUT_TICKS)) {
        s_line_uart_request_pending = false;
        ++s_line_uart_timeouts;
    }

    if (!s_line_uart_request_pending && line_uart_try_transmit(1u)) {
        s_line_uart_request_pending = true;
        s_line_uart_request_tick = now;
        ++s_line_uart_requests;
    }

    if (s_line_uart_have_state &&
        ((uint32_t)(now - s_line_uart_last_state_tick) <=
         H2026_LINE_UART_STALE_TICKS)) {
        *state = s_line_uart_state;
        return true;
    }
    return false;
}

bool h2026_bsp_start_level(void)
{
    return (DL_GPIO_readPins(GPIO_UI_START_PORT, GPIO_UI_START_PIN) != 0u);
}

void h2026_bsp_led_set(bool on)
{
    if (on) {
        DL_GPIO_setPins(GPIO_UI_STATUS_LED_PORT, GPIO_UI_STATUS_LED_PIN);
    } else {
        DL_GPIO_clearPins(GPIO_UI_STATUS_LED_PORT, GPIO_UI_STATUS_LED_PIN);
    }
}

static void oled_shift_byte(uint8_t value)
{
    for (uint8_t bit = 0u; bit < 8u; ++bit) {
        DL_GPIO_clearPins(GPIO_OLED_SCL_PORT, GPIO_OLED_SCL_PIN);
        if ((value & 0x80u) != 0u) {
            DL_GPIO_setPins(GPIO_OLED_SDA_PORT, GPIO_OLED_SDA_PIN);
        } else {
            DL_GPIO_clearPins(GPIO_OLED_SDA_PORT, GPIO_OLED_SDA_PIN);
        }
        /* About 1 MHz four-wire serial clock: ample setup/hold margin. */
        delay_cycles(H2026_OLED_HALF_PERIOD_CYCLES);
        DL_GPIO_setPins(GPIO_OLED_SCL_PORT, GPIO_OLED_SCL_PIN);
        delay_cycles(H2026_OLED_HALF_PERIOD_CYCLES);
        value <<= 1u;
    }
    DL_GPIO_clearPins(GPIO_OLED_SCL_PORT, GPIO_OLED_SCL_PIN);
}

void h2026_bsp_oled_reset(void)
{
    DL_GPIO_clearPins(GPIO_OLED_RST_PORT, GPIO_OLED_RST_PIN);
    delay_cycles(H2026_OLED_RESET_LOW_CYCLES);
    DL_GPIO_setPins(GPIO_OLED_RST_PORT, GPIO_OLED_RST_PIN);
    delay_cycles(H2026_OLED_RESET_WAIT_CYCLES);
}

void h2026_bsp_oled_write_command(uint8_t command)
{
    DL_GPIO_clearPins(GPIO_OLED_DC_PORT, GPIO_OLED_DC_PIN);
    delay_cycles(H2026_OLED_HALF_PERIOD_CYCLES);
    oled_shift_byte(command);
}

void h2026_bsp_oled_write_data(const uint8_t *data, size_t length)
{
    if (data == NULL) {
        return;
    }
    DL_GPIO_setPins(GPIO_OLED_DC_PORT, GPIO_OLED_DC_PIN);
    delay_cycles(H2026_OLED_HALF_PERIOD_CYCLES);
    for (size_t index = 0u; index < length; ++index) {
        oled_shift_byte(data[index]);
    }
}

void h2026_bsp_uart0_write(const uint8_t *data, size_t length)
{
    if (data == NULL) {
        return;
    }
    for (size_t index = 0u; index < length; ++index) {
        DL_UART_Main_transmitDataBlocking(UART_DEBUG_INST, data[index]);
    }
}

size_t h2026_bsp_uart0_try_write(const uint8_t *data, size_t length)
{
    size_t sent = 0U;

    if (data == NULL) {
        return 0U;
    }
    while ((sent < length) && !DL_UART_Main_isTXFIFOFull(UART_DEBUG_INST)) {
        DL_UART_Main_transmitData(UART_DEBUG_INST, data[sent]);
        ++sent;
    }
    return sent;
}

void h2026_bsp_diagnostics_snapshot(h2026_bsp_diagnostics_t *diagnostics)
{
    uint32_t primask;
    if (diagnostics == NULL) {
        return;
    }
    primask = __get_PRIMASK();
    __disable_irq();
    diagnostics->i2c_transactions = s_i2c_transactions;
    diagnostics->i2c_timeouts = s_i2c_timeouts;
    diagnostics->i2c_bus_errors = s_i2c_bus_errors;
    diagnostics->line_uart_requests = s_line_uart_requests;
    diagnostics->line_uart_responses = s_line_uart_responses;
    diagnostics->line_uart_timeouts = s_line_uart_timeouts;
    diagnostics->control_tick_overruns = s_control_tick_overruns;
    __set_PRIMASK(primask);
}

void GROUP1_IRQHandler(void)
{
    uint32_t left_status = DL_GPIO_getEnabledInterruptStatus(
        GPIO_ENCODER_LEFT_PORT,
        GPIO_ENCODER_LEFT_LEFT_A_PIN | GPIO_ENCODER_LEFT_LEFT_B_PIN);
    uint32_t right_status = DL_GPIO_getEnabledInterruptStatus(
        GPIO_ENCODER_RIGHT_PORT,
        GPIO_ENCODER_RIGHT_RIGHT_A_PIN | GPIO_ENCODER_RIGHT_RIGHT_B_PIN);

    if (left_status != 0u) {
        DL_GPIO_clearInterruptStatus(GPIO_ENCODER_LEFT_PORT, left_status);
        update_left_encoder();
    }
    if (right_status != 0u) {
        DL_GPIO_clearInterruptStatus(GPIO_ENCODER_RIGHT_PORT, right_status);
        update_right_encoder();
    }
}

void TIMER_CONTROL_INST_IRQHandler(void)
{
    switch (DL_TimerG_getPendingInterrupt(TIMER_CONTROL_INST)) {
        case DL_TIMER_IIDX_ZERO:
            ++s_control_ticks;
            ++s_display_tick_divider;
            if (s_display_tick_divider >= H2026_DISPLAY_PERIOD_TICKS) {
                s_display_tick_divider = 0u;
                ++s_display_counter;
                s_display_refresh_pending = true;
            }
            if (s_control_tick_pending) {
                ++s_control_tick_overruns;
            } else {
                s_control_tick_pending = true;
            }
            break;
        default:
            break;
    }
}
