#include "h2026_bsp.h"

#include "ti_msp_dl_config.h"

#include <string.h>

#define H2026_LINE_SETTLE_US               100u
#define H2026_LINE_SETTLE_CYCLES ((CPUCLK_FREQ / 1000000u) * H2026_LINE_SETTLE_US)
#define H2026_LINE_ADC_AVERAGES              4u
#define H2026_LINE_ADC_SPIN_GUARD        20000u
#define H2026_LINE_SCAN_LIMIT_US          1200u
#define H2026_LINE_US_PER_TIMER_COUNT         8u
#define H2026_OLED_I2C_ADDRESS             0x3Cu
#define H2026_OLED_I2C_MAX_PAYLOAD           16u
#define H2026_OLED_I2C_SPIN_GUARD        20000u
#define H2026_DISPLAY_PERIOD_TICKS \
    (H2026_BSP_DISPLAY_PERIOD_MS / H2026_BSP_CONTROL_PERIOD_MS)
typedef struct {
    int8_t applied_sign;
    int8_t pending_sign;
    uint32_t coast_started_tick;
} motor_direction_guard_t;

static volatile int64_t s_encoder_left;
static volatile int64_t s_encoder_right;
static volatile uint32_t s_encoder_left_invalid;
static volatile uint32_t s_encoder_right_invalid;
static volatile uint32_t s_encoder_left_events;
static volatile uint32_t s_encoder_right_events;
static volatile uint8_t s_encoder_left_phase;
static volatile uint8_t s_encoder_right_phase;

static volatile bool s_control_tick_pending;
static volatile uint32_t s_control_ticks;
static volatile uint32_t s_control_tick_overruns;
static volatile bool s_display_refresh_pending;
static uint8_t s_display_tick_divider;
static volatile uint32_t s_display_counter;

static volatile uint32_t s_line_scan_count;
static volatile uint32_t s_line_scan_failures;
static volatile uint32_t s_line_scan_max_us;
static uint32_t s_line_sample_seq;

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
        return 0u;
    }
    if (duty >= 1.0f) {
        return period;
    }
    /*
     * TIMA1 is configured low at reset and high from the zero event until
     * compare.  Thus compare/period is the high-time duty.  The previous
     * complement inverted torque: the 10 %% bench test drove at about 90 %%
     * while a closed-loop 81 %% request drove at only about 19 %%.
     */
    return (uint32_t)(duty * (float)period + 0.5f);
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

static void line_mux_select(uint8_t channel)
{
    if ((channel & 0x04u) != 0u) {
        DL_GPIO_setPins(GPIO_LINE_MUX_LINE_AD2_PORT,
                        GPIO_LINE_MUX_LINE_AD2_PIN);
    } else {
        DL_GPIO_clearPins(GPIO_LINE_MUX_LINE_AD2_PORT,
                          GPIO_LINE_MUX_LINE_AD2_PIN);
    }
    if ((channel & 0x02u) != 0u) {
        DL_GPIO_setPins(GPIO_LINE_MUX_LINE_AD1_PORT,
                        GPIO_LINE_MUX_LINE_AD1_PIN);
    } else {
        DL_GPIO_clearPins(GPIO_LINE_MUX_LINE_AD1_PORT,
                          GPIO_LINE_MUX_LINE_AD1_PIN);
    }
    if ((channel & 0x01u) != 0u) {
        DL_GPIO_setPins(GPIO_LINE_MUX_LINE_AD0_PORT,
                        GPIO_LINE_MUX_LINE_AD0_PIN);
    } else {
        DL_GPIO_clearPins(GPIO_LINE_MUX_LINE_AD0_PORT,
                          GPIO_LINE_MUX_LINE_AD0_PIN);
    }
}

static bool line_adc_convert(uint16_t *value)
{
    uint32_t guard = H2026_LINE_ADC_SPIN_GUARD;

    if (value == NULL) {
        return false;
    }
    DL_ADC12_clearInterruptStatus(
        ADC_LINE_SENSOR_INST, DL_ADC12_INTERRUPT_MEM0_RESULT_LOADED);
    DL_ADC12_startConversion(ADC_LINE_SENSOR_INST);
    while ((DL_ADC12_getRawInterruptStatus(
                ADC_LINE_SENSOR_INST,
                DL_ADC12_INTERRUPT_MEM0_RESULT_LOADED) == 0u) &&
           (guard > 0u)) {
        --guard;
    }
    if (guard == 0u) {
        DL_ADC12_stopConversion(ADC_LINE_SENSOR_INST);
        return false;
    }
    *value = DL_ADC12_getMemResult(
        ADC_LINE_SENSOR_INST, DL_ADC12_MEM_IDX_0);
    DL_ADC12_enableConversions(ADC_LINE_SENSOR_INST);
    return true;
}

static uint16_t line_elapsed_us(uint32_t start_tick, uint32_t start_count)
{
    const uint32_t end_tick = s_control_ticks;
    const uint32_t end_count = DL_TimerG_getTimerCount(TIMER_CONTROL_INST);
    uint32_t elapsed_counts =
        (end_tick - start_tick) * (TIMER_CONTROL_INST_LOAD_VALUE + 1u);

    if (start_count >= end_count) {
        elapsed_counts += start_count - end_count;
    } else {
        elapsed_counts += start_count +
            (TIMER_CONTROL_INST_LOAD_VALUE + 1u) - end_count;
    }
    return (uint16_t)(elapsed_counts * H2026_LINE_US_PER_TIMER_COUNT);
}

bool h2026_bsp_init(void)
{
    SYSCFG_DL_init();

    s_pwm_motor_period = DL_TimerA_getLoadValue(PWM_MOTOR_INST) + 1u;
    s_motor_armed = false;
    motor_coast_unconditionally();

    s_encoder_left = 0;
    s_encoder_right = 0;
    s_encoder_left_invalid = 0u;
    s_encoder_right_invalid = 0u;
    s_encoder_left_events = 0u;
    s_encoder_right_events = 0u;
    s_encoder_left_phase = read_left_phase();
    s_encoder_right_phase = read_right_phase();

    s_control_tick_pending = false;
    s_control_ticks = 0u;
    s_control_tick_overruns = 0u;
    s_display_refresh_pending = false;
    s_display_tick_divider = 0u;
    s_display_counter = 0u;
    s_line_scan_count = 0u;
    s_line_scan_failures = 0u;
    s_line_scan_max_us = 0u;
    s_line_sample_seq = 0u;

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
    snapshot->left_edge_events = s_encoder_left_events;
    snapshot->right_edge_events = s_encoder_right_events;
    snapshot->left_phase = s_encoder_left_phase;
    snapshot->right_phase = s_encoder_right_phase;
    __set_PRIMASK(primask);
}

bool h2026_bsp_line_scan(h2026_bsp_line_sample_t *sample)
{
    uint8_t channel;
    uint32_t start_tick;
    uint32_t start_count;

    if (sample == NULL) {
        return false;
    }
    memset(sample, 0, sizeof(*sample));
    start_tick = s_control_ticks;
    start_count = DL_TimerG_getTimerCount(TIMER_CONTROL_INST);

    for (channel = 0u; channel < H2026_BSP_LINE_SENSOR_COUNT; ++channel) {
        uint32_t accumulator = 0u;
        uint8_t conversion;
        uint16_t discarded;

        line_mux_select(channel);
        delay_cycles(H2026_LINE_SETTLE_CYCLES);
        if (!line_adc_convert(&discarded)) {
            goto fail;
        }
        for (conversion = 0u; conversion < H2026_LINE_ADC_AVERAGES;
             ++conversion) {
            uint16_t value;
            if (!line_adc_convert(&value)) {
                goto fail;
            }
            accumulator += value;
        }
        sample->raw_adc[channel] = (uint16_t)(
            (accumulator + (H2026_LINE_ADC_AVERAGES / 2u)) /
            H2026_LINE_ADC_AVERAGES);
    }

    sample->scan_us = line_elapsed_us(start_tick, start_count);
    if (sample->scan_us > H2026_LINE_SCAN_LIMIT_US) {
        goto fail;
    }
    ++s_line_scan_count;
    if (sample->scan_us > s_line_scan_max_us) {
        s_line_scan_max_us = sample->scan_us;
    }
    ++s_line_sample_seq;
    sample->sample_seq = s_line_sample_seq;
    sample->valid = true;
    return true;

fail:
    DL_ADC12_enableConversions(ADC_LINE_SENSOR_INST);
    ++s_line_scan_failures;
    sample->scan_us = line_elapsed_us(start_tick, start_count);
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

static bool oled_i2c_write(uint8_t control, const uint8_t *data, size_t length)
{
    uint8_t tx[H2026_OLED_I2C_MAX_PAYLOAD + 1u];
    uint32_t guard;
    uint16_t sent;
    uint16_t total;

    if ((data == NULL) || (length == 0u) ||
        (length > H2026_OLED_I2C_MAX_PAYLOAD)) {
        return false;
    }
    tx[0] = control;
    memcpy(&tx[1], data, length);
    total = (uint16_t)(length + 1u);

    guard = H2026_OLED_I2C_SPIN_GUARD;
    while (((DL_I2C_getControllerStatus(I2C_OLED_INST) &
             DL_I2C_CONTROLLER_STATUS_IDLE) == 0u) && (--guard != 0u)) {
    }
    if (guard == 0u) {
        DL_I2C_flushControllerTXFIFO(I2C_OLED_INST);
        return false;
    }

    sent = DL_I2C_fillControllerTXFIFO(I2C_OLED_INST, tx, total);
    DL_I2C_startControllerTransfer(I2C_OLED_INST, H2026_OLED_I2C_ADDRESS,
        DL_I2C_CONTROLLER_DIRECTION_TX, total);
    guard = H2026_OLED_I2C_SPIN_GUARD;
    while ((sent < total) && (--guard != 0u)) {
        sent = (uint16_t)(sent + DL_I2C_fillControllerTXFIFO(
            I2C_OLED_INST, &tx[sent], (uint16_t)(total - sent)));
    }
    delay_cycles(24u);
    while (((DL_I2C_getControllerStatus(I2C_OLED_INST) &
             DL_I2C_CONTROLLER_STATUS_BUSY) != 0u) && (--guard != 0u)) {
    }
    if ((guard == 0u) || (sent != total) ||
        ((DL_I2C_getControllerStatus(I2C_OLED_INST) &
          DL_I2C_CONTROLLER_STATUS_ERROR) != 0u)) {
        DL_I2C_flushControllerTXFIFO(I2C_OLED_INST);
        return false;
    }
    return true;
}

void h2026_bsp_oled_write_command(uint8_t command)
{
    (void)oled_i2c_write(0x00u, &command, 1u);
}

void h2026_bsp_oled_write_data(const uint8_t *data, size_t length)
{
    while (length > 0u) {
        const size_t chunk = (length > H2026_OLED_I2C_MAX_PAYLOAD)
            ? H2026_OLED_I2C_MAX_PAYLOAD : length;

        if (!oled_i2c_write(0x40u, data, chunk)) {
            return;
        }
        data += chunk;
        length -= chunk;
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
    diagnostics->line_scan_count = s_line_scan_count;
    diagnostics->line_scan_failures = s_line_scan_failures;
    diagnostics->line_scan_max_us = s_line_scan_max_us;
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
        ++s_encoder_left_events;
        update_left_encoder();
    }
    if (right_status != 0u) {
        DL_GPIO_clearInterruptStatus(GPIO_ENCODER_RIGHT_PORT, right_status);
        ++s_encoder_right_events;
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
