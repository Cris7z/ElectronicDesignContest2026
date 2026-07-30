#include "h2026_q2_display.h"

#include "../bsp/h2026_bsp.h"

#include <stddef.h>
#include <string.h>

#ifndef H2026_MOTOR_TEST_AUTORUN
#define H2026_MOTOR_TEST_AUTORUN 0
#endif

#define OLED_WIDTH 128U
/* Vendor C07A/S27F driver uses an SSD1306-compatible 128x64, eight-page OLED. */
#define OLED_PAGES 8U
#define GLYPH_WIDTH 5U
#define CELL_WIDTH 6U

static uint8_t s_framebuffer[OLED_WIDTH * OLED_PAGES];
static uint8_t s_next_page;

static void flush_page(uint8_t page)
{
    const size_t offset = (size_t)page * OLED_WIDTH;

    h2026_bsp_oled_write_command((uint8_t)(0xB0U | page));
    h2026_bsp_oled_write_command(0x00U);
    h2026_bsp_oled_write_command(0x10U);
    h2026_bsp_oled_write_data(&s_framebuffer[offset], OLED_WIDTH);
}

static const uint8_t k_digits[10][GLYPH_WIDTH] = {
    {0x3EU, 0x51U, 0x49U, 0x45U, 0x3EU},
    {0x00U, 0x42U, 0x7FU, 0x40U, 0x00U},
    {0x42U, 0x61U, 0x51U, 0x49U, 0x46U},
    {0x21U, 0x41U, 0x45U, 0x4BU, 0x31U},
    {0x18U, 0x14U, 0x12U, 0x7FU, 0x10U},
    {0x27U, 0x45U, 0x45U, 0x45U, 0x39U},
    {0x3CU, 0x4AU, 0x49U, 0x49U, 0x30U},
    {0x01U, 0x71U, 0x09U, 0x05U, 0x03U},
    {0x36U, 0x49U, 0x49U, 0x49U, 0x36U},
    {0x06U, 0x49U, 0x49U, 0x29U, 0x1EU}
};

static const uint8_t k_letters[26][GLYPH_WIDTH] = {
    {0x7EU, 0x11U, 0x11U, 0x11U, 0x7EU},
    {0x7FU, 0x49U, 0x49U, 0x49U, 0x36U},
    {0x3EU, 0x41U, 0x41U, 0x41U, 0x22U},
    {0x7FU, 0x41U, 0x41U, 0x22U, 0x1CU},
    {0x7FU, 0x49U, 0x49U, 0x49U, 0x41U},
    {0x7FU, 0x09U, 0x09U, 0x09U, 0x01U},
    {0x3EU, 0x41U, 0x49U, 0x49U, 0x7AU},
    {0x7FU, 0x08U, 0x08U, 0x08U, 0x7FU},
    {0x00U, 0x41U, 0x7FU, 0x41U, 0x00U},
    {0x20U, 0x40U, 0x41U, 0x3FU, 0x01U},
    {0x7FU, 0x08U, 0x14U, 0x22U, 0x41U},
    {0x7FU, 0x40U, 0x40U, 0x40U, 0x40U},
    {0x7FU, 0x02U, 0x0CU, 0x02U, 0x7FU},
    {0x7FU, 0x04U, 0x08U, 0x10U, 0x7FU},
    {0x3EU, 0x41U, 0x41U, 0x41U, 0x3EU},
    {0x7FU, 0x09U, 0x09U, 0x09U, 0x06U},
    {0x3EU, 0x41U, 0x51U, 0x21U, 0x5EU},
    {0x7FU, 0x09U, 0x19U, 0x29U, 0x46U},
    {0x46U, 0x49U, 0x49U, 0x49U, 0x31U},
    {0x01U, 0x01U, 0x7FU, 0x01U, 0x01U},
    {0x3FU, 0x40U, 0x40U, 0x40U, 0x3FU},
    {0x1FU, 0x20U, 0x40U, 0x20U, 0x1FU},
    {0x3FU, 0x40U, 0x38U, 0x40U, 0x3FU},
    {0x63U, 0x14U, 0x08U, 0x14U, 0x63U},
    {0x07U, 0x08U, 0x70U, 0x08U, 0x07U},
    {0x61U, 0x51U, 0x49U, 0x45U, 0x43U}
};

static const uint8_t *glyph_for(char character)
{
    static const uint8_t blank[GLYPH_WIDTH] =
        {0U, 0U, 0U, 0U, 0U};
    static const uint8_t dash[GLYPH_WIDTH] =
        {0x08U, 0x08U, 0x08U, 0x08U, 0x08U};
    static const uint8_t dot[GLYPH_WIDTH] =
        {0U, 0x60U, 0x60U, 0U, 0U};
    static const uint8_t colon[GLYPH_WIDTH] =
        {0U, 0x36U, 0x36U, 0U, 0U};
    static const uint8_t plus[GLYPH_WIDTH] =
        {0x08U, 0x08U, 0x3EU, 0x08U, 0x08U};
    static const uint8_t slash[GLYPH_WIDTH] =
        {0x20U, 0x10U, 0x08U, 0x04U, 0x02U};
    static const uint8_t question[GLYPH_WIDTH] =
        {0x02U, 0x01U, 0x51U, 0x09U, 0x06U};

    if ((character >= '0') && (character <= '9')) {
        return k_digits[(unsigned int)(character - '0')];
    }
    if ((character >= 'A') && (character <= 'Z')) {
        return k_letters[(unsigned int)(character - 'A')];
    }
    switch (character) {
    case ' ':
        return blank;
    case '-':
        return dash;
    case '.':
        return dot;
    case ':':
        return colon;
    case '+':
        return plus;
    case '/':
        return slash;
    default:
        return question;
    }
}

static void draw_character(uint8_t page, uint8_t column, char character)
{
    const uint8_t *glyph;
    size_t offset;
    size_t index;

    if ((page >= OLED_PAGES) ||
        ((uint16_t)column + CELL_WIDTH > OLED_WIDTH)) {
        return;
    }
    glyph = glyph_for(character);
    offset = ((size_t)page * OLED_WIDTH) + column;
    for (index = 0U; index < GLYPH_WIDTH; ++index) {
        s_framebuffer[offset + index] = glyph[index];
    }
    s_framebuffer[offset + GLYPH_WIDTH] = 0U;
}

static uint8_t draw_text(uint8_t page, uint8_t column, const char *text)
{
    if (text == NULL) {
        return column;
    }
    while ((*text != '\0') &&
           ((uint16_t)column + CELL_WIDTH <= OLED_WIDTH)) {
        draw_character(page, column, *text);
        column = (uint8_t)(column + CELL_WIDTH);
        ++text;
    }
    return column;
}

static uint8_t draw_u32(uint8_t page,
                        uint8_t column,
                        uint32_t value,
                        uint8_t minimum_digits)
{
    char digits[10];
    uint8_t count = 0U;

    do {
        digits[count] = (char)('0' + (value % 10U));
        value /= 10U;
        ++count;
    } while ((value != 0U) && (count < sizeof(digits)));
    while ((count < minimum_digits) && (count < sizeof(digits))) {
        digits[count] = '0';
        ++count;
    }
    while (count > 0U) {
        --count;
        draw_character(page, column, digits[count]);
        column = (uint8_t)(column + CELL_WIDTH);
    }
    return column;
}

static uint8_t draw_i32(uint8_t page,
                        uint8_t column,
                        int32_t value,
                        uint8_t minimum_digits)
{
    uint32_t magnitude;

    if (value < 0) {
        draw_character(page, column, '-');
        column = (uint8_t)(column + CELL_WIDTH);
        magnitude = (uint32_t)(-(value + 1)) + 1U;
    } else {
        draw_character(page, column, '+');
        column = (uint8_t)(column + CELL_WIDTH);
        magnitude = (uint32_t)value;
    }
    return draw_u32(page, column, magnitude, minimum_digits);
}

static uint8_t draw_hex(uint8_t page,
                        uint8_t column,
                        uint32_t value,
                        uint8_t digits)
{
    while (digits > 0U) {
        const uint8_t shift = (uint8_t)((digits - 1U) * 4U);
        const uint8_t nibble = (uint8_t)((value >> shift) & 0x0FU);
        const char character =
            (nibble < 10U)
                ? (char)('0' + nibble)
                : (char)('A' + (nibble - 10U));
        draw_character(page, column, character);
        column = (uint8_t)(column + CELL_WIDTH);
        --digits;
    }
    return column;
}

static uint8_t draw_binary8(uint8_t page, uint8_t column, uint8_t value)
{
    uint8_t bit;
    for (bit = 0U; bit < 8U; ++bit) {
        const uint8_t mask = (uint8_t)(0x80U >> bit);
        draw_character(page, column, (value & mask) ? '1' : '0');
        column = (uint8_t)(column + CELL_WIDTH);
    }
    return column;
}

static const char *short_state_name(h2026_q2_state_t state)
{
    switch (state) {
    case H2026_Q2_STATE_IDLE:
        return "IDLE";
    case H2026_Q2_STATE_START_ACQUIRE_LINE:
        return "START";
    case H2026_Q2_STATE_SEEK_START_MARKER:
        return "SEEK";
    case H2026_Q2_STATE_CLEAR_START:
        return "CLEAR";
    case H2026_Q2_STATE_LAP:
        return "LAP";
    case H2026_Q2_STATE_FINISH_ARMED:
        return "FINISH";
    case H2026_Q2_STATE_STOPPING:
        return "STOP";
    case H2026_Q2_STATE_HOLD:
        return "HOLD";
    case H2026_Q2_STATE_FAULT:
        return "FAULT";
    default:
        return "UNKNOWN";
    }
}

static int32_t scaled_i32(float value, float scale)
{
    const float scaled = value * scale;
    if (scaled >= 0.0f) {
        return (int32_t)(scaled + 0.5f);
    }
    return (int32_t)(scaled - 0.5f);
}

void h2026_q2_display_init(void)
{
    static const uint8_t init_commands[] = {
        0xAEU, 0xD5U, 0x50U, 0xA8U, 0x3FU, 0xD3U, 0x00U,
        0x40U, 0x8DU, 0x14U, 0x20U, 0x02U, 0xA1U, 0xC8U,
        0xDAU, 0x12U, 0x81U, 0xEFU, 0xD9U, 0xF1U, 0xDBU,
        0x30U, 0xA4U, 0xA6U, 0xAFU
    };
    size_t index;

    for (index = 0U; index < sizeof(init_commands); ++index) {
        h2026_bsp_oled_write_command(init_commands[index]);
    }
    s_next_page = 0U;
}

static const char *calibration_state_name(uint8_t calibration_state)
{
    switch (calibration_state) {
    case 0U: return "WAIT";
    case 1U: return "WHITE";
    case 2U: return "SWEEP";
    case 3U: return "SAVED";
    case 4U: return "FAIL";
    default: return "UNK";
    }
}

void h2026_q2_display_render(const h2026_q2_output_t *output,
                             uint8_t line_bits,
                             bool line_valid,
                             uint32_t calibration_locks,
                             uint8_t calibration_state,
                             uint8_t calibration_failure,
                             const uint16_t raw_adc[H2026_Q2_LINE_SENSOR_COUNT],
                             uint32_t app_fault_code)
{
    uint8_t column;
    uint32_t elapsed_ms = 0U;
    uint32_t distance_mm = 0U;
    uint32_t seconds;
    uint32_t milliseconds;
    int32_t line_error_milli = 0;
    int32_t speed_mm_s = 0;
    h2026_bsp_diagnostics_t bsp_diagnostics;
    h2026_q2_state_t state = H2026_Q2_STATE_IDLE;
    h2026_q2_fault_t fault = H2026_Q2_FAULT_NONE;

    if (output != NULL) {
        elapsed_ms = output->elapsed_ms;
        if (output->distance_m > 0.0f) {
            distance_mm = (uint32_t)scaled_i32(output->distance_m, 1000.0f);
        }
        line_error_milli =
            scaled_i32(output->diagnostics.filtered_line_error, 1000.0f);
        speed_mm_s =
            scaled_i32(output->center_speed_command_mps, 1000.0f);
        state = output->state;
        fault = output->fault;
    }
    h2026_bsp_diagnostics_snapshot(&bsp_diagnostics);

    memset(s_framebuffer, 0, sizeof(s_framebuffer));
    column = draw_text(0U, 0U, "COUNT ");
    (void)draw_u32(0U, column, h2026_bsp_display_counter(), 8U);

    if (calibration_locks != 0U) {
        column = draw_text(1U, 0U, "CAL ");
        column = draw_text(1U, column, calibration_state_name(calibration_state));
        if (calibration_state == 4U) {
            column = draw_text(1U, column, " E");
            column = draw_u32(1U, column, calibration_failure, 1U);
        }
        column = draw_text(1U, column, " L");
        (void)draw_hex(1U, column, calibration_locks, 3U);

        for (uint8_t pair = 0U; pair < 4U; ++pair) {
            const uint8_t first = (uint8_t)(pair * 2U);
            const uint16_t first_raw = (raw_adc != NULL) ? raw_adc[first] : 0U;
            const uint16_t second_raw = (raw_adc != NULL) ? raw_adc[first + 1U] : 0U;

            column = draw_text((uint8_t)(pair + 2U), 0U, "R");
            column = draw_u32((uint8_t)(pair + 2U), column, first, 1U);
            column = draw_text((uint8_t)(pair + 2U), column, " ");
            column = draw_u32((uint8_t)(pair + 2U), column, first_raw, 4U);
            column = draw_text((uint8_t)(pair + 2U), column, " R");
            column = draw_u32((uint8_t)(pair + 2U), column, first + 1U, 1U);
            column = draw_text((uint8_t)(pair + 2U), column, " ");
            (void)draw_u32((uint8_t)(pair + 2U), column, second_raw, 4U);
        }

        column = draw_text(6U, 0U, "BITS ");
        column = draw_binary8(6U, column, line_bits);
        (void)draw_text(6U, column, line_valid ? " OK" : " BAD");
        column = draw_text(7U, 0U, "SCAN ");
        column = draw_u32(7U, column,
                          bsp_diagnostics.line_scan_count % 1000U, 3U);
        column = draw_text(7U, column, "/");
        column = draw_u32(7U, column,
                          bsp_diagnostics.line_scan_failures % 1000U, 3U);
        column = draw_text(7U, column, "/");
        (void)draw_u32(7U, column,
                       bsp_diagnostics.line_scan_max_us % 1000U, 3U);
        return;
    }

    column = draw_text(1U, 0U, "STATE ");
    (void)draw_text(1U, column, short_state_name(state));

    seconds = elapsed_ms / 1000U;
    milliseconds = elapsed_ms % 1000U;
    column = draw_text(2U, 0U, "T ");
    column = draw_u32(2U, column, seconds / 60U, 2U);
    column = draw_text(2U, column, ":");
    column = draw_u32(2U, column, seconds % 60U, 2U);
    column = draw_text(2U, column, ".");
    (void)draw_u32(2U, column, milliseconds, 3U);

    column = draw_text(3U, 0U, "D ");
    column = draw_u32(3U, column, distance_mm, 5U);
    (void)draw_text(3U, column, " MM");

    column = draw_text(4U, 0U, "BITS ");
    column = draw_binary8(4U, column, line_bits);
    (void)draw_text(4U, column, line_valid ? " OK" : " BAD");

    column = draw_text(5U, 0U, "E ");
    column = draw_i32(5U, column, line_error_milli, 3U);
    column = draw_text(5U, column, " V ");
    (void)draw_i32(5U, column, speed_mm_s, 3U);

    column = draw_text(6U, 0U, "SCAN ");
    column = draw_u32(6U, column,
                      bsp_diagnostics.line_scan_count % 1000U, 3U);
    column = draw_text(6U, column, "/");
    column = draw_u32(6U, column,
                      bsp_diagnostics.line_scan_failures % 1000U, 3U);
    column = draw_text(6U, column, "/");
    (void)draw_u32(6U, column,
                   bsp_diagnostics.line_scan_max_us % 1000U, 3U);

    column = draw_text(7U, 0U, "F ");
    column = draw_u32(7U, column, (uint32_t)fault, 2U);
    column = draw_text(7U, column, " A ");
    (void)draw_u32(7U, column, app_fault_code, 2U);
}

bool h2026_q2_display_flush_one_page(void)
{
    const bool frame_complete = (s_next_page == (OLED_PAGES - 1U));

    flush_page(s_next_page);
    s_next_page = (uint8_t)((s_next_page + 1U) % OLED_PAGES);
    return frame_complete;
}

static const char *motor_test_stage_name(uint8_t stage)
{
    switch (stage) {
    case 0U:
        return "QUIET";
    case 1U:
        return "TAP LEFT";
    case 2U:
        return "LEFT RUN";
    case 3U:
        return "TAP RIGHT";
    case 4U:
        return "RIGHT RUN";
    case 5U:
        return "DONE COAST";
    default:
        return "UNKNOWN";
    }
}

static int32_t clamp_i64_to_i32(int64_t value)
{
    if (value > INT32_MAX) {
        return INT32_MAX;
    }
    if (value < INT32_MIN) {
        return INT32_MIN;
    }
    return (int32_t)value;
}

void h2026_q2_display_render_motor_commission(uint8_t stage,
                                               int64_t left_delta,
                                               int64_t left_test_right_delta,
                                               int64_t right_test_left_delta,
                                               int64_t right_delta,
                                               uint32_t left_invalid,
                                               uint32_t right_invalid,
                                               uint32_t left_events,
                                               uint32_t right_events,
                                               uint32_t completed_tests)
{
    uint8_t column;

    memset(s_framebuffer, 0, sizeof(s_framebuffer));
    (void)draw_text(0U, 0U, "MOTOR CROSSMAP");
    column = draw_text(1U, 0U, "STATE ");
    (void)draw_text(1U, column, motor_test_stage_name(stage));
    column = draw_text(2U, 0U, "LT L ");
    (void)draw_i32(2U, column, clamp_i64_to_i32(left_delta), 5U);
    column = draw_text(3U, 0U, "LT R ");
    (void)draw_i32(3U, column,
                   clamp_i64_to_i32(left_test_right_delta), 5U);
    column = draw_text(4U, 0U, "RT L ");
    (void)draw_i32(4U, column,
                   clamp_i64_to_i32(right_test_left_delta), 5U);
    column = draw_text(5U, 0U, "RT R ");
    (void)draw_i32(5U, column, clamp_i64_to_i32(right_delta), 5U);
    column = draw_text(6U, 0U, "I L/R ");
    column = draw_u32(6U, column, left_invalid, 1U);
    column = draw_text(6U, column, "/");
    (void)draw_u32(6U, column, right_invalid, 1U);
    column = draw_text(7U, 0U, "TESTS ");
    (void)draw_u32(7U, column, completed_tests, 1U);
    /* Keep edge counts referenced for the low-level diagnostic build. */
    (void)left_events;
    (void)right_events;
}

void h2026_q2_display_render_distance_calibration(uint8_t stage,
                                                   int64_t left_count,
                                                   int64_t right_count,
                                                   uint32_t left_invalid,
                                                   uint32_t right_invalid)
{
    uint8_t column;
    const char *state = "READY TAP";

    if (stage == 2U) {
        state = "DRIVE 3S";
    } else if (stage == 3U) {
        state = "DONE";
    } else if (stage == 4U) {
        state = "ABORT";
    } else if (stage == 0U) {
        state = "WAIT";
    }
    memset(s_framebuffer, 0, sizeof(s_framebuffer));
    (void)draw_text(0U, 0U, "GROUND DIST CAL");
    column = draw_text(1U, 0U, "STATE ");
    (void)draw_text(1U, column, state);
    (void)draw_text(2U, 0U, "AXLE START 0mm");
    (void)draw_text(3U, 0U, "BLS: RUN 3 SEC");
    column = draw_text(4U, 0U, "L FWD ");
    (void)draw_i32(4U, column, clamp_i64_to_i32(left_count), 5U);
    column = draw_text(5U, 0U, "R FWD ");
    (void)draw_i32(5U, column, clamp_i64_to_i32(right_count), 5U);
    column = draw_text(6U, 0U, "INVALID ");
    column = draw_u32(6U, column, left_invalid, 1U);
    (void)draw_text(6U, column, "/");
    (void)draw_u32(6U, column, right_invalid, 1U);
    (void)draw_text(7U, 0U, "PRESS=ABORT");
}

void h2026_q2_display_render_encoder_passive(
    int64_t left_count, int64_t right_count,
    uint32_t left_invalid, uint32_t right_invalid,
    uint8_t left_phase, uint8_t right_phase)
{
    uint8_t column;

    memset(s_framebuffer, 0, sizeof(s_framebuffer));
    (void)draw_text(0U, 0U, "ENCODER PASSIVE");
    (void)draw_text(1U, 0U, "TURN WHEEL HAND");
    column = draw_text(2U, 0U, "L ");
    (void)draw_i32(2U, column, clamp_i64_to_i32(left_count), 5U);
    column = draw_text(3U, 0U, "R ");
    (void)draw_i32(3U, column, clamp_i64_to_i32(right_count), 5U);
    column = draw_text(4U, 0U, "LI ");
    (void)draw_u32(4U, column, left_invalid, 1U);
    column = draw_text(5U, 0U, "RI ");
    (void)draw_u32(5U, column, right_invalid, 1U);
    column = draw_text(6U, 0U, "PH L");
    column = draw_u32(6U, column, left_phase & 0x03U, 1U);
    column = draw_text(6U, column, " R");
    (void)draw_u32(6U, column, right_phase & 0x03U, 1U);
    (void)draw_text(7U, 0U, "NO MOTOR OUTPUT");
}

void h2026_q2_display_render_bls_passive(bool raw_level,
                                         uint32_t edge_count)
{
    uint8_t column;

    memset(s_framebuffer, 0, sizeof(s_framebuffer));
    (void)draw_text(0U, 0U, "BLS PA18 TEST");
    (void)draw_text(1U, 0U, "PRESS RELEASE");
    column = draw_text(3U, 0U, "RAW ");
    (void)draw_u32(3U, column, raw_level ? 1U : 0U, 1U);
    column = draw_text(4U, 0U, "EDGES ");
    (void)draw_u32(4U, column, edge_count, 1U);
    (void)draw_text(6U, 0U, "NO MOTOR OUTPUT");
    (void)draw_text(7U, 0U, "POWER OFF EXIT");
}
