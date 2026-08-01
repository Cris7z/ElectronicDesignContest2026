#include "line_tracker_display.h"

#include "h2026_bsp.h"

#include <stddef.h>
#include <string.h>

#define OLED_WIDTH 128U
#define OLED_PAGES 8U
#define GLYPH_WIDTH 5U
#define CELL_WIDTH 6U
#define OLED_ASYNC_DATA_BYTES 7U

static uint8_t s_framebuffer[OLED_WIDTH * OLED_PAGES];
static uint8_t s_next_page;
static uint8_t s_page_phase;
static uint8_t s_data_offset;
static bool s_job_active;

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

static const uint8_t *glyph_for(char character)
{
    static const uint8_t blank[GLYPH_WIDTH] = {0U, 0U, 0U, 0U, 0U};
    static const uint8_t colon[GLYPH_WIDTH] = {0U, 0x36U, 0x36U, 0U, 0U};
    static const uint8_t dot[GLYPH_WIDTH] = {0U, 0x60U, 0x60U, 0U, 0U};
    static const uint8_t a[GLYPH_WIDTH] = {0x7EU, 0x11U, 0x11U, 0x11U, 0x7EU};
    static const uint8_t b[GLYPH_WIDTH] = {0x7FU, 0x49U, 0x49U, 0x49U, 0x36U};
    static const uint8_t c[GLYPH_WIDTH] = {0x3EU, 0x41U, 0x41U, 0x41U, 0x22U};
    static const uint8_t d[GLYPH_WIDTH] = {0x7FU, 0x41U, 0x41U, 0x22U, 0x1CU};
    static const uint8_t e[GLYPH_WIDTH] = {0x7FU, 0x49U, 0x49U, 0x49U, 0x41U};
    static const uint8_t f[GLYPH_WIDTH] = {0x7FU, 0x09U, 0x09U, 0x09U, 0x01U};
    static const uint8_t h[GLYPH_WIDTH] = {0x7FU, 0x08U, 0x08U, 0x08U, 0x7FU};
    static const uint8_t i[GLYPH_WIDTH] = {0x00U, 0x41U, 0x7FU, 0x41U, 0x00U};
    static const uint8_t k[GLYPH_WIDTH] = {0x7FU, 0x08U, 0x14U, 0x22U, 0x41U};
    static const uint8_t l[GLYPH_WIDTH] = {0x7FU, 0x40U, 0x40U, 0x40U, 0x40U};
    static const uint8_t m[GLYPH_WIDTH] = {0x7FU, 0x02U, 0x0CU, 0x02U, 0x7FU};
    static const uint8_t n[GLYPH_WIDTH] = {0x7FU, 0x04U, 0x08U, 0x10U, 0x7FU};
    static const uint8_t o[GLYPH_WIDTH] = {0x3EU, 0x41U, 0x41U, 0x41U, 0x3EU};
    static const uint8_t p[GLYPH_WIDTH] = {0x7FU, 0x09U, 0x09U, 0x09U, 0x06U};
    static const uint8_t r[GLYPH_WIDTH] = {0x7FU, 0x09U, 0x19U, 0x29U, 0x46U};
    static const uint8_t s[GLYPH_WIDTH] = {0x46U, 0x49U, 0x49U, 0x49U, 0x31U};
    static const uint8_t t[GLYPH_WIDTH] = {0x01U, 0x01U, 0x7FU, 0x01U, 0x01U};
    static const uint8_t u[GLYPH_WIDTH] = {0x3FU, 0x40U, 0x40U, 0x40U, 0x3FU};
    static const uint8_t v[GLYPH_WIDTH] = {0x1FU, 0x20U, 0x40U, 0x20U, 0x1FU};
    static const uint8_t w[GLYPH_WIDTH] = {0x3FU, 0x40U, 0x38U, 0x40U, 0x3FU};
    static const uint8_t y[GLYPH_WIDTH] = {0x07U, 0x08U, 0x70U, 0x08U, 0x07U};

    if ((character >= '0') && (character <= '9')) {
        return k_digits[(unsigned int)(character - '0')];
    }
    switch (character) {
    case ' ': return blank;
    case ':': return colon;
    case '.': return dot;
    case 'A': return a;
    case 'B': return b;
    case 'C': return c;
    case 'D': return d;
    case 'E': return e;
    case 'F': return f;
    case 'H': return h;
    case 'I': return i;
    case 'K': return k;
    case 'L': return l;
    case 'M': return m;
    case 'N': return n;
    case 'O': return o;
    case 'P': return p;
    case 'R': return r;
    case 'S': return s;
    case 'T': return t;
    case 'U': return u;
    case 'V': return v;
    case 'W': return w;
    case 'Y': return y;
    default: return blank;
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

static uint8_t draw_u32(uint8_t page, uint8_t column, uint32_t value,
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

static uint8_t draw_binary8(uint8_t page, uint8_t column, uint8_t bits)
{
    uint8_t shift;

    for (shift = 0U; shift < 8U; ++shift) {
        draw_character(page, column,
                       ((bits & (uint8_t)(1U << shift)) != 0U)
                           ? '1' : '0');
        column = (uint8_t)(column + CELL_WIDTH);
    }
    return column;
}

static const char *state_name(line_tracker_state_t state)
{
    switch (state) {
    case LINE_TRACKER_RUN: return "RUN";
    case LINE_TRACKER_FAULT: return "FAULT";
    case LINE_TRACKER_WAIT:
    default: return "WAIT";
    }
}

static void render(line_tracker_state_t state, uint32_t elapsed_ms,
                   float distance_m, uint8_t mode)
{
    uint8_t column;
    const uint32_t seconds = elapsed_ms / 1000U;
    const uint32_t tenths = (elapsed_ms % 1000U) / 100U;
    const uint32_t distance_cm = (distance_m <= 0.0f) ? 0U :
        (uint32_t)((distance_m * 100.0f) + 0.5f);

    memset(s_framebuffer, 0, sizeof(s_framebuffer));
    column = draw_text(0U, 0U, "M ");
    (void)draw_u32(0U, column, mode, 1U);
    column = draw_text(2U, 0U, "TIME ");
    column = draw_u32(2U, column, seconds, 2U);
    column = draw_text(2U, column, ".");
    (void)draw_u32(2U, column, tenths, 1U);
    column = draw_text(4U, 0U, "D ");
    column = draw_u32(4U, column, distance_cm / 100U, 1U);
    column = draw_text(4U, column, ".");
    column = draw_u32(4U, column, distance_cm % 100U, 2U);
    (void)draw_text(4U, column, "M");
    (void)draw_text(6U, 0U, state_name(state));
}

static const char *calibration_state_name(uint8_t state)
{
    switch (state) {
    case 1U: return "WHITE";
    case 2U: return "BLACK";
    case 3U: return "SAVE";
    case 4U: return "SAVED";
    case 5U: return "FAIL";
    default: return "WAIT";
    }
}

static void render_calibration(
    const line_tracker_calibration_view_t *calibration)
{
    uint8_t pair;
    uint8_t column;
    bool ok;

    memset(s_framebuffer, 0, sizeof(s_framebuffer));
    column = draw_text(0U, 0U, "CAL ");
    column = draw_text(0U, column,
                       calibration_state_name(calibration->state));
    column = draw_text(0U, column, " ");
    (void)draw_u32(0U, column, calibration->progress_percent, 3U);
    for (pair = 0U; pair < 4U; ++pair) {
        const uint8_t first = (uint8_t)(pair * 2U);
        const uint8_t page = (uint8_t)(pair + 1U);

        column = draw_text(page, 0U, "R");
        column = draw_u32(page, column, first, 1U);
        column = draw_text(page, column, " ");
        column = draw_u32(page, column, calibration->raw_adc[first], 4U);
        column = draw_text(page, column, " R");
        column = draw_u32(page, column, first + 1U, 1U);
        column = draw_text(page, column, " ");
        (void)draw_u32(page, column, calibration->raw_adc[first + 1U], 4U);
    }
    column = draw_text(5U, 0U, "BITS ");
    (void)draw_binary8(5U, column, calibration->live_bits);
    column = draw_text(6U, 0U, "SPAN ");
    (void)draw_binary8(6U, column, calibration->ok_bits);
    ok = calibration->frame_valid &&
         ((calibration->state == 1U) ||
          ((calibration->ok_bits == 0xFFU) &&
           (calibration->state != 5U)));
    column = draw_text(7U, 0U, ok ? "OK" : "BAD");
    column = draw_text(7U, column, " E ");
    (void)draw_u32(7U, column, calibration->error, 2U);
}

static bool flush_one_page_async(void)
{
    bool transfer_complete;

    if (s_page_phase == 0U) {
        transfer_complete = h2026_bsp_oled_try_write_command(
            (uint8_t)(0xB0U | s_next_page));
    } else if (s_page_phase == 1U) {
        transfer_complete = h2026_bsp_oled_try_write_command(0x00U);
    } else if (s_page_phase == 2U) {
        transfer_complete = h2026_bsp_oled_try_write_command(0x10U);
    } else {
        const size_t page_offset = (size_t)s_next_page * OLED_WIDTH;
        const size_t remaining = OLED_WIDTH - s_data_offset;
        const size_t chunk = (remaining > OLED_ASYNC_DATA_BYTES)
            ? OLED_ASYNC_DATA_BYTES : remaining;

        transfer_complete = h2026_bsp_oled_try_write_data(
            &s_framebuffer[page_offset + s_data_offset], chunk);
        if (transfer_complete) {
            s_data_offset = (uint8_t)(s_data_offset + chunk);
            if (s_data_offset < OLED_WIDTH) {
                return false;
            }
            s_data_offset = 0U;
            s_page_phase = 0U;
            if (s_next_page == (OLED_PAGES - 1U)) {
                s_next_page = 0U;
                return true;
            }
            ++s_next_page;
        }
        return false;
    }

    if (transfer_complete) {
        ++s_page_phase;
    }
    return false;
}

void line_tracker_display_init(void)
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
    s_page_phase = 0U;
    s_data_offset = 0U;
    s_job_active = false;
}

void line_tracker_display_service(line_tracker_state_t state,
                                  uint32_t elapsed_ms,
                                  float distance_m,
                                  uint8_t mode,
                                  const line_tracker_calibration_view_t *calibration)
{
    if (!h2026_bsp_display_refresh_pending()) {
        return;
    }
    if (!s_job_active) {
        if ((calibration != NULL) && calibration->active) {
            render_calibration(calibration);
        } else {
            render(state, elapsed_ms, distance_m, mode);
        }
        s_job_active = true;
    }
    if (flush_one_page_async()) {
        s_job_active = false;
        h2026_bsp_display_refresh_complete();
    }
}
