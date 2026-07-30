/* Read-only PA18/BLS diagnostic; motor drive is never enabled. */
#include "h2026_q2_display.h"

#include "../bsp/h2026_bsp.h"

#include "ti_msp_dl_config.h"

#include <stdbool.h>
#include <stdint.h>

int main(void)
{
    uint32_t overrun_count = 0U;
    uint32_t edge_count = 0U;
    bool display_job_active = false;
    bool previous_level;

    if (!h2026_bsp_init()) {
        for (;;) {
            h2026_bsp_motor_arm(false);
            h2026_bsp_led_set(true);
            __WFE();
        }
    }
    h2026_bsp_motor_arm(false);
    h2026_q2_display_init();
    (void)h2026_bsp_take_control_tick(&overrun_count);
    previous_level = h2026_bsp_start_level();

    for (;;) {
        const bool raw_level = h2026_bsp_start_level();

        if (!h2026_bsp_take_control_tick(&overrun_count)) {
            __WFE();
            continue;
        }
        h2026_bsp_motor_arm(false);
        if (raw_level != previous_level) {
            previous_level = raw_level;
            ++edge_count;
        }
        if (!h2026_bsp_display_refresh_pending()) {
            continue;
        }
        if (!display_job_active) {
            h2026_q2_display_render_bls_passive(raw_level, edge_count);
            display_job_active = true;
        }
        if (h2026_q2_display_flush_one_page()) {
            display_job_active = false;
            h2026_bsp_display_refresh_complete();
        }
    }
}
