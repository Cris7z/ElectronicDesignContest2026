/* Read-only GPIO quadrature diagnostic for a chassis with an unavailable BLS. */
#include "h2026_q2_display.h"

#include "../bsp/h2026_bsp.h"

#include "ti_msp_dl_config.h"

#include <stdbool.h>
#include <stdint.h>

int main(void)
{
    uint32_t overrun_count = 0U;
    bool display_job_active = false;
    h2026_bsp_encoder_snapshot_t encoders = {0};

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

    for (;;) {
        if (!h2026_bsp_take_control_tick(&overrun_count)) {
            __WFE();
            continue;
        }
        h2026_bsp_motor_arm(false);
        h2026_bsp_encoder_snapshot(&encoders);
        if (!h2026_bsp_display_refresh_pending()) {
            continue;
        }
        if (!display_job_active) {
            h2026_q2_display_render_encoder_passive(
                encoders.left_count, encoders.right_count,
                encoders.left_invalid_transitions,
                encoders.right_invalid_transitions,
                encoders.left_phase, encoders.right_phase);
            display_job_active = true;
        }
        if (h2026_q2_display_flush_one_page()) {
            display_job_active = false;
            h2026_bsp_display_refresh_complete();
        }
    }
}
