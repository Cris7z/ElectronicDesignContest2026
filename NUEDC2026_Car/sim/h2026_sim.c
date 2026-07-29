/**
 * Host tests for the H2026 protocol, LF04 processing and ball controller.
 */
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "../firmware/h2026/ball_balance.h"
#include "../firmware/h2026/ball_link.h"
#include "../firmware/h2026/lf04.h"

static void put_u16(uint8_t *p, uint16_t value)
{
    p[0] = (uint8_t)value;
    p[1] = (uint8_t)(value >> 8);
}
static void put_u32(uint8_t *p, uint32_t value)
{
    p[0] = (uint8_t)value;
    p[1] = (uint8_t)(value >> 8);
    p[2] = (uint8_t)(value >> 16);
    p[3] = (uint8_t)(value >> 24);
}

static void make_state_frame(uint8_t frame[17], uint8_t seq,
                             int16_t x_mm, int16_t vx_mm_s)
{
    memset(frame, 0, 17);
    frame[0] = 0xAA;
    frame[1] = 0x55;
    frame[2] = BALL_LINK_TYPE_STATE;
    frame[3] = seq;
    put_u32(&frame[4], 123456u);
    put_u16(&frame[8], (uint16_t)x_mm);
    put_u16(&frame[10], (uint16_t)vx_mm_s);
    frame[12] = 220u;
    frame[13] = BALL_LINK_FLAG_VALID | BALL_LINK_FLAG_CALIBRATED;
    put_u16(&frame[14], ball_link_crc16(&frame[2], 12u));
    frame[16] = 0x0D;
}

static int test_link(void)
{
    ball_link_t link;
    uint8_t frame[17];
    ball_link_init(&link);
    make_state_frame(frame, 7u, -35, 128);

    for (int i = 0; i < 17; ++i) {
        ball_link_rx_byte(&link, frame[i], 1000u);
    }
    int ok = link.frames_ok == 1u && link.x_mm == -35 &&
             link.vx_mm_s == 128 &&
             ball_link_alive(&link, 1100u, 120u, 150u);

    frame[9] ^= 0x01u;
    for (int i = 0; i < 17; ++i) {
        ball_link_rx_byte(&link, frame[i], 1110u);
    }
    ok &= link.frames_ok == 1u && link.crc_errors == 1u;
    ok &= !ball_link_alive(&link, 1121u, 120u, 150u);

    printf("[H1] UART frame + CRC + timeout: %s\n", ok ? "PASS" : "FAIL");
    return ok;
}

static int test_lf04(void)
{
    lf04_t sensor;
    lf04_init(&sensor, false);
    lf04_update(&sensor, 0x01u);
    const int left_ok = sensor.line_valid && sensor.error < -0.9f;
    lf04_update(&sensor, 0x06u);
    const int centre_ok = sensor.line_valid && fabsf(sensor.error) < 0.01f;
    lf04_update(&sensor, 0x00u);
    const int lost_ok = !sensor.line_valid && !sensor.all_dark;
    lf04_update(&sensor, 0x0Fu);
    const int cross_ok = !sensor.line_valid && sensor.all_dark;
    const int ok = left_ok && centre_ok && lost_ok && cross_ok;
    printf("[H2] LF04 four-channel decoding: %s\n", ok ? "PASS" : "FAIL");
    return ok;
}

static int test_ball_controller(void)
{
    const float dt = 0.01f;
    const float g = 9.80665f;
    ball_balance_params_t params;
    ball_balance_t ctrl;
    ball_balance_default_params(&params);
    params.max_angle_deg = 6.0f;
    ball_balance_init(&ctrl, &params);

    float x_m = -0.040f;
    float v_m_s = 0.0f;
    float peak_after_settle_mm = 0.0f;
    for (int step = 0; step < 800; ++step) {
        const float t = step * dt;
        float target_mm;
        if (t < 2.0f) {
            target_mm = 0.0f;
        } else if (t < 3.5f) {
            target_mm = ball_balance_smooth_target(0.0f, 50.0f,
                                                    t - 2.0f, 1.5f);
        } else if (t < 5.5f) {
            target_mm = ball_balance_smooth_target(50.0f, -50.0f,
                                                    t - 3.5f, 2.0f);
        } else {
            target_mm = -50.0f;
        }

        const float theta_deg = ball_balance_update(
            &ctrl, target_mm, x_m * 1000.0f, v_m_s * 1000.0f,
            0.0f, dt, true);
        const float theta_rad = theta_deg * 0.01745329252f;
        const float acceleration = (5.0f / 7.0f) * g * theta_rad
                                 - 0.45f * v_m_s;
        v_m_s += acceleration * dt;
        x_m += v_m_s * dt;

        if (t > 6.5f) {
            const float error_mm = fabsf(x_m * 1000.0f + 50.0f);
            if (error_mm > peak_after_settle_mm) {
                peak_after_settle_mm = error_mm;
            }
        }
    }

    const float final_error_mm = fabsf(x_m * 1000.0f + 50.0f);
    const int ok = final_error_mm < 10.0f && peak_after_settle_mm < 15.0f;
    printf("[H3] ball outer-loop model: final_err=%.1fmm peak_late=%.1fmm %s\n",
           final_error_mm, peak_after_settle_mm, ok ? "PASS" : "FAIL");
    return ok;
}

int main(void)
{
    int pass = 1;
    pass &= test_link();
    pass &= test_lf04();
    pass &= test_ball_controller();
    printf("======== H2026 %s ========\n", pass ? "ALL PASS" : "FAILED");
    return pass ? 0 : 1;
}
