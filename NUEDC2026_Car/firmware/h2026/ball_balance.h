/**
 * ball_balance.h - outer ball-position controller.
 *
 * Positive beam angle is defined to accelerate the ball toward +x.
 * This module outputs a beam angle target; a separate 200 Hz actuator loop
 * must close it with the MS42CG encoder and D36A STEP/DIR interface.
 */
#ifndef H2026_BALL_BALANCE_H
#define H2026_BALL_BALANCE_H

#include <stdbool.h>

typedef struct {
    float kp_rad_per_m;
    float ki_rad_per_m_s;
    float kd_rad_s_per_m;
    float accel_feedforward_gain;
    float velocity_lpf_alpha;
    float integral_limit_m_s;
    float max_angle_deg;
    float max_slew_deg_s;
} ball_balance_params_t;

typedef struct {
    ball_balance_params_t p;
    float integral_m_s;
    float velocity_m_s;
    float command_deg;
    bool measurement_valid;
} ball_balance_t;

void ball_balance_default_params(ball_balance_params_t *params);
void ball_balance_init(ball_balance_t *ctrl,
                       const ball_balance_params_t *params);
void ball_balance_reset(ball_balance_t *ctrl);

float ball_balance_update(ball_balance_t *ctrl,
                          float target_mm,
                          float measured_mm,
                          float measured_v_mm_s,
                          float vehicle_accel_m_s2,
                          float dt_s,
                          bool valid);

/** Quintic smoothstep reference with zero velocity/acceleration at both ends. */
float ball_balance_smooth_target(float start_mm, float end_mm,
                                 float elapsed_s, float duration_s);

#endif
