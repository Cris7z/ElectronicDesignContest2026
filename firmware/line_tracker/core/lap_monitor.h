#ifndef LAP_MONITOR_H
#define LAP_MONITOR_H

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    LAP_MONITOR_WAIT = 0,
    LAP_MONITOR_RUNNING,
    LAP_MONITOR_COMPLETE,
    LAP_MONITOR_TIMEOUT
} lap_monitor_state_t;

typedef struct {
    float target_distance_m;
    float approach_distance_m;
    float left_meters_per_count;
    float right_meters_per_count;
    int8_t left_forward_sign;
    int8_t right_forward_sign;
    uint32_t timeout_ms;
} lap_monitor_config_t;

typedef struct {
    lap_monitor_state_t state;
    uint32_t elapsed_ms;
    float distance_m;
    float remaining_m;
    bool approach_active;
} lap_monitor_output_t;

typedef struct {
    lap_monitor_config_t config;
    lap_monitor_output_t output;
    int64_t left_origin_count;
    int64_t right_origin_count;
    uint32_t start_ms;
} lap_monitor_t;

bool lap_monitor_init(lap_monitor_t *monitor,
                      const lap_monitor_config_t *config);
void lap_monitor_reset(lap_monitor_t *monitor);
void lap_monitor_start(lap_monitor_t *monitor,
                       int64_t left_count,
                       int64_t right_count,
                       uint32_t now_ms);
void lap_monitor_cancel(lap_monitor_t *monitor);
void lap_monitor_step(lap_monitor_t *monitor,
                      int64_t left_count,
                      int64_t right_count,
                      uint32_t now_ms);

#endif
