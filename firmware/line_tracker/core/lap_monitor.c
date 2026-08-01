#include "lap_monitor.h"

#include <string.h>

static bool config_is_valid(const lap_monitor_config_t *config)
{
    return (config != NULL) &&
           (config->target_distance_m > 0.0f) &&
           (config->approach_distance_m >= 0.0f) &&
           (config->approach_distance_m < config->target_distance_m) &&
           (config->left_meters_per_count > 0.0f) &&
           (config->right_meters_per_count > 0.0f) &&
           ((config->left_forward_sign == -1) ||
            (config->left_forward_sign == 1)) &&
           ((config->right_forward_sign == -1) ||
            (config->right_forward_sign == 1)) &&
           (config->timeout_ms > 0U);
}

static void update_output(lap_monitor_t *monitor,
                          int64_t left_count,
                          int64_t right_count,
                          uint32_t now_ms)
{
    const int64_t left_delta = left_count - monitor->left_origin_count;
    const int64_t right_delta = right_count - monitor->right_origin_count;
    const float left_distance =
        (float)(monitor->config.left_forward_sign * left_delta) *
        monitor->config.left_meters_per_count;
    const float right_distance =
        (float)(monitor->config.right_forward_sign * right_delta) *
        monitor->config.right_meters_per_count;

    monitor->output.elapsed_ms = now_ms - monitor->start_ms;
    monitor->output.distance_m = 0.5f * (left_distance + right_distance);
    if (monitor->output.distance_m < monitor->config.target_distance_m) {
        monitor->output.remaining_m =
            monitor->config.target_distance_m - monitor->output.distance_m;
    } else {
        monitor->output.remaining_m = 0.0f;
    }
    /* Keep the physical 150 mm gate stable across binary float rounding. */
    monitor->output.approach_active =
        monitor->output.remaining_m <=
        (monitor->config.approach_distance_m + 0.000001f);
}

bool lap_monitor_init(lap_monitor_t *monitor,
                      const lap_monitor_config_t *config)
{
    if ((monitor == NULL) || !config_is_valid(config)) {
        return false;
    }
    memset(monitor, 0, sizeof(*monitor));
    monitor->config = *config;
    lap_monitor_reset(monitor);
    return true;
}

void lap_monitor_reset(lap_monitor_t *monitor)
{
    if (monitor == NULL) {
        return;
    }
    memset(&monitor->output, 0, sizeof(monitor->output));
    monitor->output.state = LAP_MONITOR_WAIT;
}

void lap_monitor_start(lap_monitor_t *monitor,
                       int64_t left_count,
                       int64_t right_count,
                       uint32_t now_ms)
{
    if (monitor == NULL) {
        return;
    }
    lap_monitor_reset(monitor);
    monitor->left_origin_count = left_count;
    monitor->right_origin_count = right_count;
    monitor->start_ms = now_ms;
    monitor->output.state = LAP_MONITOR_RUNNING;
}

void lap_monitor_cancel(lap_monitor_t *monitor)
{
    if ((monitor != NULL) &&
        (monitor->output.state != LAP_MONITOR_WAIT)) {
        monitor->output.state = LAP_MONITOR_WAIT;
        monitor->output.approach_active = false;
    }
}

void lap_monitor_step(lap_monitor_t *monitor,
                      int64_t left_count,
                      int64_t right_count,
                      uint32_t now_ms)
{
    if ((monitor == NULL) ||
        (monitor->output.state != LAP_MONITOR_RUNNING)) {
        return;
    }
    update_output(monitor, left_count, right_count, now_ms);
    if (monitor->output.elapsed_ms >= monitor->config.timeout_ms) {
        monitor->output.state = LAP_MONITOR_TIMEOUT;
        monitor->output.approach_active = false;
    } else if (monitor->output.distance_m >= monitor->config.target_distance_m) {
        monitor->output.state = LAP_MONITOR_COMPLETE;
        monitor->output.approach_active = false;
    }
}
