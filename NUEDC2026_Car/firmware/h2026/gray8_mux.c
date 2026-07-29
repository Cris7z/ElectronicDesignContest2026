#include "gray8_mux.h"

void gray8_mux_init(gray8_mux_t *sensor, bool active_low)
{
    *sensor = (gray8_mux_t){0};
    sensor->active_low = active_low;
}

void gray8_mux_begin_scan(gray8_mux_t *sensor)
{
    sensor->scan_address = 0u;
    sensor->scan_bits = 0u;
}

uint8_t gray8_mux_current_address(const gray8_mux_t *sensor)
{
    return sensor->scan_address & 0x07u;
}

bool gray8_mux_push_out_sample(gray8_mux_t *sensor, bool out_high)
{
    const uint8_t address = gray8_mux_current_address(sensor);
    if (out_high) {
        sensor->scan_bits |= (uint8_t)(1u << address);
    }

    ++sensor->scan_address;
    if (sensor->scan_address < GRAY8_MUX_CHANNELS) {
        return false;
    }

    gray8_mux_update(sensor, sensor->scan_bits);
    gray8_mux_begin_scan(sensor);
    return true;
}

void gray8_mux_update(gray8_mux_t *sensor, uint8_t raw_bits)
{
    static const int8_t weights[GRAY8_MUX_CHANNELS] = {
        -7, -5, -3, -1, 1, 3, 5, 7
    };

    sensor->raw_bits = raw_bits;
    sensor->dark_bits = sensor->active_low
        ? (uint8_t)~sensor->raw_bits
        : sensor->raw_bits;

    int sum = 0;
    uint8_t count = 0u;
    for (uint8_t i = 0u; i < GRAY8_MUX_CHANNELS; ++i) {
        if ((sensor->dark_bits & (uint8_t)(1u << i)) != 0u) {
            sum += weights[i];
            ++count;
        }
    }

    sensor->dark_count = count;
    sensor->line_lost = count == 0u;
    sensor->all_dark = count == GRAY8_MUX_CHANNELS;
    sensor->line_valid = !sensor->line_lost && !sensor->all_dark;

    if (sensor->line_valid) {
        sensor->error = (float)sum / ((float)count * 7.0f);
        sensor->error_filtered +=
            0.35f * (sensor->error - sensor->error_filtered);
    }
}
