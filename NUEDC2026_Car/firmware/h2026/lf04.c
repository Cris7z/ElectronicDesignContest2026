#include "lf04.h"

void lf04_init(lf04_t *sensor, bool active_low)
{
    *sensor = (lf04_t){0};
    sensor->active_low = active_low;
}
void lf04_update(lf04_t *sensor, uint8_t raw_bits)
{
    static const int8_t weights[4] = {-3, -1, 1, 3};
    sensor->raw_bits = raw_bits & 0x0Fu;
    sensor->dark_bits = sensor->active_low
        ? (uint8_t)(~sensor->raw_bits) & 0x0Fu
        : sensor->raw_bits;

    int sum = 0;
    uint8_t count = 0u;
    for (uint8_t i = 0; i < 4u; ++i) {
        if ((sensor->dark_bits & (uint8_t)(1u << i)) != 0u) {
            sum += weights[i];
            ++count;
        }
    }

    sensor->dark_count = count;
    sensor->line_valid = count > 0u && count < 4u;
    sensor->all_dark = count == 4u;
    if (sensor->line_valid) {
        sensor->error = (float)sum / ((float)count * 3.0f);
        sensor->error_filtered += 0.35f *
            (sensor->error - sensor->error_filtered);
    }
}
