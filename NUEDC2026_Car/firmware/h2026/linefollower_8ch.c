#include "linefollower_8ch.h"

static uint16_t read_u16_le(const uint8_t data[2])
{
    return (uint16_t)data[0] | ((uint16_t)data[1] << 8);
}

static bool read_registers(linefollower_8ch_t *sensor,
                           linefollower_8ch_i2c_read_fn read,
                           void *context,
                           uint8_t register_address,
                           uint8_t *data,
                           size_t length)
{
    if (read == NULL ||
        !read(context, LINEFOLLOWER_8CH_I2C_ADDR_7BIT,
              register_address, data, length)) {
        ++sensor->i2c_errors;
        return false;
    }

    return true;
}

void linefollower_8ch_init(linefollower_8ch_t *sensor,
                           bool state_active_high)
{
    *sensor = (linefollower_8ch_t){0};
    sensor->state_active_high = state_active_high;
}

void linefollower_8ch_update_state(linefollower_8ch_t *sensor,
                                   uint8_t state_bits)
{
    static const int8_t weights[LINEFOLLOWER_8CH_CHANNELS] = {
        -7, -5, -3, -1, 1, 3, 5, 7
    };

    sensor->state_bits = state_bits;
    sensor->state_valid = true;
    sensor->line_bits = sensor->state_active_high
        ? state_bits
        : (uint8_t)~state_bits;

    int sum = 0;
    uint8_t count = 0u;
    for (uint8_t i = 0u; i < LINEFOLLOWER_8CH_CHANNELS; ++i) {
        if ((sensor->line_bits & (uint8_t)(1u << i)) != 0u) {
            sum += weights[i];
            ++count;
        }
    }

    sensor->line_count = count;
    sensor->line_lost = count == 0u;
    sensor->all_line = count == LINEFOLLOWER_8CH_CHANNELS;
    sensor->line_valid = !sensor->line_lost && !sensor->all_line;

    if (sensor->line_valid) {
        sensor->error = (float)sum / ((float)count * 7.0f);
        sensor->error_filtered +=
            0.35f * (sensor->error - sensor->error_filtered);
    }
}

bool linefollower_8ch_read_state(linefollower_8ch_t *sensor,
                                 linefollower_8ch_i2c_read_fn read,
                                 void *context)
{
    uint8_t value = 0u;
    if (!read_registers(sensor, read, context,
                        LINEFOLLOWER_8CH_REG_STATE, &value, sizeof(value))) {
        sensor->state_valid = false;
        sensor->line_valid = false;
        sensor->line_lost = false;
        sensor->all_line = false;
        return false;
    }

    linefollower_8ch_update_state(sensor, value);
    return true;
}

bool linefollower_8ch_read_analog(linefollower_8ch_t *sensor,
                                  linefollower_8ch_i2c_read_fn read,
                                  void *context)
{
    uint8_t data[LINEFOLLOWER_8CH_CHANNELS * 2u];
    if (!read_registers(sensor, read, context,
                        LINEFOLLOWER_8CH_REG_ANALOG_CH1,
                        data, sizeof(data))) {
        sensor->analog_valid = false;
        return false;
    }

    for (uint8_t i = 0u; i < LINEFOLLOWER_8CH_CHANNELS; ++i) {
        sensor->analog[i] = read_u16_le(&data[i * 2u]);
    }
    sensor->analog_valid = true;
    return true;
}

bool linefollower_8ch_read_thresholds(linefollower_8ch_t *sensor,
                                      linefollower_8ch_i2c_read_fn read,
                                      void *context)
{
    uint8_t data[LINEFOLLOWER_8CH_CHANNELS * 2u];
    if (!read_registers(sensor, read, context,
                        LINEFOLLOWER_8CH_REG_THRESHOLD_CH1,
                        data, sizeof(data))) {
        sensor->threshold_valid = false;
        return false;
    }

    for (uint8_t i = 0u; i < LINEFOLLOWER_8CH_CHANNELS; ++i) {
        sensor->threshold[i] = read_u16_le(&data[i * 2u]);
    }
    sensor->threshold_valid = true;
    return true;
}
