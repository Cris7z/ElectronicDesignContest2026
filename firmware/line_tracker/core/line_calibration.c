#include "line_calibration.h"

#include <stddef.h>
#include <string.h>

static uint16_t crc16_update(uint16_t crc, uint8_t value)
{
    uint8_t bit;

    crc ^= value;
    for (bit = 0U; bit < 8U; ++bit) {
        crc = (crc & 1U) != 0U
            ? (uint16_t)((crc >> 1U) ^ 0xA001U)
            : (uint16_t)(crc >> 1U);
    }
    return crc;
}

uint16_t line_calibration_crc16(const line_calibration_record_t *record)
{
    uint16_t crc = 0xFFFFU;
    size_t index;

    if (record == NULL) {
        return 0U;
    }
    for (index = 0U; index < LINE_TRACKER_SENSOR_COUNT; ++index) {
        const uint16_t values[2] = {
            record->white_adc[index], record->black_adc[index]
        };
        size_t value_index;

        for (value_index = 0U; value_index < 2U; ++value_index) {
            uint16_t value = values[value_index];
            uint8_t byte_index;

            for (byte_index = 0U; byte_index < 2U; ++byte_index) {
                crc = crc16_update(crc, (uint8_t)(value & 0xFFU));
                value >>= 8U;
            }
        }
        {
            uint32_t coordinate_bits;
            uint8_t byte_index;

            memcpy(&coordinate_bits, &record->sensor_x_mm[index],
                   sizeof(coordinate_bits));
            for (byte_index = 0U; byte_index < 4U; ++byte_index) {
                crc = crc16_update(
                    crc, (uint8_t)(coordinate_bits & 0xFFU));
                coordinate_bits >>= 8U;
            }
        }
    }
    crc = crc16_update(crc, (uint8_t)(record->version & 0xFFU));
    crc = crc16_update(crc, (uint8_t)(record->version >> 8U));
    return crc;
}

uint8_t line_calibration_span_ok_bits(
    const uint16_t white_adc[LINE_TRACKER_SENSOR_COUNT],
    const uint16_t black_adc[LINE_TRACKER_SENSOR_COUNT])
{
    uint8_t bits = 0U;
    uint8_t index;

    if ((white_adc == NULL) || (black_adc == NULL)) {
        return 0U;
    }
    for (index = 0U; index < LINE_TRACKER_SENSOR_COUNT; ++index) {
        if ((black_adc[index] <= 4095U) &&
            (white_adc[index] <= 4095U) &&
            ((uint32_t)black_adc[index] >=
             ((uint32_t)white_adc[index] + LINE_CALIBRATION_MIN_SPAN))) {
            bits |= (uint8_t)(1U << index);
        }
    }
    return bits;
}

void line_calibration_finalize(line_calibration_record_t *record)
{
    if (record == NULL) {
        return;
    }
    record->version = LINE_CALIBRATION_VERSION;
    record->crc16 = line_calibration_crc16(record);
}

bool line_calibration_valid(const line_calibration_record_t *record)
{
    uint8_t index;

    if ((record == NULL) ||
        (record->version != LINE_CALIBRATION_VERSION) ||
        (record->crc16 != line_calibration_crc16(record)) ||
        (line_calibration_span_ok_bits(record->white_adc,
                                       record->black_adc) != 0xFFU)) {
        return false;
    }
    for (index = 1U; index < LINE_TRACKER_SENSOR_COUNT; ++index) {
        if (!(record->sensor_x_mm[index] >
              record->sensor_x_mm[index - 1U])) {
            return false;
        }
    }
    return true;
}
