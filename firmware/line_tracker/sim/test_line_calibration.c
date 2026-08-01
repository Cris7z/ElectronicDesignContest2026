#include "line_calibration.h"
#include "line_calibration_session.h"

#include <assert.h>
#include <stdio.h>
#include <string.h>

static void fill_positions(line_calibration_record_t *record)
{
    uint8_t index;

    for (index = 0U; index < LINE_TRACKER_SENSOR_COUNT; ++index) {
        record->sensor_x_mm[index] = -35.0f + (10.0f * index);
    }
}

static void test_record_crc_and_validation(void)
{
    line_calibration_record_t record;
    uint8_t index;

    memset(&record, 0, sizeof(record));
    for (index = 0U; index < LINE_TRACKER_SENSOR_COUNT; ++index) {
        record.white_adc[index] = (uint16_t)(200U + index);
        record.black_adc[index] = (uint16_t)(1200U + index);
    }
    fill_positions(&record);
    line_calibration_finalize(&record);
    assert(line_calibration_valid(&record));
    record.black_adc[3] = 300U;
    line_calibration_finalize(&record);
    assert(!line_calibration_valid(&record));
}

static void test_frozen_record_crc(void)
{
    line_calibration_record_t record = {
        .white_adc = {174U, 174U, 172U, 173U,
                      171U, 172U, 171U, 171U},
        .black_adc = {4095U, 4095U, 4095U, 4095U,
                      4095U, 4095U, 4095U, 4095U},
        .sensor_x_mm = {-35.0f, -25.0f, -15.0f, -5.0f,
                         5.0f,  15.0f,  25.0f, 35.0f},
        .version = LINE_CALIBRATION_VERSION
    };

    assert(line_calibration_crc16(&record) == 0x74D9U);
    line_calibration_finalize(&record);
    assert(line_calibration_valid(&record));
}

static void test_white_and_black_capture(void)
{
    line_calibration_session_t session;
    uint16_t raw[LINE_TRACKER_SENSOR_COUNT];
    uint16_t tick;
    uint8_t index;

    line_calibration_session_begin(&session);
    for (index = 0U; index < LINE_TRACKER_SENSOR_COUNT; ++index) {
        raw[index] = (uint16_t)(200U + index);
    }
    for (tick = 0U; tick < LINE_CALIBRATION_WHITE_TICKS; ++tick) {
        line_calibration_session_step(&session, raw, true);
    }
    assert(session.state == LINE_CAL_BLACK);
    for (tick = 0U; tick < LINE_CALIBRATION_BLACK_TICKS; ++tick) {
        for (index = 0U; index < LINE_TRACKER_SENSOR_COUNT; ++index) {
            raw[index] = (uint16_t)(1200U + index);
        }
        line_calibration_session_step(&session, raw, true);
    }
    assert(session.state == LINE_CAL_READY_TO_SAVE);
    assert(session.ok_bits == 0xFFU);
    assert(line_calibration_session_live_bits(&session, raw) == 0xFFU);
    line_calibration_session_mark_saved(&session);
    assert(session.state == LINE_CAL_SAVED);
}

static void test_bad_span_does_not_pass(void)
{
    line_calibration_session_t session;
    uint16_t raw[LINE_TRACKER_SENSOR_COUNT] = {200U, 200U, 200U, 200U,
                                               200U, 200U, 200U, 200U};
    uint16_t tick;

    line_calibration_session_begin(&session);
    for (tick = 0U; tick < LINE_CALIBRATION_WHITE_TICKS; ++tick) {
        line_calibration_session_step(&session, raw, true);
    }
    raw[0] = 1200U;
    for (tick = 0U; tick < LINE_CALIBRATION_BLACK_TICKS; ++tick) {
        line_calibration_session_step(&session, raw, true);
    }
    assert(session.state == LINE_CAL_FAILED);
    assert(session.error == LINE_CAL_ERROR_SPAN);
    assert(session.ok_bits == 0x01U);
}

int main(void)
{
    test_record_crc_and_validation();
    test_frozen_record_crc();
    test_white_and_black_capture();
    test_bad_span_does_not_pass();
    puts("line_calibration host tests: PASS");
    return 0;
}
