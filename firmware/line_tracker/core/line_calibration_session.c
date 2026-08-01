#include "line_calibration_session.h"

#include <string.h>

static uint32_t absolute_delta(uint16_t left, uint16_t right)
{
    return (left >= right) ? (uint32_t)(left - right)
                           : (uint32_t)(right - left);
}

void line_calibration_session_begin(line_calibration_session_t *session)
{
    if (session == NULL) {
        return;
    }
    memset(session, 0, sizeof(*session));
    session->state = LINE_CAL_WHITE;
}

void line_calibration_session_cancel(line_calibration_session_t *session)
{
    if (session == NULL) {
        return;
    }
    session->state = LINE_CAL_IDLE;
}

void line_calibration_session_step(
    line_calibration_session_t *session,
    const uint16_t raw_adc[LINE_TRACKER_SENSOR_COUNT], bool frame_valid)
{
    uint8_t index;

    if ((session == NULL) || (raw_adc == NULL) ||
        !line_calibration_session_active(session)) {
        return;
    }
    if (!frame_valid) {
        session->state = LINE_CAL_FAILED;
        session->error = LINE_CAL_ERROR_ADC;
        return;
    }
    if (session->state == LINE_CAL_WHITE) {
        for (index = 0U; index < LINE_TRACKER_SENSOR_COUNT; ++index) {
            session->white_sum[index] += raw_adc[index];
        }
        ++session->ticks;
        if (session->ticks >= LINE_CALIBRATION_WHITE_TICKS) {
            for (index = 0U; index < LINE_TRACKER_SENSOR_COUNT; ++index) {
                session->white_adc[index] = (uint16_t)(
                    (session->white_sum[index] +
                     (LINE_CALIBRATION_WHITE_TICKS / 2U)) /
                    LINE_CALIBRATION_WHITE_TICKS);
                session->black_adc[index] = session->white_adc[index];
            }
            session->ticks = 0U;
            session->state = LINE_CAL_BLACK;
        }
        return;
    }
    if (session->state != LINE_CAL_BLACK) {
        return;
    }
    for (index = 0U; index < LINE_TRACKER_SENSOR_COUNT; ++index) {
        const uint32_t delta = absolute_delta(
            raw_adc[index], session->white_adc[index]);

        if ((raw_adc[index] >= session->white_adc[index]) &&
            (delta > session->black_delta[index])) {
            session->black_delta[index] = delta;
            session->black_adc[index] = raw_adc[index];
        }
    }
    session->ok_bits = line_calibration_span_ok_bits(
        session->white_adc, session->black_adc);
    ++session->ticks;
    if (session->ticks >= LINE_CALIBRATION_BLACK_TICKS) {
        if (session->ok_bits == 0xFFU) {
            session->state = LINE_CAL_READY_TO_SAVE;
        } else {
            session->state = LINE_CAL_FAILED;
            session->error = LINE_CAL_ERROR_SPAN;
        }
    }
}

void line_calibration_session_mark_saved(line_calibration_session_t *session)
{
    if ((session != NULL) &&
        (session->state == LINE_CAL_READY_TO_SAVE)) {
        session->state = LINE_CAL_SAVED;
        session->error = LINE_CAL_ERROR_NONE;
    }
}

void line_calibration_session_mark_store_failed(
    line_calibration_session_t *session, uint8_t store_error)
{
    if (session == NULL) {
        return;
    }
    session->state = LINE_CAL_FAILED;
    session->error = (uint8_t)(LINE_CAL_ERROR_STORE_BASE + store_error);
}

bool line_calibration_session_active(
    const line_calibration_session_t *session)
{
    return (session != NULL) && (session->state != LINE_CAL_IDLE);
}

uint8_t line_calibration_session_live_bits(
    const line_calibration_session_t *session,
    const uint16_t raw_adc[LINE_TRACKER_SENSOR_COUNT])
{
    uint8_t bits = 0U;
    uint8_t index;

    if ((session == NULL) || (raw_adc == NULL)) {
        return 0U;
    }
    for (index = 0U; index < LINE_TRACKER_SENSOR_COUNT; ++index) {
        const uint16_t white = session->white_adc[index];
        const uint16_t black = session->black_adc[index];
        const uint16_t threshold = (uint16_t)(
            white + (((uint32_t)black - white) / 2U));

        if (((session->ok_bits & (uint8_t)(1U << index)) != 0U) &&
            (raw_adc[index] >= threshold)) {
            bits |= (uint8_t)(1U << index);
        }
    }
    return bits;
}

uint8_t line_calibration_session_progress_percent(
    const line_calibration_session_t *session)
{
    uint32_t percent;

    if (session == NULL) {
        return 0U;
    }
    if (session->state == LINE_CAL_WHITE) {
        percent = ((uint32_t)session->ticks * 100U) /
            LINE_CALIBRATION_WHITE_TICKS;
    } else if (session->state == LINE_CAL_BLACK) {
        percent = ((uint32_t)session->ticks * 100U) /
            LINE_CALIBRATION_BLACK_TICKS;
    } else if ((session->state == LINE_CAL_READY_TO_SAVE) ||
               (session->state == LINE_CAL_SAVED)) {
        percent = 100U;
    } else {
        percent = 0U;
    }
    return (percent > 100U) ? 100U : (uint8_t)percent;
}
