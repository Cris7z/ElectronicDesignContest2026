#ifndef LINE_CALIBRATION_STORE_H
#define LINE_CALIBRATION_STORE_H

#include "line_calibration.h"

#include <stdbool.h>
#include <stdint.h>

bool line_calibration_store_load(line_calibration_record_t *record);
bool line_calibration_store_save(const line_calibration_record_t *record);
uint8_t line_calibration_store_error(void);

#endif
