#include "h2026_q2_app_config.h"

#include "ti_msp_dl_config.h"

#include <string.h>

#define H2026_Q2_PI 3.14159265358979323846f

/*
 * MSPM0G3507 main Flash is 128 KiB and is erased in 1 KiB sectors.  The
 * linker is constrained to end at 0x1FBFF, so this final sector can only hold
 * this calibration record; it can never contain executable firmware.
 */
#define H2026_Q2_LINE_CAL_FLASH_ADDRESS       0x0001FC00UL
#define H2026_Q2_LINE_CAL_FLASH_SECTOR_BYTES  DL_FLASHCTL_SECTOR_SIZE
#define H2026_Q2_LINE_CAL_STORE_MAGIC         0x47385231UL
#define H2026_Q2_LINE_CAL_STORE_COMMIT        0xC011A6EDUL
#define H2026_Q2_LINE_CAL_STORE_FORMAT        1UL

enum {
    H2026_Q2_LINE_CAL_STORE_MAGIC_INDEX = 0,
    H2026_Q2_LINE_CAL_STORE_FORMAT_INDEX,
    H2026_Q2_LINE_CAL_STORE_VERSION_CRC_INDEX,
    H2026_Q2_LINE_CAL_STORE_COMMIT_INDEX,
    H2026_Q2_LINE_CAL_STORE_SAMPLE_BASE_INDEX,
    H2026_Q2_LINE_CAL_STORE_POSITION_BASE_INDEX =
        H2026_Q2_LINE_CAL_STORE_SAMPLE_BASE_INDEX + H2026_Q2_LINE_SENSOR_COUNT,
    H2026_Q2_LINE_CAL_STORE_WORD_COUNT =
        H2026_Q2_LINE_CAL_STORE_POSITION_BASE_INDEX + H2026_Q2_LINE_SENSOR_COUNT
};

_Static_assert((H2026_Q2_LINE_CAL_STORE_WORD_COUNT % 2U) == 0U,
               "Flash writes must be 64-bit aligned");
_Static_assert((H2026_Q2_LINE_CAL_STORE_WORD_COUNT * sizeof(uint32_t)) <=
                   H2026_Q2_LINE_CAL_FLASH_SECTOR_BYTES,
               "calibration record must fit in its reserved Flash sector");

/*
 * The first power-up intentionally has no valid white/black references.  A
 * guarded BLS calibration session fills this RAM blob and commits it into the
 * reserved final Flash sector. Motors remain disarmed until the blob passes
 * the CRC/span validation.
 */
static h2026_q2_line_calibration_t s_line_calibration = {
    .sensor_x_mm = {
        H2026_Q2_SENSOR_X0_MM, H2026_Q2_SENSOR_X1_MM,
        H2026_Q2_SENSOR_X2_MM, H2026_Q2_SENSOR_X3_MM,
        H2026_Q2_SENSOR_X4_MM, H2026_Q2_SENSOR_X5_MM,
        H2026_Q2_SENSOR_X6_MM, H2026_Q2_SENSOR_X7_MM
    }
};

static bool s_line_calibration_loaded;
/* OLED-visible, so hardware write failures can be separated without UART. */
static uint8_t s_line_calibration_store_error;

enum {
    LINE_CAL_STORE_OK = 0U,
    LINE_CAL_STORE_INVALID_INPUT = 1U,
    LINE_CAL_STORE_ERASE_FAILED = 2U,
    LINE_CAL_STORE_PROGRAM_BASE = 3U,
    LINE_CAL_STORE_VERIFY_BASE = 16U,
    LINE_CAL_STORE_UNPACK_FAILED = 40U
};

static void line_calibration_pack(
    const h2026_q2_line_calibration_t *calibration,
    uint32_t words[H2026_Q2_LINE_CAL_STORE_WORD_COUNT])
{
    uint32_t i;

    memset(words, 0xFF, H2026_Q2_LINE_CAL_STORE_WORD_COUNT * sizeof(words[0]));
    words[H2026_Q2_LINE_CAL_STORE_MAGIC_INDEX] = H2026_Q2_LINE_CAL_STORE_MAGIC;
    words[H2026_Q2_LINE_CAL_STORE_FORMAT_INDEX] = H2026_Q2_LINE_CAL_STORE_FORMAT;
    words[H2026_Q2_LINE_CAL_STORE_VERSION_CRC_INDEX] =
        (uint32_t)calibration->version | ((uint32_t)calibration->crc16 << 16U);
    words[H2026_Q2_LINE_CAL_STORE_COMMIT_INDEX] = H2026_Q2_LINE_CAL_STORE_COMMIT;

    for (i = 0U; i < H2026_Q2_LINE_SENSOR_COUNT; ++i) {
        words[H2026_Q2_LINE_CAL_STORE_SAMPLE_BASE_INDEX + i] =
            (uint32_t)calibration->white_adc[i] |
            ((uint32_t)calibration->black_adc[i] << 16U);
        memcpy(&words[H2026_Q2_LINE_CAL_STORE_POSITION_BASE_INDEX + i],
               &calibration->sensor_x_mm[i], sizeof(uint32_t));
    }
}

static bool line_calibration_unpack(
    const volatile uint32_t words[H2026_Q2_LINE_CAL_STORE_WORD_COUNT],
    h2026_q2_line_calibration_t *calibration)
{
    uint32_t i;

    if ((words[H2026_Q2_LINE_CAL_STORE_MAGIC_INDEX] !=
             H2026_Q2_LINE_CAL_STORE_MAGIC) ||
        (words[H2026_Q2_LINE_CAL_STORE_FORMAT_INDEX] !=
             H2026_Q2_LINE_CAL_STORE_FORMAT) ||
        (words[H2026_Q2_LINE_CAL_STORE_COMMIT_INDEX] !=
             H2026_Q2_LINE_CAL_STORE_COMMIT)) {
        return false;
    }

    memset(calibration, 0, sizeof(*calibration));
    calibration->version = (uint16_t)
        words[H2026_Q2_LINE_CAL_STORE_VERSION_CRC_INDEX];
    calibration->crc16 = (uint16_t)
        (words[H2026_Q2_LINE_CAL_STORE_VERSION_CRC_INDEX] >> 16U);
    for (i = 0U; i < H2026_Q2_LINE_SENSOR_COUNT; ++i) {
        const uint32_t sample =
            words[H2026_Q2_LINE_CAL_STORE_SAMPLE_BASE_INDEX + i];

        calibration->white_adc[i] = (uint16_t)sample;
        calibration->black_adc[i] = (uint16_t)(sample >> 16U);
        memcpy(&calibration->sensor_x_mm[i],
               (const void *)&words[H2026_Q2_LINE_CAL_STORE_POSITION_BASE_INDEX + i],
               sizeof(uint32_t));
    }
    return h2026_q2_line_calibration_valid(calibration);
}

static void line_calibration_load_once(void)
{
    const volatile uint32_t *const flash_words =
        (const volatile uint32_t *)(uintptr_t)H2026_Q2_LINE_CAL_FLASH_ADDRESS;
    h2026_q2_line_calibration_t calibration;

    if (s_line_calibration_loaded) {
        return;
    }
    s_line_calibration_loaded = true;
    if (line_calibration_unpack(flash_words, &calibration)) {
        s_line_calibration = calibration;
    }
}

static bool line_calibration_store(const h2026_q2_line_calibration_t *calibration)
{
    uint32_t words[H2026_Q2_LINE_CAL_STORE_WORD_COUNT];
    const volatile uint32_t *const flash_words =
        (const volatile uint32_t *)(uintptr_t)H2026_Q2_LINE_CAL_FLASH_ADDRESS;
    uint32_t i;
    uint32_t primask;
    DL_FLASHCTL_COMMAND_STATUS status;
    bool erase_succeeded;
    bool verified = false;

    line_calibration_pack(calibration, words);
    s_line_calibration_store_error = LINE_CAL_STORE_OK;
    primask = __get_PRIMASK();
    __disable_irq();

    DL_FlashCTL_executeClearStatus(FLASHCTL);
    DL_FlashCTL_unprotectSector(FLASHCTL, H2026_Q2_LINE_CAL_FLASH_ADDRESS,
                                DL_FLASHCTL_REGION_SELECT_MAIN);
    status = DL_FlashCTL_eraseMemoryFromRAM(
        FLASHCTL, H2026_Q2_LINE_CAL_FLASH_ADDRESS,
        DL_FLASHCTL_COMMAND_SIZE_SECTOR);
    erase_succeeded = (status == DL_FLASHCTL_COMMAND_STATUS_PASSED);
    for (i = 0U; (status == DL_FLASHCTL_COMMAND_STATUS_PASSED) &&
                  (i < H2026_Q2_LINE_CAL_STORE_WORD_COUNT); i += 2U) {
        /* Each completed erase/program command re-protects main Flash. */
        DL_FlashCTL_executeClearStatus(FLASHCTL);
        DL_FlashCTL_unprotectSector(
            FLASHCTL, H2026_Q2_LINE_CAL_FLASH_ADDRESS,
            DL_FLASHCTL_REGION_SELECT_MAIN);
        status = DL_FlashCTL_programMemoryFromRAM64WithECCGenerated(
            FLASHCTL, H2026_Q2_LINE_CAL_FLASH_ADDRESS + (i * sizeof(uint32_t)),
            &words[i]);
    }
    DL_FlashCTL_protectSector(FLASHCTL, H2026_Q2_LINE_CAL_FLASH_ADDRESS,
                              DL_FLASHCTL_REGION_SELECT_MAIN);

    if (primask == 0U) {
        __enable_irq();
    }

    if (status != DL_FLASHCTL_COMMAND_STATUS_PASSED) {
        s_line_calibration_store_error = !erase_succeeded
            ? LINE_CAL_STORE_ERASE_FAILED
            : (uint8_t)(LINE_CAL_STORE_PROGRAM_BASE + (i / 2U));
        return false;
    }
    for (i = 0U; i < H2026_Q2_LINE_CAL_STORE_WORD_COUNT; ++i) {
        if (flash_words[i] != words[i]) {
            s_line_calibration_store_error =
                (uint8_t)(LINE_CAL_STORE_VERIFY_BASE + i);
            return false;
        }
    }
    verified = line_calibration_unpack(flash_words,
                                       &(h2026_q2_line_calibration_t){0});
    if (!verified) {
        s_line_calibration_store_error = LINE_CAL_STORE_UNPACK_FAILED;
    }
    return verified;
}

static bool is_binary_setting(int value)
{
    return (value == 0) || (value == 1);
}

static bool is_sign_setting(int value)
{
    return (value == -1) || (value == 1);
}

uint32_t h2026_q2_app_calibration_locks(void)
{
    uint32_t locks = 0U;

    line_calibration_load_once();

    if (H2026_Q2_COMMISSIONED != 1) {
        locks |= H2026_Q2_CAL_LOCK_NOT_COMMISSIONED;
    }
    if (!h2026_q2_line_calibration_valid(&s_line_calibration)) {
        locks |= H2026_Q2_CAL_LOCK_LINE_CALIBRATION;
    }
    if (!is_binary_setting(H2026_Q2_SENSOR_CH1_IS_LEFT)) {
        locks |= H2026_Q2_CAL_LOCK_LINE_ORDER;
    }
    if (!is_sign_setting(H2026_Q2_LEFT_ENCODER_SIGN) ||
        !is_sign_setting(H2026_Q2_RIGHT_ENCODER_SIGN)) {
        locks |= H2026_Q2_CAL_LOCK_ENCODER_SIGNS;
    }
    if (!is_sign_setting(H2026_Q2_LEFT_MOTOR_SIGN) ||
        !is_sign_setting(H2026_Q2_RIGHT_MOTOR_SIGN)) {
        locks |= H2026_Q2_CAL_LOCK_MOTOR_SIGNS;
    }
    if (!(H2026_Q2_LEFT_METERS_PER_ENCODER_COUNT > 0.0f) ||
        !(H2026_Q2_RIGHT_METERS_PER_ENCODER_COUNT > 0.0f)) {
        locks |= H2026_Q2_CAL_LOCK_ENCODER_SCALE;
    }
    if (!(H2026_Q2_TRACK_WIDTH_M > 0.0f)) {
        locks |= H2026_Q2_CAL_LOCK_TRACK_WIDTH;
    }
    if (!(H2026_Q2_STOP_DISTANCE_FROM_MARKER_M >= 0.0f)) {
        locks |= H2026_Q2_CAL_LOCK_STOP_OFFSET;
    }
    if (!is_binary_setting(H2026_Q2_START_ACTIVE_LEVEL)) {
        locks |= H2026_Q2_CAL_LOCK_BUTTON_LEVEL;
    }
    return locks;
}

bool h2026_q2_app_line_calibration_get(
    h2026_q2_line_calibration_t *calibration)
{
    line_calibration_load_once();
    if (calibration == NULL) {
        return false;
    }
    *calibration = s_line_calibration;
    return h2026_q2_line_calibration_valid(calibration);
}

bool h2026_q2_app_line_calibration_set(
    const h2026_q2_line_calibration_t *calibration)
{
    line_calibration_load_once();
    if (!h2026_q2_line_calibration_valid(calibration)) {
        s_line_calibration_store_error = LINE_CAL_STORE_INVALID_INPUT;
        return false;
    }
    if (!line_calibration_store(calibration)) {
        return false;
    }
    s_line_calibration = *calibration;
    return true;
}

uint8_t h2026_q2_app_line_calibration_store_error(void)
{
    return s_line_calibration_store_error;
}

static void configure_stadium_feedforward(h2026_q2_config_t *config)
{
    const float straight_m = 1.5f;
    const float half_circle_m = H2026_Q2_PI * 0.5f;
    const float clockwise_curvature_1pm = 2.0f;
    const float lap_m = (2.0f * straight_m) + (2.0f * half_circle_m);

    if (H2026_Q2_STADIUM_FEEDFORWARD_VERIFIED != 1) {
        config->curve_segment_count = 0U;
        return;
    }

    /*
     * 正曲率在 core 中定义为右转。末尾额外放一个 0 曲率段，使过终点后的
     * 停车距离不会继续套用最后一个半圆的前馈。
     */
    config->curve_segment_count = 5U;
    config->curve_segments[0].end_distance_m = straight_m;
    config->curve_segments[0].curvature_1pm = 0.0f;
    config->curve_segments[1].end_distance_m = straight_m + half_circle_m;
    config->curve_segments[1].curvature_1pm = clockwise_curvature_1pm;
    config->curve_segments[2].end_distance_m =
        (2.0f * straight_m) + half_circle_m;
    config->curve_segments[2].curvature_1pm = 0.0f;
    config->curve_segments[3].end_distance_m = lap_m;
    config->curve_segments[3].curvature_1pm = clockwise_curvature_1pm;
    config->curve_segments[4].end_distance_m = lap_m + 1.0f;
    config->curve_segments[4].curvature_1pm = 0.0f;
}

bool h2026_q2_app_build_config(h2026_q2_config_t *config)
{
    line_calibration_load_once();
    if (config == NULL) {
        return false;
    }

    memset(config, 0, sizeof(*config));

    config->wide_min_active = 3U;
    memcpy(config->sensor_x_mm, s_line_calibration.sensor_x_mm,
           sizeof(config->sensor_x_mm));

    /*
     * 5 cm 横向起止线一般只覆盖八路板的一部分，绝不能写死 0xFF。
     * 按键起跑时采集实际图案，返程只接受足够宽且与该图案相交的候选。
     */
    config->marker_capture_min_active = 3U;
    config->marker_detect_min_active = 3U;
    config->marker_count_tolerance = 1U;
    config->marker_detect_ratio = 0.70f;
    config->marker_center_limit_normalized = 0.45f;
    config->marker_release_ms = 60U;
    config->marker_confirm_ms = 25U;
    config->start_clear_distance_m = 0.12f;
    config->start_acquire_timeout_ms = 1200U;
    config->start_acquire_speed_mps = 0.10f;
    /* 轮子压在起点，灰度在前方约 10 cm：起停横线不参与本车计时。 */
    config->use_start_finish_marker = false;
    config->finish_gate_distance_m = 6.1416f;
    config->finish_gate_time_ms = 1000U;
    config->distance_finish_approach_m = 0.15f;
    config->zero_offset_approach_speed_mps = 0.05f;
    config->stop_distance_from_marker_m =
        H2026_Q2_STOP_DISTANCE_FROM_MARKER_M;
    config->stop_position_tolerance_m = 0.008f;
    config->stop_speed_tolerance_mps = 0.015f;
    config->stop_hold_ms = 250U;

    config->line_sensor_grace_ms = 20U;
    config->line_sensor_fault_ms = 150U;
    config->line_grace_ms = 40U;
    config->line_fault_ms = 300U;
    config->mission_timeout_ms = 45000U;
    config->stopping_timeout_ms = 4000U;
    config->fault_coast_max_ms = 150U;

    config->left_meters_per_encoder_count =
        H2026_Q2_LEFT_METERS_PER_ENCODER_COUNT;
    config->right_meters_per_encoder_count =
        H2026_Q2_RIGHT_METERS_PER_ENCODER_COUNT;
    config->left_encoder_sign =
        (int8_t)H2026_Q2_LEFT_ENCODER_SIGN;
    config->right_encoder_sign =
        (int8_t)H2026_Q2_RIGHT_ENCODER_SIGN;
    config->track_width_m = H2026_Q2_TRACK_WIDTH_M;

    /* 首次实车循迹按 0.20 m/s 起调，稳定后再分级提高到 0.30/0.40 m/s。 */
    config->cruise_speed_mps = 0.20f;
    config->minimum_tracking_speed_mps = 0.10f;
    config->degraded_sensor_speed_mps = 0.08f;
    config->degraded_line_speed_mps = 0.06f;
    config->maximum_wheel_speed_mps = 0.60f;
    config->acceleration_limit_mps2 = 0.70f;
    config->deceleration_limit_mps2 = 1.20f;
    config->stopping_deceleration_mps2 = 0.80f;
    config->line_error_speed_reduction = 0.15f;
    config->curvature_speed_reduction_m = 0.0625f;

    config->line_error_filter_alpha = 0.35f;
    config->line_derivative_filter_alpha = 0.20f;
    config->line_kp_center_mps = 0.14f;
    config->line_kp_edge_mps = 0.30f;
    config->line_kd_center_m = 0.005f;
    config->line_kd_edge_m = 0.012f;
    config->line_correction_limit_mps = 0.23f;

    config->curvature_feedforward_gain = 0.90f;
    config->curve_transition_m = 0.12f;
    configure_stadium_feedforward(config);

    /*
     * 这里只是首次 0.20 m/s 实车验证的保守起调值。kS/kV 和 PI 后续仍须用
     * 架空轮速阶跃日志重辨识；不得在未复核前直接提高巡线速度。
     */
    config->left_speed_pi.kp = 0.60f;
    config->left_speed_pi.ki = 3.00f;
    config->left_speed_pi.ks = 0.08f;
    config->left_speed_pi.kv = 1.80f;
    config->left_speed_pi.integral_limit = 0.25f;
    config->right_speed_pi = config->left_speed_pi;
    config->wheel_speed_filter_alpha = 0.35f;
    config->signed_duty_limit = 0.85f;

    return (h2026_q2_app_calibration_locks() == 0U) &&
           h2026_q2_config_validate(config);
}

bool h2026_q2_app_start_pressed(bool raw_level)
{
    if (!is_binary_setting(H2026_Q2_START_ACTIVE_LEVEL)) {
        return false;
    }
    return raw_level ==
           (H2026_Q2_START_ACTIVE_LEVEL == 1);
}

float h2026_q2_app_left_motor_duty(float controller_duty)
{
    return controller_duty * (float)H2026_Q2_LEFT_MOTOR_SIGN;
}

float h2026_q2_app_right_motor_duty(float controller_duty)
{
    return controller_duty * (float)H2026_Q2_RIGHT_MOTOR_SIGN;
}
