#include "line_calibration_store.h"

#include "ti_msp_dl_config.h"

#include <stdint.h>
#include <string.h>

#define LINE_CAL_FLASH_ADDRESS       0x0001FC00UL
#define LINE_CAL_STORE_MAGIC         0x47385231UL
#define LINE_CAL_STORE_COMMIT        0xC011A6EDUL
#define LINE_CAL_STORE_FORMAT        1UL

enum {
    STORE_MAGIC_INDEX = 0,
    STORE_FORMAT_INDEX,
    STORE_VERSION_CRC_INDEX,
    STORE_COMMIT_INDEX,
    STORE_SAMPLE_BASE_INDEX,
    STORE_POSITION_BASE_INDEX =
        STORE_SAMPLE_BASE_INDEX + LINE_TRACKER_SENSOR_COUNT,
    STORE_WORD_COUNT =
        STORE_POSITION_BASE_INDEX + LINE_TRACKER_SENSOR_COUNT
};

enum {
    STORE_OK = 0U,
    STORE_INVALID_INPUT = 1U,
    STORE_ERASE_FAILED = 2U,
    STORE_PROGRAM_BASE = 3U,
    STORE_VERIFY_BASE = 16U,
    STORE_UNPACK_FAILED = 40U
};

_Static_assert((STORE_WORD_COUNT % 2U) == 0U,
               "Flash writes must be 64-bit aligned");
_Static_assert((STORE_WORD_COUNT * sizeof(uint32_t)) <=
                   DL_FLASHCTL_SECTOR_SIZE,
               "calibration record must fit in one Flash sector");

static uint8_t s_store_error;

/* The linker fixes this symbol at LINE_CAL_FLASH_ADDRESS. Referencing the
 * symbol also guarantees that the default calibration section is retained. */
extern const uint32_t g_default_calibration_image[STORE_WORD_COUNT];

static void pack_record(const line_calibration_record_t *record,
                        uint32_t words[STORE_WORD_COUNT])
{
    uint32_t index;

    memset(words, 0xFF, STORE_WORD_COUNT * sizeof(words[0]));
    words[STORE_MAGIC_INDEX] = LINE_CAL_STORE_MAGIC;
    words[STORE_FORMAT_INDEX] = LINE_CAL_STORE_FORMAT;
    words[STORE_VERSION_CRC_INDEX] =
        (uint32_t)record->version | ((uint32_t)record->crc16 << 16U);
    words[STORE_COMMIT_INDEX] = LINE_CAL_STORE_COMMIT;
    for (index = 0U; index < LINE_TRACKER_SENSOR_COUNT; ++index) {
        uint32_t coordinate_bits;

        words[STORE_SAMPLE_BASE_INDEX + index] =
            (uint32_t)record->white_adc[index] |
            ((uint32_t)record->black_adc[index] << 16U);
        memcpy(&coordinate_bits, &record->sensor_x_mm[index],
               sizeof(coordinate_bits));
        words[STORE_POSITION_BASE_INDEX + index] = coordinate_bits;
    }
}

static bool unpack_record(const volatile uint32_t words[STORE_WORD_COUNT],
                          line_calibration_record_t *record)
{
    uint32_t index;

    if ((record == NULL) ||
        (words[STORE_MAGIC_INDEX] != LINE_CAL_STORE_MAGIC) ||
        (words[STORE_FORMAT_INDEX] != LINE_CAL_STORE_FORMAT) ||
        (words[STORE_COMMIT_INDEX] != LINE_CAL_STORE_COMMIT)) {
        return false;
    }
    memset(record, 0, sizeof(*record));
    record->version = (uint16_t)words[STORE_VERSION_CRC_INDEX];
    record->crc16 = (uint16_t)(words[STORE_VERSION_CRC_INDEX] >> 16U);
    for (index = 0U; index < LINE_TRACKER_SENSOR_COUNT; ++index) {
        const uint32_t sample = words[STORE_SAMPLE_BASE_INDEX + index];
        const uint32_t coordinate_bits =
            words[STORE_POSITION_BASE_INDEX + index];

        record->white_adc[index] = (uint16_t)sample;
        record->black_adc[index] = (uint16_t)(sample >> 16U);
        memcpy(&record->sensor_x_mm[index], &coordinate_bits,
               sizeof(coordinate_bits));
    }
    return line_calibration_valid(record);
}

bool line_calibration_store_load(line_calibration_record_t *record)
{
    const volatile uint32_t *const flash_words =
        g_default_calibration_image;

    return unpack_record(flash_words, record);
}

bool line_calibration_store_save(const line_calibration_record_t *record)
{
    uint32_t words[STORE_WORD_COUNT];
    const volatile uint32_t *const flash_words =
        (const volatile uint32_t *)(uintptr_t)LINE_CAL_FLASH_ADDRESS;
    DL_FLASHCTL_COMMAND_STATUS status;
    uint32_t index;
    uint32_t primask;
    bool erase_succeeded;
    line_calibration_record_t verified_record;

    if (!line_calibration_valid(record)) {
        s_store_error = STORE_INVALID_INPUT;
        return false;
    }
    pack_record(record, words);
    s_store_error = STORE_OK;
    primask = __get_PRIMASK();
    __disable_irq();
    DL_FlashCTL_executeClearStatus(FLASHCTL);
    DL_FlashCTL_unprotectSector(FLASHCTL, LINE_CAL_FLASH_ADDRESS,
                                DL_FLASHCTL_REGION_SELECT_MAIN);
    status = DL_FlashCTL_eraseMemoryFromRAM(
        FLASHCTL, LINE_CAL_FLASH_ADDRESS, DL_FLASHCTL_COMMAND_SIZE_SECTOR);
    erase_succeeded = (status == DL_FLASHCTL_COMMAND_STATUS_PASSED);
    for (index = 0U;
         (status == DL_FLASHCTL_COMMAND_STATUS_PASSED) &&
         (index < STORE_WORD_COUNT);
         index += 2U) {
        DL_FlashCTL_executeClearStatus(FLASHCTL);
        DL_FlashCTL_unprotectSector(FLASHCTL, LINE_CAL_FLASH_ADDRESS,
                                    DL_FLASHCTL_REGION_SELECT_MAIN);
        status = DL_FlashCTL_programMemoryFromRAM64WithECCGenerated(
            FLASHCTL,
            LINE_CAL_FLASH_ADDRESS + (index * sizeof(uint32_t)),
            &words[index]);
    }
    DL_FlashCTL_protectSector(FLASHCTL, LINE_CAL_FLASH_ADDRESS,
                              DL_FLASHCTL_REGION_SELECT_MAIN);
    if (primask == 0U) {
        __enable_irq();
    }
    if (status != DL_FLASHCTL_COMMAND_STATUS_PASSED) {
        s_store_error = !erase_succeeded
            ? STORE_ERASE_FAILED
            : (uint8_t)(STORE_PROGRAM_BASE + (index / 2U));
        return false;
    }
    for (index = 0U; index < STORE_WORD_COUNT; ++index) {
        if (flash_words[index] != words[index]) {
            s_store_error = (uint8_t)(STORE_VERIFY_BASE + index);
            return false;
        }
    }
    if (!unpack_record(flash_words, &verified_record)) {
        s_store_error = STORE_UNPACK_FAILED;
        return false;
    }
    return true;
}

uint8_t line_calibration_store_error(void)
{
    return s_store_error;
}
