#include <stdint.h>

/*
 * Factory/frozen calibration image at 0x1FC00. The linker places this array
 * in its own 1 KiB Flash region, so every program download restores these
 * known-good references. Runtime MODE calibration may erase and replace the
 * same data sector without touching executable code.
 *
 * CRC16=0x74D9 uses the same version-1 format as line_calibration_store.c.
 */
__attribute__((section(".calibration"), used, aligned(8)))
const uint32_t g_default_calibration_image[20] = {
    0x47385231UL, 0x00000001UL, 0x74D90001UL, 0xC011A6EDUL,
    0x0FFF00AEUL, 0x0FFF00AEUL, 0x0FFF00ACUL, 0x0FFF00ADUL,
    0x0FFF00ABUL, 0x0FFF00ACUL, 0x0FFF00ABUL, 0x0FFF00ABUL,
    0xC20C0000UL, 0xC1C80000UL, 0xC1700000UL, 0xC0A00000UL,
    0x40A00000UL, 0x41700000UL, 0x41C80000UL, 0x420C0000UL
};
