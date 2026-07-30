"""Read-only archive of the C07A line-calibration Flash sector.

This never loads a program, erases Flash, or writes target memory.  It saves
the entire reserved 1 KiB sector so an exact record can be restored before a
later firmware download if the loader performs a mass erase.
"""
from datetime import datetime
from hashlib import sha256
from pathlib import Path
import json
import re
import struct
import sys

from scripting import initScripting


FLASH_ADDRESS = 0x0001FC00
FLASH_SECTOR_BYTES = 1024
MAGIC = 0x47385231
FORMAT = 1
COMMIT = 0xC011A6ED
WORD_COUNT = 20


def crc16_update(crc, value):
    crc ^= value
    for _ in range(8):
        crc = ((crc >> 1) ^ 0xA001) if (crc & 1) else (crc >> 1)
    return crc & 0xFFFF


def calibration_crc16(words):
    crc = 0xFFFF
    for index in range(8):
        sample = words[4 + index]
        for value in (sample & 0xFFFF, sample >> 16):
            crc = crc16_update(crc, value & 0xFF)
            crc = crc16_update(crc, (value >> 8) & 0xFF)
        coordinate = words[12 + index]
        for _ in range(4):
            crc = crc16_update(crc, coordinate & 0xFF)
            coordinate >>= 8
    version = words[2] & 0xFFFF
    crc = crc16_update(crc, version & 0xFF)
    return crc16_update(crc, version >> 8)


def validate_and_describe(blob):
    words = struct.unpack("<20I", blob[:WORD_COUNT * 4])
    if words[0] != MAGIC or words[1] != FORMAT or words[3] != COMMIT:
        raise RuntimeError("calibration record magic/format/commit is invalid")
    if (words[2] >> 16) != calibration_crc16(words):
        raise RuntimeError("calibration record CRC is invalid")

    channels = []
    for index in range(8):
        sample = words[4 + index]
        white = sample & 0xFFFF
        black = sample >> 16
        if black - white < 410:
            raise RuntimeError(f"channel {index} calibration span is too small")
        coordinate = struct.unpack("<f", struct.pack("<I", words[12 + index]))[0]
        channels.append({"index": index, "white_adc": white,
                         "black_adc": black, "x_mm": coordinate})
    return {"version": words[2] & 0xFFFF,
            "crc16": words[2] >> 16,
            "channels": channels}


def main():
    project_root = Path(__file__).resolve().parents[1]
    ccxml = Path(r"D:\A-Soft\DevTools\TI\ccs2100\ccs\scripting\examples\debugger\mspm0g3507\mspm0g3507.ccxml")
    archive_dir = project_root / "calibration_backups"
    archive_dir.mkdir(exist_ok=True)

    ds = initScripting({"timeout": 15000})
    session = None
    try:
        ds.configure(str(ccxml))
        session = ds.openSession(re.compile(r"cortex", re.IGNORECASE))
        session.target.connect()
        values = session.memory.read(FLASH_ADDRESS, FLASH_SECTOR_BYTES, 8)
        blob = bytes(values)
        description = validate_and_describe(blob)
        stamp = datetime.now().strftime("%Y%m%d-%H%M%S")
        stem = archive_dir / f"c07a_line_calibration_{stamp}"
        binary_path = stem.with_suffix(".bin")
        metadata_path = stem.with_suffix(".json")
        binary_path.write_bytes(blob)
        metadata = {
            "flash_address": f"0x{FLASH_ADDRESS:08X}",
            "sector_bytes": FLASH_SECTOR_BYTES,
            "sha256": sha256(blob).hexdigest(),
            "record": description,
        }
        metadata_path.write_text(json.dumps(metadata, indent=2), encoding="utf-8")
        print(f"BACKUP OK: {binary_path}")
        print(f"METADATA: {metadata_path}")
        print(f"SHA256: {metadata['sha256']}")
    finally:
        if session is not None:
            session.target.disconnect()
        ds.shutdown()


if __name__ == "__main__":
    try:
        main()
    except Exception as error:
        print(f"BACKUP FAILED: {error}", file=sys.stderr)
        raise
