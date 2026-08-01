"""Back up runtime calibration, program factory app+calibration, and run.

The .out image owns a separate calibration section at 0x1FC00. Every program
download intentionally restores the frozen calibration. A MODE calibration
may replace that data at runtime; its pre-flash copy is retained as evidence,
but the newly downloaded image must match the factory calibration exactly.
"""

import argparse
from datetime import datetime
from hashlib import sha256
from pathlib import Path
import re
import struct
import sys

from scripting import initScripting


CALIBRATION_ADDRESS = 0x0001FC00
CALIBRATION_BYTES = 1024
DEFAULT_CCXML = Path(
    r"D:\A-Soft\DevTools\TI\ccs2100\ccs\scripting\examples\debugger"
    r"\mspm0g3507\mspm0g3507.ccxml"
)
DEFAULT_CALIBRATION_WORDS = (
    0x47385231, 0x00000001, 0x74D90001, 0xC011A6ED,
    0x0FFF00AE, 0x0FFF00AE, 0x0FFF00AC, 0x0FFF00AD,
    0x0FFF00AB, 0x0FFF00AC, 0x0FFF00AB, 0x0FFF00AB,
    0xC20C0000, 0xC1C80000, 0xC1700000, 0xC0A00000,
    0x40A00000, 0x41700000, 0x41C80000, 0x420C0000,
)
DEFAULT_CALIBRATION = (
    b"".join(struct.pack("<I", word) for word in DEFAULT_CALIBRATION_WORDS) +
    bytes([0xFF]) *
    (CALIBRATION_BYTES - (4 * len(DEFAULT_CALIBRATION_WORDS)))
)


def read_calibration(session):
    values = session.memory.read(CALIBRATION_ADDRESS, CALIBRATION_BYTES, 8)
    blob = bytes(values)
    if len(blob) != CALIBRATION_BYTES:
        raise RuntimeError(f"unexpected calibration read length: {len(blob)}")
    return blob


def save_backup(directory, blob):
    directory.mkdir(parents=True, exist_ok=True)
    stamp = datetime.now().strftime("%Y%m%d-%H%M%S")
    path = directory / f"c07a_calibration_preflash_{stamp}.bin"
    path.write_bytes(blob)
    return path


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--program", required=True, type=Path,
                        help="absolute or relative path to the .out image")
    parser.add_argument("--ccxml", type=Path, default=DEFAULT_CCXML)
    parser.add_argument("--backup-dir", type=Path,
                        default=Path(__file__).resolve().parents[1] /
                        "calibration_backups")
    args = parser.parse_args()
    program = args.program.resolve()
    ccxml = args.ccxml.resolve()

    if not program.is_file():
        raise RuntimeError(f"program image does not exist: {program}")
    if not ccxml.is_file():
        raise RuntimeError(f"debug configuration does not exist: {ccxml}")

    ds = initScripting({"timeout": 30000})
    session = None
    try:
        ds.configure(str(ccxml))
        session = ds.openSession(re.compile(r"cortex", re.IGNORECASE))
        session.target.connect()

        before = read_calibration(session)
        before_hash = sha256(before).hexdigest()
        backup = save_backup(args.backup_dir, before)
        print(f"CALIBRATION BACKUP: {backup}")
        print(f"CALIBRATION SHA256 BEFORE: {before_hash}")

        session.memory.loadProgram(str(program))

        after = read_calibration(session)
        after_hash = sha256(after).hexdigest()
        print(f"CALIBRATION SHA256 AFTER:  {after_hash}")
        expected_hash = sha256(DEFAULT_CALIBRATION).hexdigest()
        print(f"CALIBRATION SHA256 DEFAULT:{expected_hash}")
        if after != DEFAULT_CALIBRATION:
            raise RuntimeError(
                "downloaded calibration does not match the frozen default; "
                "target remains halted"
            )

        session.target.reset()
        session.target.run(False)
        print("PROGRAM VERIFIED; TARGET RESET AND RUNNING")
    finally:
        if session is not None:
            session.target.disconnect()
        ds.shutdown()


if __name__ == "__main__":
    try:
        main()
    except Exception as error:
        print(f"FLASH FAILED: {error}", file=sys.stderr)
        raise
