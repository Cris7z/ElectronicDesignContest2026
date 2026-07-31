"""Back up C07A calibration, program the application, verify, and run.

The application linker excludes 0x1FC00..0x1FFFF.  This tool still reads the
reserved sector both before and after programming: a changed hash means the
download procedure was not safe for the stored calibration and the target is
left halted for recovery from the backup.
"""

import argparse
from datetime import datetime
from hashlib import sha256
from pathlib import Path
import re
import sys

from scripting import initScripting


CALIBRATION_ADDRESS = 0x0001FC00
CALIBRATION_BYTES = 1024
DEFAULT_CCXML = Path(
    r"D:\A-Soft\DevTools\TI\ccs2100\ccs\scripting\examples\debugger"
    r"\mspm0g3507\mspm0g3507.ccxml"
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
        if after_hash != before_hash:
            raise RuntimeError(
                "calibration sector changed during programming; target remains "
                "halted and the pre-flash backup must be restored before run"
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
