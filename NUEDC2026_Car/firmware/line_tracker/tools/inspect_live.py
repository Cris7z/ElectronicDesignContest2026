"""Read-only snapshot for the currently flashed line_tracker image."""
from pathlib import Path
import re
import struct

from scripting import initScripting


CCXML = Path(r"D:\A-Soft\DevTools\TI\ccs2100\ccs\scripting\examples\debugger\mspm0g3507\mspm0g3507.ccxml")
OUTPUT_ADDRESS = 0x20200204
RAW_ADC_ADDRESS = 0x20200228
OVERRUN_ADDRESS = 0x20200238


def main():
    ds = initScripting({"timeout": 15000})
    session = None
    try:
        ds.configure(str(CCXML))
        session = ds.openSession(re.compile(r"cortex", re.IGNORECASE))
        session.target.connect()
        session.target.halt()
        output = bytes(session.memory.read(OUTPUT_ADDRESS, 40, 8))
        raw = bytes(session.memory.read(RAW_ADC_ADDRESS, 16, 8))
        # TI's bare-metal build uses a one-byte enum here; the next uint16
        # member is aligned to offset 2.
        state = output[0]
        strengths = struct.unpack_from("<8H", output, 2)
        raw_adc = struct.unpack("<8H", raw)
        lost = struct.unpack_from("<H", output, 22)[0]
        error, left, right = struct.unpack_from("<3f", output, 24)
        overruns = session.memory.readOne(OVERRUN_ADDRESS, 32)
        print(f"state={state} black_mask=0x{output[18]:02X} black_count={output[19]} center_gap={output[20]}")
        print(f"raw_adc={list(raw_adc)}")
        print(f"strength={list(strengths)} error={error:.3f}")
        print(f"left_duty={left:.4f} right_duty={right:.4f} lost_ticks={lost} overruns={overruns}")
    finally:
        if session is not None:
            session.target.run(False)
            session.target.disconnect()
        ds.shutdown()


if __name__ == "__main__":
    main()
