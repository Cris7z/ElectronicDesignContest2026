"""Read the live H-R02 diagnostics without programming or resetting C07A."""

import re

from scripting import initScripting


CCXML = (r"D:\A-Soft\DevTools\TI\ccs2100\ccs\scripting\examples"
         r"\debugger\mspm0g3507\mspm0g3507.ccxml")
STATE_ADDRESS = 0x20200700
BLACK_COUNT_ADDRESS = 0x20200715
LOST_TICKS_ADDRESS = 0x20200718
RAW_ADC_ADDRESS = 0x2020073C
DISTANCE_ADDRESS = 0x2020076C
ELAPSED_ADDRESS = 0x20200770
OVERRUN_ADDRESS = 0x2020078C
BSP_READY_ADDRESS = 0x20200790
APPROACH_ADDRESS = 0x20200791
CONTROL_TICK_OVERRUNS_ADDRESS = 0x202007B8
LINE_SCAN_COUNT_ADDRESS = 0x202007C0
LINE_SCAN_FAILURES_ADDRESS = 0x202007C4
LINE_SCAN_MAX_US_ADDRESS = 0x202007C8


def bytes_at(session, address, count):
    return bytes(int(value) & 0xFF
                 for value in session.memory.read(address, count, 8))


def u32(session, address):
    return int.from_bytes(bytes_at(session, address, 4), "little")


def u8(session, address):
    return bytes_at(session, address, 1)[0]


def u16(session, address):
    return int.from_bytes(bytes_at(session, address, 2), "little")


def main():
    ds = initScripting({"timeout": 15000})
    session = None
    try:
        ds.configure(CCXML)
        session = ds.openSession(re.compile(r"cortex", re.IGNORECASE))
        session.target.connect()
        state = u32(session, STATE_ADDRESS)
        output_raw = bytes_at(session, STATE_ADDRESS, 48)
        raw_bytes = bytes_at(session, RAW_ADC_ADDRESS, 16)
        raw_adc = [int.from_bytes(raw_bytes[index:index + 2], "little")
                   for index in range(0, len(raw_bytes), 2)]
        print(f"STATE={state} (0=WAIT, 1=RUN, 2=FAULT)")
        print("OUTPUT_HEX=" + output_raw.hex())
        print(f"BLACK_COUNT={u8(session, BLACK_COUNT_ADDRESS)}")
        print(f"LOST_TICKS={u16(session, LOST_TICKS_ADDRESS)}")
        print(f"TICK_OVERRUNS={u32(session, OVERRUN_ADDRESS)}")
        print(f"BSP_READY={u8(session, BSP_READY_ADDRESS)}")
        print(f"ELAPSED_MS={u32(session, ELAPSED_ADDRESS)}")
        print(f"APPROACH_ACTIVE={u8(session, APPROACH_ADDRESS)}")
        print(f"CONTROL_TICK_OVERRUNS={u32(session, CONTROL_TICK_OVERRUNS_ADDRESS)}")
        print(f"LINE_SCAN_COUNT={u32(session, LINE_SCAN_COUNT_ADDRESS)}")
        print(f"LINE_SCAN_FAILURES={u32(session, LINE_SCAN_FAILURES_ADDRESS)}")
        print(f"LINE_SCAN_MAX_US={u32(session, LINE_SCAN_MAX_US_ADDRESS)}")
        print("RAW_ADC=" + ",".join(str(value) for value in raw_adc))
        # Distance is a float; print its raw bits to keep this script
        # independent of target-expression evaluation.
        print(f"DISTANCE_BITS=0x{u32(session, DISTANCE_ADDRESS):08X}")
    finally:
        if session is not None:
            session.target.disconnect()
        ds.shutdown()


if __name__ == "__main__":
    main()
