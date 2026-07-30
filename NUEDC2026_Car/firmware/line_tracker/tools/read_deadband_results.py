"""Read the completed raised-chassis low-duty sweep; no target writes."""
from pathlib import Path
import re
import struct

from scripting import initScripting


CCXML = Path(r"D:\A-Soft\DevTools\TI\ccs2100\ccs\scripting\examples\debugger\mspm0g3507\mspm0g3507.ccxml")
DONE_ADDRESS = 0x2020015C
RUNNING_ADDRESS = 0x2020015D
STAGE_ADDRESS = 0x2020015E
LEFT_ADDRESS = 0x20200160
RIGHT_ADDRESS = 0x20200198
DUTY_PERCENT = (2, 3, 4, 5, 6, 8, 10)


def values_at(session, address):
    data = bytes(session.memory.read(address, 8 * len(DUTY_PERCENT), 8))
    return struct.unpack("<7q", data)


def main():
    ds = initScripting({"timeout": 15000})
    session = None
    try:
        ds.configure(str(CCXML))
        session = ds.openSession(re.compile(r"cortex", re.IGNORECASE))
        session.target.connect()
        session.target.halt()
        left = values_at(session, LEFT_ADDRESS)
        right = values_at(session, RIGHT_ADDRESS)
        done = session.memory.readOne(DONE_ADDRESS, 8)
        running = session.memory.readOne(RUNNING_ADDRESS, 8)
        stage = session.memory.readOne(STAGE_ADDRESS, 8)
        print(f"done={done} running={running} completed_steps={stage}")
        for duty, left_delta, right_delta in zip(DUTY_PERCENT, left, right):
            print(f"duty={duty:2d}% left={left_delta:5d} right={right_delta:5d}")
    finally:
        if session is not None:
            session.target.run(False)
            session.target.disconnect()
        ds.shutdown()


if __name__ == "__main__":
    main()
