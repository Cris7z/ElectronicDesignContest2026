"""Read-only two-second encoder delta while the currently running image drives."""
from pathlib import Path
import re
import time

from scripting import initScripting


CCXML = Path(r"D:\A-Soft\DevTools\TI\ccs2100\ccs\scripting\examples\debugger\mspm0g3507\mspm0g3507.ccxml")
LEFT_ADDRESS = 0x20200278
RIGHT_ADDRESS = 0x20200280


def count_at(session, address):
    return int.from_bytes(bytes(session.memory.read(address, 8, 8)),
                          byteorder="little", signed=True)


def main():
    ds = initScripting({"timeout": 15000})
    session = None
    try:
        ds.configure(str(CCXML))
        session = ds.openSession(re.compile(r"cortex", re.IGNORECASE))
        session.target.connect()
        session.target.halt()
        left_before = count_at(session, LEFT_ADDRESS)
        right_before = count_at(session, RIGHT_ADDRESS)
        session.target.run(False)
        time.sleep(2.0)
        session.target.halt()
        left_after = count_at(session, LEFT_ADDRESS)
        right_after = count_at(session, RIGHT_ADDRESS)
        print(f"left={left_before}->{left_after}, delta={left_after-left_before}")
        print(f"right={right_before}->{right_after}, delta={right_after-right_before}")
    finally:
        if session is not None:
            session.target.run(False)
            session.target.disconnect()
        ds.shutdown()


if __name__ == "__main__":
    main()
