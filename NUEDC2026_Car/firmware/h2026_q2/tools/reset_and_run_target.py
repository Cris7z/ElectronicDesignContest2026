"""Reset and run the already-programmed target without loading or writing Flash."""
from pathlib import Path
import re

from scripting import initScripting


def main():
    ccxml = Path(r"D:\A-Soft\DevTools\TI\ccs2100\ccs\scripting\examples\debugger\mspm0g3507\mspm0g3507.ccxml")
    ds = initScripting({"timeout": 15000})
    session = None
    try:
        ds.configure(str(ccxml))
        session = ds.openSession(re.compile(r"cortex", re.IGNORECASE))
        session.target.connect()
        session.target.reset()
        session.target.run(False)
        print("TARGET RESET AND RUNNING")
    finally:
        if session is not None:
            session.target.disconnect()
        ds.shutdown()


if __name__ == "__main__":
    main()
