"""Read transition diagnostics from a running line-tracker image.

The target is never halted or written.  Start this script, then drive one
ordinary lap so the CSV captures approach, entry, apex and exit transitions.
"""
from pathlib import Path
import argparse
import re
import struct
import time

from scripting import initScripting


CCXML = Path(r"D:\A-Soft\DevTools\TI\ccs2100\ccs\scripting\examples\debugger\mspm0g3507\mspm0g3507.ccxml")


def symbol_address(map_text, symbol):
    pattern = (r"(?m)^\s*([0-9a-fA-F]{8})\s+\S+\s+"
               r"\(\.common:" + re.escape(symbol) + r"\)")
    match = re.search(pattern, map_text)
    if match is None:
        raise RuntimeError("symbol not found in matched map: " + symbol)
    return int(match.group(1), 16)


def read_float(session, address):
    return struct.unpack("<f", bytes(session.memory.read(address, 4, 8)))[0]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--map", type=Path, required=True,
                        help="map belonging to the flashed image")
    parser.add_argument("--duration", type=float, default=25.0)
    parser.add_argument("--interval", type=float, default=0.02)
    args = parser.parse_args()
    if args.duration <= 0.0 or args.interval <= 0.0:
        raise ValueError("duration and interval must be positive")

    map_text = args.map.read_text(encoding="utf-8", errors="replace")
    symbols = (
        "g_line_tracker_debug_p_duty",
        "g_line_tracker_debug_d_duty",
        "g_line_tracker_debug_pd_target_yaw",
        "g_line_tracker_debug_final_yaw",
        "g_line_tracker_debug_base_duty",
        "g_line_tracker_debug_edge_blend",
        "g_line_tracker_applied_left_duty",
        "g_line_tracker_applied_right_duty",
        "g_line_tracker_shadow_left_measured",
        "g_line_tracker_shadow_right_measured",
    )
    addresses = {symbol: symbol_address(map_text, symbol) for symbol in symbols}
    ds = initScripting({"timeout": 15000})
    session = None
    try:
        ds.configure(str(CCXML))
        session = ds.openSession(re.compile(r"cortex", re.IGNORECASE))
        session.target.connect()
        session.target.run(False)
        print("t,p,d,pd_yaw,final_yaw,base,edge,left_pwm,right_pwm,left_measured,right_measured")
        deadline = time.monotonic() + args.duration
        while time.monotonic() < deadline:
            values = [read_float(session, addresses[symbol]) for symbol in symbols]
            print(f"{time.monotonic():.3f}," +
                  ",".join(f"{value:.5f}" for value in values), flush=True)
            time.sleep(args.interval)
    finally:
        if session is not None:
            try:
                session.target.run(False)
            except Exception as error:
                if "not halted" not in str(error).lower():
                    raise
            session.target.disconnect()
        ds.shutdown()


if __name__ == "__main__":
    main()
