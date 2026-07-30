"""Live, read-only shadow-speed-PI monitor for one exact line_tracker build.

The target is left running while SRAM is sampled through XDS110; this tool
never writes target memory and never calls halt.  It is therefore suitable for
observing an operating car without injecting a 5 ms control-loop pause.
"""
from pathlib import Path
import argparse
import re
import struct
import time

from scripting import initScripting


CCXML = Path(r"D:\A-Soft\DevTools\TI\ccs2100\ccs\scripting\examples\debugger\mspm0g3507\mspm0g3507.ccxml")
DEFAULT_MAP = (Path(__file__).resolve().parents[1] / "Build" /
               "line_tracker_speed_pi_shadow_yaw90_20260731" /
               "line_tracker.map")


def address_for_symbol(map_text, symbol):
    pattern = (r"(?m)^\s*([0-9a-fA-F]{8})\s+\S+\s+"
               r"\(\.common:" + re.escape(symbol) + r"\)")
    match = re.search(pattern, map_text)
    if match is None:
        raise RuntimeError(f"symbol not found in matched map: {symbol}")
    return int(match.group(1), 16)


def optional_address_for_symbol(map_text, symbol):
    try:
        return address_for_symbol(map_text, symbol)
    except RuntimeError:
        return None


def read_float(session, address):
    data = bytes(session.memory.read(address, 4, 8))
    return struct.unpack("<f", data)[0]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=20.0,
                        help="capture duration in seconds (default: 20)")
    parser.add_argument("--interval", type=float, default=0.10,
                        help="sample interval in seconds (default: 0.10)")
    parser.add_argument("--map", type=Path, default=DEFAULT_MAP,
                        help="map belonging to the currently flashed image")
    args = parser.parse_args()
    if args.duration <= 0.0 or args.interval <= 0.0:
        raise ValueError("duration and interval must be positive")

    map_text = args.map.read_text(encoding="utf-8", errors="replace")
    shadow = address_for_symbol(map_text, "g_line_tracker_shadow_left_correction")
    addresses = {
        "pwm_left": address_for_symbol(map_text, "g_line_tracker_applied_left_duty"),
        "pwm_right": address_for_symbol(map_text, "g_line_tracker_applied_right_duty"),
        "balance": optional_address_for_symbol(
            map_text, "g_line_tracker_wheel_balance_trim"),
        "updated": address_for_symbol(map_text, "g_line_tracker_shadow_updated"),
    }
    bench_state = optional_address_for_symbol(map_text, "g_wheel_pi_bench_state")
    tracker_state = optional_address_for_symbol(map_text, "g_line_tracker_output")
    if bench_state is None and tracker_state is None:
        raise RuntimeError("matched map has neither bench nor tracker state")
    ds = initScripting({"timeout": 15000})
    session = None
    try:
        ds.configure(str(CCXML))
        session = ds.openSession(re.compile(r"cortex", re.IGNORECASE))
        session.target.connect()
        session.target.run(False)
        print("t,state,updated,target_l,target_r,measured_l,measured_r,"
              "correction_l,correction_r,pwm_l,pwm_r,balance")
        deadline = time.monotonic() + args.duration
        while time.monotonic() < deadline:
            # Addresses 0x328..0x33f are six contiguous float variables:
            # L corr, L measured, L target, R corr, R measured, R target.
            block = bytes(session.memory.read(shadow, 24, 8))
            left_correction, left_measured, left_target, right_correction, \
                right_measured, right_target = struct.unpack("<6f", block)
            state_address = bench_state if bench_state is not None else tracker_state
            state = bytes(session.memory.read(state_address, 1, 8))[0]
            updated = bytes(session.memory.read(addresses["updated"], 1, 8))[0]
            pwm_left = read_float(session, addresses["pwm_left"])
            pwm_right = read_float(session, addresses["pwm_right"])
            balance = (read_float(session, addresses["balance"])
                       if addresses["balance"] is not None else 0.0)
            print(f"{time.monotonic():.3f},{state},{updated},"
                  f"{left_target:.4f},{right_target:.4f},"
                  f"{left_measured:.4f},{right_measured:.4f},"
                  f"{left_correction:.4f},{right_correction:.4f},"
                  f"{pwm_left:.4f},{pwm_right:.4f},{balance:.4f}", flush=True)
            time.sleep(args.interval)
    finally:
        if session is not None:
            # Reading live SRAM leaves the target running.  CCS reports an
            # exception if run is requested again in that state; it is not a
            # capture failure and the target must remain running either way.
            try:
                session.target.run(False)
            except Exception as error:
                if "not halted" not in str(error).lower():
                    raise
            session.target.disconnect()
        ds.shutdown()


if __name__ == "__main__":
    main()
