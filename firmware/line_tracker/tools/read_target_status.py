"""Read the live H-R02 diagnostics without programming or resetting C07A."""

import re
from pathlib import Path

from scripting import initScripting


CCXML = (r"D:\A-Soft\DevTools\TI\ccs2100\ccs\scripting\examples"
         r"\debugger\mspm0g3507\mspm0g3507.ccxml")
CALIBRATION_ADDRESS = 0x0001FC00
MAP_PATH = Path(__file__).resolve().parents[1] / "Build/default/line_tracker.map"


def load_symbols():
    text = MAP_PATH.read_text(encoding="utf-8", errors="replace")
    names = (
        "g_line_tracker_output", "g_line_tracker_raw_adc",
        "g_line_tracker_distance_m", "g_line_tracker_elapsed_ms",
        "g_line_tracker_tick_overruns", "g_line_tracker_bsp_ready",
        "g_line_tracker_lap_approach_active", "g_line_tracker_mode",
        "g_line_tracker_mode_button_pressed",
        "g_line_tracker_start_button_pressed",
        "g_line_tracker_calibration_state",
        "g_line_tracker_calibration_bits",
        "g_line_tracker_calibration_ok_bits",
        "g_line_tracker_calibration_error",
        "g_line_tracker_calibration_loaded",
    )
    symbols = {}
    for name in names:
        match = re.search(rf"(?m)^([0-9A-Fa-f]{{8}})\s+{name}\s*$", text)
        if match is None:
            raise RuntimeError(f"symbol missing from {MAP_PATH}: {name}")
        symbols[name] = int(match.group(1), 16)
    return symbols


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
    symbols = load_symbols()
    output_address = symbols["g_line_tracker_output"]
    raw_adc_address = symbols["g_line_tracker_raw_adc"]
    ds = initScripting({"timeout": 15000})
    session = None
    try:
        ds.configure(CCXML)
        session = ds.openSession(re.compile(r"cortex", re.IGNORECASE))
        session.target.connect()
        state = u8(session, output_address)
        output_raw = bytes_at(session, output_address, 60)
        raw_bytes = bytes_at(session, raw_adc_address, 16)
        raw_adc = [int.from_bytes(raw_bytes[index:index + 2], "little")
                   for index in range(0, len(raw_bytes), 2)]
        print(f"STATE={state} (0=WAIT, 1=RUN, 2=FAULT)")
        print("OUTPUT_HEX=" + output_raw.hex())
        print(f"BLACK_COUNT={u8(session, output_address + 19)}")
        print(f"LOST_TICKS={u16(session, output_address + 22)}")
        print(f"TICK_OVERRUNS={u32(session, symbols['g_line_tracker_tick_overruns'])}")
        print(f"BSP_READY={u8(session, symbols['g_line_tracker_bsp_ready'])}")
        print(f"ELAPSED_MS={u32(session, symbols['g_line_tracker_elapsed_ms'])}")
        print("APPROACH_ACTIVE=" + str(u8(
            session, symbols["g_line_tracker_lap_approach_active"])))
        print(f"MODE={u8(session, symbols['g_line_tracker_mode'])}")
        print("MODE_BUTTON_PRESSED=" + str(u8(
            session, symbols["g_line_tracker_mode_button_pressed"])))
        print("START_BUTTON_PRESSED=" + str(u8(
            session, symbols["g_line_tracker_start_button_pressed"])))
        print("CAL_STATE=" + str(u8(
            session, symbols["g_line_tracker_calibration_state"])))
        print("CAL_BITS=" + format(u8(
            session, symbols["g_line_tracker_calibration_bits"]), "08b"))
        print("CAL_OK_BITS=" + format(u8(
            session, symbols["g_line_tracker_calibration_ok_bits"]), "08b"))
        print("CAL_ERROR=" + str(u8(
            session, symbols["g_line_tracker_calibration_error"])))
        print("CAL_LOADED=" + str(u8(
            session, symbols["g_line_tracker_calibration_loaded"])))
        print("RAW_ADC=" + ",".join(str(value) for value in raw_adc))
        calibration_bytes = bytes_at(session, CALIBRATION_ADDRESS, 80)
        calibration_words = [
            int.from_bytes(calibration_bytes[index:index + 4], "little")
            for index in range(0, len(calibration_bytes), 4)
        ]
        print("CAL_HEADER=" + ",".join(
            f"0x{word:08X}" for word in calibration_words[:4]))
        print("CAL_WHITE=" + ",".join(
            str(word & 0xFFFF) for word in calibration_words[4:12]))
        print("CAL_BLACK=" + ",".join(
            str(word >> 16) for word in calibration_words[4:12]))
        # Distance is a float; print its raw bits to keep this script
        # independent of target-expression evaluation.
        print("DISTANCE_BITS=0x" + format(u32(
            session, symbols["g_line_tracker_distance_m"]), "08X"))
    finally:
        if session is not None:
            session.target.disconnect()
        ds.shutdown()


if __name__ == "__main__":
    main()
