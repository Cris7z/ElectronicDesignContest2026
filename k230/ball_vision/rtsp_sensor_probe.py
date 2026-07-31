"""Stage 6 primary RTSP probe: network stability and full camera coverage.

No detection model, calibration or Display writeback runs here.  The program
uses the direct-sensor implementation adapted from CanMV's ``rtsp_server.py``
so the host's RTSP evidence isolates AP and encoder behavior.
"""

import os
import sys
import time

import network

# CanMV VS Code's “run remote file” executes through the REPL rather than as
# ``python /path/file.py``.  Keep this sibling import deterministic in either
# launch mode without changing the process-wide application path elsewhere.
try:
    _SOURCE_DIR = os.path.dirname(__file__)
except NameError:
    _SOURCE_DIR = "/sdcard/ball_vision_rtsp_probe"
if _SOURCE_DIR and _SOURCE_DIR not in sys.path:
    sys.path.append(_SOURCE_DIR)

from rtsp_sensor_transport import SensorRtspServer

try:
    from private_rtsp_config import CONFIG
except ImportError:
    from rtsp_probe_config_example import CONFIG


def _start_access_point(config):
    if not hasattr(network, "get_dev_list") or "w1" not in network.get_dev_list():
        raise RuntimeError("K230 Wi-Fi AP device w1 is unavailable")
    ap = network.WLAN(network.AP_IF)
    if ap.config(ssid=config["ap_ssid"], key=config["ap_password"]) is False:
        raise RuntimeError("K230 Wi-Fi AP configuration failed")
    if not hasattr(network, "set_default_dev") or network.set_default_dev("w1") is False:
        raise RuntimeError("K230 Wi-Fi AP default-device selection failed")
    deadline = time.ticks_add(time.ticks_ms(), 15000)
    while ap.ifconfig()[0] == "0.0.0.0" and time.ticks_diff(deadline, time.ticks_ms()) > 0:
        os.exitpoint()
        time.sleep_ms(100)
    ip = ap.ifconfig()[0]
    if ip == "0.0.0.0":
        raise RuntimeError("K230 Wi-Fi AP did not obtain an address")
    print("AP_READY", ip, config["ap_ssid"])
    return ap, ip


def main():
    config = dict(CONFIG)
    if config["ap_password"].startswith("REPLACE_"):
        raise RuntimeError("create private_rtsp_config.py before running RTSP probe")

    _, ip = _start_access_point(config)
    server = SensorRtspServer(
        session_name=config["rtsp_session"],
        port=config["rtsp_port"],
        width=config["rgb888p_size"][0],
        height=config["rgb888p_size"][1],
        bitrate_kbps=config["rtsp_bitrate_kbps"],
        gop_len=config.get("rtsp_gop", 30),
        sensor_id=config["sensor_id"],
    )
    try:
        server.start()
        print("RTSP_READY", server.get_rtsp_url(ip))
        last_report = time.ticks_ms()
        while True:
            os.exitpoint()
            now = time.ticks_ms()
            if time.ticks_diff(now, last_report) >= 5000:
                print("RTSP_STATUS", server.get_stats())
                last_report = now
            time.sleep_ms(100)
    except KeyboardInterrupt:
        print("RTSP_SENSOR_PROBE_STOP_REQUESTED")
    finally:
        server.stop()


if __name__ == "__main__":
    main()
