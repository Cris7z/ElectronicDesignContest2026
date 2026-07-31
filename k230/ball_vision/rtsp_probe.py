"""Stage 6 transport-only probe: GC2093 preview -> H.264 RTSP.

Run this before deploying measured vision.  It owns no model, calibration,
UART, or actuator interface, so a failed RTSP bring-up cannot be confused with
steel-ball detection or Stage 7 integration.
"""

import gc
import os
import time

import network
from libs.PipeLine import PipeLine

from rtsp_writeback import WritebackRtsp

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
    if hasattr(network, "set_default_dev") and network.set_default_dev("w1") is False:
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


def _draw_status(pipeline, ip, fps):
    canvas = pipeline.osd_img
    canvas.clear()
    canvas.draw_string_advanced(8, 8, 24, "RTSP %s:8554/ball" % ip,
                                color=(80, 255, 80, 255))
    canvas.draw_string_advanced(8, 40, 20, "H264 %.1f FPS" % fps,
                                color=(180, 220, 255, 255))


def main():
    config = dict(CONFIG)
    if config["ap_password"].startswith("REPLACE_"):
        raise RuntimeError("create private_rtsp_config.py before running RTSP probe")

    pipeline = PipeLine(
        rgb888p_size=config["rgb888p_size"],
        display_mode=config["display_mode"],
        display_size=config["display_size"],
    )
    rtsp = WritebackRtsp(
        config["rtsp_session"], config["rtsp_port"], config["rtsp_bitrate_kbps"]
    )
    clock = time.clock()
    try:
        pipeline.create(sensor_id=config["sensor_id"], to_ide=False)
        _, ip = _start_access_point(config)
        rtsp.start()
        print("RTSP_READY", "rtsp://%s:%d/%s" % (
            ip, config["rtsp_port"], config["rtsp_session"]
        ))
        while True:
            clock.tick()
            frame = pipeline.get_frame()
            del frame
            _draw_status(pipeline, ip, clock.fps())
            pipeline.show_image()
            gc.collect()
            os.exitpoint()
    except KeyboardInterrupt:
        print("RTSP_PROBE_STOP_REQUESTED")
    finally:
        rtsp.stop()
        pipeline.destroy()


if __name__ == "__main__":
    main()
