"""Copy to private_rtsp_config.py and set a private WPA2 password locally.

This probe intentionally has no model or calibration dependency.  It proves
the K230 AP + H.264 RTSP transport before the measured-vision app is deployed.
"""

CONFIG = {
    "sensor_id": 0,
    "rgb888p_size": [1280, 720],
    "display_mode": "virt",
    "display_size": [1280, 720],
    "ap_ssid": "K230-BALL-G6",
    "ap_password": "REPLACE_WITH_PRIVATE_WPA2_PASSWORD",
    "rtsp_session": "ball",
    "rtsp_port": 8554,
    "rtsp_bitrate_kbps": 2048,
    "rtsp_gop": 30,
}
