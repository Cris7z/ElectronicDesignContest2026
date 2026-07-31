"""Copy to private_config.py and replace only the local values.

This example is safe to commit.  Do not put AP credentials, models or actual
calibration parameters in Git.
"""

CONFIG = {
    "sensor_id": 2,
    "ai_size": [1280, 720],
    "display_mode": "virt",
    "display_size": [1280, 720],
    "roi": [0, 150, 1280, 150],
    "model_root": "/sdcard/ball_vision/models/",
    "deploy_config": "/sdcard/ball_vision/models/deploy_config.json",
    "calibration_path": "/sdcard/ball_vision/calibration.json",
    "model_sha256": "REPLACE_WITH_ACTIVE_MODEL_SHA256",
    "ap_ssid": "K230-BALL",
    "ap_password": "CHANGE_THIS_ON_DEVICE",
    "ap_channel": 6,
    "rtsp_session": "ball",
    "rtsp_port": 8554,
    "rtsp_bitrate_kbps": 2048,
    "min_confidence": 0.50,
    "min_box_px": 4,
    "max_box_px": 240,
    "max_speed_mm_s": 300,
    "temporal_margin_mm": 8,
    "diagnostic_jsonl": "/sdcard/ball_vision/logs/vision.jsonl",
}
