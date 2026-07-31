"""Stage 6 K230 ball measurement + annotated RTSP entry point.

Requires CanMV K230 v1.8 on the physical 01Studio board.  It deliberately has
no UART import or actuator API: formal inter-board communication is Stage 7.
"""

import gc
import os
import time
import ujson

import image
import network
import nncase_runtime as nn
from libs.PlatTasks import DetectionApp
from libs.Utils import center_pad_param, read_json
from libs.PipeLine import PipeLine

from rtsp_writeback import WritebackRtsp
from vision_contract import MeasurementTracker, PiecewiseCalibration

try:
    from private_config import CONFIG as PRIVATE_CONFIG
except ImportError:
    from config_example import CONFIG as PRIVATE_CONFIG


def _config():
    return dict(PRIVATE_CONFIG)


class PipeDetectionApp(DetectionApp):
    """Kmodel detector with a fixed full-resolution pipe ROI crop."""

    def __init__(self, config, deploy_config, labels, anchors, model_type):
        self.roi = config["roi"]
        self.full_size = config["ai_size"]
        self.deploy_config = deploy_config
        model_root = config["model_root"].rstrip("/") + "/"
        kmodel_path = deploy_config["kmodel_path"]
        if not kmodel_path.startswith("/"):
            kmodel_path = model_root + kmodel_path
        super().__init__(
            "video",
            kmodel_path,
            labels,
            deploy_config["img_size"],
            anchors,
            model_type,
            deploy_config["confidence_threshold"],
            deploy_config["nms_threshold"],
            self.full_size,
            config["display_size"],
            debug_mode=0,
        )

    def config_preprocess(self, input_image_size=None):
        full_size = input_image_size if input_image_size else self.full_size
        roi_x, roi_y, roi_w, roi_h = self.roi
        top, bottom, left, right, _ = center_pad_param(
            [roi_w, roi_h], self.model_input_size
        )
        self.ai2d.crop(roi_x, roi_y, roi_w, roi_h)
        self.ai2d.pad(
            [0, 0, 0, 0, top, bottom, left, right], 0, [114, 114, 114]
        )
        self.ai2d.resize(nn.interp_method.tf_bilinear, nn.interp_mode.half_pixel)
        self.ai2d.build(
            [1, 3, full_size[1], full_size[0]],
            [1, 3, self.model_input_size[1], self.model_input_size[0]],
        )

    def postprocess(self, results):
        full_size = self.rgb888p_size
        roi_x, roi_y, roi_w, roi_h = self.roi
        self.rgb888p_size = [roi_w, roi_h]
        result = super().postprocess(results)
        self.rgb888p_size = full_size
        for box in result["boxes"]:
            box[0] += roi_x
            box[1] += roi_y
            box[2] += roi_x
            box[3] += roi_y
        return result


def _load_calibration(path):
    with open(path, "r") as handle:
        return PiecewiseCalibration.from_dict(ujson.load(handle))


def _start_access_point(config):
    if not hasattr(network, "get_dev_list") or "w1" not in network.get_dev_list():
        raise RuntimeError("K230 Wi-Fi AP device w1 is unavailable")
    ap = network.WLAN(network.AP_IF)
    if ap.config(ssid=config["ap_ssid"], key=config["ap_password"]) is False:
        raise RuntimeError("K230 Wi-Fi AP configuration failed")
    if hasattr(network, "set_default_dev") and network.set_default_dev("w1") is False:
        raise RuntimeError("K230 Wi-Fi AP default-device selection failed")
    deadline = time.ticks_add(time.ticks_ms(), 5000)
    while ap.ifconfig()[0] == "0.0.0.0" and time.ticks_diff(deadline, time.ticks_ms()) > 0:
        time.sleep_ms(100)
    if ap.ifconfig()[0] == "0.0.0.0":
        raise RuntimeError("K230 Wi-Fi AP did not obtain an address")
    print("AP_READY", ap.ifconfig()[0], config["ap_ssid"])
    return ap


def _anchors(deploy_config):
    if deploy_config["model_type"] != "AnchorBaseDet":
        return []
    return deploy_config["anchors"][0] + deploy_config["anchors"][1] + deploy_config["anchors"][2]


def _best_candidate(result):
    best = None
    for index, box in enumerate(result["boxes"]):
        confidence = float(result["scores"][index])
        candidate = {
            "pixel_x": (box[0] + box[2]) / 2.0,
            "pixel_y": (box[1] + box[3]) / 2.0,
            "box_w": box[2] - box[0],
            "box_h": box[3] - box[1],
            "confidence": confidence,
            "box": box,
        }
        if best is None or candidate["confidence"] > best["confidence"]:
            best = candidate
    return best


def _map_axis(value, source, destination):
    return int(value * destination // source)


def _draw_overlay(pipeline, config, candidate, sample, fps):
    canvas = pipeline.osd_img
    canvas.clear()
    display_w, display_h = config["display_size"]
    source_w, source_h = config["ai_size"]
    roi_x, roi_y, roi_w, roi_h = config["roi"]
    canvas.draw_rectangle(
        _map_axis(roi_x, source_w, display_w),
        _map_axis(roi_y, source_h, display_h),
        _map_axis(roi_w, source_w, display_w),
        _map_axis(roi_h, source_h, display_h),
        color=(0, 220, 255, 255),
        thickness=2,
    )
    if candidate is not None:
        box = candidate["box"]
        x1 = _map_axis(box[0], source_w, display_w)
        y1 = _map_axis(box[1], source_h, display_h)
        x2 = _map_axis(box[2], source_w, display_w)
        y2 = _map_axis(box[3], source_h, display_h)
        canvas.draw_rectangle(x1, y1, x2 - x1, y2 - y1, color=(255, 80, 80, 255), thickness=2)
        canvas.draw_cross((x1 + x2) // 2, (y1 + y2) // 2, color=(80, 255, 80, 255), thickness=2)
    x_text = "--" if sample.x_mm is None else "%+d mm" % sample.x_mm
    canvas.draw_string_advanced(8, 8, 24, "X %s" % x_text, color=(0, 255, 0, 255))
    canvas.draw_string_advanced(8, 38, 18, sample.status, color=(255, 230, 80, 255))
    canvas.draw_string_advanced(
        8, 62, 18, "Q %d  SEQ %u" % (sample.quality, sample.seq), color=(180, 220, 255, 255)
    )
    canvas.draw_string_advanced(8, 86, 18, "FPS %.1f" % fps, color=(180, 220, 255, 255))


def _append_jsonl(handle, sample, frame_ms, every_n):
    if handle is None or sample.seq % every_n != 0:
        return
    payload = sample.as_dict()
    payload["frame_ms"] = int(frame_ms)
    handle.write(ujson.dumps(payload) + "\n")
    handle.flush()


def _open_diagnostic_log(path):
    try:
        return open(path, "a")
    except BaseException as error:
        print("DIAGNOSTIC_LOG_DISABLED", repr(error))
        return None


def main():
    config = _config()
    calibration = _load_calibration(config["calibration_path"])
    if calibration.orientation.get("positive_direction") != "hinge_to_actuator":
        raise ValueError("calibration coordinate direction must be hinge_to_actuator")
    if calibration.roi is not None and list(calibration.roi) != list(config["roi"]):
        raise ValueError("calibration ROI does not match the active detector ROI")
    if calibration.model_sha256 != config["model_sha256"]:
        raise ValueError("calibration model SHA-256 does not match the active model")

    deploy_config = read_json(config["deploy_config"])
    detector = PipeDetectionApp(
        config,
        deploy_config,
        deploy_config["categories"],
        _anchors(deploy_config),
        deploy_config["model_type"],
    )
    detector.config_preprocess()
    tracker = MeasurementTracker(
        calibration,
        config["roi"],
        min_confidence=config["min_confidence"],
        min_box_px=config["min_box_px"],
        max_box_px=config["max_box_px"],
        max_speed_mm_s=config["max_speed_mm_s"],
        temporal_margin_mm=config["temporal_margin_mm"],
    )
    pipeline = PipeLine(
        rgb888p_size=config["ai_size"],
        display_mode=config["display_mode"],
        display_size=config["display_size"],
    )
    pipeline.create(sensor_id=config["sensor_id"], to_ide=False)
    log_handle = _open_diagnostic_log(config["diagnostic_jsonl"])
    rtsp = WritebackRtsp(
        config["rtsp_session"], config["rtsp_port"], config["rtsp_bitrate_kbps"]
    )
    clock = time.clock()
    frame_count = 0
    try:
        _start_access_point(config)
        rtsp.start()
        while True:
            frame_start = time.ticks_ms()
            clock.tick()
            frame = pipeline.get_frame()
            capture_ms = time.ticks_ms()
            result = detector.run(frame.to_numpy_ref()) if frame.format() == image.RGBP888 else {"boxes": [], "scores": [], "idx": []}
            del frame
            candidate = _best_candidate(result)
            sample = tracker.process(capture_ms, candidate)
            _draw_overlay(pipeline, config, candidate, sample, clock.fps())
            pipeline.show_image()
            _append_jsonl(log_handle, sample, time.ticks_diff(time.ticks_ms(), frame_start), 10)
            frame_count += 1
            if frame_count % 30 == 0:
                print("VISION", ujson.dumps(sample.as_dict()), "fps=%.1f" % clock.fps())
                gc.collect()
            os.exitpoint()
    except KeyboardInterrupt:
        print("VISION_STOP_REQUESTED")
    except BaseException as error:
        print("VISION_FAULT", repr(error))
        raise
    finally:
        if log_handle is not None:
            log_handle.close()
        try:
            detector.deinit()
        except BaseException:
            pass
        rtsp.stop()
        pipeline.destroy()


if __name__ == "__main__":
    main()
