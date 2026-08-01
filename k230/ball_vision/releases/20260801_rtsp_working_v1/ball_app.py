import os, gc, time
import network
import image
import nncase_runtime as nn
from libs.PlatTasks import DetectionApp
from libs.Utils import *
from media.sensor import *
from media.display import *
from media.media import *
from rtsp_writeback import WritebackRtsp

# --- Camera and display ---
# The 01Studio LCD reports its own native orientation at runtime.  Do not
# force 800x480 here: that rotates the image on the portrait-mounted panel.
DISPLAY_W, DISPLAY_H = 0, 0
AI_W, AI_H = 1280, 720

# --- Local H.264 RTSP transport ---
# CanMV v1.8's minimal ``sys`` module has no ``sys.path``; keep the private
# AP configuration alongside this root-level application.
try:
    from private_rtsp_config import CONFIG as RTSP_CONFIG
except ImportError:
    RTSP_CONFIG = {
        "ap_ssid": "K230-CAMERA",
        "ap_password": "REPLACE_WITH_PRIVATE_WPA2_PASSWORD",
        "rtsp_session": "ball",
        "rtsp_port": 8554,
        "rtsp_bitrate_kbps": 2048,
    }

# Preserve the currently verified LCD detection, guides, and distance display.
SHOW_LCD_DETECTION_BOX = True
# Keep the basic bitmap font because the advanced renderer stalled this panel.
SHOW_LCD_ADVANCED_TEXT = False
# UART is intentionally decoupled from the vision loop.  The publisher owns
# the UART in a background thread; this loop only replaces the latest sample.
ENABLE_ASYNC_UART = True
UART_BAUD = 115200
UART_STATUS_PATH = "/sdcard/uart_test_status.txt"

# --- Detection model ---
root_path = "/sdcard/mp_deployment_source/"
deploy_conf = read_json(root_path + "/deploy_config.json")
kmodel_path = root_path + deploy_conf["kmodel_path"]
labels = deploy_conf["categories"]
confidence_threshold = deploy_conf["confidence_threshold"]
nms_threshold = deploy_conf["nms_threshold"]
model_input_size = deploy_conf["img_size"]
model_type = deploy_conf["model_type"]
anchors = []
if model_type == "AnchorBaseDet":
    anchors = (deploy_conf["anchors"][0] + deploy_conf["anchors"][1]
               + deploy_conf["anchors"][2])

# --- Pipe calibration and AI crop ---
position_calibration = deploy_conf["position_calibration"]
left_reference = position_calibration["left_reference"]
right_reference = position_calibration["right_reference"]
left_cm = left_reference["position_cm"]
right_cm = right_reference["position_cm"]
left_px = left_reference["center_pixel_x"]
right_px = right_reference["center_pixel_x"]
ROI_X, ROI_Y, ROI_W, ROI_H = 0, 150, 1280, 150


class PipeDetectionApp(DetectionApp):
    """Use AI2D to crop the pipe before inference, then restore box coords."""
    def config_preprocess(self, input_image_size=None):
        full_size = input_image_size if input_image_size else self.rgb888p_size
        top, bottom, left, right, _ = center_pad_param(
            [ROI_W, ROI_H], self.model_input_size
        )
        self.ai2d.crop(ROI_X, ROI_Y, ROI_W, ROI_H)
        self.ai2d.pad([0, 0, 0, 0, top, bottom, left, right], 0,
                      [114, 114, 114])
        self.ai2d.resize(nn.interp_method.tf_bilinear,
                         nn.interp_mode.half_pixel)
        self.ai2d.build([1, 3, full_size[1], full_size[0]],
                        [1, 3, self.model_input_size[1], self.model_input_size[0]])

    def postprocess(self, results):
        full_size = self.rgb888p_size
        self.rgb888p_size = [ROI_W, ROI_H]
        res = super().postprocess(results)
        self.rgb888p_size = full_size
        for box in res["boxes"]:
            box[0] += ROI_X
            box[1] += ROI_Y
            box[2] += ROI_X
            box[3] += ROI_Y
        return res


def start_rtsp_access_point():
    if not hasattr(network, "get_dev_list") or "w1" not in network.get_dev_list():
        raise RuntimeError("K230 Wi-Fi AP device w1 is unavailable")
    ap = network.WLAN(network.AP_IF)
    if ap.config(ssid=RTSP_CONFIG["ap_ssid"], key=RTSP_CONFIG["ap_password"]) is False:
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
    print("AP_READY", ip, RTSP_CONFIG["ap_ssid"])
    return ap, ip


def map_x(source_x, width):
    return int(source_x * width // AI_W)


def map_y(source_y, height):
    return int(source_y * height // AI_H)


def draw_guides(canvas, width, height, position_cm, show_text=True,
                show_distance=False, show_basic_text=False):
    # Pipe ROI
    rx, ry = map_x(ROI_X, width), map_y(ROI_Y, height)
    rw, rh = map_x(ROI_W, width), map_y(ROI_H, height)
    canvas.draw_rectangle(rx, ry, rw, rh, color=(0, 255, 255), thickness=2)

    plus_11_x = map_x(right_px, width)
    minus_11_x = map_x(left_px, width)
    zero_x = (plus_11_x + minus_11_x) // 2
    scale_y = max(48, ry - 22)
    label_y = max(4, scale_y - 28)
    tick_bottom = ry - 2
    color = (255, 255, 0)
    canvas.draw_line(plus_11_x, scale_y, minus_11_x, scale_y, color=color, thickness=2)
    for tick_x in (plus_11_x, zero_x, minus_11_x):
        canvas.draw_line(tick_x, scale_y - 5, tick_x, tick_bottom, color=color, thickness=2)
        canvas.draw_circle(tick_x, scale_y, 3, color=color, thickness=1)
    for mark_cm, mark_text in ((6, "+6"), (5, "+5"), (4, "+4"),
                               (-4, "-4"), (-5, "-5"), (-6, "-6")):
        mark_source_x = left_px + (mark_cm - left_cm) * (right_px - left_px) / (right_cm - left_cm)
        mark_x = map_x(mark_source_x, width)
        canvas.draw_line(mark_x, scale_y - 3, mark_x, scale_y + 4, color=color, thickness=1)
        if show_text:
            canvas.draw_string_advanced(mark_x - 11, label_y + 3, 14, mark_text, color=color)
        elif show_basic_text:
            canvas.draw_string(mark_x - 7, label_y + 3, mark_text, color=color, scale=1)
    if show_text:
        canvas.draw_string_advanced(max(2, plus_11_x - 36), label_y, 18, "+11 cm", color=color)
        canvas.draw_string_advanced(zero_x - 20, label_y, 18, "0 cm", color=color)
        canvas.draw_string_advanced(minus_11_x - 52, label_y, 18, "-11 cm", color=color)
        if position_cm is not None:
            canvas.draw_string_advanced(8, min(height - 34, ry + rh + 6), 26,
                                        "X = %.2f cm" % position_cm, color=(0, 255, 0))
        canvas.draw_string_advanced(8, height - 24, 16, "RTSP H264 ON",
                                    color=(80, 220, 120))
    elif show_basic_text:
        canvas.draw_string(max(2, plus_11_x - 24), label_y, "+11", color=color, scale=1)
        canvas.draw_string(zero_x - 4, label_y, "0", color=color, scale=1)
        canvas.draw_string(minus_11_x - 22, label_y, "-11", color=color, scale=1)
        canvas.draw_string(8, height - 16, "RTSP ON",
                           color=(80, 220, 120), scale=1)
    # Basic bitmap text is safe on the 01Studio ARGB OSD.  The advanced font
    # renderer is intentionally not used here because it can stall this panel.
    if show_distance and position_cm is not None:
        canvas.draw_string(8, min(height - 20, ry + rh + 6),
                           "X=%+.2fcm" % position_cm,
                           color=(0, 255, 0), scale=2)


def draw_lcd_detection(canvas, res):
    """Draw only the box on the 01Studio OSD.

    DetectionApp.draw_result() clears the OSD a second time and draws a
    label with the library's aligned display coordinates.  That first OSD
    label draw is where this firmware stalls.  The model already returns
    full AI-frame coordinates, so map them directly to the native LCD.
    """
    for i in range(len(res["boxes"])):
        box = res["boxes"][i]
        x1 = map_x(box[0], DISPLAY_W)
        y1 = map_y(box[1], DISPLAY_H)
        x2 = map_x(box[2], DISPLAY_W)
        y2 = map_y(box[3], DISPLAY_H)
        canvas.draw_rectangle(x1, y1, max(1, x2 - x1), max(1, y2 - y1),
                              color=(255, 0, 0), thickness=2)


def write_uart_status(text):
    try:
        with open(UART_STATUS_PATH, "w") as handle:
            handle.write(text)
            handle.write("\n")
    except Exception:
        pass


# Camera and display must start before the RTSP writeback server.
sensor = Sensor()
sensor.reset()
sensor.set_hmirror(False)
sensor.set_vflip(False)

# This is deliberately the same LCD setup as libs/PipeLine.py.  Its default
# size is board-specific, so it keeps the 01Studio panel in the right
# orientation.
Display.init(Display.ST7701, osd_num=1, to_ide=True)
DISPLAY_W, DISPLAY_H = Display.width(), Display.height()

# Channel 0 is owned by the hardware video layer for the LCD and RTSP
# writeback. Channel 2 remains the independent AI input.
sensor.set_framesize(width=DISPLAY_W, height=DISPLAY_H, chn=CAM_CHN_ID_0)
sensor.set_pixformat(Sensor.YUV420SP, chn=CAM_CHN_ID_0)
sensor.set_framesize(width=AI_W, height=AI_H, chn=CAM_CHN_ID_2)
sensor.set_pixformat(PIXEL_FORMAT_RGB_888_PLANAR, chn=CAM_CHN_ID_2)

osd_img = image.Image(DISPLAY_W, DISPLAY_H, image.ARGB8888)
MediaManager.init()
sensor_bind_info = sensor.bind_info(x=0, y=0, chn=CAM_CHN_ID_0)
Display.bind_layer(**sensor_bind_info, layer=Display.LAYER_VIDEO1)
sensor.run()

_, rtsp_ip = start_rtsp_access_point()
rtsp = WritebackRtsp(
    RTSP_CONFIG.get("rtsp_session", "ball"),
    RTSP_CONFIG.get("rtsp_port", 8554),
    RTSP_CONFIG.get("rtsp_bitrate_kbps", 2048),
)
rtsp.start()
print("RTSP_READY", rtsp.url())

det_app = PipeDetectionApp("video", kmodel_path, labels, model_input_size,
                           anchors, model_type, confidence_threshold,
                           nms_threshold, [AI_W, AI_H], [DISPLAY_W, DISPLAY_H],
                           debug_mode=0)
det_app.config_preprocess()
uart_publisher = None
if ENABLE_ASYNC_UART:
    write_uart_status("before_uart_init")
    try:
        from rct6_uart import Rct6UartPublisher
        uart_publisher = Rct6UartPublisher(UART_BAUD)
        write_uart_status("uart_async_ready")
    except BaseException as error:
        write_uart_status("uart_init_error: %r" % error)

seq = 0
# One-time first-frame breadcrumb.  Channel 0 remains hardware-bound to the
# LCD; this file pinpoints the first AI/OSD call that fails to return.
_first_frame_diagnostic = True
_first_frame_stage_path = "/sdcard/first_frame_stage.txt"


def _mark_first_frame_stage(stage):
    try:
        with open(_first_frame_stage_path, "w") as handle:
            handle.write(stage)
            handle.write("\n")
    except Exception:
        pass


try:
    while True:
        if _first_frame_diagnostic:
            _mark_first_frame_stage("before_ai_snapshot")
        ai_img = sensor.snapshot(chn=CAM_CHN_ID_2)
        if _first_frame_diagnostic:
            _mark_first_frame_stage("after_ai_snapshot")
        if ai_img.format() == image.RGBP888:
            if _first_frame_diagnostic:
                _mark_first_frame_stage("before_detector_run")
            res = det_app.run(ai_img.to_numpy_ref())
            if _first_frame_diagnostic:
                _mark_first_frame_stage("after_detector_run")
        else:
            res = {"boxes": [], "scores": [], "idx": []}
        del ai_img

        position_cm = None
        position_confidence = 0.0
        valid = []
        for i in range(len(res["boxes"])):
            box = res["boxes"][i]
            cx = (box[0] + box[2]) / 2
            cy = (box[1] + box[3]) / 2
            if ROI_X <= cx <= ROI_X + ROI_W and ROI_Y <= cy <= ROI_Y + ROI_H:
                valid.append(i)
        if valid:
            best = valid[0]
            for i in valid[1:]:
                if res["scores"][i] > res["scores"][best]:
                    best = i
            if res["scores"][best] >= 0.5:
                res["boxes"] = [res["boxes"][best]]
                res["scores"] = [res["scores"][best]]
                res["idx"] = [res["idx"][best]]
                box = res["boxes"][0]
                center_px = (box[0] + box[2]) / 2
                position_cm = left_cm + (center_px - left_px) * (right_cm - left_cm) / (right_px - left_px)
                position_cm = max(left_cm, min(right_cm, position_cm))
                position_confidence = res["scores"][0]
            else:
                res = {"boxes": [], "scores": [], "idx": []}
        else:
            res = {"boxes": [], "scores": [], "idx": []}

        if uart_publisher is not None:
            if position_cm is None:
                uart_publisher.publish(seq, None, 0)
            else:
                uart_publisher.publish(seq, int(position_cm * 10.0),
                                       int(position_confidence * 100.0))

        osd_img.clear()
        if _first_frame_diagnostic:
            _mark_first_frame_stage("before_draw_result")
        if SHOW_LCD_DETECTION_BOX:
            draw_lcd_detection(osd_img, res)
        if _first_frame_diagnostic:
            _mark_first_frame_stage("after_draw_result")
        draw_guides(osd_img, DISPLAY_W, DISPLAY_H, position_cm,
                    show_text=SHOW_LCD_ADVANCED_TEXT,
                    show_distance=True,
                    show_basic_text=not SHOW_LCD_ADVANCED_TEXT)
        if _first_frame_diagnostic:
            _mark_first_frame_stage("after_osd_guides")

        if _first_frame_diagnostic:
            _mark_first_frame_stage("before_osd_show")
        Display.show_image(osd_img, 0, 0, Display.LAYER_OSD3)
        if _first_frame_diagnostic:
            _mark_first_frame_stage("first_frame_complete")
            _first_frame_diagnostic = False
        seq = (seq + 1) & 0xFF
        gc.collect()
except KeyboardInterrupt:
    pass
finally:
    det_app.deinit()
    rtsp.stop()
    sensor.stop()
    Display.deinit()
    MediaManager.deinit()
