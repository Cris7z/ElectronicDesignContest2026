import os, gc, time
import _thread
import socket
import network
import image
import nncase_runtime as nn
from machine import UART
from machine import FPIOA
from libs.PlatTasks import DetectionApp
from libs.Utils import *
from media.sensor import *
from media.display import *
from media.media import *

# --- Camera and display ---
# The 01Studio LCD reports its own native orientation at runtime.  Do not
# force 800x480 here: that rotates the image on the portrait-mounted panel.
DISPLAY_W, DISPLAY_H = 0, 0
AI_W, AI_H = 1280, 720
# Browser preview is deliberately smaller than the AI frame.  The detector
# still uses the full 1280x720 image, while 480x270 keeps JPEG/network work
# from stealing too much time from inference.
STREAM_W, STREAM_H = 480, 270

# --- Wireless JPEG push: K230 -> PC receiver -> browser ---
# The browser JPEG preview is useful during camera calibration, but it costs
# CPU time and introduces frame-rate jitter.  Keep it off during the final
# ball-control task.  It can be set to True again for troubleshooting.
ENABLE_WIRELESS_PREVIEW = False
# Wireless preview is disabled for the real-time control build.  If it is
# enabled locally for camera calibration, provide the local network settings
# here; do not commit credentials or a private LAN address.
WIFI_SSID = ""
WIFI_PASSWORD = ""
PC_IP = "0.0.0.0"
PC_PORT = 9000
JPEG_QUALITY = 35
STREAM_EVERY_N = 2
SEND_CHUNK = 1400

# --- Real-time position link: K230 UART1 -> STM32 USART1 ---
# 01Studio CanMV K230: GPIO3 / physical pin 8 is UART1_TX.
# The matching STM32F103 pin is PA10 (USART1_RX).  Both boards use 3.3 V IO.
STM32_UART_BAUD = 115200
MIN_BALL_CONFIDENCE = 0.40
STREAM_MAGIC = b"KFRM"

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


class PushHub:
    def __init__(self):
        self.lock = _thread.allocate_lock()
        self.jpeg = None
        self.seq = 0
        self.connected = False

    def publish(self, jpeg):
        self.lock.acquire()
        try:
            self.jpeg = jpeg
            self.seq += 1
        finally:
            self.lock.release()

    def fetch(self, previous_seq):
        self.lock.acquire()
        try:
            if self.jpeg is None or self.seq == previous_seq:
                return None, previous_seq
            return self.jpeg, self.seq
        finally:
            self.lock.release()


push_hub = PushHub()
uart_tx_count = 0
uart_tx_error_count = 0


def init_stm32_uart():
    """Initialise K230 UART1 on IO3/IO4 for the STM32 position link."""
    fpioa = FPIOA()
    fpioa.set_function(3, FPIOA.UART1_TXD)  # GPIO3, physical pin 8, TX1
    fpioa.set_function(4, FPIOA.UART1_RXD)  # GPIO4, physical pin 10, RX1
    return UART(UART.UART1, STM32_UART_BAUD)


def send_ball_position(uart, sequence, position_cm, confidence):
    """Send one checked text position packet to STM32.

    Format: $B,SEQ,POS_MM,CONF*XOR\r\n .  POS is signed millimetres
    relative to O and CONF is 0..100.  The printable frame also makes it easy
    to verify the camera link with a normal USB-TTL serial assistant.
    """
    position_mm = int(position_cm * 10.0 + (0.5 if position_cm >= 0 else -0.5))
    if position_mm > 32767:
        position_mm = 32767
    elif position_mm < -32768:
        position_mm = -32768
    confidence_u8 = int(confidence * 100.0 + 0.5)
    if confidence_u8 < 0:
        confidence_u8 = 0
    elif confidence_u8 > 100:
        confidence_u8 = 100
    body = "B,%d,%d,%d" % (sequence & 0xFF, position_mm, confidence_u8)
    checksum = 0
    for value in body:
        checksum ^= ord(value)
    global uart_tx_count, uart_tx_error_count
    try:
        uart.write("$%s*%02X\r\n" % (body, checksum))
        uart_tx_count += 1
    except Exception as error:
        uart_tx_error_count += 1
        print("STM32_UART_WRITE_ERROR:", error)


def send_all(sock, data):
    view = memoryview(data)
    offset = 0
    retries = 0
    while offset < len(view):
        try:
            sent = sock.send(view[offset:min(offset + SEND_CHUNK, len(view))])
        except OSError as error:
            code = error.args[0] if error.args else 0
            if code in (11, 35) and retries < 400:
                retries += 1
                time.sleep_ms(5)
                continue
            raise
        if not sent:
            raise OSError("preview connection closed")
        offset += sent
        retries = 0


def connect_wifi():
    wlan = network.WLAN(network.STA_IF)
    wlan.active(True)
    if not wlan.isconnected():
        wlan.connect(WIFI_SSID, WIFI_PASSWORD)
        deadline = time.ticks_add(time.ticks_ms(), 15000)
        while not wlan.isconnected() and time.ticks_diff(deadline, time.ticks_ms()) > 0:
            time.sleep_ms(250)
    return wlan.isconnected()


def push_loop():
    last_seq = 0
    while True:
        sock = None
        try:
            if not connect_wifi():
                raise OSError("Wi-Fi unavailable")
            sock = socket.socket()
            sock.connect(socket.getaddrinfo(PC_IP, PC_PORT)[0][-1])
            push_hub.connected = True
            print("WIRELESS_PREVIEW: connected", PC_IP)
            while True:
                jpeg, seq = push_hub.fetch(last_seq)
                if jpeg is None:
                    time.sleep_ms(10)
                    continue
                last_seq = seq
                send_all(sock, STREAM_MAGIC + len(jpeg).to_bytes(4, "little"))
                send_all(sock, jpeg)
        except Exception as error:
            print("WIRELESS_PREVIEW_RETRY:", error)
        finally:
            push_hub.connected = False
            if sock is not None:
                try:
                    sock.close()
                except:
                    pass
        time.sleep_ms(3000)


def start_wireless_preview():
    if ENABLE_WIRELESS_PREVIEW:
        _thread.stack_size(64 * 1024)
        _thread.start_new_thread(push_loop, ())


def encode_jpeg(img):
    try:
        encoded = img.compressed(quality=JPEG_QUALITY)
    except Exception:
        encoded = img.compress(quality=JPEG_QUALITY)
    return bytes(encoded.bytearray())


def map_x(source_x, width):
    return int(source_x * width // AI_W)


def map_y(source_y, height):
    return int(source_y * height // AI_H)


def draw_guides(canvas, width, height, position_cm):
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
        canvas.draw_string_advanced(mark_x - 11, label_y + 3, 14, mark_text, color=color)
    canvas.draw_string_advanced(max(2, plus_11_x - 36), label_y, 18, "+11 cm", color=color)
    canvas.draw_string_advanced(zero_x - 20, label_y, 18, "0 cm", color=color)
    canvas.draw_string_advanced(minus_11_x - 52, label_y, 18, "-11 cm", color=color)
    if position_cm is not None:
        canvas.draw_string_advanced(8, min(height - 34, ry + rh + 6), 26,
                                    "X = %.2f cm" % position_cm, color=(0, 255, 0))
    status = "WiFi PUSH OK" if push_hub.connected else "WiFi PUSH ..."
    status_color = (80, 220, 120) if push_hub.connected else (200, 180, 80)
    canvas.draw_string_advanced(8, height - 24, 16, status, color=status_color)
    canvas.draw_string_advanced(max(180, width - 220), height - 24, 16,
                                "UART_TX:%d E:%d" % (uart_tx_count, uart_tx_error_count),
                                color=(80, 220, 120) if uart_tx_error_count == 0 else (255, 80, 80))


def draw_stream_detection(img, res):
    for i in range(len(res["boxes"])):
        box = res["boxes"][i]
        x1, y1 = map_x(box[0], STREAM_W), map_y(box[1], STREAM_H)
        x2, y2 = map_x(box[2], STREAM_W), map_y(box[3], STREAM_H)
        img.draw_rectangle(x1, y1, x2 - x1, y2 - y1, color=(255, 0, 0), thickness=2)
        img.draw_string_advanced(x1, max(0, y1 - 22), 16,
                                 labels[res["idx"][i]] + " %.2f" % res["scores"][i],
                                 color=(255, 0, 0))


# Camera must start before the Wi-Fi thread on this firmware family.
sensor = Sensor()
sensor.reset()
sensor.set_hmirror(False)
sensor.set_vflip(False)

# This is deliberately the same LCD setup as libs/PipeLine.py.  Its default
# size is board-specific, so it keeps the 01Studio panel in the right
# orientation.
Display.init(Display.ST7701, osd_num=1, to_ide=True)
DISPLAY_W, DISPLAY_H = Display.width(), Display.height()

# Channel 0 is owned by the hardware video layer for the LCD.  Channel 1 is
# an independent RGB565 frame for JPEG streaming, so the tutorial's Wi-Fi
# transport remains available without drawing the LCD image a second time.
sensor.set_framesize(width=DISPLAY_W, height=DISPLAY_H, chn=CAM_CHN_ID_0)
sensor.set_pixformat(Sensor.YUV420SP, chn=CAM_CHN_ID_0)
sensor.set_framesize(width=STREAM_W, height=STREAM_H, chn=CAM_CHN_ID_1)
sensor.set_pixformat(Sensor.RGB565, chn=CAM_CHN_ID_1)
sensor.set_framesize(width=AI_W, height=AI_H, chn=CAM_CHN_ID_2)
sensor.set_pixformat(PIXEL_FORMAT_RGB_888_PLANAR, chn=CAM_CHN_ID_2)

osd_img = image.Image(DISPLAY_W, DISPLAY_H, image.ARGB8888)
MediaManager.init()
sensor_bind_info = sensor.bind_info(x=0, y=0, chn=CAM_CHN_ID_0)
Display.bind_layer(**sensor_bind_info, layer=Display.LAYER_VIDEO1)
sensor.run()

det_app = PipeDetectionApp("video", kmodel_path, labels, model_input_size,
                           anchors, model_type, confidence_threshold,
                           nms_threshold, [AI_W, AI_H], [DISPLAY_W, DISPLAY_H],
                           debug_mode=0)
det_app.config_preprocess()
stm32_uart = init_stm32_uart()
start_wireless_preview()

seq = 0
stream_counter = 0
gc_counter = 0
try:
    while True:
        ai_img = sensor.snapshot(chn=CAM_CHN_ID_2)
        if ai_img.format() == image.RGBP888:
            res = det_app.run(ai_img.to_numpy_ref())
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
            if res["scores"][best] >= MIN_BALL_CONFIDENCE:
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

        # The STM32 receives the physical coordinate, not an image frame.
        # If detection is lost, no packet is sent; STM32 stops after its
        # 100 ms communication watchdog expires.
        if position_cm is not None:
            send_ball_position(stm32_uart, seq, position_cm, position_confidence)

        osd_img.clear()
        det_app.draw_result(osd_img, res)
        draw_guides(osd_img, DISPLAY_W, DISPLAY_H, position_cm)

        # This frame only serves the browser receiver.  The LCD's camera
        # picture is already produced by the hardware-bound channel 0.
        stream_img = sensor.snapshot(chn=CAM_CHN_ID_1)
        draw_stream_detection(stream_img, res)
        draw_guides(stream_img, STREAM_W, STREAM_H, position_cm)
        # Show the annotations once.  Rendering stream_img as well was the
        # source of the duplicated boxes/labels seen on the LCD.
        Display.show_image(osd_img, 0, 0, Display.LAYER_OSD3)
        if push_hub.connected:
            stream_counter += 1
            if stream_counter >= STREAM_EVERY_N:
                stream_counter = 0
                try:
                    push_hub.publish(encode_jpeg(stream_img))
                except Exception as error:
                    print("WIRELESS_JPEG_ERROR:", error)
        del stream_img
        seq = (seq + 1) & 0xFF
        # Full collection is expensive on K230; collecting every frame made
        # both detection and the browser preview visibly stall.
        gc_counter += 1
        if gc_counter >= 30:
            gc_counter = 0
            gc.collect()
except KeyboardInterrupt:
    pass
finally:
    det_app.deinit()
    sensor.stop()
    Display.deinit()
    MediaManager.deinit()
