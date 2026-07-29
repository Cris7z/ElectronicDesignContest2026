# target_detect.py — 目标识别示例: 色块 + NCC 模板匹配(数字/图形)
# 发送帧: 0xAA 0x55 | 0x02 | class_u8 cx_i16 cy_i16 rsv_u8 | sum | 0x0D
# 用法:
#   模式A 色块: 识别指定颜色目标(如红色标志物), 直接可用
#   模式B 模板: 把赛题目标(数字1~8等)拍成 template/*.pgm 放到 SD 卡,
#              印刷体数字识别 NCC 够用; 光照差异大时改用 MaixHub 训练 CNN
import sensor, time, image, struct
from pyb import UART

MODE = "blob"                      # "blob" 或 "template"
RED_THRESHOLD = [(30, 70, 30, 90, 10, 70)]   # LAB, 现场必调
TEMPLATES = ["/1.pgm", "/2.pgm", "/3.pgm"]   # class_id = 下标+1
MATCH_THRESH = 0.70

sensor.reset()
sensor.set_pixformat(sensor.RGB565 if MODE == "blob" else sensor.GRAYSCALE)
sensor.set_framesize(sensor.QQVGA)
sensor.skip_frames(time=1500)
sensor.set_auto_gain(False)
sensor.set_auto_whitebal(False)
uart = UART(3, 115200)
clock = time.clock()

templates = []
if MODE == "template":
    for p in TEMPLATES:
        try:
            templates.append(image.Image(p))
        except OSError:
            templates.append(None)

def send_target(cls, cx, cy):
    payload = struct.pack("<Bhh B", cls, cx, cy, 0)
    body = bytes([0x02]) + payload
    s = sum(body) & 0xFF
    uart.write(bytes([0xAA, 0x55]) + body + bytes([s, 0x0D]))

while True:
    clock.tick()
    img = sensor.snapshot()

    if MODE == "blob":
        blobs = img.find_blobs(RED_THRESHOLD, pixels_threshold=80,
                               area_threshold=100, merge=True)
        if blobs:
            b = max(blobs, key=lambda b: b.pixels())
            img.draw_rectangle(b.rect())
            send_target(1, b.cx() - img.width()//2, b.cy() - img.height()//2)
        else:
            send_target(0, 0, 0)
    else:
        best_cls, best_r, best_xy = 0, MATCH_THRESH, (0, 0)
        for i, t in enumerate(templates):
            if t is None:
                continue
            r = img.find_template(t, best_r, step=4,
                                  search=image.SEARCH_EX)
            if r:
                best_cls = i + 1
                best_xy = (r[0] + r[2]//2 - img.width()//2,
                           r[1] + r[3]//2 - img.height()//2)
        send_target(best_cls, best_xy[0], best_xy[1])
