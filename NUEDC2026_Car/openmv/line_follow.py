# line_follow.py — OpenMV 视觉巡线 (线性回归 get_regression, 最稳方案)
# 输出帧协议与主控 openmv_link.c 配套:
#   0xAA 0x55 | 0x01 | err_i16 angle_i16 flags_u8 rsv_u8 | sum | 0x0D
# err: 线中心偏差 * 1000 (归一化到 [-1000,1000], 左负右正)
# angle: 线倾角 * 10 (度, 车头相对线的角度, 预瞄用)
# flags: bit0=有效 bit1=十字 bit2=看到目标色块
#
# 现场必调: GRAYSCALE_THRESHOLD (按键/IDE 实时看直方图调)
import sensor, time, struct
from pyb import UART, LED

# ---------------- 配置 ----------------
GRAYSCALE_THRESHOLD = [(0, 60)]      # 黑线阈值(灰度), 现场必调!
ROI_MAIN  = (0, 20, 160, 80)          # 主检测区(QQVGA 160x120)
ROI_NEAR  = (0, 80, 160, 40)          # 近端区: 十字检测用
CROSS_PIX_MIN = 2600                  # 近端区黑像素超过此值判十字
UART_NUM, BAUD = 3, 115200            # P4=TX P5=RX

sensor.reset()
sensor.set_pixformat(sensor.GRAYSCALE)
sensor.set_framesize(sensor.QQVGA)    # 160x120, 帧率优先
sensor.skip_frames(time=1500)
sensor.set_auto_gain(False)           # 锁增益/曝光/白平衡: 光照鲁棒的关键!
sensor.set_auto_exposure(False)
sensor.set_auto_whitebal(False)
clock = time.clock()
uart = UART(UART_NUM, BAUD)
led = LED(3)

def send_line(err_norm, angle_deg, valid, cross, target):
    e = int(max(-1.0, min(1.0, err_norm)) * 1000)
    a = int(max(-90.0, min(90.0, angle_deg)) * 10)
    flags = (1 if valid else 0) | (2 if cross else 0) | (4 if target else 0)
    payload = struct.pack("<hhBB", e, a, flags, 0)
    body = bytes([0x01]) + payload
    s = sum(body) & 0xFF
    uart.write(bytes([0xAA, 0x55]) + body + bytes([s, 0x0D]))

while True:
    clock.tick()
    img = sensor.snapshot()
    img.binary(GRAYSCALE_THRESHOLD)   # 二值化后回归更稳

    # 线性回归: 对整个 ROI 内的黑色像素拟合一条线
    line = img.get_regression([(100, 255)], roi=ROI_MAIN, robust=True)

    # 十字检测: 近端 ROI 黑像素统计
    stats = img.get_statistics(roi=ROI_NEAR)
    cross = stats.mean() > (CROSS_PIX_MIN * 255 // (160 * 40)) # 平均亮度近似

    if line and line.magnitude() > 4:   # magnitude 小=散点噪声, 丢弃
        # rho: 线到图像中心的垂距(像素) -> 归一化误差
        # theta: 0..179, 90=竖直线
        rho_err = line.rho() - (img.width() // 2)
        theta = line.theta()            # 0..179, 90 = 竖直
        angle = theta - 90              # 相对竖直的倾角, 左倾负右倾正
        if angle > 90:
            angle -= 180
        send_line(rho_err / (img.width() / 2), -angle, True, cross, False)
        led.on()
    else:
        send_line(0.0, 0.0, False, cross, False)
        led.off()
    # print(clock.fps())  # 调试看帧率, 稳定后注释掉
