# 01Studio CanMV K230 bring-up

## Frozen target

- Board: 01Studio **CanMV K230 standard board**, not K230D and not CM-K230.
- Memory: start with the 1 GB SKU as an engineering/cost assumption; accept it
  only after the two-hour peak-memory stress test.
- Camera: the included 70-degree, 24-pin GC2093 on CSI2.
- Storage/accessories: the pictured kit's 16 GB MicroSD, reader, heatsink,
  acrylic base, Type-C cable, and XH-1.25-to-2.54 mm 4-pin cable.
- Firmware: freeze CanMV v1.8 and the matching non-eMMC 01Studio board image:
  `CanMV_K230_01Studio_micropython_v1.8-0-gc2d1f5c_nncase_v2.11.0.img.gz`.
  Verify it with the same release page's `.md5` file. Do not mix the older
  01Studio resource-bundle examples with v1.8 APIs.

Official entry points:

- Board parameters: <https://wiki.01studio.cc/docs/canmv_k230/intro/canmv_k230/>
- 01Studio tutorial index: <https://wiki.01studio.cc/docs/canmv_k230/>
- 01Studio K230 resource pointer:
  <https://github.com/01studio-lab/K230_Resource>
- 01Studio GitHub organisation: <https://github.com/01studio-lab>
- Camera/CSI mapping: <https://wiki.01studio.cc/docs/canmv_k230/machine_vision/camera/>
- UART pins and 4-pin connector: <https://wiki.01studio.cc/docs/canmv_k230/basic_examples/uart/>
- Power inputs: <https://wiki.01studio.cc/docs/canmv_k230/getting_start/power_supply/>
- Wi-Fi STA/AP: <https://wiki.01studio.cc/docs/canmv_k230/network/wifi_connect/>
- CanMV v1.8 release and exact image:
  <https://github.com/kendryte/canmv_k230/releases/tag/v1.8>
- CanMV v1.8 display-free H.264/RTSP example:
  <https://github.com/kendryte/canmv_k230/blob/v1.8/resources/examples/02-Media/rtsp_server.py>
- CanMV v1.8 AI + RTSP example:
  <https://github.com/kendryte/canmv_k230/blob/v1.8/resources/examples/02-Media/ai_rtsp.py>

## Fixed wiring

| K230 standard board | C07A/MSPM0G3507 | Purpose |
|---|---|---|
| UART2 TX / GPIO11 | PB7 / UART1 RX | ball-state frames, 115200 8N1 |
| GND | GND | signal reference |
| UART2 RX / GPIO12 | not connected initially | reserved |
| 3.3 V / 5 V | not connected | never parallel the two boards' supplies |

Read the board-side 4-pin silkscreen as
`GND / 3V3 / IO12-RX2 / IO11-TX2`; never infer a signal from the adapter
wire colour. Disconnect the C07A onboard Bluetooth TX physically before using
PB7.

For bench bring-up, power the K230 through Type-C. For the final vehicle, use
a strain-relieved locking harness from the independent regulator to a red 5 V
and black GND pin on the standard board's 40-pin header. The regulator must
provide a strict 5 V and at least 2 A continuous output; the official nominal
board requirement is 5 V @ 1 A. Never power the board from the UART
connector's 3V3 pin.

## Bring-up order

1. Install the heatsink without allowing it to touch surrounding components.
2. Burn the pinned v1.8 01Studio standard-board image, check its MD5, and run
   the v1.8 camera example on the onboard CSI2 GC2093.
3. Run `ball_tracker.py` with UART2 connected first to a 3.3 V USB-TTL adapter.
   Confirm 115200 8N1 frames, sequence progression, CRC, and valid timeout.
4. Connect the K230 and laptop to a dedicated 2.4 GHz portable router. Reserve
   the K230 DHCP address.
5. Run the display-free official `rtsp_server.py`; open its reported RTSP URL
   (normally `rtsp://<K230_IP>:8554/test`) in VLC and OBS. Confirm the direct
   1280x720 H.264 stream and recording.
6. Do not run `ai_rtsp.py` unchanged: it defaults to an LCD-backed WBC stream,
   while this kit has no LCD. Use it only as an AI+RTSP reference. The final
   pipeline should use one `Sensor(id=2)`, a low-resolution RGB/gray channel
   for ball detection, and a 1280x720 YUV420SP channel bound directly to the
   H.264 VENC/RTSP server.
7. Move UART2 output into the merged loop. Detection/UART has priority; RTSP
   disconnect or restart must not delay state frames beyond 120 ms.
8. Run a two-hour combined test: ball detection + UART + RTSP + OBS recording.

Keep Wi-Fi credentials in an SD-card-local configuration file that is ignored
by version control. Never commit a real SSID or password.

## Camera field-of-view decision

First test whether the included 70-degree GC2093 covers the whole 25 cm beam
with enough ball pixels at a mechanically stable mounting height. If 70
degrees is treated as the horizontal field of view, the geometric lower bound
for covering 25 cm is about 18 cm; verify the real orientation and distortion.
Prefer the official 24-pin, 15 cm extension cable so the camera can be high
while the K230 board stays low. Only if the stock lens cannot meet both full
beam coverage and ball-pixel requirements should an official 100-degree
manual-focus GC2093 be added on CSI0. Do not plug the existing K230D camera
into this board until its FPC pin count and pinout are proven compatible.
