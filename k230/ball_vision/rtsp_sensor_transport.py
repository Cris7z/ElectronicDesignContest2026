"""Direct-sensor RTSP transport for CanMV K230 v1.8.

This is a narrow, H.264-configurable adaptation of CanMV's official
``examples/02-Media/rtsp_server.py``.  It binds a YUV420SP camera channel
directly to the hardware encoder, so the first Stage 6 validation measures
network stability and complete camera coverage without Display writeback or
the ball detector in the path.
"""

import _thread
import os
import time
import uctypes

import multimedia as mm
from media.media import MediaManager
from media.sensor import Sensor
# CanMV's v1.8 public RTSP example exports these helpers from
# ``media.vencoder``.  They are not importable from the ``mpp`` package.
from media.vencoder import (
    ALIGN_UP,
    VENC_DEV_ID,
    VIDEO_ENCODE_MOD_ID,
    ChnAttrStr,
    Encoder,
    StreamData,
)


class SensorRtspServer:
    """One direct camera-to-H.264 RTSP session, with deterministic teardown."""

    def __init__(self, session_name="ball", port=8554, width=1280, height=720,
                 bitrate_kbps=2048, gop_len=30, sensor_id=0,
                 src_frame_rate=60, dst_frame_rate=60):
        if bitrate_kbps < 100 or bitrate_kbps > 20000:
            raise ValueError("bitrate_kbps must be 100..20000")
        self.session_name = session_name
        self.port = int(port)
        self.width = ALIGN_UP(int(width), 16)
        self.height = int(height)
        self.bitrate_kbps = int(bitrate_kbps)
        self.gop_len = int(gop_len)
        self.src_frame_rate = int(src_frame_rate)
        self.dst_frame_rate = int(dst_frame_rate)
        self.sensor_id = int(sensor_id)
        self.server = mm.rtsp_server()
        self.running = False
        self.thread_done = True
        self.sensor = None
        self.encoder = None
        # The 01Studio v1.8 image exposes the encoder through the legacy
        # explicit-channel API (rather than ``Encoder.chn``).
        self.encoder_chn = 0
        self.link = None
        self.frame_count = 0
        self.byte_count = 0

    def start(self):
        if self.running:
            return
        if self.server.rtspserver_init(self.port) != 0:
            raise RuntimeError("RTSP bind failed on port %d" % self.port)
        try:
            if self.server.rtspserver_createsession(
                    self.session_name, mm.multi_media_type.media_h264, False) != 0:
                raise RuntimeError("RTSP session creation failed")
            self._init_camera_encoder_link()
            self.server.rtspserver_start()
            self.encoder.Start(self.encoder_chn)
            self.sensor.run()
        except BaseException:
            self._teardown_media()
            self.server.rtspserver_deinit()
            raise
        self.running = True
        self.thread_done = False
        _thread.start_new_thread(self._stream_loop, ())

    def _init_camera_encoder_link(self):
        # CanMV v1.8's public ``rtsp_server.py`` constructs the active
        # camera with ``Sensor()``.  The 01Studio 1G image rejects the
        # otherwise plausible ``id=0`` keyword even though GC2093 is the
        # discovered physical sensor.
        self.sensor = Sensor()
        self.sensor.reset()
        self.sensor.set_framesize(width=self.width, height=self.height, alignment=12)
        self.sensor.set_pixformat(Sensor.YUV420SP)
        self.encoder = Encoder()
        self.encoder.SetOutBufs(self.encoder_chn, 8, self.width, self.height)
        attributes = ChnAttrStr(
            self.encoder.PAYLOAD_TYPE_H264,
            self.encoder.H264_PROFILE_MAIN,
            self.width,
            self.height,
            bit_rate=self.bitrate_kbps,
            gopLen=self.gop_len,
            src_frame_rate=self.src_frame_rate,
            dst_frame_rate=self.dst_frame_rate,
        )
        self.encoder.Create(self.encoder_chn, attributes)
        self.link = MediaManager.link(
            self.sensor.bind_info()["src"],
            (VIDEO_ENCODE_MOD_ID, VENC_DEV_ID, self.encoder_chn),
        )

    def _stream_loop(self):
        stream = StreamData()
        try:
            while self.running:
                os.exitpoint()
                if self.encoder.GetStream(self.encoder_chn, stream) != 0:
                    continue
                try:
                    for index in range(stream.pack_cnt):
                        size = stream.data_size[index]
                        payload = bytes(uctypes.bytearray_at(stream.data[index], size))
                        self.server.rtspserver_sendvideodata(
                            self.session_name, payload, size, 1000
                        )
                        self.byte_count += size
                    self.frame_count += 1
                finally:
                    self.encoder.ReleaseStream(self.encoder_chn, stream)
        except BaseException as error:
            print("RTSP_STREAM_ERROR", repr(error))
        finally:
            self.thread_done = True

    def get_rtsp_url(self, host):
        return "rtsp://%s:%d/%s" % (host, self.port, self.session_name)

    def get_stats(self):
        return {"frames": self.frame_count, "bytes": self.byte_count}

    def stop(self):
        if not self.running:
            return
        self.running = False
        deadline = time.ticks_add(time.ticks_ms(), 1500)
        while not self.thread_done and time.ticks_diff(deadline, time.ticks_ms()) > 0:
            time.sleep_ms(20)
        self._teardown_media()
        try:
            self.server.rtspserver_stop()
        finally:
            self.server.rtspserver_deinit()

    def _teardown_media(self):
        if self.sensor is not None:
            try:
                self.sensor.stop()
            except BaseException:
                pass
        if self.link is not None:
            try:
                self.link.destroy()
            except BaseException:
                pass
        if self.encoder is not None:
            try:
                self.encoder.Stop(self.encoder_chn)
                self.encoder.Destroy(self.encoder_chn)
            except BaseException:
                pass
        self.sensor = None
        self.link = None
        self.encoder = None
