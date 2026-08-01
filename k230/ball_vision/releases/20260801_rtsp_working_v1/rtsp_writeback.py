"""Configurable H.264 writeback RTSP server for CanMV K230 v1.8.

Derived from the public v1.8 WBCRtsp example, but keeps the competition
session name and bitrate in Stage 6 configuration instead of hard-coding
`/test` and a hidden default.
"""

import _thread
import os
import time

import multimedia as mm
from _media import Display
from media.vencoder import ALIGN_UP, Encoder, ChnAttrStr, StreamData


class WritebackRtsp:
    def __init__(self, session_name="ball", port=8554, bitrate_kbps=2048):
        self.session_name = session_name
        self.port = int(port)
        self.bitrate_kbps = int(bitrate_kbps)
        self._running = False
        self._thread_done = True
        self._encoder = None
        self._server = None
        # CanMV v1.8 on this board uses the explicit encoder channel API.
        self._channel = 0

    def start(self):
        if self._running:
            return
        if not Display.inited():
            raise RuntimeError("Display must be initialized before RTSP")

        width = ALIGN_UP(Display.width(), 16)
        height = Display.height()
        self._encoder = Encoder()
        self._encoder.SetOutBufs(self._channel, 16, width, height)
        attributes = ChnAttrStr(
            self._encoder.PAYLOAD_TYPE_H264,
            self._encoder.H264_PROFILE_MAIN,
            width,
            height,
            bit_rate=self.bitrate_kbps,
        )
        self._encoder.Create(self._channel, attributes)
        self._server = mm.rtsp_server()
        self._server.rtspserver_init(self.port)
        self._server.rtspserver_createsession(
            self.session_name, mm.multi_media_type.media_h264, False
        )
        self._server.rtspserver_start()
        self._encoder.Start(self._channel)
        if not Display.writeback(True):
            self.stop()
            raise RuntimeError("Display writeback start failed")
        self._running = True
        self._thread_done = False
        _thread.start_new_thread(self._pump, ())
        print("RTSP_READY", self.url())

    def url(self):
        if self._server is None:
            return None
        return self._server.rtspserver_getrtspurl(self.session_name)

    def _pump(self):
        try:
            while self._running:
                os.exitpoint()
                frame = Display.writeback_dump(100)
                if frame:
                    self._send_frame(frame)
                time.sleep_ms(10)
        except BaseException as error:
            print("RTSP_PUMP_ERROR", repr(error))
        finally:
            self._thread_done = True

    def _send_frame(self, frame):
        if not self._running:
            return
        if self._encoder.SendFrame(self._channel, frame, timeout=-1) != 0:
            return
        stream = StreamData()
        if self._encoder.GetStream(self._channel, stream, timeout=-1) != 0:
            return
        try:
            for index in range(stream.pack_cnt):
                self._server.rtspserver_sendvideodata_byphyaddr(
                    self.session_name,
                    stream.phy_addr[index],
                    stream.data_size[index],
                    1000,
                )
        finally:
            self._encoder.ReleaseStream(self._channel, stream)

    def stop(self):
        self._running = False
        deadline = time.ticks_add(time.ticks_ms(), 1500)
        while not self._thread_done and time.ticks_diff(deadline, time.ticks_ms()) > 0:
            time.sleep_ms(20)
        try:
            if Display.inited():
                Display.writeback(False)
        except BaseException as error:
            print("RTSP_WRITEBACK_STOP_ERROR", repr(error))
        if self._encoder is not None:
            try:
                self._encoder.Stop(self._channel)
                self._encoder.Destroy(self._channel)
            except BaseException as error:
                print("RTSP_ENCODER_STOP_ERROR", repr(error))
        if self._server is not None:
            try:
                self._server.rtspserver_stop()
                self._server.rtspserver_deinit()
            except BaseException as error:
                print("RTSP_SERVER_STOP_ERROR", repr(error))
        self._encoder = None
        self._server = None
