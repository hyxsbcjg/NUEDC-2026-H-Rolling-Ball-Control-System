"""Steel-ball detection with hardware-bound LCD and H.264 RTSP streaming.

The camera is opened only once.  CH0 is bound directly to the ST7701, CH1 is
the small grayscale detection image, and CH2 is bound directly to hardware
VENC.  The RTSP worker sends VENC physical packs directly when the frozen
firmware exposes that API; it never takes a second camera snapshot or converts
a video frame in Python.

Target firmware:
    CanMV K230 v1.8-0-gc2d1f5c, LCKFB standard board image.
"""

import _thread
import gc
import image
import os
import sys
import time
import uctypes

import multimedia as mm
import network
from machine import FPIOA, TOUCH, UART, WDT
from media.sensor import *
from media.display import *
from media.media import (
    ALIGN_UP,
    MediaManager,
    VIDEO_ENCODE_MOD_ID,
    VENC_DEV_ID,
)
from media.vencoder import ChnAttrStr, Encoder, StreamData, VENC_CHN_ID_0


APP_VERSION = "steel-ball-rtsp-1.1.0"

# Camera and display settings. Keep the current 1080p60 request as the default.
# Enable 720p90 only after the 60 Hz media/CV path passes the board tests.
SENSOR_ID = 2
STABLE_SENSOR_MODE = (1920, 1080, 60)
FAST_SENSOR_MODE = (1280, 720, 90)
PREFER_720P90 = False

DISPLAY_WIDTH = 800
DISPLAY_HEIGHT = 480
DETECT_WIDTH = 400
DETECT_HEIGHT = 240
DISPLAY_TO_IDE = True
IDE_JPEG_QUALITY = 60

DISPLAY_CHN = CAM_CHN_ID_0
DETECT_CHN = CAM_CHN_ID_1
STREAM_CHN = CAM_CHN_ID_2

# Wi-Fi AP and RTSP settings retained from Project/图传功能/main.py.
ENABLE_RTSP = False
AP_SSID = "CHANGE_ME"
AP_PASSWORD = None
AP_CHANNEL = 6
AP_IP = "192.168.169.1"
AP_NETMASK = "255.255.255.0"
AP_DEVICE = "w1"
AP_DEVICE_WAIT_MS = 10000
AP_DEVICE_POLL_MS = 100

RTSP_PORT = 8554
RTSP_SESSION = "live"
RTSP_USE_PHYSICAL_PACKS = True

# CH2 is throttled before VENC.  Encoding is done by hardware and therefore
# does not add a full-resolution Python snapshot to the detection loop.
VIDEO_WIDTH = ALIGN_UP(1280, 16)
VIDEO_HEIGHT = 720
VIDEO_FPS = 20
VIDEO_BITRATE_KBIT = 2000
VIDEO_GOP = 10
VENC_CHANNEL = VENC_CHN_ID_0
VENC_OUTPUT_BUFFERS = 8

FRAME_PERIOD_MS = 1000 // VIDEO_FPS
VENC_PTS_PER_RTSP_MS = 1000
STREAM_GET_TIMEOUT_MS = 1000
STREAM_STALL_TIMEOUT_MS = 4000
WATCHDOG_TIMEOUT_SECONDS = 10

# SD-card logging is disabled by default so log writes cannot stall detection.
# Serial boot/error logs remain enabled.  Set this True only for diagnosis.
RTSP_LOG_TO_SD = False
RTSP_LOG_PATH = "/sdcard/k230-camera.log"

# UART2 on the LushanPi GH1.25 socket marked "2":
# GPIO11/TX -> TC264 RX, GPIO12/RX <- TC264 TX, and GND -> GND.
UART2_TX_GPIO = 11
UART2_RX_GPIO = 12
UART2_BAUDRATE = 115200
UART2_FLAG_VALID = 0x00
UART2_FLAG_NO_HISTORY = 0x01
UART2_FRAME_SIZE = 5
_uart2_tx_frame = bytearray((0x40, 0x00, 0x00, 0x00, 0x50))

# Touch marker settings. The latest clicked point stays visible until the next
# click. TOUCH(0) uses the system touch device whose coordinates match the LCD.
TOUCH_CROSS_SIZE = 15
TOUCH_CROSS_THICKNESS = 3
TOUCH_TEXT_SIZE = 24
TOUCH_TEXT_GAP = 4
TOUCH_COLOR = (255, 0, 255)
TOUCH_OSD_WIDTH = 160
TOUCH_OSD_HEIGHT = 64

# A new touch click sends Touch_X as a big-endian uint16 in three consecutive
# copies of the same fixed 5-byte frame.
TOUCH_FRAME_HEAD = 0x40
TOUCH_FRAME_FLAG = 0x0A
TOUCH_FRAME_TAIL = 0x50
TOUCH_FRAME_REPEAT_COUNT = 3

# Detection ROI corner meanings: top-left, top-right, bottom-left, bottom-right.
BALL_ROI_TOP_LEFT = (50, 297)
BALL_ROI_TOP_RIGHT = (775, 308)
BALL_ROI_BOTTOM_LEFT = (50, 373)
BALL_ROI_BOTTOM_RIGHT = (775, 382)

# Traverse around the quadrilateral without crossing its edges.
BALL_ROI_DISPLAY = (
    BALL_ROI_TOP_LEFT,
    BALL_ROI_TOP_RIGHT,
    BALL_ROI_BOTTOM_RIGHT,
    BALL_ROI_BOTTOM_LEFT
)

# Corresponding bounding box in the 400x240 detection image.
BALL_ROI = (13, 148, 384, 44)

MIN_PIXELS = 50
MIN_SIZE = 6
MAX_SIZE = 80
MIN_ROUND_PERCENT = 10
MIN_FILL_PERCENT = 1

# Accuracy-sensitive options keep the original values in the first optimized
# version. Values above 1 reuse Otsu and refresh it on the frame after a miss.
# Change these only after positive/negative sample A/B tests.
OTSU_EVERY_N_FRAMES = 1
BLOB_X_STRIDE = 1
BLOB_Y_STRIDE = 1

SCALE_X = DISPLAY_WIDTH // DETECT_WIDTH
SCALE_Y = DISPLAY_HEIGHT // DETECT_HEIGHT

# CH0 is hardware-bound to the video layer. This small transparent strip is
# copied to the OSD layer on every processed camera frame.
OSD_X = 0
OSD_Y = 272
OSD_WIDTH = DISPLAY_WIDTH
OSD_HEIGHT = 128
OSD_PERIOD_MS = 0

# A separate small OSD keeps the live coordinate and FPS at the top-left
# without expanding the lower detection-overlay strip.
STATUS_OSD_X = 0
STATUS_OSD_Y = 0
STATUS_OSD_WIDTH = 256
STATUS_OSD_HEIGHT = 32
STATUS_TEXT_SIZE = 20

# The display may briefly retain a result to avoid visible flicker. The
# current_ball result and performance statistics still use the current frame.
DISPLAY_HOLD_MS = 70

PERF_REPORT_MS = 1000
GC_INTERVAL_MS = 2000
LATENCY_BIN_US = 500
LATENCY_BIN_COUNT = 128


def clamp(value, low, high):
    if value < low:
        return low
    if value > high:
        return high
    return value


def read_touch_point(touch_device):
    points = touch_device.read(1)
    if not points:
        return None

    point = points[0]
    x = clamp(int(point.x), 0, DISPLAY_WIDTH - 1)
    y = clamp(int(point.y), 0, DISPLAY_HEIGHT - 1)
    return x, y


def build_touch_x_frame(touch_x):
    return bytes([
        TOUCH_FRAME_HEAD,
        TOUCH_FRAME_FLAG,
        (touch_x >> 8) & 0xFF,
        touch_x & 0xFF,
        TOUCH_FRAME_TAIL
    ])


def send_touch_x_frames(uart_device, touch_x):
    frame = build_touch_x_frame(touch_x)
    burst = frame * TOUCH_FRAME_REPEAT_COUNT
    written = uart_device.write(burst)

    if written != len(burst):
        raise OSError(
            "UART short write: %s/%d"
            % (str(written), len(burst))
        )

    return frame


def touch_overlay_position(touch_point):
    x, y = touch_point
    return (
        clamp(
            x - TOUCH_OSD_WIDTH // 2,
            0,
            DISPLAY_WIDTH - TOUCH_OSD_WIDTH
        ),
        clamp(
            y - TOUCH_CROSS_SIZE - TOUCH_TEXT_GAP - TOUCH_TEXT_SIZE,
            0,
            DISPLAY_HEIGHT - TOUCH_OSD_HEIGHT
        )
    )


def draw_touch_marker(img, touch_point, offset_x, offset_y):
    if touch_point is None:
        return

    x, y = touch_point
    img.draw_cross(
        x - offset_x,
        y - offset_y,
        color=TOUCH_COLOR,
        size=TOUCH_CROSS_SIZE,
        thickness=TOUCH_CROSS_THICKNESS
    )

    text = "(%d, %d)" % (x, y)
    # ASCII glyphs are approximately half the selected font size in width.
    text_width = len(text) * TOUCH_TEXT_SIZE // 2
    text_x = clamp(
        x - text_width // 2,
        0,
        max(0, DISPLAY_WIDTH - text_width)
    )
    text_y = max(
        0,
        y - TOUCH_CROSS_SIZE - TOUCH_TEXT_GAP - TOUCH_TEXT_SIZE
    )
    img.draw_string_advanced(
        text_x - offset_x,
        text_y - offset_y,
        TOUCH_TEXT_SIZE,
        text,
        color=TOUCH_COLOR
    )


def rtsp_log(message):
    line = "[RTSP %s ms] %s" % (time.ticks_ms(), message)
    print(line)
    if not RTSP_LOG_TO_SD:
        return

    try:
        with open(RTSP_LOG_PATH, "a") as output:
            output.write(line)
            output.write("\n")
    except BaseException:
        # Serial logging remains available if the SD card is read-only/full.
        pass


class PtsToMilliseconds:
    """Map the documented v1.8 VENC microsecond PTS to RTSP milliseconds."""

    def __init__(self):
        self.monotonic_origin = time.ticks_ms()
        self.source_origin = None
        self.output_origin = 0
        self.last_source = None
        self.last_output = -1

    def map(self, source_pts):
        monotonic_ms = max(
            0,
            time.ticks_diff(time.ticks_ms(), self.monotonic_origin),
        )
        if source_pts is None or source_pts <= 0:
            return self._commit(monotonic_ms)

        if self.source_origin is None:
            self.source_origin = source_pts
            self.output_origin = monotonic_ms
            self.last_source = source_pts
            return self._commit(self.output_origin)

        source_delta = source_pts - self.last_source
        if source_delta <= 0 or source_delta > (10 * 1000 * 1000):
            self.source_origin = source_pts
            self.output_origin = max(
                self.last_output + FRAME_PERIOD_MS,
                monotonic_ms,
            )
            self.last_source = source_pts
            return self._commit(self.output_origin)

        self.last_source = source_pts
        mapped = self.output_origin + (
            (source_pts - self.source_origin) // VENC_PTS_PER_RTSP_MS
        )
        return self._commit(mapped)

    def _commit(self, value):
        if value <= self.last_output:
            value = self.last_output + 1
        self.last_output = value
        return value


def frame_pts(stream_data):
    """Choose the I/P pack PTS, or the first non-zero PTS, for one frame."""
    fallback = None

    for index in range(stream_data.pack_cnt):
        pts = stream_data.pts[index]
        if pts and fallback is None:
            fallback = pts

        stream_type = stream_data.stream_type[index]
        if (
            stream_type == Encoder.STREAM_TYPE_I
            or stream_type == Encoder.STREAM_TYPE_P
        ):
            if pts:
                return pts

    return fallback


def wait_for_ap_ip(ap, timeout_ms=5000):
    started = time.ticks_ms()

    while time.ticks_diff(time.ticks_ms(), started) < timeout_ms:
        config = ap.ifconfig()
        if config and config[0] == AP_IP:
            return config

        os.exitpoint()
        time.sleep_ms(100)

    raise RuntimeError("AP did not reach fixed address %s" % AP_IP)


def wait_for_network_device(timeout_ms=AP_DEVICE_WAIT_MS):
    started = time.ticks_ms()
    last_devices = None
    last_error = None

    while time.ticks_diff(time.ticks_ms(), started) < timeout_ms:
        try:
            last_devices = network.get_dev_list()
            last_error = None
            if last_devices is not None and AP_DEVICE in last_devices:
                return last_devices
        except BaseException as exc:
            last_error = exc

        os.exitpoint()
        time.sleep_ms(AP_DEVICE_POLL_MS)

    if last_error is not None:
        detail = "last error: %s" % last_error
    else:
        detail = "found: %s" % last_devices

    raise RuntimeError(
        "Wi-Fi AP device %s not ready after %d ms; %s"
        % (AP_DEVICE, timeout_ms, detail)
    )


def start_access_point():
    if not AP_PASSWORD:
        raise RuntimeError(
            "Set AP_SSID and AP_PASSWORD before enabling RTSP"
        )

    if not hasattr(network, "get_dev_list"):
        raise RuntimeError("firmware does not expose network.get_dev_list()")

    devices = wait_for_network_device()

    ap = network.WLAN(network.AP_IF)
    try:
        if ap.status():
            ap.stop()
            time.sleep_ms(300)
    except BaseException:
        # A fresh boot may not expose status until the first configuration.
        pass

    if ap.config(ssid=AP_SSID, key=AP_PASSWORD) is False:
        raise RuntimeError("failed to start Wi-Fi AP")
    time.sleep_ms(300)

    info = ap.info()
    if info is None:
        raise RuntimeError("firmware did not return AP configuration")

    info.channel = AP_CHANNEL
    info.band = ap.BAND_2_4GHZ
    info.security = ap.SECURITY_WPA2_AES_PSK
    if ap.stop() is False:
        raise RuntimeError("failed to restart AP for fixed radio configuration")
    time.sleep_ms(300)

    if ap.config(info=info, key=AP_PASSWORD) is False:
        raise RuntimeError(
            "failed to start fixed WPA2 AP on channel %d" % AP_CHANNEL
        )
    time.sleep_ms(300)

    if ap.ifconfig((AP_IP, AP_NETMASK, AP_IP, AP_IP)) is False:
        raise RuntimeError("failed to assign fixed AP address")
    if network.set_default_dev(AP_DEVICE) is False:
        raise RuntimeError(
            "failed to make %s the default network device" % AP_DEVICE
        )

    config = wait_for_ap_ip(ap)
    actual = ap.info()
    if actual is None:
        raise RuntimeError("AP radio readback failed")
    if (
        actual.channel != AP_CHANNEL
        or actual.band != ap.BAND_2_4GHZ
        or actual.security != ap.SECURITY_WPA2_AES_PSK
    ):
        raise RuntimeError("AP channel/band/security readback failed")

    rtsp_log(
        "AP ready: ssid=%s channel=%d band=%d security=%d ifconfig=%s"
        % (
            AP_SSID,
            actual.channel,
            actual.band,
            actual.security,
            config,
        )
    )
    return ap


class CameraRtspStreamer:
    """Use sensor CH2 -> hardware VENC -> RTSP without Python video frames."""

    def __init__(self):
        self.ap = None
        self.encoder = None
        self.link = None
        self.server = None

        self.running = False
        self.thread_started = False
        self.thread_done = True
        self.thread_error = None
        self.heartbeat = 0
        self.heartbeat_ticks = time.ticks_ms()

        self.pts = PtsToMilliseconds()
        self.last_logged_pts = None
        self.pts_logs_remaining = 3

        self.encoder_created = False
        self.encoder_running = False
        self.server_initialized = False
        self.server_running = False
        self.use_physical_packs = False

        self.frames_sent = 0
        self.bytes_sent = 0
        self.stats_ticks = time.ticks_ms()
        self.stats_frames = 0
        self.stats_bytes = 0

    def prepare(self, sensor):
        """Allocate/link resources that must exist before MediaManager.init."""
        self.ap = start_access_point()

        self.server = mm.rtsp_server()
        self.use_physical_packs = (
            RTSP_USE_PHYSICAL_PACKS
            and hasattr(
                self.server,
                "rtspserver_sendvideodata_byphyaddr",
            )
        )
        result = self.server.rtspserver_init(RTSP_PORT)
        if result != 0:
            raise RuntimeError(
                "RTSP port %d bind failed: %s" % (RTSP_PORT, result)
            )
        self.server_initialized = True

        result = self.server.rtspserver_createsession(
            RTSP_SESSION,
            mm.multi_media_type.media_h264,
            False,
        )
        if result != 0:
            raise RuntimeError("RTSP session creation failed: %s" % result)

        self.encoder = Encoder()
        self.encoder.SetOutBufs(
            VENC_CHANNEL,
            VENC_OUTPUT_BUFFERS,
            VIDEO_WIDTH,
            VIDEO_HEIGHT,
        )
        self.link = MediaManager.link(
            sensor.bind_info(chn=STREAM_CHN)["src"],
            (VIDEO_ENCODE_MOD_ID, VENC_DEV_ID, VENC_CHANNEL),
        )

    def create_encoder(self):
        """Create VENC after the shared media buffer manager is initialized."""
        attributes = ChnAttrStr(
            self.encoder.PAYLOAD_TYPE_H264,
            self.encoder.H264_PROFILE_BASELINE,
            VIDEO_WIDTH,
            VIDEO_HEIGHT,
            bit_rate=VIDEO_BITRATE_KBIT,
            gopLen=VIDEO_GOP,
            src_frame_rate=VIDEO_FPS,
            dst_frame_rate=VIDEO_FPS,
        )
        self.encoder.Create(VENC_CHANNEL, attributes)
        self.encoder_created = True

    def start_pipeline(self):
        """Start RTSP and VENC before the shared sensor starts."""
        self.server.rtspserver_start()
        self.server_running = True

        self.encoder.Start(VENC_CHANNEL)
        self.encoder_running = True

    def start_worker(self):
        """Start draining encoded buffers after sensor.run()."""
        self.running = True
        self.thread_done = False
        self.thread_error = None
        self.heartbeat_ticks = time.ticks_ms()
        self.stats_ticks = self.heartbeat_ticks
        self.stats_frames = self.frames_sent
        self.stats_bytes = self.bytes_sent

        try:
            _thread.start_new_thread(self._stream_loop, ())
            self.thread_started = True
        except BaseException:
            self.running = False
            self.thread_done = True
            raise

        rtsp_log(
            "ready: rtsp://%s:%d/%s H264 Baseline %dx%d@%dfps "
            "%dkbit/s send=%s"
            % (
                AP_IP,
                RTSP_PORT,
                RTSP_SESSION,
                VIDEO_WIDTH,
                VIDEO_HEIGHT,
                VIDEO_FPS,
                VIDEO_BITRATE_KBIT,
                "phyaddr" if self.use_physical_packs else "bytes",
            )
        )

    def _send_encoded_frame(self, stream_data, timestamp_ms):
        access_unit_size = 0

        if self.use_physical_packs:
            physical_addresses = getattr(stream_data, "phy_addr", None)
            if physical_addresses is None:
                self.use_physical_packs = False
                rtsp_log(
                    "VENC physical addresses unavailable; "
                    "falling back to byte copies"
                )
            else:
                for pack_index in range(stream_data.pack_cnt):
                    pack_size = stream_data.data_size[pack_index]
                    send_result = (
                        self.server.rtspserver_sendvideodata_byphyaddr(
                            RTSP_SESSION,
                            physical_addresses[pack_index],
                            pack_size,
                            timestamp_ms,
                        )
                    )
                    if send_result != 0:
                        raise RuntimeError(
                            "RTSP physical send failed: result=%s"
                            % send_result
                        )
                    access_unit_size += pack_size

                return access_unit_size

        # Compatibility path for firmware without the physical-address API.
        if stream_data.pack_cnt == 1:
            access_unit_size = stream_data.data_size[0]
            access_unit = bytes(
                uctypes.bytearray_at(
                    stream_data.data[0],
                    access_unit_size,
                )
            )
        else:
            parts = []
            for pack_index in range(stream_data.pack_cnt):
                pack_size = stream_data.data_size[pack_index]
                parts.append(
                    bytes(
                        uctypes.bytearray_at(
                            stream_data.data[pack_index],
                            pack_size,
                        )
                    )
                )
            access_unit = b"".join(parts)
            access_unit_size = len(access_unit)

        send_result = self.server.rtspserver_sendvideodata(
            RTSP_SESSION,
            access_unit,
            access_unit_size,
            timestamp_ms,
        )
        if send_result != 0:
            raise RuntimeError(
                "RTSP send failed: result=%s" % send_result
            )

        return access_unit_size

    def _stream_loop(self):
        stream_data = StreamData()
        consecutive_timeouts = 0

        try:
            while self.running:
                os.exitpoint()
                acquired = False

                try:
                    result = self.encoder.GetStream(
                        VENC_CHANNEL,
                        stream_data,
                        timeout=STREAM_GET_TIMEOUT_MS,
                    )
                    if result != 0:
                        consecutive_timeouts += 1
                        if (
                            consecutive_timeouts * STREAM_GET_TIMEOUT_MS
                            >= STREAM_STALL_TIMEOUT_MS
                        ):
                            raise RuntimeError("VENC stream stalled")
                        continue

                    acquired = True
                    consecutive_timeouts = 0
                    source_pts = frame_pts(stream_data)
                    timestamp_ms = self.pts.map(source_pts)

                    if self.pts_logs_remaining > 0:
                        delta = None
                        if source_pts and self.last_logged_pts:
                            delta = source_pts - self.last_logged_pts
                        rtsp_log(
                            "VENC sample: packs=%d pts_us=%s "
                            "delta_us=%s rtsp_ms=%d"
                            % (
                                stream_data.pack_cnt,
                                source_pts,
                                delta,
                                timestamp_ms,
                            )
                        )
                        if source_pts:
                            self.last_logged_pts = source_pts
                        self.pts_logs_remaining -= 1

                    access_unit_size = self._send_encoded_frame(
                        stream_data,
                        timestamp_ms,
                    )

                finally:
                    if acquired:
                        self.encoder.ReleaseStream(
                            VENC_CHANNEL,
                            stream_data,
                        )

                # The frame counts only after send and buffer release succeed.
                self.frames_sent += 1
                self.bytes_sent += access_unit_size
                self.heartbeat += 1
                self.heartbeat_ticks = time.ticks_ms()

        except BaseException as exc:
            self.thread_error = exc
            rtsp_log("stream thread failed: %s" % exc)
            try:
                sys.print_exception(exc)
            except BaseException:
                pass
        finally:
            self.thread_done = True

    def check_health(self):
        if self.thread_error is not None:
            raise RuntimeError("RTSP worker failed: %s" % self.thread_error)
        if self.thread_started and self.thread_done:
            raise RuntimeError("RTSP worker stopped unexpectedly")
        if (
            self.thread_started
            and time.ticks_diff(
                time.ticks_ms(),
                self.heartbeat_ticks,
            )
            >= STREAM_STALL_TIMEOUT_MS
        ):
            raise RuntimeError("RTSP heartbeat stopped")

    def sample_performance(self, now_ms):
        elapsed_ms = time.ticks_diff(now_ms, self.stats_ticks)
        if elapsed_ms <= 0:
            return (0.0, 0.0)

        frame_delta = self.frames_sent - self.stats_frames
        byte_delta = self.bytes_sent - self.stats_bytes
        fps = frame_delta * 1000.0 / elapsed_ms
        bitrate_kbit = byte_delta * 8.0 / elapsed_ms

        self.stats_ticks = now_ms
        self.stats_frames = self.frames_sent
        self.stats_bytes = self.bytes_sent
        return (fps, bitrate_kbit)

    def stop_worker(self):
        if not self.thread_started:
            return

        self.running = False
        started = time.ticks_ms()
        while (
            not self.thread_done
            and time.ticks_diff(time.ticks_ms(), started) < 2500
        ):
            time.sleep_ms(50)

        if not self.thread_done:
            raise RuntimeError(
                "RTSP worker did not stop; media resources must be retained"
            )

        self.thread_started = False

    def deinit_pipeline(self):
        """Release shared-media dependencies after the sensor has stopped."""
        if self.link is not None:
            try:
                del self.link
            except BaseException as exc:
                rtsp_log("media unlink warning: %s" % exc)
            self.link = None

        if self.encoder_running:
            try:
                self.encoder.Stop(VENC_CHANNEL)
            except BaseException as exc:
                rtsp_log("encoder stop warning: %s" % exc)
            self.encoder_running = False

        if self.encoder_created:
            try:
                self.encoder.Destroy(VENC_CHANNEL)
            except BaseException as exc:
                rtsp_log("encoder destroy warning: %s" % exc)
            self.encoder_created = False

        if self.server_running:
            try:
                self.server.rtspserver_stop()
            except BaseException as exc:
                rtsp_log("RTSP stop warning: %s" % exc)
            self.server_running = False

        if self.server_initialized:
            try:
                self.server.rtspserver_destroysession(RTSP_SESSION)
            except BaseException:
                pass
            try:
                self.server.rtspserver_deinit()
            except BaseException as exc:
                rtsp_log("RTSP deinit warning: %s" % exc)
            self.server_initialized = False

        if self.ap is not None:
            try:
                self.ap.stop()
            except BaseException as exc:
                rtsp_log("AP stop warning: %s" % exc)
            self.ap = None


def uart2_init():
    """Initialize UART2 on GPIO11/GPIO12 at 115200, 8N1."""
    fpioa = FPIOA()
    fpioa.set_function(UART2_TX_GPIO, FPIOA.UART2_TXD)
    fpioa.set_function(UART2_RX_GPIO, FPIOA.UART2_RXD)

    return UART(
        UART.UART2,
        baudrate=UART2_BAUDRATE,
        bits=UART.EIGHTBITS,
        parity=UART.PARITY_NONE,
        stop=UART.STOPBITS_ONE
    )


def uart2_send_ball_x(uart2, ball_x):
    """Send one 40 Flag Ball_X_H Ball_X_L 50 frame."""
    if ball_x is None:
        flag = UART2_FLAG_NO_HISTORY
        ball_x = 0
    else:
        flag = UART2_FLAG_VALID
        ball_x = int(ball_x)
        if ball_x < 0 or ball_x > 0xFFFF:
            raise ValueError("Ball_X is outside uint16: %d" % ball_x)

    _uart2_tx_frame[1] = flag
    _uart2_tx_frame[2] = (ball_x >> 8) & 0xFF
    _uart2_tx_frame[3] = ball_x & 0xFF

    written = uart2.write(_uart2_tx_frame)
    if written is not None and written != UART2_FRAME_SIZE:
        raise OSError(
            "UART2 short write: %d/%d" % (written, UART2_FRAME_SIZE)
        )


def resolve_sensor_mode():
    requested = FAST_SENSOR_MODE if PREFER_720P90 else STABLE_SENSOR_MODE

    try:
        mode_result = Sensor.list_mode(id=SENSOR_ID)
        if mode_result is None:
            raise RuntimeError("Sensor.list_mode returned None")

        sensor_name, modes = mode_result
        for mode in modes or ():
            width = int(mode.get("width", 0))
            height = int(mode.get("height", 0))
            fps = int(mode.get("fps", 0))
            if (width, height, fps) == requested:
                print(
                    "[SENSOR] %s exposes %dx%d@%d; requesting it"
                    % (sensor_name, width, height, fps)
                )
                return requested

        if PREFER_720P90:
            print(
                "[WARN] 1280x720@90 is not exposed; falling back to "
                "1920x1080@60"
            )
        else:
            print(
                "[WARN] exact 1920x1080@60 mode was not listed; "
                "requesting the stable fallback"
            )
    except BaseException as exc:
        print("[WARN] sensor mode query failed: %s" % exc)

    return STABLE_SENSOR_MODE


def histogram_percentile_us(histogram, sample_count, percentile):
    if sample_count <= 0:
        return 0

    target = (sample_count * percentile + 99) // 100
    cumulative = 0

    for index in range(len(histogram)):
        cumulative += histogram[index]
        if cumulative >= target:
            return (index + 1) * LATENCY_BIN_US

    return len(histogram) * LATENCY_BIN_US


class PerformanceMeter:
    def __init__(self):
        self.period_histogram = [0] * LATENCY_BIN_COUNT
        self.last_fps = 0.0
        self.started_ms = time.ticks_ms()
        self._reset_interval()

    def _reset_interval(self):
        self.frames = 0
        self.hits = 0
        self.period_samples = 0
        self.snapshot_us_total = 0
        self.otsu_us_total = 0
        self.otsu_calls = 0
        self.blob_us_total = 0
        self.work_us_max = 0
        self.gc_us_max = 0
        self.period_us_max = 0
        self.period_overflow_count = 0

        for index in range(len(self.period_histogram)):
            self.period_histogram[index] = 0

    def add(
        self,
        period_us,
        work_us,
        snapshot_us,
        otsu_us,
        otsu_ran,
        blob_us,
        gc_us,
        ball,
        otsu_threshold,
        rtsp_streamer,
    ):
        self.frames += 1
        self.snapshot_us_total += snapshot_us
        self.blob_us_total += blob_us

        if ball is not None:
            self.hits += 1

        if otsu_ran:
            self.otsu_calls += 1
            self.otsu_us_total += otsu_us

        if work_us > self.work_us_max:
            self.work_us_max = work_us

        if gc_us > self.gc_us_max:
            self.gc_us_max = gc_us

        if period_us > 0:
            if period_us > self.period_us_max:
                self.period_us_max = period_us

            bin_index = period_us // LATENCY_BIN_US
            if bin_index >= len(self.period_histogram):
                bin_index = len(self.period_histogram) - 1
                self.period_overflow_count += 1
            self.period_histogram[bin_index] += 1
            self.period_samples += 1

        now_ms = time.ticks_ms()
        elapsed_ms = time.ticks_diff(now_ms, self.started_ms)
        if elapsed_ms < PERF_REPORT_MS:
            return

        self.last_fps = self.frames * 1000.0 / max(elapsed_ms, 1)
        p95_us = histogram_percentile_us(
            self.period_histogram,
            self.period_samples,
            95
        )
        p99_us = histogram_percentile_us(
            self.period_histogram,
            self.period_samples,
            99
        )
        snapshot_avg_us = self.snapshot_us_total / max(self.frames, 1)
        otsu_avg_us = self.otsu_us_total / max(self.otsu_calls, 1)
        blob_avg_us = self.blob_us_total / max(self.frames, 1)
        hit_percent = self.hits * 100.0 / max(self.frames, 1)
        free_bytes = gc.mem_free() if hasattr(gc, "mem_free") else -1
        if rtsp_streamer is None:
            stream_fps = 0.0
            stream_bitrate_kbit = 0.0
        else:
            stream_fps, stream_bitrate_kbit = (
                rtsp_streamer.sample_performance(now_ms)
            )

        if ball is None:
            result_text = "NO_BALL"
        else:
            result_text = "BALL,%d,%d,%d" % (
                ball[4] * SCALE_X,
                ball[5] * SCALE_Y,
                ball[2] + ball[3]
            )

        print(
            "[PERF] detect_calls=%.1fHz period_p95=%.2fms p99=%.2fms "
            "period_max=%.2fms overflow=%d work_max=%.2fms "
            "snap_avg=%.2fms otsu_avg=%.2fms blob_avg=%.2fms "
            "gc_max=%.2fms stream=%.1fHz stream_rate=%.0fkbit/s "
            "hit=%.1f%% threshold=%d free=%d result=%s"
            % (
                self.last_fps,
                p95_us / 1000.0,
                p99_us / 1000.0,
                self.period_us_max / 1000.0,
                self.period_overflow_count,
                self.work_us_max / 1000.0,
                snapshot_avg_us / 1000.0,
                otsu_avg_us / 1000.0,
                blob_avg_us / 1000.0,
                self.gc_us_max / 1000.0,
                stream_fps,
                stream_bitrate_kbit,
                hit_percent,
                otsu_threshold,
                free_bytes,
                result_text
            )
        )

        self.started_ms = now_ms
        self._reset_interval()


def point_in_polygon(x, y, points):
    last_cross = 0

    for index in range(len(points)):
        x1, y1 = points[index]
        x2, y2 = points[(index + 1) % len(points)]
        cross = (x2 - x1) * (y - y1) - (y2 - y1) * (x - x1)

        if cross == 0:
            continue

        if last_cross == 0:
            last_cross = cross
        elif (cross > 0) != (last_cross > 0):
            return False

    return True


def blob_fully_inside_roi(blob):
    x0 = blob.x()
    y0 = blob.y()
    x1 = x0 + blob.w() - 1
    y1 = y0 + blob.h() - 1

    corners = (
        (x0, y0),
        (x1, y0),
        (x1, y1),
        (x0, y1)
    )

    for x, y in corners:
        if not point_in_polygon(
            x * SCALE_X,
            y * SCALE_Y,
            BALL_ROI_DISPLAY
        ):
            return False

    return True


def draw_roi_outline(img, points, offset_x=0, offset_y=0):
    for index in range(len(points)):
        x1, y1 = points[index]
        x2, y2 = points[(index + 1) % len(points)]
        img.draw_line(
            x1 + offset_x,
            y1 + offset_y,
            x2 + offset_x,
            y2 + offset_y,
            color=(0, 0, 255),
            thickness=2
        )


def get_otsu_threshold(gray_img):
    histogram = gray_img.get_histogram(roi=BALL_ROI)
    return histogram.get_threshold().value()


def find_steel_ball(gray_img, otsu_threshold):
    # Directly find the dark range in the grayscale ROI. This preserves the
    # original pixel classification without a 400x240 copy and binary pass.
    blobs = gray_img.find_blobs(
        [(0, otsu_threshold)],
        False,
        BALL_ROI,
        x_stride=BLOB_X_STRIDE,
        y_stride=BLOB_Y_STRIDE,
        pixels_threshold=MIN_PIXELS,
        merge=False,
        margin=4
    )

    best = None
    best_score = 0

    for blob in blobs:
        w = blob.w()
        h = blob.h()
        pixels = blob.pixels()

        if w < MIN_SIZE or h < MIN_SIZE:
            continue

        if w > MAX_SIZE or h > MAX_SIZE:
            continue

        long_side = max(w, h)
        short_side = min(w, h)

        round_percent = short_side * 100 // long_side
        fill_percent = pixels * 100 // (w * h)

        if round_percent < MIN_ROUND_PERCENT:
            continue

        if fill_percent < MIN_FILL_PERCENT:
            continue

        # Keep the original trapezoid rule, but run it only after the cheaper
        # size/shape filters have accepted the candidate.
        if not blob_fully_inside_roi(blob):
            continue

        score = pixels * round_percent

        if score > best_score:
            best_score = score
            best = (
                blob.x(),
                blob.y(),
                w,
                h,
                blob.cx(),
                blob.cy(),
                pixels
            )

    return best


def draw_overlay(osd_img, display_ball):
    osd_img.clear()
    draw_roi_outline(
        osd_img,
        BALL_ROI_DISPLAY,
        offset_x=-OSD_X,
        offset_y=-OSD_Y
    )

    if display_ball is None:
        return

    cx = display_ball[4] * SCALE_X - OSD_X
    cy = display_ball[5] * SCALE_Y - OSD_Y

    osd_img.draw_cross(
        cx,
        cy,
        color=(255, 0, 0),
        size=8,
        thickness=2
    )


def draw_status_overlay(status_img, current_ball, fps):
    status_img.clear()

    if current_ball is None:
        text = "X:---  FPS:%.1f" % fps
    else:
        text = "X:%d  FPS:%.1f" % (
            current_ball[4] * SCALE_X,
            fps
        )

    status_img.draw_string_advanced(
        4,
        4,
        STATUS_TEXT_SIZE,
        text,
        color=(255, 0, 0)
    )


sensor = None
touch = None
uart2 = None
rtsp_streamer = None
watchdog = None
display_started = False
video_bound = False
media_initialized = False
osd_img = None
touch_osd_img = None
status_osd_img = None

os.exitpoint(os.EXITPOINT_ENABLE)
sensor_width, sensor_height, sensor_fps = resolve_sensor_mode()

try:
    if OTSU_EVERY_N_FRAMES < 1:
        raise ValueError("OTSU_EVERY_N_FRAMES must be at least 1")
    if VIDEO_FPS < 1:
        raise ValueError("VIDEO_FPS must be at least 1")

    uart2 = uart2_init()
    print(
        "[BOOT] %s UART2 tx=GPIO11 rx=GPIO12 baud=115200 8N1"
        % APP_VERSION
    )

    sensor = Sensor(
        id=SENSOR_ID,
        width=sensor_width,
        height=sensor_height,
        fps=sensor_fps
    )
    sensor.reset()

    # CH0 continuously reaches the LCD video layer without a Python snapshot,
    # RGB conversion, full-frame drawing, or full-frame show_image copy.
    sensor.set_framesize(
        width=DISPLAY_WIDTH,
        height=DISPLAY_HEIGHT,
        chn=DISPLAY_CHN
    )
    sensor.set_pixformat(Sensor.YUV420SP, chn=DISPLAY_CHN)

    # CH1 is the only channel touched by the detection loop.
    sensor.set_framesize(
        width=DETECT_WIDTH,
        height=DETECT_HEIGHT,
        chn=DETECT_CHN
    )
    sensor.set_pixformat(Sensor.GRAYSCALE, chn=DETECT_CHN)

    if ENABLE_RTSP:
        # CH2 is consumed directly by hardware VENC.  Decimating it to 20 fps
        # before encoding limits memory bandwidth while CH1 keeps sampling at
        # the full sensor cadence for detection.
        sensor.set_framesize(
            width=VIDEO_WIDTH,
            height=VIDEO_HEIGHT,
            alignment=12,
            chn=STREAM_CHN
        )
        sensor.set_pixformat(Sensor.YUV420SP, chn=STREAM_CHN)
        sensor._set_chn_fps(chn=STREAM_CHN, fps=VIDEO_FPS)

    bind_info = sensor.bind_info(x=0, y=0, chn=DISPLAY_CHN)
    Display.bind_layer(**bind_info, layer=Display.LAYER_VIDEO1)
    video_bound = True

    Display.init(
        Display.ST7701,
        width=DISPLAY_WIDTH,
        height=DISPLAY_HEIGHT,
        osd_num=2,
        to_ide=DISPLAY_TO_IDE,
        quality=IDE_JPEG_QUALITY
    )
    display_started = True

    osd_img = image.Image(
        OSD_WIDTH,
        OSD_HEIGHT,
        image.ARGB8888
    )
    touch_osd_img = image.Image(
        TOUCH_OSD_WIDTH,
        TOUCH_OSD_HEIGHT,
        image.ARGB8888
    )
    status_osd_img = image.Image(
        STATUS_OSD_WIDTH,
        STATUS_OSD_HEIGHT,
        image.ARGB8888
    )

    if ENABLE_RTSP:
        rtsp_streamer = CameraRtspStreamer()
        rtsp_streamer.prepare(sensor)

    # Display, camera buffers, and VENC share one media manager.
    MediaManager.init()
    media_initialized = True

    if rtsp_streamer is not None:
        rtsp_streamer.create_encoder()
        rtsp_streamer.start_pipeline()

    sensor.run()
    touch = TOUCH(0)
    print("[BOOT] touch ready")

    if rtsp_streamer is not None:
        # The watchdog is fed only by the live detection loop after the stream
        # worker has passed its heartbeat check.
        watchdog = WDT(
            1,
            timeout=WATCHDOG_TIMEOUT_SECONDS,
            auto_close=False
        )
        rtsp_streamer.start_worker()

    gc.collect()

    try:
        lcd_fps = Display.fps()
    except BaseException:
        lcd_fps = -1

    print(
        "[BOOT] steel ball detector request=%dx%d@%d "
        "detect=%dx%d stream=%s lcd_fps_reported=%d ide=%s"
        % (
            sensor_width,
            sensor_height,
            sensor_fps,
            DETECT_WIDTH,
            DETECT_HEIGHT,
            (
                "%dx%d@%d" % (VIDEO_WIDTH, VIDEO_HEIGHT, VIDEO_FPS)
                if rtsp_streamer is not None
                else "off"
            ),
            lcd_fps,
            DISPLAY_TO_IDE
        )
    )

    performance = PerformanceMeter()
    frame_id = 0
    current_ball = None
    display_ball = None
    display_ball_seen_ms = 0
    last_ball_x = None
    last_otsu_threshold = 0
    last_overlay_ms = None
    last_gc_ms = time.ticks_ms()
    previous_frame_started_us = None
    last_touch_point = None
    touch_was_pressed = False

    while True:
        os.exitpoint()

        if rtsp_streamer is not None:
            rtsp_streamer.check_health()
        if watchdog is not None:
            watchdog.feed()

        current_touch_point = read_touch_point(touch)
        if current_touch_point is None:
            touch_was_pressed = False
        elif not touch_was_pressed:
            last_touch_point = current_touch_point
            touch_was_pressed = True
            try:
                send_touch_x_frames(
                    uart2,
                    last_touch_point[0]
                )
                print(
                    "TOUCH_TX,%d,%d,REPEAT:%d"
                    % (
                        last_touch_point[0],
                        last_touch_point[1],
                        TOUCH_FRAME_REPEAT_COUNT
                    )
                )
            except Exception as exc:
                print("touch UART send failed:", exc)

            print(
                "TOUCH,%d,%d"
                % (last_touch_point[0], last_touch_point[1])
            )

            touch_osd_x, touch_osd_y = touch_overlay_position(
                last_touch_point
            )
            touch_osd_img.clear()
            draw_touch_marker(
                touch_osd_img,
                last_touch_point,
                touch_osd_x,
                touch_osd_y
            )
            Display.show_image(
                touch_osd_img,
                x=touch_osd_x,
                y=touch_osd_y,
                layer=Display.LAYER_OSD1
            )

        frame_started_us = time.ticks_us()

        if previous_frame_started_us is None:
            period_us = 0
        else:
            period_us = time.ticks_diff(
                frame_started_us,
                previous_frame_started_us
            )
        previous_frame_started_us = frame_started_us

        snapshot_started_us = time.ticks_us()
        detect_img = sensor.snapshot(chn=DETECT_CHN)
        snapshot_us = time.ticks_diff(
            time.ticks_us(),
            snapshot_started_us
        )

        otsu_ran = (
            frame_id % OTSU_EVERY_N_FRAMES == 0
            or current_ball is None
        )
        otsu_us = 0
        if otsu_ran:
            otsu_started_us = time.ticks_us()
            last_otsu_threshold = get_otsu_threshold(detect_img)
            otsu_us = time.ticks_diff(
                time.ticks_us(),
                otsu_started_us
            )

        blob_started_us = time.ticks_us()
        current_ball = find_steel_ball(
            detect_img,
            last_otsu_threshold
        )
        blob_us = time.ticks_diff(
            time.ticks_us(),
            blob_started_us
        )

        # Send exactly one protocol frame for every processed camera frame.
        # A fresh detection replaces the saved coordinate. A miss repeats the
        # last valid coordinate; before the first hit, Flag=0x01 marks no data.
        if current_ball is not None:
            last_ball_x = current_ball[4] * SCALE_X
        uart2_send_ball_x(uart2, last_ball_x)

        # display_ball may persist briefly for UI stability independently of
        # the UART hold-last-value behavior above.
        now_ms = time.ticks_ms()
        if current_ball is not None:
            display_ball = current_ball
            display_ball_seen_ms = now_ms
        elif (
            display_ball is not None
            and time.ticks_diff(now_ms, display_ball_seen_ms)
            >= DISPLAY_HOLD_MS
        ):
            display_ball = None

        if (
            last_overlay_ms is None
            or time.ticks_diff(now_ms, last_overlay_ms) >= OSD_PERIOD_MS
        ):
            draw_overlay(
                osd_img,
                display_ball
            )
            Display.show_image(
                osd_img,
                x=OSD_X,
                y=OSD_Y,
                layer=Display.LAYER_OSD0
            )
            draw_status_overlay(
                status_osd_img,
                current_ball,
                performance.last_fps
            )
            Display.show_image(
                status_osd_img,
                x=STATUS_OSD_X,
                y=STATUS_OSD_Y,
                layer=Display.LAYER_OSD2
            )
            last_overlay_ms = now_ms

        gc_us = 0
        if time.ticks_diff(now_ms, last_gc_ms) >= GC_INTERVAL_MS:
            gc_started_us = time.ticks_us()
            gc.collect()
            gc_us = time.ticks_diff(time.ticks_us(), gc_started_us)
            last_gc_ms = time.ticks_ms()

        work_us = time.ticks_diff(time.ticks_us(), frame_started_us)
        performance.add(
            period_us,
            work_us,
            snapshot_us,
            otsu_us,
            otsu_ran,
            blob_us,
            gc_us,
            current_ball,
            last_otsu_threshold,
            rtsp_streamer
        )
        frame_id += 1

except KeyboardInterrupt:
    print("[STOP] user stopped")

except BaseException as exc:
    print("[FATAL] %s" % exc)

finally:
    media_cleanup_safe = True

    if rtsp_streamer is not None:
        try:
            rtsp_streamer.stop_worker()
        except BaseException as exc:
            media_cleanup_safe = False
            print("[WARN] RTSP worker stop failed: %s" % exc)

    if touch is not None:
        try:
            touch.deinit()
        except BaseException as exc:
            print("[WARN] touch deinit failed: %s" % exc)

    if uart2 is not None:
        try:
            uart2.deinit()
        except BaseException as exc:
            print("[WARN] UART2 deinit failed: %s" % exc)

    if isinstance(sensor, Sensor) and media_cleanup_safe:
        try:
            sensor.stop()
        except BaseException as exc:
            media_cleanup_safe = False
            print("[WARN] sensor stop failed: %s" % exc)

    if rtsp_streamer is not None and media_cleanup_safe:
        rtsp_streamer.deinit_pipeline()

    if display_started and media_cleanup_safe:
        try:
            Display.deinit()
        except BaseException as exc:
            media_cleanup_safe = False
            print("[WARN] display deinit failed: %s" % exc)
    elif video_bound and media_cleanup_safe:
        try:
            Display.unbind_layer(Display.LAYER_VIDEO1)
        except BaseException:
            pass

    try:
        os.exitpoint(os.EXITPOINT_ENABLE_SLEEP)
        time.sleep_ms(100)
    except BaseException:
        pass

    if media_initialized and media_cleanup_safe:
        try:
            MediaManager.deinit()
        except BaseException as exc:
            media_cleanup_safe = False
            print("[WARN] media manager deinit failed: %s" % exc)

    if watchdog is not None and media_cleanup_safe:
        try:
            watchdog.close()
        except BaseException as exc:
            print("[WARN] watchdog close failed: %s" % exc)

    if not media_cleanup_safe and watchdog is not None:
        print("[FATAL] unsafe media cleanup; waiting for watchdog reset")
        while True:
            # Keep media objects referenced and stop feeding the watchdog.
            _ = sensor
            _ = rtsp_streamer
            _ = watchdog
            time.sleep_ms(1000)

    gc.collect()
