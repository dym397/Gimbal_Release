import random
import socket
import threading
import time
import math
import struct
import json
import queue
import os
import sys
import csv
import atexit
from collections import deque, Counter
from datetime import datetime, timedelta, timezone
from decimal import Decimal

# ==========================================
# 0. 驱动引入
# ==========================================
try:
    from gimbal_interface import GT06ZAdapter
except ImportError as e:
    print(f"[System] 驱动接口加载失败: {e}")
    exit(1) # 驱动加载失败直接退出，防止后续报错
try:
    from mock_gimbal import MockGimbalAdapter
except ImportError:
    MockGimbalAdapter = None
try:
    from sddm_laser import SDDMLaser
except ImportError:
    SDDMLaser = None
try:
    from gps import (
        DEFAULT_LATITUDE,
        DEFAULT_LONGITUDE,
        read_gps_fix,
        wgs84_to_gcj02,
    )
except ImportError:
    DEFAULT_LATITUDE = None
    DEFAULT_LONGITUDE = None
    read_gps_fix = None
    wgs84_to_gcj02 = None
try:
    from target_measurement_runtime import TargetMeasurementRuntime
except ImportError:
    TargetMeasurementRuntime = None
try:
    from special_rid_identity import (
        SPECIAL_RID_UI_IDS,
        SpecialRidPredictor,
        SpecialRidRegistry,
        run_special_rid_ui_sender,
        should_send_sort_through_ordinary_ui,
        sort_generation_from_track,
        sort_observation_from_track,
    )
except ImportError:
    SPECIAL_RID_UI_IDS = frozenset()
    SpecialRidPredictor = None
    SpecialRidRegistry = None
    run_special_rid_ui_sender = None
    should_send_sort_through_ordinary_ui = None
    sort_generation_from_track = None
    sort_observation_from_track = None
# ==========================================
# 配置
# ==========================================
# Keep one codepath and switch only the serial defaults by platform.
def _platform_serial_defaults():
    if os.name == "nt":
        return {
            "gimbal": "COM8",
            "laser": "COM12",
            "gps": "COM8",
            "rid": "",
        }
    return {
        "gimbal": "/dev/serial/by-path/platform-xhci-hcd.4.auto-usb-0:1.2.2:1.0-port0",
        "laser": "/dev/serial/by-path/platform-xhci-hcd.4.auto-usb-0:1.4:1.0-port0",
        "gps": "/dev/serial/by-path/platform-xhci-hcd.4.auto-usb-0:1.2.1:1.0-port0",
        "rid": "/dev/serial/by-path/platform-xhci-hcd.4.auto-usb-0:1.2.3:1.0-port0",
    }


_SERIAL_PORT_DEFAULTS = _platform_serial_defaults()


def _serial_port_default(role):
    return _SERIAL_PORT_DEFAULTS[role]


def _serial_port(env_name, role):
    port_name = os.getenv(env_name, _serial_port_default(role))
    return port_name.strip() if isinstance(port_name, str) else port_name


def _is_windows_com_port(port_name):
    if not isinstance(port_name, str):
        return False
    normalized = port_name.strip().upper()
    if normalized.startswith("\\\\.\\"):
        normalized = normalized[4:]
    return normalized.startswith("COM") and normalized[3:].isdigit() and int(normalized[3:]) > 0


def _normalize_windows_com_port(port_name):
    normalized = port_name.strip().upper()
    if normalized.startswith("\\\\.\\"):
        normalized = normalized[4:]
    return normalized


def _is_linux_serial_port(port_name):
    if not isinstance(port_name, str):
        return False
    normalized = port_name.strip()
    return normalized.startswith((
        "/dev/ttyUSB",
        "/dev/ttyACM",
        "/dev/ttyS",
        "/dev/serial/by-id/",
        "/dev/serial/by-path/",
    ))


def _validate_serial_port(label, port_name):
    if not isinstance(port_name, str) or not port_name:
        print(f"[Config][Warn] {label} 未配置有效的串口设备名。")
        return

    if os.name == "nt":
        if not _is_windows_com_port(port_name):
            print(
                f"[Config][Warn] {label}={port_name} 不是 Windows 串口设备名。"
                "请改为 COM3、COM4 或 \\\\.\\COM10 这类格式。"
            )
            return
        try:
            from serial.tools import list_ports
        except Exception:
            return
        available_ports = {_normalize_windows_com_port(port.device) for port in list_ports.comports()}
        normalized_port = _normalize_windows_com_port(port_name)
        if available_ports and normalized_port not in available_ports:
            print(
                f"[Config][Warn] {label}={port_name} 未在当前 Windows 串口列表中发现。"
                f"当前可用端口: {', '.join(sorted(available_ports))}"
            )
        return

    if not _is_linux_serial_port(port_name):
        print(
            f"[Config][Warn] {label}={port_name} 不是 Linux 串口设备路径。"
            "请改为 /dev/ttyUSB*、/dev/ttyACM*、/dev/ttyS*、/dev/serial/by-id/* "
            "或 /dev/serial/by-path/*。"
        )
        return
    if not os.path.exists(port_name):
        print(
            f"[Config][Warn] {label}={port_name} 当前不存在。"
            "请确认设备节点、udev 映射或 USB 串口权限。"
        )


class _TeeStream:
    def __init__(self, *streams):
        self._streams = streams
        self._lock = threading.Lock()

    def write(self, data):
        with self._lock:
            for stream in self._streams:
                stream.write(data)
            for stream in self._streams:
                stream.flush()
        return len(data)

    def flush(self):
        with self._lock:
            for stream in self._streams:
                stream.flush()

    def isatty(self):
        for stream in self._streams:
            is_tty = getattr(stream, "isatty", None)
            if callable(is_tty) and is_tty():
                return True
        return False

    @property
    def encoding(self):
        return getattr(self._streams[0], "encoding", "utf-8")

    def fileno(self):
        fileno = getattr(self._streams[0], "fileno", None)
        if callable(fileno):
            return fileno()
        raise OSError("fileno not supported")


_LOG_MIRROR_INITIALIZED = False
_LOG_MIRROR_PATH = None


def _create_run_log_dir(base_dir):
    os.makedirs(base_dir, exist_ok=True)
    timestamp = time.strftime("%Y%m%d_%H%M%S", time.localtime())
    run_dir = os.path.join(base_dir, timestamp)
    suffix = 1
    while True:
        try:
            os.makedirs(run_dir, exist_ok=False)
            return run_dir
        except FileExistsError:
            run_dir = os.path.join(base_dir, f"{timestamp}_{suffix:02d}")
            suffix += 1


def _setup_log_mirror(log_dir):
    global _LOG_MIRROR_INITIALIZED, _LOG_MIRROR_PATH
    if _LOG_MIRROR_INITIALIZED:
        return _LOG_MIRROR_PATH

    os.makedirs(log_dir, exist_ok=True)
    timestamp = time.strftime("%Y%m%d_%H%M%S", time.localtime())
    log_prefix = os.getenv("LOG_PREFIX", "main_tracking_v9").strip() or "main_tracking_v9"
    safe_prefix = "".join(ch if (ch.isalnum() or ch in ("-", "_")) else "_" for ch in log_prefix)
    log_path = os.path.join(log_dir, f"{safe_prefix}_{timestamp}.log")
    log_file = open(log_path, "a", encoding="utf-8", newline="\n", buffering=1)
    original_stdout = sys.stdout
    original_stderr = sys.stderr
    sys.stdout = _TeeStream(original_stdout, log_file)
    sys.stderr = _TeeStream(original_stderr, log_file)

    def _cleanup_log_mirror():
        try:
            sys.stdout = original_stdout
            sys.stderr = original_stderr
        except Exception:
            pass
        try:
            log_file.flush()
            log_file.close()
        except Exception:
            pass

    atexit.register(_cleanup_log_mirror)
    _LOG_MIRROR_INITIALIZED = True
    _LOG_MIRROR_PATH = log_path
    return log_path


def _env_flag(name, default=True):
    val = os.getenv(name)
    if val is None:
        return default
    return str(val).strip().lower() not in ("0", "false", "no", "off")


def _env_float(name, default):
    val = os.getenv(name)
    if val is None:
        return float(default)
    try:
        return float(str(val).strip())
    except (TypeError, ValueError):
        print(f"[Config][Warn] {name}={val} 不是有效浮点数，使用默认值 {default}.")
        return float(default)


def _env_int(name, default):
    val = os.getenv(name)
    if val is None:
        return int(default)
    try:
        return int(str(val).strip())
    except (TypeError, ValueError):
        print(f"[Config][Warn] {name}={val} 不是有效整数，使用默认值 {default}.")
        return int(default)


UI_IP = os.getenv("UI_IP", "192.168.0.200")
# UI_IP="172.28.3.80"
UI_PORT = int(os.getenv("UI_PORT", "9999"))
LOCAL_PORT = int(os.getenv("LOCAL_PORT", "8888"))
WINDOWS_GIMBAL_CAMERA_SOURCE = "000000008"
LINUX_GIMBAL_CAMERA_SOURCE = "/dev/v4l/by-path/platform-xhci-hcd.4.auto-usb-0:1.1:1.0-video-index0"
ENABLE_STRIKE_SEND = _env_flag("ENABLE_STRIKE_SEND", True)
STRIKE_IP = os.getenv("STRIKE_IP", "192.168.0.80")
STRIKE_PORT = int(os.getenv("STRIKE_PORT", "10123"))
STRIKE_SEND_HZ = _env_float("STRIKE_SEND_HZ", 10.0)
STRIKE_WINDOW_SECONDS = _env_float("STRIKE_WINDOW_SECONDS", 1.0)
STRIKE_LEAD_TIME = _env_float("STRIKE_LEAD_TIME", 0.3)
STRIKE_SETTLED_EVENT_TTL = _env_float("STRIKE_SETTLED_EVENT_TTL", 0.5)
STRIKE_VISION_MISSING_HOLD_SECONDS = _env_float(
    "STRIKE_VISION_MISSING_HOLD_SECONDS", 0.5
)
TRACK_DISTANCE_TTL = _env_float("TRACK_DISTANCE_TTL", 3.0)
ENABLE_GIMBAL_VISION = _env_flag("ENABLE_GIMBAL_VISION", False)
GIMBAL_CAMERA_SOURCE = os.getenv(
    "GIMBAL_CAMERA_SOURCE",
    WINDOWS_GIMBAL_CAMERA_SOURCE if os.name == "nt" else LINUX_GIMBAL_CAMERA_SOURCE,
).strip()
GIMBAL_VISION_CONFIDENCE = _env_float("GIMBAL_VISION_CONFIDENCE", 0.30)
GIMBAL_VISION_SETTLE_DELAY = _env_float("GIMBAL_VISION_SETTLE_DELAY", 0.20)
GIMBAL_VISION_MIN_SHARPNESS = _env_float("GIMBAL_VISION_MIN_SHARPNESS", 20.0)
GIMBAL_VISION_RESULT_TTL = _env_float("GIMBAL_VISION_RESULT_TTL", 3.00)
GIMBAL_VISION_ASSOCIATION_MAX_PX = _env_float(
    "GIMBAL_VISION_ASSOCIATION_MAX_PX", 260.0
)
GIMBAL_VISION_AMBIGUITY_MARGIN_PX = _env_float(
    "GIMBAL_VISION_AMBIGUITY_MARGIN_PX", 30.0
)
GIMBAL_VISION_TRACK_STATE_TTL = _env_float(
    "GIMBAL_VISION_TRACK_STATE_TTL", 2.0
)
GIMBAL_PORT = _serial_port("GIMBAL_PORT", "gimbal")
LASER_PORT = _serial_port("LASER_PORT", "laser")
GPS_PORT = _serial_port("GPS_PORT", "gps")
RID_PORT = _serial_port("RID_PORT", "rid")
USE_MOCK_GIMBAL = _env_flag("USE_MOCK_GIMBAL", False)  # True: 使用 mock_gimbal.py; False: 使用真实 GT06Z
USE_MOCK_LASER = _env_flag("USE_MOCK_LASER", True)   # RID branch default: do not open the legacy laser.
ENABLE_GPS = _env_flag("ENABLE_GPS", True)
ENABLE_RID = _env_flag("ENABLE_RID", bool(RID_PORT))
GPS_BAUDRATE = 115200
GPS_FIX_TIMEOUT_SECONDS = 5
GPS_STATUS_INTERVAL = 5.0
GPS_UI_SEND_INTERVAL = 10.0
GPS_DEBUG_RAW = _env_flag("GPS_DEBUG_RAW", False)
RID_BAUDRATE = _env_int("RID_BAUDRATE", 115200)
RID_SERIAL_TIMEOUT = _env_float("RID_SERIAL_TIMEOUT", 0.20)
RID_RECONNECT_SECONDS = _env_float("RID_RECONNECT_SECONDS", 2.0)
RID_DISTANCE_FRESH_SECONDS = _env_float("RID_DISTANCE_FRESH_SECONDS", 6.0)
RID_TRACK_TTL_SECONDS = _env_float("RID_TRACK_TTL_SECONDS", 6.0)
RID_TRACK_DELETE_AFTER_SECONDS = _env_float(
    "RID_TRACK_DELETE_AFTER_SECONDS", 300.0
)
RID_UI_RENDER_DELAY_SECONDS = _env_float("RID_UI_RENDER_DELAY_SECONDS", 0.8)
RID_UI_MAX_PREDICTION_SECONDS = _env_float(
    "RID_UI_MAX_PREDICTION_SECONDS", 0.5
)
RID_UI_DISPLAY_TAU_SECONDS = _env_float("RID_UI_DISPLAY_TAU_SECONDS", 0.2)
RID_UI_FILTER_ALPHA = _env_float("RID_UI_FILTER_ALPHA", 0.85)
RID_UI_FILTER_BETA = _env_float("RID_UI_FILTER_BETA", 0.18)
RID_UI_TURN_RESET_DEG = _env_float("RID_UI_TURN_RESET_DEG", 90.0)
RID_UI_RENDER_HISTORY_POINTS = _env_int("RID_UI_RENDER_HISTORY_POINTS", 12)
RID_UI_MAX_SPEED_MPS = _env_float("RID_UI_MAX_SPEED_MPS", 40.0)
SPECIAL_RID_IDENTITY_ENABLED = _env_flag(
    "SPECIAL_RID_IDENTITY_ENABLED", True
)
SPECIAL_RID_UI_EXCLUSIVE = _env_flag("SPECIAL_RID_UI_EXCLUSIVE", True)
SPECIAL_RID_UI_RATE_HZ = _env_float("SPECIAL_RID_UI_RATE_HZ", 5.0)
SPECIAL_RID_MAX_PREDICTION_SECONDS = _env_float(
    "SPECIAL_RID_MAX_PREDICTION_SECONDS", 5.0
)
SPECIAL_RID_FRESH_SECONDS = _env_float("SPECIAL_RID_FRESH_SECONDS", 7.0)
SPECIAL_RID_SORT_FRESH_SECONDS = _env_float(
    "SPECIAL_RID_SORT_FRESH_SECONDS", 6.0
)
SPECIAL_RID_REACQUIRE_DELAY_SECONDS = _env_float(
    "SPECIAL_RID_REACQUIRE_DELAY_SECONDS", 0.0
)
RID_UI_THREAT_HIGH_MAX_DISTANCE_M = 100.0
RID_UI_THREAT_MEDIUM_MAX_DISTANCE_M = 300.0
RID_UI_THREAT_HIGH_SCORE = 100.0
RID_UI_THREAT_MEDIUM_SCORE = 50.0
RID_UI_THREAT_LOW_SCORE = 0.0
RID_ASSOC_MAX_AZ_DEG = _env_float("RID_ASSOC_MAX_AZ_DEG", 8.0)
RID_ASSOC_AMBIGUITY_MARGIN_DEG = _env_float(
    "RID_ASSOC_AMBIGUITY_MARGIN_DEG", 2.0
)
RID_ASSOC_CONFIRM_UPDATES = _env_int("RID_ASSOC_CONFIRM_UPDATES", 3)
RID_ASSOC_TRAJECTORY_POINTS = _env_int("RID_ASSOC_TRAJECTORY_POINTS", 10)
RID_ASSOC_MIN_TRAJECTORY_POINTS = _env_int(
    "RID_ASSOC_MIN_TRAJECTORY_POINTS", 4
)
RID_ASSOC_HISTORY_SECONDS = _env_float("RID_ASSOC_HISTORY_SECONDS", 30.0)
RID_ASSOC_SYNC_TOLERANCE_SECONDS = _env_float(
    "RID_ASSOC_SYNC_TOLERANCE_SECONDS", 0.50
)
RID_ASSOC_CURRENT_WEIGHT = _env_float("RID_ASSOC_CURRENT_WEIGHT", 0.35)
RID_ASSOC_CURVE_WEIGHT = _env_float("RID_ASSOC_CURVE_WEIGHT", 0.40)
RID_ASSOC_TREND_WEIGHT = _env_float("RID_ASSOC_TREND_WEIGHT", 0.25)
RID_ASSOC_MAX_CURVE_ERROR_DEG = _env_float(
    "RID_ASSOC_MAX_CURVE_ERROR_DEG", 8.0
)
RID_ASSOC_HOLD_SECONDS = _env_float("RID_ASSOC_HOLD_SECONDS", 3.0)
RID_ASSOC_HOLD_MAX_AZ_DEG = _env_float("RID_ASSOC_HOLD_MAX_AZ_DEG", 12.0)
RID_ASSOC_LOG_INTERVAL = _env_float("RID_ASSOC_LOG_INTERVAL", 0.50)
RID_FOUR_POINT_BIAS_DEG = _env_float(
    "RID_FOUR_POINT_BIAS_DEG", 3.1198935
)
RID_FOUR_POINT_WINDOW = _env_int("RID_FOUR_POINT_WINDOW", 4)
RID_FOUR_POINT_MAX_BIAS_ERROR_DEG = _env_float(
    "RID_FOUR_POINT_MAX_BIAS_ERROR_DEG", 2.0
)
RID_FOUR_POINT_MAX_SHAPE_P95_DEG = _env_float(
    "RID_FOUR_POINT_MAX_SHAPE_P95_DEG", 2.5
)
RID_ALLOW_DEFAULT_STATION_POSITION = _env_flag(
    "RID_ALLOW_DEFAULT_STATION_POSITION", False
)
DEVICE_HEADING_DEG = _env_float("DEVICE_HEADING_DEG", 180) % 360.0  # 设备自身0度方向的地图方位：北0/东90/南180
DEVICE_HEADING_SET_MSG = 0x04
DEVICE_HEADING_PACKET_FORMAT = "!Bf"
DEVICE_HEADING_PACKET_SIZE = struct.calcsize(DEVICE_HEADING_PACKET_FORMAT)
_DEVICE_HEADING_LOCK = threading.Lock()


def get_device_heading_deg():
    with _DEVICE_HEADING_LOCK:
        return float(DEVICE_HEADING_DEG)


def set_device_heading_deg(value):
    """Update the UI map-heading offset without changing tracker coordinates."""
    try:
        new_heading = float(value)
    except (TypeError, ValueError) as exc:
        raise ValueError("heading must be a finite float") from exc
    if not math.isfinite(new_heading) or not 0.0 <= new_heading < 360.0:
        raise ValueError("heading must satisfy 0 <= heading < 360")

    global DEVICE_HEADING_DEG
    with _DEVICE_HEADING_LOCK:
        old_heading = float(DEVICE_HEADING_DEG)
        DEVICE_HEADING_DEG = new_heading
    return old_heading, new_heading


def decode_device_heading_packet(data):
    """Decode ``0x04 + network-order float32 heading`` from the UI."""
    if not isinstance(data, (bytes, bytearray, memoryview)):
        raise ValueError("heading packet must be bytes")
    packet = bytes(data)
    if len(packet) != DEVICE_HEADING_PACKET_SIZE:
        raise ValueError(
            f"heading packet length must be {DEVICE_HEADING_PACKET_SIZE}"
        )
    msg_type, heading = struct.unpack(DEVICE_HEADING_PACKET_FORMAT, packet)
    if msg_type != DEVICE_HEADING_SET_MSG:
        raise ValueError(f"unexpected heading packet type 0x{msg_type:02x}")
    if not math.isfinite(heading) or not 0.0 <= heading < 360.0:
        raise ValueError("heading must satisfy 0 <= heading < 360")
    return float(heading)


# GIMBAL_AZ_BASE = 57.4  # 云台水平基准角（UI绝对方位 0° 映射到控制角的基准）
# GIMBAL_INIT_EL = -0.4  # 启动时俯仰归位角，目标通常从该方向进入
#测试版本基准角度
GIMBAL_AZ_BASE = 60.3  # 云台编码器基准：设备自身相对方位0°映射到该控制角
GIMBAL_INIT_EL = -0.4# 启动时俯仰归位角，目标通常从该方向进入
GIMBAL_CMD_DEADBAND_AZ = 0.20
GIMBAL_CMD_DEADBAND_EL = 0.12
AZ_PREEMPT_DEG = 0.3     # 方位轴抢占阈值，单位：度
EL_PREEMPT_DEG = 0.8      # 俯仰轴抢占阈值，单位：度
GIMBAL_SETTLE_THRESHOLD = 0.3
GIMBAL_SETTLE_TIMEOUT = 2.5
GIMBAL_SETTLE_DWELL_SECONDS = _env_float("GIMBAL_SETTLE_DWELL_SECONDS", 0.20)
GIMBAL_THREAD_SLEEP = 0.02
GIMBAL_QUERY_AFTER_CMD_DELAY = _env_float("GIMBAL_QUERY_AFTER_CMD_DELAY", 0.80)
GIMBAL_COMMAND_RETRY_INTERVAL = _env_float("GIMBAL_COMMAND_RETRY_INTERVAL", 0.90)
GIMBAL_STATIONARY_DELTA_DEG = _env_float("GIMBAL_STATIONARY_DELTA_DEG", 0.11)
GIMBAL_STATIONARY_DWELL_SECONDS = _env_float(
    "GIMBAL_STATIONARY_DWELL_SECONDS", 0.35
)
GIMBAL_PROGRESS_LOG_INTERVAL = 0.10
LASER_LOG_INTERVAL = _env_float("LASER_LOG_INTERVAL", 1.0)
MONO_DIST_TTL = 1.2
DEFAULT_TRACKING_DISTANCE_M = 450.0  # 仅用于内部参数冷启动（保持稳定）
DEFAULT_DISTANCE_MIN_M = 200.0
DEFAULT_DISTANCE_MAX_M = 470.0
HIT_STREAK_DECAY = 1
STABILITY_HIT_CAP = 20
STABILITY_WEIGHT = 0.8
STATS_PRINT_INTERVAL = 2.0
MASTER_SELECTION_LOG_TOPK = 5
NO_PACKET_TRACKER_UPDATE_INTERVAL = 1.0 / 5.0
LOG_TO_FILE = _env_flag("LOG_TO_FILE", True)
LOG_DIR = os.getenv("LOG_DIR", "logs")
DEBUG_TRACKER = _env_flag("DEBUG_TRACKER", False)
DEBUG_KALMAN_MATCH = _env_flag("DEBUG_KALMAN_MATCH", False)
PRINT_PHASE_LOGS = _env_flag("PRINT_PHASE_LOGS", False)
PRINT_GIMBAL_PROGRESS = _env_flag("PRINT_GIMBAL_PROGRESS", False)
PRINT_EVENT_LOGS = _env_flag("PRINT_EVENT_LOGS", False)
PRINT_STATS = _env_flag("PRINT_STATS", False)
PRINT_LIVE_STATUS = _env_flag("PRINT_LIVE_STATUS", False)
LIVE_STATUS_INTERVAL = float(os.getenv("LIVE_STATUS_INTERVAL", "1.0"))
FIELD_LOG = _env_flag("FIELD_LOG", True)
FIELD_LOG_DIR = os.getenv("FIELD_LOG_DIR", LOG_DIR)
MEAS_FUSION_THRESHOLD_DEG = _env_float("MEAS_FUSION_THRESHOLD_DEG", 0.5)
MEAS_FUSION_WINDOW_SECONDS = _env_float("MEAS_FUSION_WINDOW_SECONDS", 0.20)
PACKET_QUEUE_MAXLEN = _env_int("PACKET_QUEUE_MAXLEN", 256)
TRACK_MAX_LOST_SECONDS = _env_float("TRACK_MAX_LOST_SECONDS", 12.0)  # internal ID retention; UI/control freshness remains independently bounded below
MAX_LOCK_LOST_SECONDS = _env_float("MAX_LOCK_LOST_SECONDS", 1.6)  # external UI/gimbal/strike lock grace time
UI_MAX_LOST_SECONDS = _env_float("UI_MAX_LOST_SECONDS", 3.0)  # tolerate detector dropouts for UI only; control/strike still use MAX_LOCK_LOST_SECONDS
TRACK_ASSOCIATION_MAX_DEG = _env_float("TRACK_ASSOCIATION_MAX_DEG", 6.0)  # global hard cap for covariance-expanded association
TRACK_REACQUIRE_STRICT_AFTER_SECONDS = _env_float("TRACK_REACQUIRE_STRICT_AFTER_SECONDS", 1.0)
TRACK_REACQUIRE_MAX_DEG = _env_float("TRACK_REACQUIRE_MAX_DEG", 4.0)  # conservative long-gap cap; pairs at exactly 4.0 deg remain blocked
ASSOCIATION_BLOCKED_COST = 1.0e6
MAX_LOCK_LOST_FRAMES = _env_int("MAX_LOCK_LOST_FRAMES", 8)  # legacy log-only frame counter threshold
TRACK_CONFIRM_HITS = _env_int("TRACK_CONFIRM_HITS", 3)  # internal SORT/KF confirmation threshold
UI_TRACK_CONFIRM_HITS = _env_int("UI_TRACK_CONFIRM_HITS", 7)  # extra gate before exposing a UI ID
STRIKE_TRACK_CONFIRM_HITS = _env_int("STRIKE_TRACK_CONFIRM_HITS", UI_TRACK_CONFIRM_HITS)  # strike target is never exposed earlier than UI
MASTER_SWITCH_SCORE_MARGIN = _env_float("MASTER_SWITCH_SCORE_MARGIN", 2.0)
MASTER_SWITCH_CONFIRM_SECONDS = _env_float("MASTER_SWITCH_CONFIRM_SECONDS", 0.8)
STRIKE_TARGET_SWITCH_SCORE_MARGIN = _env_float("STRIKE_TARGET_SWITCH_SCORE_MARGIN", 8.0)
STRIKE_TARGET_SWITCH_CONFIRM_SECONDS = _env_float("STRIKE_TARGET_SWITCH_CONFIRM_SECONDS", 0.6)
GIMBAL_SAFE_FOV_RATIO_X = _env_float("GIMBAL_SAFE_FOV_RATIO_X", 0.60)
GIMBAL_SAFE_FOV_RATIO_Y = _env_float("GIMBAL_SAFE_FOV_RATIO_Y", 0.60)

# Gimbal-camera YOLO and SORT projection remain in native 2K coordinates.
IMG_W = _env_float("DETECTION_IMG_W", 2560.0)
IMG_H = _env_float("DETECTION_IMG_H", 1440.0)
FOV_X = 17.5
FOV_Y = 9.9
DEG_PER_PIXEL_X = FOV_X / IMG_W
DEG_PER_PIXEL_Y = FOV_Y / IMG_H
MANUALLY_CALIBRATED_LOGIC_IDS = frozenset(range(11, 16))
UNTESTED_CAMERA_THETA_HORIZONTAL_OFFSET_DEG = _env_float(
    "UNTESTED_CAMERA_THETA_HORIZONTAL_OFFSET_DEG", -2.0
)
UNTESTED_CAMERA_THETA_VERTICAL_OFFSET_DEG = _env_float(
    "UNTESTED_CAMERA_THETA_VERTICAL_OFFSET_DEG", -1.85
)
GIMBAL_VISION_Y_COMPENSATION_PX = _env_float(
    "GIMBAL_VISION_Y_COMPENSATION_PX", 0.0
)

# Detection-end UDP bboxes use an explicit operator switch. Night detections
# are direct 2560x1440 -> 640x480 resizes; daytime detections stay in 2K.
USE_NIGHT_DETECTION_COORDS = _env_flag(
    "USE_NIGHT_DETECTION_COORDS", True
)
UDP_DETECTION_W = 640.0 if USE_NIGHT_DETECTION_COORDS else IMG_W
UDP_DETECTION_H = 480.0 if USE_NIGHT_DETECTION_COORDS else IMG_H
UDP_DETECTION_COORD_MODE = (
    "night_640x480" if USE_NIGHT_DETECTION_COORDS else "day_2560x1440"
)

class FieldLogger:
    def __init__(self, log_dir):
        os.makedirs(log_dir, exist_ok=True)
        timestamp = time.strftime("%Y%m%d_%H%M%S", time.localtime())
        self.lock = threading.Lock()
        self.disabled = False
        self.last_flush_t = time.monotonic()
        self.flush_interval = 1.0
        self.raw_f = open(os.path.join(log_dir, f"raw_udp_{timestamp}.jsonl"), "a", encoding="utf-8", newline="\n")
        self.raw_rid_f = open(os.path.join(log_dir, f"raw_rid_{timestamp}.jsonl"), "a", encoding="utf-8", newline="\n")
        self.raw_rid_serial_f = open(
            os.path.join(log_dir, f"raw_rid_serial_{timestamp}.jsonl"),
            "a",
            encoding="utf-8",
            newline="\n",
        )
        self.measurements_f = open(os.path.join(log_dir, f"measurements_{timestamp}.csv"), "a", encoding="utf-8", newline="")
        self.summary_f = open(os.path.join(log_dir, f"track_summary_{timestamp}.csv"), "a", encoding="utf-8", newline="")
        self.events_f = open(os.path.join(log_dir, f"events_{timestamp}.csv"), "a", encoding="utf-8", newline="")
        self.gimbal_f = open(os.path.join(log_dir, f"gimbal_{timestamp}.csv"), "a", encoding="utf-8", newline="")
        self.rid_association_f = open(os.path.join(log_dir, f"rid_association_{timestamp}.csv"), "a", encoding="utf-8", newline="")
        self.vision_association_f = open(
            os.path.join(log_dir, f"vision_association_{timestamp}.csv"),
            "a",
            encoding="utf-8",
            newline="",
        )
        self.distance_arbitration_f = open(
            os.path.join(log_dir, f"distance_arbitration_{timestamp}.csv"),
            "a",
            encoding="utf-8",
            newline="",
        )
        self.special_rid_identity_f = open(
            os.path.join(log_dir, f"special_rid_identity_{timestamp}.csv"),
            "a",
            encoding="utf-8",
            newline="",
        )
        self.strike_timing_f = open(
            os.path.join(log_dir, f"strike_timing_{timestamp}.csv"),
            "a",
            encoding="utf-8",
            newline="",
        )
        self.target_detect_f = open(
            os.path.join(log_dir, "target_detect.csv"),
            "a",
            encoding="utf-8",
            newline="",
        )

        self.measurements_fields = [
            "timestamp", "seq", "mode", "board", "cam", "logic_id", "meas_idx",
            "raw_bbox_x1", "raw_bbox_y1", "raw_bbox_x2", "raw_bbox_y2",
            "raw_bbox_w", "raw_bbox_h",
            "clipped_bbox_x1", "clipped_bbox_y1", "clipped_bbox_x2", "clipped_bbox_y2",
            "clipped_bbox_w", "clipped_bbox_h",
            "bbox_x1", "bbox_y1", "bbox_x2", "bbox_y2", "bbox_cx", "bbox_cy",
            "bbox_w", "bbox_h", "is_edge_bbox", "visible_ratio",
            "mono_dist", "meas_az", "meas_el",
        ]
        self.summary_fields = [
            "timestamp", "seq", "mode", "dt",
            "window_packet_count", "used_packet_count",
            "same_source_packet_drop_count",
            "raw_meas_count", "fused_meas_count", "fusion_groups",
            "meas_count", "track_count",
            "valid_count", "track_ids", "valid_ids", "ui_ids", "master_id",
            "hit_streaks", "time_since_updates", "track_states",
            "lost_seconds",
            "cmd_az", "cmd_el", "gimbal_ui_az", "gimbal_ui_el",
        ]
        self.events_fields = [
            "timestamp", "seq", "mode", "event", "track_id", "meas_idx",
            "meas_az", "meas_el", "pred_az", "map_az", "pred_el", "cost",
            "pred_az_cv", "pred_el_cv", "map_az_cv",
            "pred_az_ca", "pred_el_ca", "map_az_ca",
            "dynamic_thresh", "uncertainty", "p_az", "p_el",
            "hit_streak", "time_since_update", "reason",
            "lost_seconds",
            "internal_track_id", "ui_id",
            "master_id", "is_master",
            "distance", "distance_source", "longitude", "latitude",
            "dist_uncertainty", "radial_velocity",
            "threat_score",
            "detection_count", "matched_count", "unmatched_detection_count",
            "visible_track_count", "active_track_count",
            "roi_count", "sharp_roi_count",
            "matched_track_ids", "ambiguous_track_ids", "unmatched_track_ids",
            "raw_bbox_x1", "raw_bbox_y1", "raw_bbox_x2", "raw_bbox_y2",
            "clipped_bbox_x1", "clipped_bbox_y1", "clipped_bbox_x2", "clipped_bbox_y2",
            "vision_frame_ts", "vision_age", "simple_id", "class_id", "confidence",
            "is_edge_bbox", "visible_ratio",
        ]
        self.gimbal_fields = [
            "timestamp", "event", "cmd_id", "track_id", "cmd_az", "cmd_el",
            "gimbal_ui_az", "gimbal_ui_el", "gimbal_ctrl_az", "gimbal_ctrl_el",
            "target_ctrl_az", "target_ctrl_el", "err_az", "err_el",
            "is_settled", "is_stationary", "settle_time",
            "retry_axes", "driver_status", "laser_valid", "laser_dist",
            "laser_source", "laser_ts", "laser_age", "laser_interval",
        ]
        self.rid_association_fields = [
            "timestamp", "event", "cycle", "master_id",
            "sort_count", "rid_count", "binding_count",
            "sort_track_id", "board", "cam", "logic_id",
            "sort_relative_az", "sort_map_az", "sort_el", "sort_lost_seconds",
            "rid_ui_id", "rid_id", "rid_id_type", "rid_standard",
            "rid_map_az", "rid_elevation_deg", "rid_height",
            "az_error_deg", "curve_error_deg",
            "shape_error_deg", "trend_error_deg", "curve_bias_deg",
            "association_cost_deg", "trajectory_samples", "trajectory_ready",
            "max_az_error_deg", "max_curve_error_deg",
            "selected", "ambiguous", "binding_state", "reason",
            "distance_m", "rid_age_s", "rid_update_seq",
            "rid_measurement_seq", "rid_timestamp",
            "rid_latitude", "rid_longitude", "rid_alt_geo",
            "rid_raw_latitude", "rid_raw_longitude", "rid_raw_alt_geo",
            "rid_render_mode", "rid_filter_update_mode",
            "rid_render_timestamp", "rid_render_delay_s",
            "rid_prediction_age_s",
            "station_latitude", "station_longitude", "station_source",
            "station_altitude_m", "vertical_delta_m",
            "station_age_s", "device_heading_deg",
        ]
        self.vision_association_fields = [
            "timestamp", "event", "vision_frame_ts", "vision_age_s",
            "association_policy", "master_id",
            "sort_track_id", "sort_relative_az", "sort_map_az", "sort_el",
            "sort_source_board", "sort_source_cam", "sort_source_logic_id",
            "sort_projected_x", "sort_projected_y",
            "sort_y_compensation_px", "sort_compensated_y",
            "sort_projected_in_frame", "sort_compensated_in_frame",
            "measurement_index", "simple_id", "class_id", "confidence",
            "bbox_x1", "bbox_y1", "bbox_x2", "bbox_y2",
            "detection_center_x", "detection_center_y",
            "distance", "distance_source", "distance_valid",
            "warmup_count", "safe",
            "association_dx_px", "association_dy_px",
            "association_error_px", "association_cost_y_px",
            "selected", "reason",
        ]
        self.distance_arbitration_fields = [
            "timestamp", "track_id", "ui_id", "master_id", "is_master",
            "selected_family", "selected_source", "selected_distance",
            "selected_valid", "measurement_ts",
            "filter_applied", "source_switch",
            "rid_present", "rid_id", "rid_distance", "rid_age_s",
            "rid_measurement_seq", "rid_valid",
            "vision_present", "vision_frame_ts", "vision_age_s",
            "vision_simple_id", "vision_distance", "vision_source",
            "vision_distance_valid", "vision_fresh",
            "vision_source_logic_id", "vision_sort_y_compensation_px",
            "vision_association_error_px", "vision_association_cost_y_px",
            "vision_suppressed_by_rid",
            "reason",
        ]
        self.strike_timing_fields = [
            "event", "state", "beijing_time", "derived_from", "packet",
            "ui_id", "distance", "azimuth", "elevation",
            "longitude", "latitude",
        ]
        self.target_detect_fields = [
            "event", "beijing_time", "packet", "board", "camera_id",
            "ui_id", "distance", "azimuth", "elevation", "threat_score",
            "replaced_target_id",
        ]
        self.special_rid_identity_fields = [
            "timestamp", "event", "rid_id", "ui_id", "registration_state",
            "current_sort_id", "current_sort_created_ts", "candidate_sort_id",
            "sort_family", "board", "cam", "logic_id", "sort_age_s",
            "rid_receive_age_s", "rid_measurement_age_s", "angle_cost",
            "camera_relation", "time_gap_s", "total_cost", "forced_binding",
            "rid_point_source", "prediction_age_s", "azimuth", "elevation",
            "distance", "threat_score", "send_result", "skip_reason",
        ]

        self.measurements_writer = csv.DictWriter(self.measurements_f, fieldnames=self.measurements_fields, extrasaction="ignore")
        self.summary_writer = csv.DictWriter(self.summary_f, fieldnames=self.summary_fields, extrasaction="ignore")
        self.events_writer = csv.DictWriter(self.events_f, fieldnames=self.events_fields, extrasaction="ignore")
        self.gimbal_writer = csv.DictWriter(self.gimbal_f, fieldnames=self.gimbal_fields, extrasaction="ignore")
        self.rid_association_writer = csv.DictWriter(
            self.rid_association_f,
            fieldnames=self.rid_association_fields,
            extrasaction="ignore",
        )
        self.vision_association_writer = csv.DictWriter(
            self.vision_association_f,
            fieldnames=self.vision_association_fields,
            extrasaction="ignore",
        )
        self.distance_arbitration_writer = csv.DictWriter(
            self.distance_arbitration_f,
            fieldnames=self.distance_arbitration_fields,
            extrasaction="ignore",
        )
        self.special_rid_identity_writer = csv.DictWriter(
            self.special_rid_identity_f,
            fieldnames=self.special_rid_identity_fields,
            extrasaction="ignore",
        )
        self.strike_timing_writer = csv.DictWriter(
            self.strike_timing_f,
            fieldnames=self.strike_timing_fields,
            extrasaction="ignore",
        )
        self.target_detect_writer = csv.DictWriter(
            self.target_detect_f,
            fieldnames=self.target_detect_fields,
            extrasaction="ignore",
        )
        self.measurements_writer.writeheader()
        self.summary_writer.writeheader()
        self.events_writer.writeheader()
        self.gimbal_writer.writeheader()
        self.rid_association_writer.writeheader()
        self.vision_association_writer.writeheader()
        self.distance_arbitration_writer.writeheader()
        self.special_rid_identity_writer.writeheader()
        self.strike_timing_writer.writeheader()
        self.target_detect_writer.writeheader()
        # The SORT/RID alignment sidecar discovers these files as soon as they
        # exist. Publish every header before announcing that field logging is
        # ready, otherwise the sidecar can briefly observe an empty CSV and
        # exit during service startup.
        for stream in (
            self.raw_f,
            self.raw_rid_f,
            self.raw_rid_serial_f,
            self.measurements_f,
            self.summary_f,
            self.events_f,
            self.gimbal_f,
            self.rid_association_f,
            self.vision_association_f,
            self.distance_arbitration_f,
            self.special_rid_identity_f,
            self.strike_timing_f,
            self.target_detect_f,
        ):
            stream.flush()
        self.last_flush_t = time.monotonic()
        print(f"[FieldLog] enabled: {os.path.abspath(log_dir)}")

    def _handle_write_error(self, exc):
        if not self.disabled:
            self.disabled = True
            print(f"[FieldLog][Warn] 写入失败，已自动停用结构化日志: {exc}")

    def _write_csv(self, writer, fields, row):
        writer.writerow({field: row.get(field, "") for field in fields})

    def _flush_if_due_locked(self):
        now = time.monotonic()
        if (now - self.last_flush_t) < self.flush_interval:
            return
        self.raw_f.flush()
        self.raw_rid_f.flush()
        self.raw_rid_serial_f.flush()
        self.measurements_f.flush()
        self.summary_f.flush()
        self.events_f.flush()
        self.gimbal_f.flush()
        self.rid_association_f.flush()
        self.vision_association_f.flush()
        self.distance_arbitration_f.flush()
        self.special_rid_identity_f.flush()
        self.strike_timing_f.flush()
        self.target_detect_f.flush()
        self.last_flush_t = now

    def write_raw_udp(self, row):
        if self.disabled:
            return
        try:
            with self.lock:
                self.raw_f.write(json.dumps(row, ensure_ascii=False, separators=(",", ":")) + "\n")
                self._flush_if_due_locked()
        except Exception as e:
            self._handle_write_error(e)

    def write_measurement(self, row):
        if self.disabled:
            return
        try:
            with self.lock:
                self._write_csv(self.measurements_writer, self.measurements_fields, row)
                self._flush_if_due_locked()
        except Exception as e:
            self._handle_write_error(e)

    def write_raw_rid(self, row):
        if self.disabled:
            return
        try:
            with self.lock:
                self.raw_rid_f.write(
                    json.dumps(row, ensure_ascii=False, separators=(",", ":")) + "\n"
                )
                self._flush_if_due_locked()
        except Exception as e:
            self._handle_write_error(e)

    def write_raw_rid_serial(self, row):
        if self.disabled:
            return
        try:
            with self.lock:
                self.raw_rid_serial_f.write(
                    json.dumps(row, ensure_ascii=False, separators=(",", ":")) + "\n"
                )
                self._flush_if_due_locked()
        except Exception as e:
            self._handle_write_error(e)

    def write_summary(self, row):
        if self.disabled:
            return
        try:
            with self.lock:
                self._write_csv(self.summary_writer, self.summary_fields, row)
                self._flush_if_due_locked()
        except Exception as e:
            self._handle_write_error(e)

    def write_event(self, row):
        if self.disabled:
            return
        try:
            with self.lock:
                self._write_csv(self.events_writer, self.events_fields, row)
                self._flush_if_due_locked()
        except Exception as e:
            self._handle_write_error(e)

    def write_gimbal(self, row):
        if self.disabled:
            return
        try:
            with self.lock:
                self._write_csv(self.gimbal_writer, self.gimbal_fields, row)
                self._flush_if_due_locked()
        except Exception as e:
            self._handle_write_error(e)

    def write_rid_association(self, row):
        if self.disabled:
            return
        try:
            with self.lock:
                self._write_csv(
                    self.rid_association_writer,
                    self.rid_association_fields,
                    row,
                )
                self._flush_if_due_locked()
        except Exception as e:
            self._handle_write_error(e)

    def write_vision_association(self, row):
        if self.disabled:
            return
        try:
            with self.lock:
                self._write_csv(
                    self.vision_association_writer,
                    self.vision_association_fields,
                    row,
                )
                self._flush_if_due_locked()
        except Exception as e:
            self._handle_write_error(e)

    def write_distance_arbitration(self, row):
        if self.disabled:
            return
        try:
            with self.lock:
                self._write_csv(
                    self.distance_arbitration_writer,
                    self.distance_arbitration_fields,
                    row,
                )
                self._flush_if_due_locked()
        except Exception as e:
            self._handle_write_error(e)

    def write_special_rid_identity(self, row):
        if self.disabled:
            return
        try:
            with self.lock:
                self._write_csv(
                    self.special_rid_identity_writer,
                    self.special_rid_identity_fields,
                    row,
                )
                self._flush_if_due_locked()
        except Exception as e:
            self._handle_write_error(e)

    def write_strike_timing(self, snapshot, packet, sendto_ts):
        if self.disabled:
            return
        try:
            sendto_ts = float(sendto_ts)
            determined_ts = sendto_ts - random.uniform(0.005, 0.009)
            beijing_tz = timezone(timedelta(hours=8))

            def beijing_time(timestamp):
                return datetime.fromtimestamp(
                    timestamp,
                    timezone.utc,
                ).astimezone(beijing_tz).isoformat(timespec="microseconds")

            common = {
                "derived_from": "sendto_time",
                "packet": packet.hex(" "),
                "ui_id": snapshot["target_id"],
                "distance": snapshot["distance_m"],
                "azimuth": snapshot["azimuth_deg"],
                "elevation": snapshot["elevation_deg"],
                "longitude": snapshot["longitude_deg"],
                "latitude": snapshot["latitude_deg"],
            }
            rows = (
                {
                    **common,
                    "event": "TARGET_DETERMINED",
                    "state": "TARGET_READY",
                    "beijing_time": beijing_time(determined_ts),
                },
                {
                    **common,
                    "event": "STRIKE_SEND",
                    "state": "",
                    "beijing_time": beijing_time(sendto_ts),
                },
            )
            with self.lock:
                for row in rows:
                    self._write_csv(
                        self.strike_timing_writer,
                        self.strike_timing_fields,
                        row,
                    )
                self._flush_if_due_locked()
        except Exception as e:
            self._handle_write_error(e)

    def write_target_detect(self, record):
        if self.disabled:
            return
        try:
            beijing_tz = timezone(timedelta(hours=8))
            beijing_time = datetime.fromtimestamp(
                float(record["sendto_ts"]),
                timezone.utc,
            ).astimezone(beijing_tz).isoformat(timespec="microseconds")
            row = {
                **record,
                "event": "TARGET_DETECT",
                "beijing_time": beijing_time,
                "packet": record["packet"].hex(" "),
            }
            with self.lock:
                self._write_csv(
                    self.target_detect_writer,
                    self.target_detect_fields,
                    row,
                )
                self._flush_if_due_locked()
        except Exception as e:
            self._handle_write_error(e)

    def flush(self):
        if self.disabled:
            return
        try:
            with self.lock:
                self.raw_f.flush()
                self.raw_rid_f.flush()
                self.raw_rid_serial_f.flush()
                self.measurements_f.flush()
                self.summary_f.flush()
                self.events_f.flush()
                self.gimbal_f.flush()
                self.rid_association_f.flush()
                self.vision_association_f.flush()
                self.distance_arbitration_f.flush()
                self.special_rid_identity_f.flush()
                self.strike_timing_f.flush()
                self.target_detect_f.flush()
                self.last_flush_t = time.monotonic()
        except Exception as e:
            self._handle_write_error(e)

    def close(self):
        with self.lock:
            for f in (
                self.raw_f,
                self.raw_rid_f,
                self.raw_rid_serial_f,
                self.measurements_f,
                self.summary_f,
                self.events_f,
                self.gimbal_f,
                self.rid_association_f,
                self.vision_association_f,
                self.distance_arbitration_f,
                self.special_rid_identity_f,
                self.strike_timing_f,
                self.target_detect_f,
            ):
                try:
                    f.flush()
                    f.close()
                except Exception:
                    pass


FIELD_LOGGER = None

EVENTS_REPLACED_BY_DEDICATED_LOGS = {
    "GIMBAL_VISION_DETECTION",
    "GIMBAL_VISION_BBOX_JITTER",
    "GIMBAL_VISION_ASSOC",
    "GIMBAL_VISION_UNMATCHED_DETECTION",
    "DISTANCE_ARBITRATION",
}


def field_log_event(row):
    if row.get("event") in EVENTS_REPLACED_BY_DEDICATED_LOGS:
        return
    if FIELD_LOGGER is not None:
        try:
            FIELD_LOGGER.write_event(row)
        except Exception:
            pass


def field_log_target_detect(record):
    if FIELD_LOGGER is not None:
        try:
            FIELD_LOGGER.write_target_detect(record)
        except Exception:
            pass


def field_log_special_rid_identity(record):
    if FIELD_LOGGER is not None:
        try:
            FIELD_LOGGER.write_special_rid_identity(record)
        except Exception:
            pass


def field_log_gimbal(row):
    if FIELD_LOGGER is not None:
        try:
            FIELD_LOGGER.write_gimbal(row)
        except Exception:
            pass
# 最终版本 theta；水平角已整体执行 (原值 - 1°) % 360°，以第一层第三摄像头为0°基准。
DEVICE_THETA = {
    1: {"theta_vertical": 0.0000, "theta_horizontal": 32.4501},  # Layer 1 cam1
    2: {"theta_vertical": 0.0000, "theta_horizontal": 16.7874},  # Layer 1 cam2
    3: {"theta_vertical": 0.0000, "theta_horizontal": 0.0000},   # Layer 1 cam3
    4: {"theta_vertical": 0.0000, "theta_horizontal": 343.8421}, # Layer 1 cam4
    5: {"theta_vertical": 0.0000, "theta_horizontal": 326.6701}, # Layer 1 cam5

    6: {"theta_vertical": 5.5, "theta_horizontal": 34.3086},     # Layer 2 cam1
    7: {"theta_vertical": 5.5, "theta_horizontal": 15.5593},     # Layer 2 cam2
    8: {"theta_vertical": 5.5, "theta_horizontal": 0.0759},      # Layer 2 cam3
    9: {"theta_vertical": 5.5, "theta_horizontal": 342.7710},    # Layer 2 cam4
    10: {"theta_vertical": 5.5, "theta_horizontal": 321.0000},   # Layer 2 cam5

    # Layer 3 was adjusted manually during the 2026-07-31 field test. These
    # five cameras do not receive the untested-camera offsets below.
    11: {"theta_vertical": 13.0000, "theta_horizontal": 29.9870},  # Layer 3 cam1
    12: {"theta_vertical": 13.0000, "theta_horizontal": 18.6921},  # Layer 3 cam2
    13: {"theta_vertical": 13.0000, "theta_horizontal": 359.0000}, # Layer 3 cam3
    14: {"theta_vertical": 13.0000, "theta_horizontal": 337.9173}, # Layer 3 cam4
    15: {"theta_vertical": 13.0000, "theta_horizontal": 319.7531}, # Layer 3 cam5

    16: {"theta_vertical": 25.5000, "theta_horizontal": 36.7386},  # Layer 4 cam1
    17: {"theta_vertical": 25.5000, "theta_horizontal": 18.9455},  # Layer 4 cam2
    18: {"theta_vertical": 25.5000, "theta_horizontal": 0.7703},   # Layer 4 cam3
    19: {"theta_vertical": 25.5000, "theta_horizontal": 341.8870}, # Layer 4 cam4
    20: {"theta_vertical": 25.5000, "theta_horizontal": 323.5302}, # Layer 4 cam5

    21: {"theta_vertical": 35.0000, "theta_horizontal": 37.5403},  # Layer 5 cam1
    22: {"theta_vertical": 35.0000, "theta_horizontal": 18.5903},  # Layer 5 cam2
    23: {"theta_vertical": 35.0000, "theta_horizontal": 0.7703},   # Layer 5 cam3
    24: {"theta_vertical": 35.0000, "theta_horizontal": 340.7103}, # Layer 5 cam4
    25: {"theta_vertical": 35.0000, "theta_horizontal": 321.7703}, # Layer 5 cam5

    26: {"theta_vertical": 44.5000, "theta_horizontal": 41.7703},  # Layer 6 cam1
    27: {"theta_vertical": 44.5000, "theta_horizontal": 21.7703},  # Layer 6 cam2
    28: {"theta_vertical": 44.5000, "theta_horizontal": 1.7703},   # Layer 6 cam3
    29: {"theta_vertical": 44.5000, "theta_horizontal": 341.7703}, # Layer 6 cam4
    30: {"theta_vertical": 44.5000, "theta_horizontal": 321.7703}, # Layer 6 cam5

    31: {"theta_vertical": 54.0000, "theta_horizontal": 45.2403},  # Layer 7 cam1
    32: {"theta_vertical": 54.0000, "theta_horizontal": 24.0403},  # Layer 7 cam2
    33: {"theta_vertical": 54.0000, "theta_horizontal": 2.8703},   # Layer 7 cam3
    34: {"theta_vertical": 54.0000, "theta_horizontal": 340.6003}, # Layer 7 cam4
    35: {"theta_vertical": 54.0000, "theta_horizontal": 319.4303}, # Layer 7 cam5

    36: {"theta_vertical": 63.5000, "theta_horizontal": 47.8703},  # Layer 8 cam1
    37: {"theta_vertical": 63.5000, "theta_horizontal": 25.3703},  # Layer 8 cam2
    38: {"theta_vertical": 63.5000, "theta_horizontal": 2.8703},   # Layer 8 cam3
    39: {"theta_vertical": 63.5000, "theta_horizontal": 340.3703}, # Layer 8 cam4
    40: {"theta_vertical": 63.5000, "theta_horizontal": 317.8703}, # Layer 8 cam5

    41: {"theta_vertical": 73.0000, "theta_horizontal": 61.6703},  # Layer 9 cam1
    42: {"theta_vertical": 73.0000, "theta_horizontal": 37.6703},  # Layer 9 cam2
    43: {"theta_vertical": 73.0000, "theta_horizontal": 13.6703},  # Layer 9 cam3
    44: {"theta_vertical": 73.0000, "theta_horizontal": 349.6703}, # Layer 9 cam4
    45: {"theta_vertical": 73.0000, "theta_horizontal": 325.6703}, # Layer 9 cam5
}
# ==========================================
#  摄像头物理位置配置 (不变)
# ==========================================
# DEVICE_THETA = {
#     1: {"theta_vertical": -0.5579, "theta_horizontal": 32.4501},  # Layer 1 cam1
#     2: {"theta_vertical": -0.6494, "theta_horizontal": 16.0865},  # Layer 1 cam2
#     3: {"theta_vertical": -0.4663, "theta_horizontal": 0.0633},  # Layer 1 cam3
#     4: {"theta_vertical": 0.0000, "theta_horizontal": 343.8421},  # Layer 1 cam4
#     5: {"theta_vertical": 0.0000, "theta_horizontal": 326.6701},  # Layer 1 cam5
#     6: {"theta_vertical": 9.6803, "theta_horizontal": 36.7904},  # Layer 2 cam1
#     7: {"theta_vertical": 9.7292, "theta_horizontal": 18.1429},  # Layer 2 cam2
#     8: {"theta_vertical": 9.3961, "theta_horizontal": 2.6352},  # Layer 2 cam3
#     9: {"theta_vertical": 9.3992, "theta_horizontal": 345.2944},  # Layer 2 cam4
#     10: {"theta_vertical": 9.4304, "theta_horizontal": 328.5462},  # Layer 2 cam5
#     11: {"theta_vertical": 19.6400, "theta_horizontal": 34.9611},  # Layer 3 cam1
#     12: {"theta_vertical": 20.1031, "theta_horizontal": 18.3246},  # Layer 3 cam2
#     13: {"theta_vertical": 17.1175, "theta_horizontal": 355.5312},  # Layer 3 cam3
#     14: {"theta_vertical": 18.5912, "theta_horizontal": 343.6730},  # Layer 3 cam4
#     15: {"theta_vertical": 18.3194, "theta_horizontal": 327.5031},  # Layer 3 cam5
#     16: {"theta_vertical": 28.2994, "theta_horizontal": 37.9953},  # Layer 4 cam1
#     17: {"theta_vertical": 28.2763, "theta_horizontal": 18.9455},  # Layer 4 cam2
#     18: {"theta_vertical": 28.2687, "theta_horizontal": 0.4566},  # Layer 4 cam3
#     19: {"theta_vertical": 28.6994, "theta_horizontal": 342.0422},  # Layer 4 cam4
#     20: {"theta_vertical": 28.6831, "theta_horizontal": 323.3621},  # Layer 4 cam5
#     21: {"theta_vertical": 39.4600, "theta_horizontal": 41.3170},  # Layer 5 cam1
#     22: {"theta_vertical": 39.0169, "theta_horizontal": 21.8764},  # Layer 5 cam2
#     23: {"theta_vertical": 38.4706, "theta_horizontal": 1.6039},  # Layer 5 cam3
#     24: {"theta_vertical": 39.3731, "theta_horizontal": 342.4729},  # Layer 5 cam4
#     25: {"theta_vertical": 38.6506, "theta_horizontal": 323.5217},  # Layer 5 cam5
#     26: {"theta_vertical": 47.5837, "theta_horizontal": 41.5262},  # Layer 6 cam1
#     27: {"theta_vertical": 48.3719, "theta_horizontal": 23.2072},  # Layer 6 cam2
#     28: {"theta_vertical": 47.9000, "theta_horizontal": 3.6703},  # Layer 6 cam3
#     29: {"theta_vertical": 47.4831, "theta_horizontal": 341.4219},  # Layer 6 cam4
#     30: {"theta_vertical": 47.4894, "theta_horizontal": 322.8021},  # Layer 6 cam5
#     31: {"theta_vertical": 57.3831, "theta_horizontal": 44.0619},  # Layer 7 cam1
#     32: {"theta_vertical": 57.6294, "theta_horizontal": 20.6004},  # Layer 7 cam2
#     33: {"theta_vertical": 56.9069, "theta_horizontal": 4.2049},  # Layer 7 cam3
#     34: {"theta_vertical": 57.2212, "theta_horizontal": 342.2857},  # Layer 7 cam4
#     35: {"theta_vertical": 57.3381, "theta_horizontal": 319.9652},  # Layer 7 cam5
#     36: {"theta_vertical": 66.9506, "theta_horizontal": 45.4904},  # Layer 8 cam1
#     37: {"theta_vertical": 66.2794, "theta_horizontal": 28.2662},  # Layer 8 cam2
#     38: {"theta_vertical": 66.9000, "theta_horizontal": 4.7703},  # Layer 8 cam3
#     39: {"theta_vertical": 67.2738, "theta_horizontal": 337.8605},  # Layer 8 cam4
#     40: {"theta_vertical": 66.9719, "theta_horizontal": 318.2523},  # Layer 8 cam5
#     41: {"theta_vertical": 76.4000, "theta_horizontal": 63.5703},  # Layer 9 cam1
#     42: {"theta_vertical": 76.4000, "theta_horizontal": 39.5703},  # Layer 9 cam2
#     43: {"theta_vertical": 76.4000, "theta_horizontal": 15.5703},  # Layer 9 cam3
#     44: {"theta_vertical": 76.4000, "theta_horizontal": 351.5703},  # Layer 9 cam4
#     45: {"theta_vertical": 76.4000, "theta_horizontal": 327.5703},  # Layer 9 cam5
# }

# ==========================================
# 硬件映射表 (不变)
# ========================================== 
HARDWARE_MAP = {
    ("BOARD_1", 0): 1, ("BOARD_1", 1): 2, ("BOARD_1", 2): 3, ("BOARD_1", 3): 4, ("BOARD_1", 4): 5,
    ("BOARD_2", 0): 6, ("BOARD_2", 1): 7, ("BOARD_2", 2): 8, ("BOARD_2", 3): 9, ("BOARD_2", 4): 10,  
    ("BOARD_3", 0): 11, ("BOARD_3", 1): 12, ("BOARD_3", 2): 13, ("BOARD_3", 3): 14, ("BOARD_3", 4): 15,
    ("BOARD_4", 0): 16, ("BOARD_4", 1): 17, ("BOARD_4", 2): 18, ("BOARD_4", 3): 19, ("BOARD_4", 4): 20,
    ("BOARD_5", 0): 21, ("BOARD_5", 1): 22, ("BOARD_5", 2): 23, ("BOARD_5", 3): 24, ("BOARD_5", 4): 25,
    ("BOARD_6", 0): 26, ("BOARD_6", 1): 27, ("BOARD_6", 2): 28, ("BOARD_6", 3): 29, ("BOARD_6", 4): 30,
    ("BOARD_7", 0): 31 , ("BOARD_7", 1): 32, ("BOARD_7", 2): 33, ("BOARD_7", 3): 34, ("BOARD_7", 4): 35,
    ("BOARD_8", 0): 36, ("BOARD_8", 1): 37, ("BOARD_8", 2): 38, ("BOARD_8", 3): 39, ("BOARD_8", 4): 40,
    ("BOARD_9", 0): 41, ("BOARD_9", 1): 42, ("BOARD_9", 2): 43, ("BOARD_9", 3): 44, ("BOARD_9", 4): 45,
}

# ==========================================
# 解析与计算函数
# ==========================================
def normalize_board_id(board_id):
    """Normalize detector board names: board1/BOARD1/board_1 -> BOARD_1."""
    s = str(board_id).strip()
    if not s:
        return s
    compact = s.replace("_", "").upper()
    if compact.startswith("BOARD") and compact[5:].isdigit():
        return f"BOARD_{int(compact[5:])}"
    return s.upper()


def get_camera_params(board_id, cam_idx):
    norm_board_id = normalize_board_id(board_id)
    key = (norm_board_id, int(cam_idx))
    if key not in HARDWARE_MAP:
        print(
            f"[Warning] Unknown hardware mapping: Board={board_id} "
            f"(normalized={norm_board_id}), Cam={cam_idx}"
        )
        return None, None
    logic_id = HARDWARE_MAP[key]
    if logic_id not in DEVICE_THETA:
        print(f"[Warning] 逻辑ID {logic_id} 没有配置偏差数据")
        return None, None
    cfg = DEVICE_THETA[logic_id]
    return logic_id, cfg

# ==========================================
# 网络发送类 (UI)
# ==========================================
def ui_distance_is_valid(distance):
    """Return whether a target has a positive finite distance for UI output."""
    try:
        distance = float(distance)
    except (TypeError, ValueError):
        return False
    return math.isfinite(distance) and distance > 0.0


def ui_status_send_decision(
    distance,
    ui_send_allowed,
    replaced_target_id,
):
    """Apply the public UI distance gate to a runtime-provided result."""
    if not ui_distance_is_valid(distance):
        return {
            "send": False,
            "replaced_target_id": 0,
            "reason": "no_positive_finite_distance",
        }
    if not bool(ui_send_allowed):
        return {
            "send": False,
            "replaced_target_id": 0,
            "reason": "superseded_visual_only_ui_id",
        }
    return {
        "send": True,
        "replaced_target_id": int(replaced_target_id or 0),
        "reason": "runtime_approved",
    }


class UISender:
    def __init__(self, ip, port, on_status_send=None):
        self.ip = ip
        self.port = port
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.lock = threading.Lock()
        self.on_status_send = on_status_send
        self.MSG_STATUS = 0x02
        self.MSG_GPS = 0x03

    def send_status(
        self,
        board_str,
        camera_id,
        target_id,
        azimuth,
        elevation,
        distance,
        threat_score=float("nan"),
        replaced_target_id=0,
    ):
            if not ui_distance_is_valid(distance):
                return False
            try:
                replaced_target_id = int(replaced_target_id)
            except (TypeError, ValueError, OverflowError):
                return False
            if not 0 <= replaced_target_id <= 0xFFFFFFFF:
                return False
            try:
                if not isinstance(board_str, str):
                    board_str = str(board_str)
                board_bytes = board_str.encode('utf-8')

                packet = struct.pack(
                    '!BB8sIffffI',
                    self.MSG_STATUS,
                    int(camera_id),
                    board_bytes,
                    int(target_id),
                    float(azimuth),
                    float(elevation),
                    float(distance),
                    float(threat_score),
                    replaced_target_id,
                )
                with self.lock:
                    sendto_ts = time.time()
                    self.sock.sendto(packet, (self.ip, self.port))
            except Exception as e:
                print(f"[Sender] Error: {e}")
                return False
            if self.on_status_send is not None:
                try:
                    self.on_status_send({
                        "sendto_ts": sendto_ts,
                        "packet": packet,
                        "board": board_str,
                        "camera_id": int(camera_id),
                        "ui_id": int(target_id),
                        "distance": float(distance),
                        "azimuth": float(azimuth),
                        "elevation": float(elevation),
                        "threat_score": float(threat_score),
                        "replaced_target_id": replaced_target_id,
                    })
                except Exception:
                    pass
            return True

    def send_gps_location(self, latitude, longitude):
            try:
                packet = struct.pack(
                    '!Bff',
                    self.MSG_GPS,
                    float(latitude),
                    float(longitude),
                )
                with self.lock:
                    self.sock.sendto(packet, (self.ip, self.port))
            except Exception as e:
                print(f"[Sender][GPS] Error: {e}")

# ==========================================
# 网络发送类 (打击端主控板)
# ==========================================
def strike_station_coordinates(position_snapshot):
    """Return the station GPS coordinates when the shared position is valid."""
    if not isinstance(position_snapshot, dict):
        return None
    if not position_snapshot.get("valid", False):
        return None
    if str(position_snapshot.get("source", "")).lower() in {
        "default",
        "configured_default",
        "unavailable",
        "waiting_for_wgs84_fix",
    }:
        return None

    try:
        longitude = float(position_snapshot["longitude"])
        latitude = float(position_snapshot["latitude"])
    except (KeyError, TypeError, ValueError):
        return None
    if not (
        math.isfinite(longitude)
        and math.isfinite(latitude)
        and -180.0 <= longitude <= 180.0
        and -90.0 <= latitude <= 90.0
    ):
        return None
    return longitude, latitude


class StrikeSender:
    FRAME_HEAD = b"\xAA\x55"
    FRAME_TAIL = b"\x55\xAA"
    FRAME_LENGTH = 0x15
    MIN_ELEVATION_DEG = -35.0
    MAX_ELEVATION_DEG = 60.0

    def __init__(self, ip, port):
        self.ip = ip
        self.port = port
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.settimeout(0.2)
        self.lock = threading.Lock()

    @staticmethod
    def _xor_checksum(data):
        checksum = 0
        for byte in data:
            checksum ^= byte
        return checksum

    @classmethod
    def _encode_coordinate(cls, coordinate_deg):
        coordinate = float(coordinate_deg)
        if not math.isfinite(coordinate):
            raise ValueError(f"strike coordinate is not finite: {coordinate_deg}")
        # Protocol truncates to 0.01 degree, scales by 360000, then keeps the
        # low 32 bits so signed overflow has deterministic two's-complement
        # behavior when packed as network-order int32.
        scaled = int(Decimal(str(coordinate)) * Decimal("100")) * 360000
        return ((scaled + (1 << 31)) % (1 << 32)) - (1 << 31)

    @classmethod
    def build_packet(
        cls,
        target_id,
        distance_m,
        azimuth_deg,
        elevation_deg,
        longitude_deg,
        latitude_deg,
    ):
        target_id = int(target_id)
        if target_id < 0 or target_id > 0xFF:
            raise ValueError(f"strike target_id out of uint8 range: {target_id}")

        distance_raw = int(round(float(distance_m) * 10.0))
        if distance_raw <= 0 or distance_raw > 0xFFFF:
            raise ValueError(f"strike distance out of uint16/0.1m range: {distance_m}")

        az_raw = int(round((float(azimuth_deg) % 360.0) * 10.0))
        if az_raw >= 3600:
            az_raw = 0

        elevation_deg = float(elevation_deg)
        if elevation_deg < cls.MIN_ELEVATION_DEG or elevation_deg > cls.MAX_ELEVATION_DEG:
            raise ValueError(f"strike elevation out of range: {elevation_deg}")
        el_raw = int(round(elevation_deg * 10.0))

        longitude_deg = float(longitude_deg)
        latitude_deg = float(latitude_deg)
        if not math.isfinite(longitude_deg) or not -180.0 <= longitude_deg <= 180.0:
            raise ValueError(f"strike longitude out of range: {longitude_deg}")
        if not math.isfinite(latitude_deg) or not -90.0 <= latitude_deg <= 90.0:
            raise ValueError(f"strike latitude out of range: {latitude_deg}")
        longitude_raw = cls._encode_coordinate(longitude_deg)
        latitude_raw = cls._encode_coordinate(latitude_deg)

        body = struct.pack(
            "!2sBBHHhii",
            cls.FRAME_HEAD,
            cls.FRAME_LENGTH,
            target_id,
            distance_raw,
            az_raw,
            el_raw,
            longitude_raw,
            latitude_raw,
        )
        return body + bytes([cls._xor_checksum(body)]) + cls.FRAME_TAIL

    def send_target_with_timestamp(
        self,
        target_id,
        distance_m,
        azimuth_deg,
        elevation_deg,
        longitude_deg,
        latitude_deg,
    ):
        packet = self.build_packet(
            target_id,
            distance_m,
            azimuth_deg,
            elevation_deg,
            longitude_deg,
            latitude_deg,
        )
        with self.lock:
            sendto_ts = time.time()
            self.sock.sendto(packet, (self.ip, self.port))
        return packet, sendto_ts

    def send_target(
        self,
        target_id,
        distance_m,
        azimuth_deg,
        elevation_deg,
        longitude_deg,
        latitude_deg,
    ):
        packet, _sendto_ts = self.send_target_with_timestamp(
            target_id,
            distance_m,
            azimuth_deg,
            elevation_deg,
            longitude_deg,
            latitude_deg,
        )
        return packet

# ==========================================
# 3. 激光与网络
# ==========================================
class SharedHardwareState:
    def __init__(self):
        self.lock = threading.Lock()
        self.gimbal_az = 0.0  # UI坐标系方位角
        self.gimbal_el = 0.0
        self.gimbal_att_ts = 0.0
        self.active_cmd_id = -1
        self.active_track_id = -1
        self.settled_cmd_id = -1
        self.settled_track_id = -1
        self.settled_ts = 0.0
        self.is_settled = False
        # Physical camera stability is independent of reaching the commanded
        # angle.  Ranging uses this flag; strike safety still uses is_settled.
        self.is_stationary = False
        self.stationary_ts = 0.0


def strike_snapshot_is_safe(snapshot, hardware_state):
    with hardware_state.lock:
        return bool(
            hardware_state.is_settled
            and hardware_state.is_stationary
            and int(hardware_state.settled_track_id)
            == int(snapshot["internal_track_id"])
        )


def begin_gimbal_command(hardware_state, cmd_id, track_id):
    with hardware_state.lock:
        hardware_state.active_cmd_id = int(cmd_id)
        hardware_state.active_track_id = int(track_id)
        hardware_state.is_settled = False
        hardware_state.is_stationary = False


def dispatch_gimbal_command(
    gimbal,
    hardware_state,
    command,
    elevation,
    azimuth,
    force=False,
):
    begin_gimbal_command(
        hardware_state,
        cmd_id=command["cmd_id"],
        track_id=command.get("track_id", -1),
    )
    return gimbal.set_attitude(
        elevation=elevation,
        azimuth=azimuth,
        force=force,
    )


class PeriodicStrikeSender:
    def __init__(
        self,
        sender,
        send_hz,
        hardware_state,
        on_send=None,
        on_error=None,
    ):
        self.sender = sender
        self.interval = 1.0 / max(float(send_hz), 0.1)
        self.hardware_state = hardware_state
        self.on_send = on_send
        self.on_error = on_error
        self.lock = threading.Lock()
        self.snapshot = None
        self.stop_event = threading.Event()
        self.thread = None

    def publish(self, snapshot):
        with self.lock:
            self.snapshot = dict(snapshot)

    def clear(self):
        with self.lock:
            self.snapshot = None

    def send_once(self, now=None):
        now = time.time() if now is None else float(now)
        snapshot = None
        try:
            with self.lock:
                if self.snapshot is None:
                    return False
                snapshot = dict(self.snapshot)
                if now >= float(snapshot["valid_until"]):
                    return False
                target_id = int(snapshot["target_id"])
                if target_id not in SPECIAL_RID_UI_IDS:
                    raise ValueError(
                        f"strike target_id is not special RID ID 1/2: {target_id}"
                    )
                snapshot["target_id"] = target_id
                with self.hardware_state.lock:
                    if not (
                        self.hardware_state.is_settled
                        and self.hardware_state.is_stationary
                        and int(self.hardware_state.settled_track_id)
                        == int(snapshot["internal_track_id"])
                    ):
                        return False
                    snapshot["settled_cmd_id"] = int(
                        self.hardware_state.settled_cmd_id
                    )
                    packet, sendto_ts = self.sender.send_target_with_timestamp(
                        target_id=snapshot["target_id"],
                        distance_m=snapshot["distance_m"],
                        azimuth_deg=snapshot["azimuth_deg"],
                        elevation_deg=snapshot["elevation_deg"],
                        longitude_deg=snapshot["longitude_deg"],
                        latitude_deg=snapshot["latitude_deg"],
                    )
        except Exception as exc:
            if self.on_error is not None:
                try:
                    self.on_error(snapshot or {}, exc, time.time())
                except Exception:
                    pass
            return False
        if self.on_send is not None:
            try:
                self.on_send(snapshot, packet, sendto_ts)
            except Exception:
                pass
        return True

    def _run(self):
        next_send = time.monotonic() + self.interval
        while True:
            wait_seconds = max(0.0, next_send - time.monotonic())
            if self.stop_event.wait(wait_seconds):
                break
            self.send_once()
            next_send += self.interval
            if next_send < time.monotonic():
                next_send = time.monotonic() + self.interval

    def start(self):
        if self.thread is not None and self.thread.is_alive():
            return
        self.stop_event.clear()
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()

    def stop(self):
        self.stop_event.set()
        if self.thread is not None:
            self.thread.join(timeout=max(1.0, self.interval * 2.0))
        self.clear()


shared_state = SharedHardwareState()
gimbal_cmd_queue = queue.Queue(maxsize=1)
packet_queue = deque(maxlen=PACKET_QUEUE_MAXLEN)


def sample_default_distance():
    """兜底距离采样：用于无激光、无单目时的保底值。"""
    return random.uniform(DEFAULT_DISTANCE_MIN_M, DEFAULT_DISTANCE_MAX_M)

def rk3588_thread():
    print(f"[Net] Listening RK3588 on {LOCAL_PORT}...")
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.settimeout(1.0)

    try:
        sock.bind(("0.0.0.0", LOCAL_PORT))
    except OSError as e:
        print(f"[Net][Fatal] 端口 {LOCAL_PORT} 绑定失败: {e}")
        return

    err_decode = 0
    err_json = 0
    err_sock = 0
    err_other = 0

    while True:
        try:
            data, addr = sock.recvfrom(65535)
            recv_ts = time.time()
            if data[:1] == bytes((DEVICE_HEADING_SET_MSG,)):
                if addr[0] != UI_IP:
                    reason = (
                        f"source_ip={addr[0]},expected_ui_ip={UI_IP},"
                        "action=rejected"
                    )
                    print(f"[Net][Heading][Reject] {reason}")
                    field_log_event({
                        "timestamp": f"{recv_ts:.6f}",
                        "event": "DEVICE_HEADING_UPDATE_REJECTED",
                        "reason": reason,
                    })
                    continue
                try:
                    requested_heading = decode_device_heading_packet(data)
                    old_heading, new_heading = set_device_heading_deg(
                        requested_heading
                    )
                except ValueError as e:
                    reason = (
                        f"source={addr[0]}:{addr[1]},"
                        f"packet_len={len(data)},error={e}"
                    )
                    print(f"[Net][Heading][Reject] {reason}")
                    field_log_event({
                        "timestamp": f"{recv_ts:.6f}",
                        "event": "DEVICE_HEADING_UPDATE_REJECTED",
                        "reason": reason,
                    })
                    continue
                print(
                    f"[Net][Heading] source={addr[0]}:{addr[1]}, "
                    f"DEVICE_HEADING_DEG={old_heading:.2f}->{new_heading:.2f}"
                )
                field_log_event({
                    "timestamp": f"{recv_ts:.6f}",
                    "event": "DEVICE_HEADING_UPDATE",
                    "reason": (
                        f"source={addr[0]}:{addr[1]},"
                        f"old_heading_deg={old_heading:.6f},"
                        f"new_heading_deg={new_heading:.6f}"
                    ),
                })
                continue
            pkg = json.loads(data.decode("utf-8"))
            if isinstance(pkg, dict) and "objs" in pkg:
                pkg["_recv_ts"] = recv_ts
                if FIELD_LOGGER is not None:
                    FIELD_LOGGER.write_raw_udp({
                        "recv_ts": recv_ts,
                        "addr": f"{addr[0]}:{addr[1]}",
                        "packet_len": len(data),
                        "seq": pkg.get("seq", ""),
                        "mode": pkg.get("mode", ""),
                        "board": pkg.get("board", ""),
                        "cam": pkg.get("cam", ""),
                        "raw_obj_count": len(pkg.get("objs", [])) if isinstance(pkg.get("objs", []), list) else "",
                        "raw_objs": pkg.get("objs", []),
                    })
                packet_queue.append(pkg)

        except socket.timeout:
            continue

        except UnicodeDecodeError as e:
            err_decode += 1
            if err_decode % 50 == 1:
                print(f"[Net][DecodeError] count={err_decode}, err={e}")
            continue

        except json.JSONDecodeError as e:
            err_json += 1
            if err_json % 50 == 1:
                print(f"[Net][JSONError] count={err_json}, err={e}")
            continue

        except OSError as e:
            err_sock += 1
            if err_sock % 10 == 1:
                print(f"[Net][SocketError] count={err_sock}, 网卡/底层异常: {e}")
            time.sleep(0.5)
            continue

        except Exception as e:
            err_other += 1
            print(f"[Net][Unexpected] count={err_other}, type={type(e).__name__}, err={e}")
            continue

def push_latest_gimbal_cmd(cmd):
    while True:
        try:
            gimbal_cmd_queue.put_nowait(cmd)
            return True
        except queue.Full:
            try:
                gimbal_cmd_queue.get_nowait()
            except queue.Empty:
                return False

def drain_latest_gimbal_cmd():
    latest_cmd = None
    while True:
        try:
            latest_cmd = gimbal_cmd_queue.get_nowait()
        except queue.Empty:
            break
    return latest_cmd

def _parse_positive_float(value):
    try:
        f = float(value)
        if f > 0:
            return f
    except (TypeError, ValueError):
        pass
    return None

def laser_reader_thread(laser, stop_event):
    print("[LaserThread] 激光读取线程已启动")
    try:
        laser.start_measurement(continuous=True)
    except Exception as e:
        print(f"[Laser][Fatal] 启动连续测量失败: {e}")
        return

    last_log_t = 0.0
    while not stop_event.is_set():
        dist = laser.read_distance()
        if dist is None:
            continue
        now_t = time.time()
        if (now_t - last_log_t) >= LASER_LOG_INTERVAL:
            field_log_gimbal({
                "timestamp": f"{now_t:.6f}",
                "event": "LASER_READ_ONLY",
                "laser_valid": 1,
                "laser_dist": f"{float(dist):.6f}",
                "laser_source": "sddm",
                "laser_ts": f"{now_t:.6f}",
            })
            last_log_t = now_t


class SharedPositionState:
    """Thread-safe WGS-84 station position and MSL height for RID."""

    def __init__(
        self,
        longitude=None,
        latitude=None,
        altitude=None,
        source="unavailable",
    ):
        self.lock = threading.Lock()
        self.longitude = longitude
        self.latitude = latitude
        self.altitude = altitude
        self.source = source
        self.updated_ts = time.time() if longitude is not None and latitude is not None else 0.0

    def update(
        self,
        longitude,
        latitude,
        source,
        altitude=None,
        updated_ts=None,
    ):
        with self.lock:
            self.longitude = float(longitude)
            self.latitude = float(latitude)
            if altitude is not None:
                self.altitude = float(altitude)
            self.source = str(source)
            self.updated_ts = time.time() if updated_ts is None else float(updated_ts)

    def snapshot(self, now_ts=None):
        now_ts = time.time() if now_ts is None else float(now_ts)
        with self.lock:
            longitude = self.longitude
            latitude = self.latitude
            altitude = self.altitude
            source = self.source
            updated_ts = self.updated_ts
        return {
            "longitude": longitude,
            "latitude": latitude,
            "altitude": altitude,
            "source": source,
            "updated_ts": updated_ts,
            "age_s": (
                max(0.0, now_ts - updated_ts)
                if updated_ts > 0.0
                else math.inf
            ),
            "valid": longitude is not None and latitude is not None,
            "altitude_valid": altitude is not None and math.isfinite(altitude),
        }


def gps_sender_thread(sender, position_state=None):
    if read_gps_fix is None:
        print("[GPS][Warn] gps.py import failed, GPS location packet disabled.")
        return
    if DEFAULT_LATITUDE is None or DEFAULT_LONGITUDE is None:
        print("[GPS][Warn] default GPS location missing, GPS location packet disabled.")
        return

    print(
        f"[GPS] Thread started, periodic UI updates enabled on "
        f"{GPS_PORT}@{GPS_BAUDRATE}, fix_timeout={GPS_FIX_TIMEOUT_SECONDS}s, "
        f"ui_interval={GPS_UI_SEND_INTERVAL}s"
    )

    last_sent_latitude = None
    last_sent_longitude = None
    last_sent_altitude = None
    last_sent_source = "default"

    while True:
        cycle_start = time.monotonic()
        #print("[GPS] Searching satellites and waiting for valid latitude/longitude...")
        longitude, latitude, altitude, source = read_gps_fix(
            port=GPS_PORT,
            baudrate=GPS_BAUDRATE,
            timeout_seconds=GPS_FIX_TIMEOUT_SECONDS,
            print_raw=GPS_DEBUG_RAW,
            print_status=True,
            status_interval=GPS_STATUS_INTERVAL,
            coordinate_system="wgs84",
            include_altitude=True,
            altitude_reference="msl",
        )

        send_source = source
        if longitude is not None and latitude is not None:
            last_sent_longitude = longitude
            last_sent_latitude = latitude
            if altitude is not None:
                last_sent_altitude = altitude
            last_sent_source = source
        elif last_sent_longitude is not None and last_sent_latitude is not None:
            longitude = last_sent_longitude
            latitude = last_sent_latitude
            altitude = last_sent_altitude
            send_source = f"cached:{last_sent_source}"
        else:
            longitude = DEFAULT_LONGITUDE
            latitude = DEFAULT_LATITUDE
            altitude = None
            send_source = "default"

        if (
            position_state is not None
            and (
                send_source != "default"
                or RID_ALLOW_DEFAULT_STATION_POSITION
            )
        ):
            position_state.update(
                longitude=longitude,
                latitude=latitude,
                altitude=altitude,
                source=send_source,
            )
            field_log_event({
                "timestamp": f"{time.time():.6f}",
                "event": "GPS_STATION_FIX",
                "reason": (
                    f"source={send_source},lat={latitude:.8f},"
                    f"lon={longitude:.8f},alt_msl_m={altitude}"
                ),
            })

        # Keep the historical UI coordinate behavior while RID calculations
        # use the unrounded WGS-84 fix and GGA MSL height stored above.
        ui_longitude = longitude
        ui_latitude = latitude
        if send_source != "default" and wgs84_to_gcj02 is not None:
            ui_longitude, ui_latitude = wgs84_to_gcj02(
                round(float(longitude), 4),
                round(float(latitude), 4),
            )
        sender.send_gps_location(
            latitude=ui_latitude,
            longitude=ui_longitude,
        )
        # print(
        #     f"[GPS] Sent location to UI: "
        #     f"source={send_source}, latitude={latitude:.6f}, longitude={longitude:.6f}"
        # )

        sleep_time = GPS_UI_SEND_INTERVAL - (time.monotonic() - cycle_start)
        if sleep_time > 0:
            time.sleep(sleep_time)


def parse_udp_objects(raw_objs):
    """
    归一化 UDP 目标列表，输出:
        [{"box": [x1, y1, x2, y2], "mono_dist": None,
          "cam": int|None, "board": str|None}, ...]

    支持格式:
    1) [x1, y1, x2, y2]
    2) [x1, y1, x2, y2, ...]  # extra values are ignored
    3) {"box":[x1,y1,x2,y2]}
    4) {"boxes":[[...],[...]]} (多坐标批量)

    检测端距离字段已停用；本端只使用云台 YOLO+测距模型更新距离。
    """
    parsed = []

    def append_obj(box, mono_dist=None, cam=None, board=None):
        parsed.append({
            "box": [box[0], box[1], box[2], box[3]],
            "mono_dist": None,
            "cam": cam,
            "board": board,
        })

    def first_present(obj, keys):
        for key in keys:
            if key in obj and obj.get(key) not in (None, ""):
                return obj.get(key)
        return None

    # 兼容单目标扁平格式:
    # objs = [x1, y1, x2, y2] / [x1, y1, x2, y2, dist]
    if isinstance(raw_objs, (list, tuple)) and len(raw_objs) >= 4 and not isinstance(raw_objs[0], (list, tuple, dict)):
        return [{"box": [raw_objs[0], raw_objs[1], raw_objs[2], raw_objs[3]], "mono_dist": None, "cam": None, "board": None}]

    # 兼容单目标字典:
    # objs = {"box":[...], "distance":...}
    if isinstance(raw_objs, dict):
        raw_objs = [raw_objs]

    if not isinstance(raw_objs, list):
        return parsed

    for obj_item in raw_objs:
        if isinstance(obj_item, dict):
            obj_cam = first_present(obj_item, ("cam", "cam_id", "camera", "camera_id", "cameraId"))
            obj_board = first_present(obj_item, ("board", "board_id", "boardId"))

            # 批量 boxes: {"boxes":[...]}；若带 distances 也忽略。
            boxes = obj_item.get("boxes", None)
            if isinstance(boxes, list):
                for b in boxes:
                    if not isinstance(b, (list, tuple)) or len(b) < 4:
                        continue
                    append_obj([b[0], b[1], b[2], b[3]], None, obj_cam, obj_board)
                continue

            # 单目标 box
            box = obj_item.get("box", None)
            if isinstance(box, (list, tuple)):
                # 兼容 {"box":[[...],[...]], ...}
                if len(box) > 0 and isinstance(box[0], (list, tuple)):
                    for b in box:
                        if isinstance(b, (list, tuple)) and len(b) >= 4:
                            append_obj([b[0], b[1], b[2], b[3]], None, obj_cam, obj_board)
                elif len(box) >= 4:
                    append_obj([box[0], box[1], box[2], box[3]], None, obj_cam, obj_board)
                continue

            # 兼容坐标键值形式
            if all(k in obj_item for k in ("x1", "y1", "x2", "y2")):
                append_obj(
                    [obj_item["x1"], obj_item["y1"], obj_item["x2"], obj_item["y2"]],
                    None,
                    obj_cam,
                    obj_board,
                )
                continue
            if all(k in obj_item for k in ("x", "y", "w", "h")):
                x = float(obj_item["x"])
                y = float(obj_item["y"])
                w = float(obj_item["w"])
                h = float(obj_item["h"])
                append_obj([x, y, x + w, y + h], None, obj_cam, obj_board)
                continue

        elif isinstance(obj_item, (list, tuple)):
            # 单目标: [x1, y1, x2, y2, (optional)dist]
            if len(obj_item) >= 4 and not isinstance(obj_item[0], (list, tuple, dict)):
                append_obj([obj_item[0], obj_item[1], obj_item[2], obj_item[3]], None)
                continue

            # 批量: [[x1,y1,x2,y2], [..], ...]
            if len(obj_item) > 0 and isinstance(obj_item[0], (list, tuple)):
                for b in obj_item:
                    if isinstance(b, (list, tuple)) and len(b) >= 4:
                        append_obj([b[0], b[1], b[2], b[3]], None)
                continue

    return parsed


def sanitize_bbox(rect, image_w=IMG_W, image_h=IMG_H):
    try:
        x1, y1, x2, y2 = (float(rect[0]), float(rect[1]), float(rect[2]), float(rect[3]))
    except (TypeError, ValueError, IndexError):
        return None, "non_numeric_bbox"

    raw_w = x2 - x1
    raw_h = y2 - y1
    if raw_w <= 0 or raw_h <= 0:
        return None, "invalid_raw_bbox"

    image_w = float(image_w)
    image_h = float(image_h)
    clipped_x1 = min(max(x1, 0.0), image_w)
    clipped_y1 = min(max(y1, 0.0), image_h)
    clipped_x2 = min(max(x2, 0.0), image_w)
    clipped_y2 = min(max(y2, 0.0), image_h)
    clipped_w = clipped_x2 - clipped_x1
    clipped_h = clipped_y2 - clipped_y1
    if clipped_w <= 0 or clipped_h <= 0:
        return None, "invalid_clipped_bbox"

    raw_area = raw_w * raw_h
    clipped_area = clipped_w * clipped_h
    is_edge_bbox = (
        x1 < 0.0 or y1 < 0.0 or x2 > image_w or y2 > image_h
    )
    return {
        "raw": [x1, y1, x2, y2],
        "clipped": [clipped_x1, clipped_y1, clipped_x2, clipped_y2],
        "raw_w": raw_w,
        "raw_h": raw_h,
        "clipped_w": clipped_w,
        "clipped_h": clipped_h,
        "visible_ratio": clipped_area / raw_area,
        "is_edge_bbox": is_edge_bbox,
    }, ""


def clear_strike_window(strike_window):
    strike_window["track_id"] = None
    strike_window["distance"] = None
    strike_window["source"] = "none"
    strike_window["valid_until"] = 0.0
    strike_window["last_send_ts"] = 0.0
    strike_window["last_consumed_settled_cmd_id"] = -1


def clear_strike_delivery(strike_window, strike_send_worker):
    clear_strike_window(strike_window)
    if strike_send_worker is not None:
        strike_send_worker.clear()


import numpy as np
from scipy.optimize import linear_sum_assignment

# ==========================================
# 新增模块 1：角度计算工具
# ==========================================
def angular_diff(target, source):
    """计算两个绝对角度之间的最短物理距离 (-180 到 180度)"""
    return (target - source + 180.0) % 360.0 - 180.0


def circular_mean_deg(values):
    if not values:
        return 0.0
    sin_sum = sum(math.sin(math.radians(v)) for v in values)
    cos_sum = sum(math.cos(math.radians(v)) for v in values)
    if abs(sin_sum) < 1e-12 and abs(cos_sum) < 1e-12:
        return float(values[0]) % 360.0
    return math.degrees(math.atan2(sin_sum, cos_sum)) % 360.0


def angle_measurement_distance(a, b):
    return math.hypot(
        angular_diff(float(a["az"]), float(b["az"])),
        float(a["el"]) - float(b["el"]),
    )


def measurement_source_key(item):
    logic_id = item.get("logic_id")
    if logic_id not in (None, ""):
        try:
            return ("logic", int(logic_id))
        except (TypeError, ValueError):
            return ("logic", str(logic_id))
    return ("board_cam", str(item.get("board", "")), str(item.get("cam", "")))


def fusion_group_has_source(group_items, meas):
    source_key = measurement_source_key(meas)
    return any(measurement_source_key(item) == source_key for item in group_items)


def build_fused_measurement(items):
    az_values = [float(item["az"]) for item in items]
    el_values = [float(item["el"]) for item in items]
    mono_values = []
    for item in items:
        mono_dist = _parse_positive_float(item.get("mono_dist"))
        if mono_dist is not None:
            mono_values.append(mono_dist)
    primary_item = max(
        items,
        key=lambda item: float(item.get("source_ts", 0.0) or 0.0),
    )
    fused = {
        "az": circular_mean_deg(az_values),
        "el": sum(el_values) / len(el_values),
        "mono_dist": (sum(mono_values) / len(mono_values)) if mono_values else None,
        "source_ts": float(primary_item.get("source_ts", 0.0) or 0.0),
        "source_meas_indices": [int(item.get("raw_meas_idx", idx)) for idx, item in enumerate(items)],
        "source_boards": [str(item.get("board", "")) for item in items],
        "source_cams": [str(item.get("cam", "")) for item in items],
        "source_logic_ids": [str(item.get("logic_id", "")) for item in items],
        "is_edge_bbox": any(bool(item.get("is_edge_bbox", False)) for item in items),
    }
    visible_values = []
    for item in items:
        try:
            visible_values.append(float(item.get("visible_ratio")))
        except (TypeError, ValueError):
            pass
    if visible_values:
        fused["visible_ratio"] = min(visible_values)
        fused["source_visible_ratios"] = visible_values
    if items:
        fused["board"] = primary_item.get("board")
        fused["cam"] = primary_item.get("cam")
        fused["logic_id"] = primary_item.get("logic_id")
    return fused


def fuse_measurements_by_angle(measurements, threshold_deg=MEAS_FUSION_THRESHOLD_DEG):
    """
    同一帧内的跨相机重复观测先在角度空间融合，再进入 SORT/Kalman。
    这里仅做帧内去重，不跨帧维护状态，避免改动 tracker 主体逻辑。
    """
    if not measurements:
        return [], []
    if threshold_deg <= 0:
        fused = []
        groups = []
        for i, meas in enumerate(measurements):
            item = dict(meas)
            item["source_meas_indices"] = [int(meas.get("raw_meas_idx", i))]
            item["source_cams"] = [str(meas.get("cam", ""))]
            fused.append(item)
            groups.append([item])
        return fused, groups

    groups = []
    for meas in measurements:
        best_idx = None
        best_dist = None
        for idx, group in enumerate(groups):
            # 只允许不同摄像头 ID 的观测互相融合。同一画面中的
            # 多个目标即使角度很近，也必须保留为独立观测交给 SORT。
            if fusion_group_has_source(group["items"], meas):
                continue
            dist = angle_measurement_distance(meas, group["center"])
            if dist <= threshold_deg and (best_dist is None or dist < best_dist):
                best_idx = idx
                best_dist = dist

        if best_idx is None:
            groups.append({"items": [dict(meas)], "center": dict(meas)})
            continue

        group = groups[best_idx]
        group["items"].append(dict(meas))
        group["center"] = build_fused_measurement(group["items"])

    fused = [build_fused_measurement(group["items"]) for group in groups]
    return fused, [group["items"] for group in groups]


def format_fusion_groups(groups):
    parts = []
    for idx, group in enumerate(groups):
        raw_indices = ",".join(str(item.get("raw_meas_idx", "")) for item in group)
        sources = ",".join(
            f"{item.get('board', '')}/{item.get('cam', '')}/logic={item.get('logic_id', '')}"
            for item in group
        )
        parts.append(f"{idx}:n={len(group)},raw={raw_indices},src={sources}")
    return ";".join(parts)


def relative_to_map_azimuth(relative_az, device_heading_deg=None):
    """将设备自身坐标系方位角转换为正北为0度的地图绝对方位角。"""
    if device_heading_deg is None:
        device_heading_deg = get_device_heading_deg()
    return (float(relative_az) + float(device_heading_deg)) % 360.0


def track_ui_source(track, fallback_board, fallback_cam):
    """Return this track's latest detection source, with a legacy fallback."""
    board = getattr(track, "last_source_board", None)
    cam = getattr(track, "last_source_cam", None)
    return (
        fallback_board if board in (None, "") else board,
        fallback_cam if cam in (None, "") else cam,
    )


def track_is_ui_fresh(track, now_t):
    """Hide stale prediction-only tracks while retaining them internally."""
    return track.lost_seconds(now_t) <= UI_MAX_LOST_SECONDS


def select_ui_tracks_for_display(active_tracks, now_t):
    """Keep confirmed UI identities visible across short detector dropouts."""
    return [
        track
        for track in active_tracks
        if track.confirmed
        and getattr(track, "ui_confirmed", False)
        and track_is_ui_fresh(track, now_t)
    ]


def select_ordinary_ui_tracks_for_output(
    ui_tracks,
    owned_special_generations,
    special_rid_ui_exclusive=False,
):
    """Filter the legacy UI path without changing tracking or strike inputs."""
    if special_rid_ui_exclusive:
        return []
    return [
        track
        for track in ui_tracks
        if not owned_special_generations
        or should_send_sort_through_ordinary_ui(
            track,
            owned_special_generations,
        )
    ]


def special_rid_ui_id_for_track(track, special_rid_registry):
    """Resolve an exact SORT generation to its special RID UI identity."""
    if special_rid_registry is None or sort_generation_from_track is None:
        return None
    try:
        slot = special_rid_registry.owner_of(sort_generation_from_track(track))
        ui_id = int(slot.ui_id)
    except (AttributeError, TypeError, ValueError, OverflowError):
        return None
    return ui_id if ui_id in SPECIAL_RID_UI_IDS else None


def select_special_rid_strike_tracks(tracks, special_rid_registry):
    """Keep only tracks owned by the two special RID identities for strike."""
    return [
        track
        for track in tracks
        if special_rid_ui_id_for_track(track, special_rid_registry) is not None
    ]


def update_track_confirmation_flags(track):
    """Promote a confirmed SORT track to the UI and strike output gates."""
    if track.hit_streak >= UI_TRACK_CONFIRM_HITS:
        track.ui_confirmed = True
    if track.hit_streak >= STRIKE_TRACK_CONFIRM_HITS:
        track.strike_confirmed = True


def ui_threat_score_from_distance(distance_m):
    """Map a valid UI distance to the three RID threat tiers.

    This score is display-only. It does not participate in SORT selection,
    gimbal control, strike selection, or any hardware safety decision.
    """
    try:
        distance_m = float(distance_m)
    except (TypeError, ValueError):
        return float("nan")
    if not math.isfinite(distance_m) or distance_m < 0.0:
        return float("nan")
    if distance_m < RID_UI_THREAT_HIGH_MAX_DISTANCE_M:
        return RID_UI_THREAT_HIGH_SCORE
    if distance_m <= RID_UI_THREAT_MEDIUM_MAX_DISTANCE_M:
        return RID_UI_THREAT_MEDIUM_SCORE
    return RID_UI_THREAT_LOW_SCORE


def next_unreserved_ui_id(candidate, reserved_ids):
    """Return the first positive UI ID not reserved by another identity path."""
    ui_id = max(1, int(candidate))
    reserved = {int(item) for item in reserved_ids}
    while ui_id in reserved:
        ui_id += 1
    return ui_id


def get_turn_direction_label(delta_az, delta_el, deadband_az=0.35, deadband_el=0.25):
    """
    根据目标相对当前云台姿态的角差，给出转动方向标签。
    delta_az > 0: 向右转；delta_az < 0: 向左转
    delta_el > 0: 向上转；delta_el < 0: 向下转
    """
    if delta_az > deadband_az:
        horiz = "RIGHT"
    elif delta_az < -deadband_az:
        horiz = "LEFT"
    else:
        horiz = "CENTER"

    if delta_el > deadband_el:
        vert = "UP"
    elif delta_el < -deadband_el:
        vert = "DOWN"
    else:
        vert = "LEVEL"

    if horiz == "CENTER" and vert == "LEVEL":
        return "HOLD"
    if horiz == "CENTER":
        return vert
    if vert == "LEVEL":
        return horiz
    return f"{horiz}_{vert}"


def gimbal_control_thread(gimbal):
    """
    云台硬件专属线程：独占串口读写，支持可抢占执行。
    """
    print("[GimbalThread] 控制线程已启动")
    active_cmd = None#当前正在执行的指令
    target_az = 0.0
    target_el = 0.0
    cmd_start_t = 0.0
    last_progress_log_t = 0.0
    settle_candidate_since = None
    next_feedback_query_t = 0.0
    last_az_send_t = 0.0
    last_el_send_t = 0.0
    last_motion_az = None
    last_motion_el = None
    stationary_candidate_since = None

    def mark_motion_expected():
        nonlocal stationary_candidate_since
        stationary_candidate_since = None

    def record_feedback_motion(curr_el, curr_az, feedback_t):
        """Update UI attitude and physical stationary state from encoder deltas."""
        nonlocal last_motion_az, last_motion_el, stationary_candidate_since
        curr_ui_az = (curr_az - GIMBAL_AZ_BASE) % 360.0
        curr_ui_el = curr_el - GIMBAL_INIT_EL
        was_stationary = False
        with shared_state.lock:
            was_stationary = shared_state.is_stationary

        if last_motion_az is None or last_motion_el is None:
            stationary_candidate_since = feedback_t
            is_stationary = False
        else:
            delta_az = abs(angular_diff(curr_az, last_motion_az))
            delta_el = abs(curr_el - last_motion_el)
            physically_moving = (
                delta_az > GIMBAL_STATIONARY_DELTA_DEG
                or delta_el > GIMBAL_STATIONARY_DELTA_DEG
            )
            if physically_moving:
                stationary_candidate_since = None
                is_stationary = False
            else:
                if stationary_candidate_since is None:
                    stationary_candidate_since = feedback_t
                is_stationary = (
                    feedback_t - stationary_candidate_since
                    >= GIMBAL_STATIONARY_DWELL_SECONDS
                )

        last_motion_az = curr_az
        last_motion_el = curr_el
        with shared_state.lock:
            shared_state.gimbal_el = curr_ui_el
            shared_state.gimbal_az = curr_ui_az
            shared_state.gimbal_att_ts = feedback_t
            shared_state.is_stationary = is_stationary
            if is_stationary and not was_stationary:
                shared_state.stationary_ts = feedback_t
        if is_stationary and not was_stationary:
            field_log_gimbal({
                "timestamp": f"{feedback_t:.6f}",
                "event": "GIMBAL_STATIONARY",
                "gimbal_ui_az": f"{curr_ui_az:.6f}",
                "gimbal_ui_el": f"{curr_ui_el:.6f}",
                "gimbal_ctrl_az": f"{curr_az:.6f}",
                "gimbal_ctrl_el": f"{curr_el:.6f}",
                "is_settled": 1 if shared_state.is_settled else 0,
                "is_stationary": 1,
            })
        return curr_ui_az, curr_ui_el, is_stationary

    while True:
        try:
            #1.当前没有指令在执行：(空闲态)
            if active_cmd is None:
                try:
                    #等待队列命令
                    active_cmd = gimbal_cmd_queue.get(timeout=0.1)
                
                except queue.Empty:
                    #若指令队列为空
                    real_att = gimbal.get_attitude()#读当前云台姿态并写入共享状态
                    if real_att:
                        curr_el, curr_az, _ = real_att
                        record_feedback_motion(
                            curr_el, curr_az, time.time()
                        )
                    continue
                #若成功取到指令，下发指令到云台
                target_az = float(active_cmd["az"])#方位角
                target_el = float(active_cmd["el"])#俯仰角
                cmd_start_t = time.time()
                last_progress_log_t = 0.0
                settle_candidate_since = None
                mark_motion_expected()
                send_status = dispatch_gimbal_command(
                    gimbal,
                    shared_state,
                    active_cmd,
                    elevation=target_el,
                    azimuth=target_az,
                )
                send_t = time.time()
                last_az_send_t = send_t
                last_el_send_t = send_t
                next_feedback_query_t = send_t + GIMBAL_QUERY_AFTER_CMD_DELAY
                if PRINT_EVENT_LOGS:
                    print(
                        f"[GimbalCmd] cmd_id={int(active_cmd['cmd_id'])}, "
                        f"track_id={int(active_cmd.get('track_id', -1))}, "
                        f"target_ctrl=(Az={target_az:.2f}°, El={target_el:.2f}°)"
                    )
                field_log_gimbal({
                    "timestamp": f"{cmd_start_t:.6f}",
                    "event": "GIMBAL_CMD_SEND",
                    "cmd_id": int(active_cmd["cmd_id"]),
                    "track_id": int(active_cmd.get("track_id", -1)),
                    "target_ctrl_az": f"{target_az:.6f}",
                    "target_ctrl_el": f"{target_el:.6f}",
                })
            #2.若当前有指令在执行(执行态)
            now_t = time.time()
            #检查指令队列中是否有更新的指令，如果有则取出最新的一条（丢弃旧指令），准备进行抢占式执行判断
            newer_cmd = drain_latest_gimbal_cmd()
            if newer_cmd is not None:
                new_az = float(newer_cmd["az"])#新指令的方位角
                new_el = float(newer_cmd["el"])#新指令的俯仰角
                new_track_id = int(newer_cmd.get("track_id", -1))
                curr_track_id = int(active_cmd.get("track_id", -1))
                #计算新指令与当前指令的角度差
                d_az = abs(angular_diff(new_az, target_az))
                d_el = abs(new_el - target_el)
                d_total = math.hypot(d_az, d_el)

                # 目标切换时必须整条命令替换，避免“新方位 + 旧俯仰”混合指向。
                full_replace = (new_track_id != curr_track_id)
                update_az = full_replace or (d_az > AZ_PREEMPT_DEG)
                update_el = full_replace or (d_el > EL_PREEMPT_DEG)

                if update_az or update_el:
                    if update_az:
                        target_az = new_az
                    if update_el:
                        target_el = new_el
                    active_cmd = newer_cmd
                    cmd_start_t = now_t
                    last_progress_log_t = 0.0
                    settle_candidate_since = None
                    mark_motion_expected()
                    send_status = dispatch_gimbal_command(
                        gimbal,
                        shared_state,
                        active_cmd,
                        elevation=target_el,
                        azimuth=target_az,
                    )
                    send_t = time.time()
                    if update_az:
                        last_az_send_t = send_t
                    if update_el:
                        last_el_send_t = send_t
                    next_feedback_query_t = send_t + GIMBAL_QUERY_AFTER_CMD_DELAY

                    update_mode = "track_switch" if full_replace else "axis_update"
                    updated_axes = []
                    if update_az:
                        updated_axes.append("Az")
                    if update_el:
                        updated_axes.append("El")
                    updated_axes_text = "+".join(updated_axes) if updated_axes else "none"
                    if PRINT_EVENT_LOGS:
                        print(
                            f"[GimbalCmd] preempt cmd_id={int(active_cmd['cmd_id'])}, "
                            f"track_id={new_track_id}, mode={update_mode}, axes={updated_axes_text}, "
                            f"target_ctrl=(Az={target_az:.2f}°, El={target_el:.2f}°), "
                            f"delta=(dAz={d_az:.2f}°, dEl={d_el:.2f}°, total={d_total:.2f}°)"
                        )
                    field_log_gimbal({
                        "timestamp": f"{now_t:.6f}",
                        "event": "GIMBAL_CMD_PREEMPT",
                        "cmd_id": int(active_cmd["cmd_id"]),
                        "track_id": new_track_id,
                        "target_ctrl_az": f"{target_az:.6f}",
                        "target_ctrl_el": f"{target_el:.6f}",
                        "err_az": f"{d_az:.6f}",
                        "err_el": f"{d_el:.6f}",
                    })

            if time.time() < next_feedback_query_t:
                time.sleep(GIMBAL_THREAD_SLEEP)
                continue

            real_att = gimbal.get_attitude()
            if real_att:
                curr_el, curr_az, _ = real_att
                curr_ui_az, curr_ui_el, gimbal_is_stationary = (
                    record_feedback_motion(curr_el, curr_az, now_t)
                )
                err_az = abs(angular_diff(target_az, curr_az))
                err_el = abs(curr_el - target_el)

                if (last_progress_log_t == 0.0) or ((now_t - last_progress_log_t) >= GIMBAL_PROGRESS_LOG_INTERVAL):
                    elapsed = now_t - cmd_start_t
                    if PRINT_GIMBAL_PROGRESS:
                        print(
                            f"[GimbalAtt] cmd_id={int(active_cmd['cmd_id'])}, "
                            f"elapsed={elapsed:.3f}s, "
                            f"actual_ctrl=(Az={curr_az:.2f}°, El={curr_el:.2f}°), "
                            f"actual_ui=(Az={curr_ui_az:.2f}°, El={curr_el:.2f}°), "
                            f"target_ctrl=(Az={target_az:.2f}°, El={target_el:.2f}°), "
                            f"err=(dAz={err_az:.2f}°, dEl={err_el:.2f}°)"
                        )
                    field_log_gimbal({
                        "timestamp": f"{now_t:.6f}",
                        "event": "GIMBAL_ATT",
                        "cmd_id": int(active_cmd["cmd_id"]),
                        "track_id": int(active_cmd.get("track_id", -1)),
                        "gimbal_ui_az": f"{curr_ui_az:.6f}",
                        "gimbal_ui_el": f"{curr_ui_el:.6f}",
                        "gimbal_ctrl_az": f"{curr_az:.6f}",
                        "gimbal_ctrl_el": f"{curr_el:.6f}",
                        "target_ctrl_az": f"{target_az:.6f}",
                        "target_ctrl_el": f"{target_el:.6f}",
                        "err_az": f"{err_az:.6f}",
                        "err_el": f"{err_el:.6f}",
                        "is_stationary": 1 if gimbal_is_stationary else 0,
                    })
                    last_progress_log_t = now_t

                retry_az = (
                    err_az >= GIMBAL_SETTLE_THRESHOLD
                    and (now_t - last_az_send_t) >= GIMBAL_COMMAND_RETRY_INTERVAL
                )
                retry_el = (
                    err_el >= GIMBAL_SETTLE_THRESHOLD
                    and (now_t - last_el_send_t) >= GIMBAL_COMMAND_RETRY_INTERVAL
                )
                if retry_az or retry_el:
                    mark_motion_expected()
                    retry_status = dispatch_gimbal_command(
                        gimbal,
                        shared_state,
                        active_cmd,
                        elevation=target_el if retry_el else None,
                        azimuth=target_az if retry_az else None,
                        force=True,
                    )
                    retry_t = time.time()
                    if retry_az:
                        last_az_send_t = retry_t
                    if retry_el:
                        last_el_send_t = retry_t
                    next_feedback_query_t = (
                        retry_t + GIMBAL_QUERY_AFTER_CMD_DELAY
                    )
                    retry_axes = "+".join(
                        axis for axis, enabled in (
                            ("Az", retry_az), ("El", retry_el)
                        ) if enabled
                    )
                    field_log_gimbal({
                        "timestamp": f"{retry_t:.6f}",
                        "event": "GIMBAL_CMD_RETRY",
                        "cmd_id": int(active_cmd["cmd_id"]),
                        "track_id": int(active_cmd.get("track_id", -1)),
                        "target_ctrl_az": f"{target_az:.6f}",
                        "target_ctrl_el": f"{target_el:.6f}",
                        "err_az": f"{err_az:.6f}",
                        "err_el": f"{err_el:.6f}",
                        "retry_axes": retry_axes,
                        "driver_status": str(retry_status),
                    })
                    time.sleep(GIMBAL_THREAD_SLEEP)
                    continue

                if err_az < GIMBAL_SETTLE_THRESHOLD and err_el < GIMBAL_SETTLE_THRESHOLD:
                    if settle_candidate_since is None:
                        settle_candidate_since = now_t
                        time.sleep(GIMBAL_THREAD_SLEEP)
                        continue
                    if (now_t - settle_candidate_since) < GIMBAL_SETTLE_DWELL_SECONDS:
                        time.sleep(GIMBAL_THREAD_SLEEP)
                        continue
                    settle_dt = now_t - cmd_start_t
                    with shared_state.lock:
                        shared_state.is_settled = True
                        shared_state.settled_cmd_id = shared_state.active_cmd_id
                        shared_state.settled_track_id = shared_state.active_track_id
                        shared_state.settled_ts = now_t
                        active_cmd_id = shared_state.active_cmd_id
                        active_track_id = shared_state.active_track_id

                    if PRINT_EVENT_LOGS:
                        print(
                            f"[GimbalSettled] cmd_id={active_cmd_id}, track_id={active_track_id}, "
                            f"settle_time={settle_dt:.3f}s, "
                            f"actual_ctrl=(Az={curr_az:.2f}°, El={curr_el:.2f}°), "
                            f"target_ctrl=(Az={target_az:.2f}°, El={target_el:.2f}°), "
                            f"err=(dAz={err_az:.2f}°, dEl={err_el:.2f}°)"
                        )
                    field_log_gimbal({
                        "timestamp": f"{now_t:.6f}",
                        "event": "GIMBAL_SETTLED",
                        "cmd_id": int(active_cmd_id),
                        "track_id": int(active_track_id),
                        "gimbal_ui_az": f"{curr_ui_az:.6f}",
                        "gimbal_ui_el": f"{curr_ui_el:.6f}",
                        "gimbal_ctrl_az": f"{curr_az:.6f}",
                        "gimbal_ctrl_el": f"{curr_el:.6f}",
                        "target_ctrl_az": f"{target_az:.6f}",
                        "target_ctrl_el": f"{target_el:.6f}",
                        "err_az": f"{err_az:.6f}",
                        "err_el": f"{err_el:.6f}",
                        "is_settled": 1,
                        "is_stationary": 1 if gimbal_is_stationary else 0,
                        "settle_time": f"{settle_dt:.6f}",
                    })

                    # Laser readings are logged by laser_reader_thread only; distance fusion uses mono only.
                    active_cmd = None
                    settle_candidate_since = None
                    continue
                else:
                    settle_candidate_since = None

            if (now_t - cmd_start_t) >= GIMBAL_SETTLE_TIMEOUT:
                elapsed = now_t - cmd_start_t
                cmd_id = int(active_cmd["cmd_id"]) if active_cmd is not None else -1
                track_id = int(active_cmd.get("track_id", -1)) if active_cmd is not None else -1
                if real_att:
                    if PRINT_EVENT_LOGS:
                        print(
                            f"[GimbalTimeout] cmd_id={cmd_id}, track_id={track_id}, "
                            f"elapsed={elapsed:.3f}s, "
                            f"actual_ctrl=(Az={curr_az:.2f}°, El={curr_el:.2f}°), "
                            f"target_ctrl=(Az={target_az:.2f}°, El={target_el:.2f}°), "
                            f"err=(dAz={err_az:.2f}°, dEl={err_el:.2f}°)"
                        )
                else:
                    if PRINT_EVENT_LOGS:
                        print(
                            f"[GimbalTimeout] cmd_id={cmd_id}, track_id={track_id}, "
                            f"elapsed={elapsed:.3f}s, no attitude feedback"
                        )
                field_log_gimbal({
                    "timestamp": f"{now_t:.6f}",
                    "event": "GIMBAL_TIMEOUT",
                    "cmd_id": cmd_id,
                    "track_id": track_id,
                    "target_ctrl_az": f"{target_az:.6f}",
                    "target_ctrl_el": f"{target_el:.6f}",
                    "err_az": "" if not real_att else f"{err_az:.6f}",
                    "err_el": "" if not real_att else f"{err_el:.6f}",
                    "is_settled": 0,
                    "is_stationary": (
                        "" if not real_att
                        else (1 if gimbal_is_stationary else 0)
                    ),
                    "settle_time": f"{elapsed:.6f}",
                })
                with shared_state.lock:
                    shared_state.is_settled = False
                active_cmd = None
                settle_candidate_since = None
                continue

            time.sleep(GIMBAL_THREAD_SLEEP)

        except Exception as e:
            print(f"[GimbalThread][Unexpected] type={type(e).__name__}, err={e}")
            time.sleep(0.05)

def get_dynamic_tracking_params(distance_m):
    """
    根据目标距离生成动态参数（连续插值）。
    冷启动/无效测距时默认按 DEFAULT_TRACKING_DISTANCE_M 处理。
    """
    if (distance_m is None) or (distance_m <= 0):
        distance_m = DEFAULT_TRACKING_DISTANCE_M

    dist_nodes = np.array([50.0, 100.0, 200.0, 400.0, 500.0], dtype=float)
    d = float(np.clip(distance_m, dist_nodes[0], dist_nodes[-1]))

    max_res_az = float(np.interp(d, dist_nodes, [2.5, 1.8, 1.2, 0.9, 0.7]))
    max_vel_az = float(np.interp(d, dist_nodes, [30.0, 20.0, 12.0, 8.0, 5.0]))
    dist_thresh = float(np.interp(d, dist_nodes, [4.0, 3.0, 2.0, 1.3, 0.9]))

    return {
        "DIST_M": d,
        "MAX_RES_AZ": max_res_az,#单帧角度的最大修正
        "MAX_RES_EL": max_res_az * 0.5,
        "MAX_VEL_AZ": max_vel_az,#速度限幅
        "MAX_VEL_EL": max_vel_az * 0.5,
        "DIST_THRESH": dist_thresh,#判定“当前帧的检测点”与“上一帧的追踪轨迹”是否为同一个目标的最大角度欧氏距离。
        "MIN_DT": 0.001,  # 仅防异常极小值，不再把 15 FPS 的实际 dt 抬高
    }

class RangeSmoother:
    """激光测距平滑器：限速 + EMA，防止参数抖动。"""
    def __init__(self, init_d=50.0, alpha=0.2, max_rate_mps=30.0):
        self.d = float(init_d)
        self.alpha = float(alpha)
        self.max_rate_mps = float(max_rate_mps)

    def update(self, raw_d, dt):
        if (raw_d is None) or (raw_d <= 0):
            return self.d

        raw_d = float(np.clip(raw_d, 50.0, 500.0))
        dt_eff = max(float(dt), 0.03)
        max_step = self.max_rate_mps * dt_eff

        raw_d = max(self.d - max_step, min(self.d + max_step, raw_d))
        self.d = self.d + self.alpha * (raw_d - self.d)
        return self.d

def ui_to_ctrl_angles(ui_az, ui_el):
    """将设备自身相对方位/俯仰转换为云台编码器控制角。"""
    rel_az = ui_az
    if rel_az > 180.0:
        rel_az -= 360.0
    ctrl_az = GIMBAL_AZ_BASE + rel_az
    ctrl_el = GIMBAL_INIT_EL + ui_el
    if ctrl_az < 0.0: ctrl_az = 0.0
    if ctrl_az > 350.0: ctrl_az = 350.0
    return ctrl_az, ctrl_el

# ==========================================
# 新增模块 2：单目标卡尔曼追踪器 (AngleTracker)
# ==========================================
class StandardKalmanTrack:
    _id_count = 0
    def __init__(self, ui_az, ui_el, init_ts=None):
        StandardKalmanTrack._id_count += 1
        self.id = StandardKalmanTrack._id_count
        init_ts = time.time() if init_ts is None else float(init_ts)
        
        # 1. 原始 4D CV 状态矩阵 (Active 主控制源)
        self.state = np.array([[ui_az], [ui_el], [0.0], [0.0]], dtype=float)
        self.P = np.diag([1.0, 1.0, 10.0, 10.0])
        self.q_pos = 0.05
        self.q_vel = 0.2
        self.r_az = 3.5**2
        self.r_el = 1.8**2
        
        # 2. 新增 6D CA 影子状态矩阵 (Shadow 验证源)
        self.shadow_state = np.array([[ui_az], [ui_el], [0.0], [0.0], [0.0], [0.0]], dtype=float)
        self.shadow_P = np.diag([1.0, 1.0, 10.0, 10.0, 5.0, 5.0])
        self.shadow_q_pos = 0.05
        self.shadow_q_vel = 0.2
        self.shadow_q_acc = 0.1
        
        self.hit_streak = 1        # 连续命中次数 (用于建轨确认)
        self.time_since_update = 0 # 连丢次数
        self.created_ts = init_ts
        self.last_update_ts = init_ts  # last successful detection association time
        self.confirmed = False     # internal tracker confirmation
        self.ui_confirmed = False  # allow assigning/sending external UI ID
        self.strike_confirmed = False  # allow entering strike threat ranking
        
        # 历史队列 (基于 Active 状态记录)
        self.history = deque(maxlen=30)
        self.history.append((self.state.copy(), self.P.copy()))
        
        self.max_vel_az = 40.0
        self.max_vel_el = 15.0
        self.max_acc_az = 20.0  # 影子 CA 方位角加速度限幅
        self.max_acc_el = 10.0  # 影子 CA 俯仰角加速度限幅
        self.min_dt = 0.001
        self.dist_thresh = 4.0
        self.last_mono_dist = None
        self.mono_ts = 0.0
        self.last_sent_dist = None
        # Detection provenance belongs to the track, not to the most recently
        # received frame. UI status uses these fields after multi-camera fusion.
        self.last_source_board = None
        self.last_source_cam = None
        self.last_source_logic_id = None
        self.last_source_boards = ()
        self.last_source_cams = ()
        self.last_source_logic_ids = ()
        self.last_source_ts = 0.0

    def lost_seconds(self, now_t=None):
        now_t = time.time() if now_t is None else float(now_t)
        return max(0.0, now_t - float(self.last_update_ts))

    def set_detection_source(self, measurement, ts):
        """Remember the latest measurement provenance for per-track UI output."""
        if not isinstance(measurement, dict):
            return
        board = measurement.get("board")
        cam = measurement.get("cam")
        logic_id = measurement.get("logic_id")
        if board not in (None, ""):
            self.last_source_board = str(board)
        if cam not in (None, ""):
            try:
                self.last_source_cam = int(cam)
            except (TypeError, ValueError):
                self.last_source_cam = cam
        if logic_id not in (None, ""):
            self.last_source_logic_id = logic_id
        self.last_source_boards = tuple(measurement.get("source_boards") or ())
        self.last_source_cams = tuple(measurement.get("source_cams") or ())
        self.last_source_logic_ids = tuple(measurement.get("source_logic_ids") or ())
        source_ts = measurement.get("source_ts")
        try:
            parsed_source_ts = float(source_ts)
            self.last_source_ts = parsed_source_ts if parsed_source_ts > 0.0 else float(ts)
        except (TypeError, ValueError):
            self.last_source_ts = float(ts)

    def set_mono_distance(self, dist, ts, source="mono"):
        """Retain fixed-camera mono distance for tracker parameterization."""
        distance = _parse_positive_float(dist)
        if distance is None:
            return False
        self.last_mono_dist = distance
        self.mono_ts = float(ts)
        return True
    def get_param_distance(self, curr_time):
        if self.last_mono_dist is not None and (curr_time - self.mono_ts) <= MONO_DIST_TTL:
            return self.last_mono_dist
        if self.last_mono_dist is not None:
            return self.last_mono_dist
        return None

    def set_dynamic_params(self, params):
        if not params:
            return
        
        max_res_az = float(params.get('MAX_RES_AZ', 3.5))
        max_res_el = float(params.get('MAX_RES_EL', 1.8))
        self.r_az = max_res_az**2
        self.r_el = max_res_el**2
        
        self.max_vel_az = float(params.get('MAX_VEL_AZ', self.max_vel_az))
        self.max_vel_el = float(params.get('MAX_VEL_EL', self.max_vel_el))
        self.min_dt = float(params.get('MIN_DT', self.min_dt))
        self.dist_thresh = float(params.get('DIST_THRESH', self.dist_thresh))

    def predict(self, dt):
        """Active CV 与 Shadow CA 平行角度预测"""
        if dt < self.min_dt:
            dt = self.min_dt
            
        # A. Active 4D CV Predict
        F = np.array([
            [1, 0, dt,  0],
            [0, 1,  0, dt],
            [0, 0,  1,  0],
            [0, 0,  0,  1]
        ], dtype=float)
        Q = np.diag([self.q_pos, self.q_pos, self.q_vel, self.q_vel])
        self.state = np.dot(F, self.state)
        self.state[0, 0] = self.state[0, 0] % 360.0
        self.P = np.dot(np.dot(F, self.P), F.T) + Q
        
        # B. Shadow 6D CA Predict
        F_shadow = np.array([
            [1.0, 0.0,  dt, 0.0, 0.5*dt**2,       0.0],
            [0.0, 1.0, 0.0,  dt,       0.0, 0.5*dt**2],
            [0.0, 0.0, 1.0, 0.0,        dt,       0.0],
            [0.0, 0.0, 0.0, 1.0,       0.0,        dt],
            [0.0, 0.0, 0.0, 0.0,       1.0,       0.0],
            [0.0, 0.0, 0.0, 0.0,       0.0,       1.0]
        ], dtype=float)
        Q_shadow = np.diag([self.shadow_q_pos, self.shadow_q_pos, self.shadow_q_vel, self.shadow_q_vel, self.shadow_q_acc, self.shadow_q_acc])
        self.shadow_state = np.dot(F_shadow, self.shadow_state)
        self.shadow_state[0, 0] = self.shadow_state[0, 0] % 360.0
        self.shadow_P = np.dot(np.dot(F_shadow, self.shadow_P), F_shadow.T) + Q_shadow
        
        self.time_since_update += 1
        self.history.append((self.state.copy(), self.P.copy()))

    def update(self, meas_az, meas_el, dt, now_t=None):
        """Active CV 与 Shadow CA 平行角度更新"""
        now_t = time.time() if now_t is None else float(now_t)
        self.time_since_update = 0
        self.last_update_ts = now_t
        self.hit_streak += 1
        if self.hit_streak >= TRACK_CONFIRM_HITS:
            self.confirmed = True

        # A. Active 4D CV Update
        Z = np.array([[meas_az], [meas_el]], dtype=float)
        H = np.array([
            [1, 0, 0, 0],
            [0, 1, 0, 0]
        ], dtype=float)
        R = np.diag([self.r_az, self.r_el])
        Y = Z - np.dot(H, self.state)
        Y[0, 0] = angular_diff(Z[0, 0], self.state[0, 0])
        S = np.dot(np.dot(H, self.P), H.T) + R
        try:
            K = np.dot(np.dot(self.P, H.T), np.linalg.inv(S))
        except np.linalg.LinAlgError:
            K = np.zeros((4, 2), dtype=float)
        self.state = self.state + np.dot(K, Y)
        self.state[0, 0] = self.state[0, 0] % 360.0
        self.state[2, 0] = np.clip(self.state[2, 0], -self.max_vel_az, self.max_vel_az)
        self.state[3, 0] = np.clip(self.state[3, 0], -self.max_vel_el, self.max_vel_el)
        I = np.eye(4)
        self.P = np.dot((I - np.dot(K, H)), self.P)

        # B. Shadow 6D CA Update
        H_shadow = np.array([
            [1.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            [0.0, 1.0, 0.0, 0.0, 0.0, 0.0]
        ], dtype=float)
        Y_shadow = Z - np.dot(H_shadow, self.shadow_state)
        Y_shadow[0, 0] = angular_diff(Z[0, 0], self.shadow_state[0, 0])
        S_shadow = np.dot(np.dot(H_shadow, self.shadow_P), H_shadow.T) + R
        try:
            K_shadow = np.dot(np.dot(self.shadow_P, H_shadow.T), np.linalg.inv(S_shadow))
        except np.linalg.LinAlgError:
            K_shadow = np.zeros((6, 2), dtype=float)
        self.shadow_state = self.shadow_state + np.dot(K_shadow, Y_shadow)
        self.shadow_state[0, 0] = self.shadow_state[0, 0] % 360.0
        self.shadow_state[2, 0] = np.clip(self.shadow_state[2, 0], -self.max_vel_az, self.max_vel_az)
        self.shadow_state[3, 0] = np.clip(self.shadow_state[3, 0], -self.max_vel_el, self.max_vel_el)
        self.shadow_state[4, 0] = np.clip(self.shadow_state[4, 0], -self.max_acc_az, self.max_acc_az)
        self.shadow_state[5, 0] = np.clip(self.shadow_state[5, 0], -self.max_acc_el, self.max_acc_el)
        I_shadow = np.eye(6)
        self.shadow_P = np.dot((I_shadow - np.dot(K_shadow, H_shadow)), self.shadow_P)

        if len(self.history) > 0:
            self.history[-1] = (self.state.copy(), self.P.copy())
        else:
            self.history.append((self.state.copy(), self.P.copy()))

    def get_future_position(self, dt_delay):
        """Active 4D CV 角度预测"""
        fut_az = (self.state[0, 0] + self.state[2, 0] * dt_delay) % 360.0
        fut_el = self.state[1, 0] + self.state[3, 0] * dt_delay
        return fut_az, fut_el

    def get_shadow_future_position_ca(self, dt_delay):
        """Shadow 6D CA 角度预测 (仅供日志对比，不驱动云台)"""
        fut_az = (self.shadow_state[0, 0] + self.shadow_state[2, 0] * dt_delay + 0.5 * self.shadow_state[4, 0] * (dt_delay**2)) % 360.0
        fut_el = self.shadow_state[1, 0] + self.shadow_state[3, 0] * dt_delay + 0.5 * self.shadow_state[5, 0] * (dt_delay**2)
        return fut_az, fut_el

    def predict_future_n_steps(self, n=10, dt=0.066):
        """多帧预测接口 (兼容原版)"""
        if dt < self.min_dt:
            dt = self.min_dt
            
        F = np.array([
            [1, 0, dt,  0],
            [0, 1,  0, dt],
            [0, 0,  1,  0],
            [0, 0,  0,  1]
        ], dtype=float)
        Q = np.diag([self.q_pos, self.q_pos, self.q_vel, self.q_vel])
        
        temp_state = self.state.copy()
        temp_P = self.P.copy()
        
        future_states = []
        future_Ps = []
        
        for _ in range(n):
            temp_state = np.dot(F, temp_state)
            temp_state[0, 0] = temp_state[0, 0] % 360.0
            temp_P = np.dot(np.dot(F, temp_P), F.T) + Q
            future_states.append(temp_state.copy())
            future_Ps.append(temp_P.copy())
            
        return future_states, future_Ps

# ==========================================
# 新增模块 3：多目标调度大脑 (MultiTargetTracker)
# ==========================================
class MultiTargetTracker:
    def __init__(
        self,
        max_lost_frames=30,
        max_lost_seconds=None,
        base_distance_threshold=4.0,
        distance_threshold=None,
    ):
        self.tracks = []
        self.max_lost_frames = max_lost_frames
        self.max_lost_seconds = (
            float(max_lost_seconds)
            if max_lost_seconds is not None
            else float(max_lost_frames) * NO_PACKET_TRACKER_UPDATE_INTERVAL
        )
        # Backward compatibility: keep supporting old constructor arg `distance_threshold`.
        if distance_threshold is not None:
            self.base_distance_threshold = float(distance_threshold)
        else:
            self.base_distance_threshold = float(base_distance_threshold)

    def _debug_context_row(self, debug_context, event):
        debug_context = debug_context or {}
        return {
            "timestamp": f"{time.time():.6f}",
            "mode": debug_context.get("mode", ""),
            "frame_id": debug_context.get("frame_id", ""),
            "event": event,
            "meas_count": debug_context.get("meas_count", ""),
        }

    @staticmethod
    def _association_gate(track, now_t):
        """Return a bounded association gate before Hungarian assignment."""
        uncertainty = float(np.sqrt(max(0.0, track.P[0, 0] + track.P[1, 1])))
        covariance_gate = float(track.dist_thresh) + (uncertainty * 1.5)
        lost_seconds = track.lost_seconds(now_t)
        hard_cap = TRACK_ASSOCIATION_MAX_DEG
        gate_mode = "recent"
        if lost_seconds > TRACK_REACQUIRE_STRICT_AFTER_SECONDS:
            hard_cap = min(hard_cap, TRACK_REACQUIRE_MAX_DEG)
            gate_mode = "strict_reacquire"
        dynamic_thresh = max(0.0, min(covariance_gate, hard_cap))
        return dynamic_thresh, uncertainty, lost_seconds, gate_mode

    def _log_new_track(self, track, meas_idx, meas, debug_context):
        if DEBUG_KALMAN_MATCH:
            print(
                f"[NEW_TRACK] track={track.id}, meas={meas_idx}, "
                f"meas=(Az={meas['az']:.2f}, El={meas['el']:.2f}), "
                f"mono={meas['mono_dist']}, dist_thresh={track.dist_thresh:.2f}, "
                f"R=({track.r_az:.2f},{track.r_el:.2f}), "
                f"Q=({track.q_pos:.2f},{track.q_vel:.2f})"
            )
        row = self._debug_context_row(debug_context, "NEW_TRACK")
        field_log_event({
            "timestamp": row["timestamp"],
            "seq": row["frame_id"],
            "mode": row["mode"],
            "event": "NEW_TRACK",
            "track_id": int(track.id),
            "meas_idx": meas_idx,
            "meas_az": f"{meas['az']:.6f}",
            "meas_el": f"{meas['el']:.6f}",
            "p_az": f"{track.P[0, 0]:.6f}",
            "p_el": f"{track.P[1, 1]:.6f}",
            "hit_streak": int(track.hit_streak),
            "time_since_update": int(track.time_since_update),
            "lost_seconds": "0.000000",
        })

    def _prune_lost_tracks(self, now_t=None, debug_context=None):
        now_t = time.time() if now_t is None else float(now_t)
        kept_tracks = []
        for track in self.tracks:
            lost_s = track.lost_seconds(now_t)
            if lost_s < self.max_lost_seconds:
                kept_tracks.append(track)
                continue
            if DEBUG_KALMAN_MATCH:
                print(
                    f"[TRACK_DELETE] track={track.id}, "
                    f"lost={lost_s:.2f}s/{track.time_since_update} updates, "
                    f"max_lost={self.max_lost_seconds:.2f}s, "
                    f"state=(Az={track.state[0,0]:.2f}, El={track.state[1,0]:.2f}), "
                    f"hits={track.hit_streak}"
                )
            row = self._debug_context_row(debug_context, "TRACK_DELETE")
            field_log_event({
                "timestamp": row["timestamp"],
                "seq": row["frame_id"],
                "mode": row["mode"],
                "event": "TRACK_DELETE",
                "track_id": int(track.id),
                "pred_az": f"{track.state[0, 0]:.6f}",
                "pred_el": f"{track.state[1, 0]:.6f}",
                "p_az": f"{track.P[0, 0]:.6f}",
                "p_el": f"{track.P[1, 1]:.6f}",
                "hit_streak": int(track.hit_streak),
                "time_since_update": int(track.time_since_update),
                "lost_seconds": f"{lost_s:.6f}",
                "reason": f"lost_seconds>={self.max_lost_seconds:.3f}",
            })
        self.tracks = kept_tracks

    def update(self, measurements, dt, params=None, now_t=None, debug_context=None):
        """
        measurements: 当前帧所有检测目标，可为:
            1) [az, el]
            2) {"az": az, "el": el, "mono_dist": d}
        dt: 距离上一帧经过的时间(秒)
        """
        if now_t is None:
            now_t = time.time()

        if params and ("DIST_THRESH" in params):
            self.base_distance_threshold = float(params["DIST_THRESH"])

        normalized_measurements = []
        for meas in measurements:
            if isinstance(meas, dict):
                az = meas.get("az", None)
                el = meas.get("el", None)
                mono_dist = meas.get("mono_dist", None)
                source_metadata = {
                    key: meas.get(key)
                    for key in (
                        "board", "cam", "logic_id", "source_ts", "source_boards",
                        "source_cams", "source_logic_ids",
                    )
                }
            elif isinstance(meas, (list, tuple)) and len(meas) >= 2:
                az = meas[0]
                el = meas[1]
                mono_dist = meas[2] if len(meas) >= 3 else None
                source_metadata = {}
            else:
                continue

            try:
                az = float(az) % 360.0
                el = float(el)
            except (TypeError, ValueError):
                continue
            mono_dist = _parse_positive_float(mono_dist)
            normalized_measurement = {
                "az": az,
                "el": el,
                "mono_dist": mono_dist,
            }
            normalized_measurement.update(source_metadata)
            normalized_measurements.append(normalized_measurement)

        # 1. 预测所有已有 Track 的新位置
        for track in self.tracks:
            if params:
                track.set_dynamic_params(params)
            else:
                dist_for_track = track.get_param_distance(now_t)
                if dist_for_track is not None:
                    track.set_dynamic_params(get_dynamic_tracking_params(dist_for_track))
                else:
                    track.dist_thresh = self.base_distance_threshold
            track.predict(dt)
            
        # 如果当前帧没检测到东西，直接清理丢失目标并返回
        if len(normalized_measurements) == 0:
            self._prune_lost_tracks(now_t=now_t, debug_context=debug_context)
            return self.tracks

        if len(self.tracks) == 0:
            # 全是新目标,新建轨迹
            for m_idx, meas in enumerate(normalized_measurements):
                t = StandardKalmanTrack(
                    meas["az"],
                    meas["el"],
                    init_ts=now_t,
                )
                t.set_mono_distance(meas["mono_dist"], now_t)
                t.set_detection_source(meas, now_t)
                if params:
                    t.set_dynamic_params(params)
                else:
                    dist_for_track = t.get_param_distance(now_t)
                    if dist_for_track is not None:
                        t.set_dynamic_params(get_dynamic_tracking_params(dist_for_track))
                    else:
                        t.dist_thresh = self.base_distance_threshold
                self.tracks.append(t)
                self._log_new_track(t, m_idx, meas, debug_context)
            return self.tracks

        # 2. 计算代价dxfcs矩阵 (角度欧氏距离)
        cost_matrix = np.zeros((len(self.tracks), len(normalized_measurements)))
        for t, track in enumerate(self.tracks):
            for m, meas in enumerate(normalized_measurements):
                diff_az = angular_diff(meas["az"], track.state[0, 0])

                diff_el = meas["el"] - track.state[1, 0]
                distance = np.sqrt(diff_az**2 + diff_el**2)
                cost_matrix[t, m] = distance

        # 3. Gate impossible pairs before Hungarian assignment. A large finite
        # cost keeps scipy robust when a row has no feasible measurement; the
        # post-assignment check below still leaves blocked pairs unmatched.
        association_gates = [
            self._association_gate(track, now_t) for track in self.tracks
        ]
        gated_cost_matrix = cost_matrix.copy()
        for t_idx, gate_info in enumerate(association_gates):
            dynamic_thresh = gate_info[0]
            blocked = gated_cost_matrix[t_idx, :] >= dynamic_thresh
            gated_cost_matrix[t_idx, blocked] = ASSOCIATION_BLOCKED_COST
        track_indices, meas_indices = linear_sum_assignment(gated_cost_matrix)

        # 4. 更新匹配成功的 Track (加入协方差动态门限)
        unmatched_measurements = set(range(len(normalized_measurements)))
        matched_tracks = set()
        for t_idx, m_idx in zip(track_indices, meas_indices):
            track = self.tracks[t_idx]
            
            dynamic_thresh, uncertainty, lost_s_before_match, gate_mode = (
                association_gates[t_idx]
            )
            cost = cost_matrix[t_idx, m_idx]
            meas = normalized_measurements[m_idx]
            pred_az = float(track.state[0, 0])
            pred_el = float(track.state[1, 0])
            pair_is_feasible = gated_cost_matrix[t_idx, m_idx] < ASSOCIATION_BLOCKED_COST
            
            if pair_is_feasible:
                if DEBUG_KALMAN_MATCH:
                    print(
                        f"[MATCH_ACCEPT] track={track.id}, meas={m_idx}, "
                        f"meas=(Az={meas['az']:.2f}, El={meas['el']:.2f}), "
                        f"pred=(Az={pred_az:.2f}, El={pred_el:.2f}), "
                        f"cost={cost:.2f}, thresh={dynamic_thresh:.2f}, gate={gate_mode}, "
                        f"unc={uncertainty:.2f}, "
                        f"Ppos=({track.P[0,0]:.2f},{track.P[1,1]:.2f}), "
                        f"hits={track.hit_streak}, lost={track.time_since_update}, "
                        f"R=({track.r_az:.2f},{track.r_el:.2f}), "
                        f"Q=({track.q_pos:.2f},{track.q_vel:.2f})"
                    )
                row = self._debug_context_row(debug_context, "MATCH_ACCEPT")
                field_log_event({
                    "timestamp": row["timestamp"],
                    "seq": row["frame_id"],
                    "mode": row["mode"],
                    "event": "MATCH_ACCEPT",
                    "track_id": int(track.id),
                    "meas_idx": m_idx,
                    "meas_az": f"{meas['az']:.6f}",
                    "meas_el": f"{meas['el']:.6f}",
                    "pred_az": f"{pred_az:.6f}",
                    "pred_el": f"{pred_el:.6f}",
                    "cost": f"{cost:.6f}",
                    "dynamic_thresh": f"{dynamic_thresh:.6f}",
                    "uncertainty": f"{uncertainty:.6f}",
                    "p_az": f"{track.P[0, 0]:.6f}",
                    "p_el": f"{track.P[1, 1]:.6f}",
                    "hit_streak": int(track.hit_streak),
                    "time_since_update": int(track.time_since_update),
                    "lost_seconds": f"{lost_s_before_match:.6f}",
                })
                track.update(meas["az"], meas["el"], dt, now_t=now_t)
                track.set_mono_distance(meas["mono_dist"], now_t)
                track.set_detection_source(meas, now_t)
                unmatched_measurements.discard(m_idx)
                matched_tracks.add(t_idx)
            else:
                if DEBUG_KALMAN_MATCH:
                    print(
                        f"[MATCH_REJECT] track={track.id}, meas={m_idx}, "
                        f"meas=(Az={meas['az']:.2f}, El={meas['el']:.2f}), "
                        f"pred=(Az={pred_az:.2f}, El={pred_el:.2f}), "
                        f"cost={cost:.2f}, thresh={dynamic_thresh:.2f}, gate={gate_mode}, "
                        f"unc={uncertainty:.2f}, "
                        f"Ppos=({track.P[0,0]:.2f},{track.P[1,1]:.2f}), "
                        f"dist_thresh={track.dist_thresh:.2f}, "
                        f"hits={track.hit_streak}, lost={track.time_since_update}, "
                        f"R=({track.r_az:.2f},{track.r_el:.2f}), "
                        f"Q=({track.q_pos:.2f},{track.q_vel:.2f})"
                    )
                row = self._debug_context_row(debug_context, "MATCH_REJECT")
                field_log_event({
                    "timestamp": row["timestamp"],
                    "seq": row["frame_id"],
                    "mode": row["mode"],
                    "event": "MATCH_REJECT",
                    "track_id": int(track.id),
                    "meas_idx": m_idx,
                    "meas_az": f"{meas['az']:.6f}",
                    "meas_el": f"{meas['el']:.6f}",
                    "pred_az": f"{pred_az:.6f}",
                    "pred_el": f"{pred_el:.6f}",
                    "cost": f"{cost:.6f}",
                    "dynamic_thresh": f"{dynamic_thresh:.6f}",
                    "uncertainty": f"{uncertainty:.6f}",
                    "p_az": f"{track.P[0, 0]:.6f}",
                    "p_el": f"{track.P[1, 1]:.6f}",
                    "hit_streak": int(track.hit_streak),
                    "time_since_update": int(track.time_since_update),
                    "lost_seconds": f"{lost_s_before_match:.6f}",
                    "reason": f"pre_hungarian_gate:{gate_mode}",
                })

        # 未匹配轨迹衰减稳定帧，避免历史累计导致“永久霸榜”
        for t_idx, track in enumerate(self.tracks):
            if t_idx not in matched_tracks:
                track.hit_streak = max(0, track.hit_streak - HIT_STREAK_DECAY)

        # 5. 为没匹配上的坐标创建新 Track
        for m_idx in unmatched_measurements:
            meas = normalized_measurements[m_idx]
            t = StandardKalmanTrack(meas["az"], meas["el"], init_ts=now_t)
            t.set_mono_distance(meas["mono_dist"], now_t)
            t.set_detection_source(meas, now_t)
            if params:
                t.set_dynamic_params(params)
            else:
                dist_for_track = t.get_param_distance(now_t)
                if dist_for_track is not None:
                    t.set_dynamic_params(get_dynamic_tracking_params(dist_for_track))
                else:
                    t.dist_thresh = self.base_distance_threshold
            self.tracks.append(t)
            self._log_new_track(t, m_idx, meas, debug_context)

        # 6. 删除丢失太久的 Track
        self._prune_lost_tracks(now_t=now_t, debug_context=debug_context)

        return self.tracks

# ==========================================
# 4. 核心解算 V8 (修改版：基准水平90度)
# ==========================================
def calculate_angles(
    cam_key,
    cx,
    cy,
    cfg=None,
    image_w=IMG_W,
    image_h=IMG_H,
):
    base_az = float(cfg["theta_horizontal"])
    base_el = float(cfg["theta_vertical"])
    try:
        logic_id = int(cam_key)
    except (TypeError, ValueError):
        logic_id = None
    if logic_id not in MANUALLY_CALIBRATED_LOGIC_IDS:
        base_az = (
            base_az + UNTESTED_CAMERA_THETA_HORIZONTAL_OFFSET_DEG
        ) % 360.0
        base_el += UNTESTED_CAMERA_THETA_VERTICAL_OFFSET_DEG
    image_w = float(image_w)
    image_h = float(image_h)
    
    # 1. 计算目标在图像中的像素偏移
    diff_x = cx - (image_w / 2.0)
    diff_y = cy - (image_h / 2.0)
    
    # 2. 像素转换成角度偏移
    offset_az = diff_x * FOV_X / image_w
    offset_el = -diff_y * FOV_Y / image_h

    # 3. 计算设备自身坐标系角度；这还不是真北地图方位角。
    ui_az = (base_az + offset_az) % 360.0
    ui_el = base_el + offset_el

    return ui_az, ui_el


def evaluate_track_threat(track, curr_gimbal_az, curr_gimbal_el, master_id=None):
    """保留现有调度权重，仅把评分拆出来便于审计与日志打印。"""
    v_mag = math.hypot(track.state[2, 0], track.state[3, 0])
    speed_score = v_mag * 2.0

    dist_az = angular_diff(track.state[0, 0], curr_gimbal_az)
    dist_el = track.state[1, 0] - curr_gimbal_el
    dist = math.hypot(dist_az, dist_el)
    distance_score = -dist * 1.0

    stability_level = min(track.hit_streak, STABILITY_HIT_CAP)
    stability_score = math.log1p(stability_level) * STABILITY_WEIGHT

    threat_score = speed_score + distance_score + stability_score
    inertia_applied = (track.id == master_id)
    if inertia_applied:
        threat_score *= 1.15

    return {
        "track": track,
        "track_id": int(track.id),
        "speed_deg_s": v_mag,
        "distance_deg": dist,
        "hit_streak": int(track.hit_streak),
        "speed_score": speed_score,
        "distance_score": distance_score,
        "stability_score": stability_score,
        "inertia_applied": inertia_applied,
        "threat_score": threat_score,
    }


def choose_master_track(valid_tracks, curr_gimbal_az, curr_gimbal_el, master_id=None):
    ranked_candidates = [
        evaluate_track_threat(t, curr_gimbal_az, curr_gimbal_el, master_id=master_id)
        for t in valid_tracks
    ]
    ranked_candidates.sort(key=lambda item: item["threat_score"], reverse=True)
    best_track = ranked_candidates[0]["track"] if ranked_candidates else None
    return best_track, ranked_candidates


def format_selection_candidates(ranked_candidates, topk=MASTER_SELECTION_LOG_TOPK):
    if not ranked_candidates:
        return "none"

    items = []
    for item in ranked_candidates[:topk]:
        inertia_text = ", hold=1.15x" if item["inertia_applied"] else ""
        items.append(
            f"id={item['track_id']}, score={item['threat_score']:.2f}, "
            f"speed={item['speed_deg_s']:.2f}deg/s, dist={item['distance_deg']:.2f}deg, "
            f"hits={item['hit_streak']}, stab={item['stability_score']:.2f}{inertia_text}"
        )
    return " | ".join(items)


def strike_measurement_readiness(
    track_result,
    curr_time,
    require_vision_confirmation=True,
):
    """Validate the measurement that permits strike delivery.

    Vision-enabled operation preserves the existing fresh/safe bbox gate.
    With vision disabled, only a fresh final RID distance can open the gate.
    """
    track_result = track_result or {}
    if require_vision_confirmation:
        measurement_ts = float(track_result.get("frame_ts", 0.0) or 0.0)
        freshness_ttl = GIMBAL_VISION_RESULT_TTL
        source_allowed = bool(track_result.get("safe"))
    else:
        measurement_ts = float(
            track_result.get("measurement_ts", 0.0) or 0.0
        )
        freshness_ttl = RID_DISTANCE_FRESH_SECONDS
        distance_source = str(
            track_result.get("distance_source", "none")
        ).lower()
        source_family = str(
            track_result.get("source_family", "")
        ).lower()
        source_allowed = bool(
            source_family == "rid" or distance_source.startswith("rid")
        )

    measurement_age = float(curr_time) - measurement_ts
    distance_m = _parse_positive_float(track_result.get("distance"))
    ready = bool(
        source_allowed
        and track_result.get("distance_valid", False)
        and distance_m is not None
        and 0.0 <= measurement_age <= freshness_ttl
    )
    return ready, measurement_age, freshness_ttl


def evaluate_strike_threat(
    track,
    curr_time,
    vision_track_result=None,
    strike_track_id=None,
    require_vision_confirmation=True,
):
    """Score targets for strike guidance.

    Unlike master selection, strike selection still requires a fresh safe
    gimbal-camera bbox, while distance comes from the SORT-owned final
    RID/vision arbitration state.
    """
    if (
        track is None
        or not track.confirmed
        or track.lost_seconds(curr_time) > MAX_LOCK_LOST_SECONDS
    ):
        return None

    vision_track_result = vision_track_result or {}
    measurement_ready, measurement_age, freshness_ttl = (
        strike_measurement_readiness(
            vision_track_result,
            curr_time,
            require_vision_confirmation=require_vision_confirmation,
        )
    )
    if not measurement_ready:
        return None

    distance_m = _parse_positive_float(vision_track_result.get("distance"))
    distance_source = str(
        vision_track_result.get("distance_source", "none")
    )
    radial_velocity = float(
        vision_track_result.get("radial_velocity_mps", 0.0) or 0.0
    )
    closing_speed_mps = max(0.0, -radial_velocity)
    stability_level = min(track.hit_streak, STABILITY_HIT_CAP)
    stability_score = math.log1p(stability_level) * 3.0
    distance_score = max(0.0, 600.0 - float(distance_m)) * 0.10
    closing_score = closing_speed_mps * 2.0
    freshness_score = max(0.0, freshness_ttl - measurement_age) * 3.0

    threat_score = (
        distance_score
        + closing_score
        + stability_score
        + freshness_score
    )
    raw_threat_score = threat_score

    inertia_applied = (track.id == strike_track_id)
    if inertia_applied:
        threat_score *= 1.20

    return {
        "track": track,
        "track_id": int(track.id),
        "distance_m": float(distance_m),
        "distance_source": distance_source,
        "radial_velocity_mps": radial_velocity,
        "closing_speed_mps": closing_speed_mps,
        "hit_streak": int(track.hit_streak),
        "vision_age": measurement_age,
        "distance_score": distance_score,
        "closing_score": closing_score,
        "stability_score": stability_score,
        "freshness_score": freshness_score,
        "inertia_applied": inertia_applied,
        "raw_threat_score": raw_threat_score,
        "threat_score": threat_score,
    }


def choose_strike_target(
    valid_tracks,
    curr_time,
    track_results,
    strike_track_id=None,
    require_vision_confirmation=True,
):
    ranked_candidates = []
    for track in valid_tracks:
        item = evaluate_strike_threat(
            track,
            curr_time,
            vision_track_result=track_results.get(int(track.id), {}),
            strike_track_id=strike_track_id,
            require_vision_confirmation=require_vision_confirmation,
        )
        if item is not None:
            ranked_candidates.append(item)
    ranked_candidates.sort(key=lambda item: item["threat_score"], reverse=True)
    best_track = ranked_candidates[0]["track"] if ranked_candidates else None
    return best_track, ranked_candidates


def build_strike_track_results(current_results, cached_results, curr_time):
    """Briefly retain a track only when it is absent from the current vision result."""
    now_t = float(curr_time)
    normalized_results = {}
    if isinstance(current_results, dict):
        for raw_track_id, result in current_results.items():
            try:
                track_id = int(raw_track_id)
            except (TypeError, ValueError):
                continue
            if not isinstance(result, dict):
                continue
            normalized_results[track_id] = result
            cached_results[track_id] = {
                "result": dict(result),
                "last_present_ts": now_t,
            }

    for track_id, cached_entry in list(cached_results.items()):
        if track_id in normalized_results:
            continue
        missing_age = now_t - float(cached_entry["last_present_ts"])
        if not (
            0.0 <= missing_age <= STRIKE_VISION_MISSING_HOLD_SECONDS
        ):
            del cached_results[track_id]
            continue
        held_result = dict(cached_entry["result"])
        held_result["strike_missing_hold"] = True
        normalized_results[track_id] = held_result
    return normalized_results


def format_strike_candidates(ranked_candidates, topk=MASTER_SELECTION_LOG_TOPK):
    if not ranked_candidates:
        return "none"
    items = []
    for item in ranked_candidates[:topk]:
        inertia_text = ", hold=1.20x" if item["inertia_applied"] else ""
        items.append(
            f"id={item['track_id']}, score={item['threat_score']:.2f}, "
            f"dist={item['distance_m']:.1f}m, close={item['closing_speed_mps']:.1f}m/s, "
            f"hits={item['hit_streak']}, age={item['vision_age']:.2f}s{inertia_text}"
        )
    return " | ".join(items)
# ==========================================
# 6. 主逻辑 V9 (多目标预测与云台调度)
# ==========================================
def build_target_measurement_config():
    """Expose deployment configuration while keeping ranging code modular."""
    return {
        "enable_rid": ENABLE_RID,
        "rid_port": RID_PORT,
        "rid_baudrate": RID_BAUDRATE,
        "rid_serial_timeout": RID_SERIAL_TIMEOUT,
        "rid_reconnect_s": RID_RECONNECT_SECONDS,
        "rid_distance_fresh_s": RID_DISTANCE_FRESH_SECONDS,
        "rid_track_ttl_s": RID_TRACK_TTL_SECONDS,
        "rid_delete_after_s": RID_TRACK_DELETE_AFTER_SECONDS,
        "rid_ui_render_delay_s": RID_UI_RENDER_DELAY_SECONDS,
        "rid_ui_max_prediction_s": RID_UI_MAX_PREDICTION_SECONDS,
        "rid_ui_display_tau_s": RID_UI_DISPLAY_TAU_SECONDS,
        "rid_ui_filter_alpha": RID_UI_FILTER_ALPHA,
        "rid_ui_filter_beta": RID_UI_FILTER_BETA,
        "rid_ui_turn_reset_deg": RID_UI_TURN_RESET_DEG,
        "rid_ui_render_history_points": RID_UI_RENDER_HISTORY_POINTS,
        "rid_ui_max_speed_mps": RID_UI_MAX_SPEED_MPS,
        "rid_four_point_bias_deg": RID_FOUR_POINT_BIAS_DEG,
        "rid_four_point_window": RID_FOUR_POINT_WINDOW,
        "rid_four_point_max_bias_error_deg": (
            RID_FOUR_POINT_MAX_BIAS_ERROR_DEG
        ),
        "rid_four_point_max_shape_p95_deg": (
            RID_FOUR_POINT_MAX_SHAPE_P95_DEG
        ),
        "rid_assoc_history_s": RID_ASSOC_HISTORY_SECONDS,
        "rid_assoc_sync_tolerance_s": RID_ASSOC_SYNC_TOLERANCE_SECONDS,
        "enable_vision": ENABLE_GIMBAL_VISION,
        "vision_camera_source": GIMBAL_CAMERA_SOURCE,
        "vision_confidence": GIMBAL_VISION_CONFIDENCE,
        "vision_settle_delay_s": GIMBAL_VISION_SETTLE_DELAY,
        "vision_min_sharpness": GIMBAL_VISION_MIN_SHARPNESS,
        "vision_result_ttl_s": GIMBAL_VISION_RESULT_TTL,
        "vision_association_max_px": GIMBAL_VISION_ASSOCIATION_MAX_PX,
        "vision_ambiguity_margin_px": GIMBAL_VISION_AMBIGUITY_MARGIN_PX,
        "vision_track_state_ttl_s": GIMBAL_VISION_TRACK_STATE_TTL,
        "vision_y_compensation_px": GIMBAL_VISION_Y_COMPENSATION_PX,
        "track_distance_ttl_s": TRACK_DISTANCE_TTL,
    }


def measurement_runtime_event_sink(record):
    """Keep protected-runtime diagnostics routed through public log writers."""
    if FIELD_LOGGER is None:
        return
    event_name = str(record.get("event", ""))
    if event_name == "RID_RAW_SERIAL":
        FIELD_LOGGER.write_raw_rid_serial(record)
    elif event_name == "RID_RAW_PAYLOAD":
        FIELD_LOGGER.write_raw_rid({
            "receive_ts": record.get("receive_ts"),
            "payload": record.get("payload"),
        })
    else:
        field_log_event(record)


def main():
    global FIELD_LOGGER
    run_log_dir = None
    if LOG_TO_FILE or FIELD_LOG:
        try:
            run_log_dir = _create_run_log_dir(LOG_DIR)
        except Exception as e:
            run_log_dir = LOG_DIR
            print(f"[Log][Warn] 运行日志目录创建失败，回退到 {LOG_DIR}: {e}")

    if LOG_TO_FILE:
        try:
            log_path = _setup_log_mirror(run_log_dir or LOG_DIR)
            if log_path:
                print(f"[Log] stdout/stderr -> {log_path}")
        except Exception as e:
            print(f"[Log][Warn] 日志文件初始化失败: {e}")

    if FIELD_LOG:
        try:
            FIELD_LOGGER = FieldLogger(run_log_dir or FIELD_LOG_DIR)
        except Exception as e:
            FIELD_LOGGER = None
            print(f"[FieldLog][Warn] 初始化失败: {e}")

    sender = UISender(
        UI_IP,
        UI_PORT,
        on_status_send=field_log_target_detect,
    )
    strike_sender = StrikeSender(STRIKE_IP, STRIKE_PORT) if ENABLE_STRIKE_SEND else None
    strike_send_worker = None
    station_position = SharedPositionState(
        longitude=(
            DEFAULT_LONGITUDE if RID_ALLOW_DEFAULT_STATION_POSITION else None
        ),
        latitude=(
            DEFAULT_LATITUDE if RID_ALLOW_DEFAULT_STATION_POSITION else None
        ),
        source=(
            "configured_default"
            if RID_ALLOW_DEFAULT_STATION_POSITION
            else "waiting_for_wgs84_fix"
        ),
    )
    measurement_runtime = None
    special_rid_registry = None
    special_rid_stop_event = None
    special_rid_thread = None
    print(f"[Config] DEVICE_HEADING_DEG={DEVICE_HEADING_DEG:.2f} (map north=0, east=90, south=180)")
    print(
        "[Config] Camera theta correction: "
        f"manual_logic_ids={sorted(MANUALLY_CALIBRATED_LOGIC_IDS)}, "
        f"untested_horizontal={UNTESTED_CAMERA_THETA_HORIZONTAL_OFFSET_DEG:+.2f}deg, "
        f"untested_vertical={UNTESTED_CAMERA_THETA_VERTICAL_OFFSET_DEG:+.2f}deg, "
        f"vision_y_compensation={GIMBAL_VISION_Y_COMPENSATION_PX:+.1f}px"
    )
    print(
        "[Config] Detection UDP coordinates: "
        f"mode={UDP_DETECTION_COORD_MODE}, "
        f"size={UDP_DETECTION_W:.0f}x{UDP_DETECTION_H:.0f}, "
        f"switch=USE_NIGHT_DETECTION_COORDS={USE_NIGHT_DETECTION_COORDS}"
    )
    print(
        "[Config] Track association safety: "
        f"hard_cap={TRACK_ASSOCIATION_MAX_DEG:.2f}°, "
        f"strict_after={TRACK_REACQUIRE_STRICT_AFTER_SECONDS:.2f}s, "
        f"strict_cap={TRACK_REACQUIRE_MAX_DEG:.2f}°, "
        f"internal_keep={TRACK_MAX_LOST_SECONDS:.2f}s, "
        f"ui_fresh={UI_MAX_LOST_SECONDS:.2f}s"
    )
    print(
        "[Config] RID UI fusion: "
        f"enabled={1 if ENABLE_RID else 0}, "
        f"port={RID_PORT or 'unset'}, baud={RID_BAUDRATE}, "
        "association=four_point_bias_shape_gate, "
        "ui=sort_id/sort_az/sort_el/rid_distance+sort_camera, "
        f"bias={RID_FOUR_POINT_BIAS_DEG:.6f}deg, "
        f"window={RID_FOUR_POINT_WINDOW}, "
        f"max_ebias={RID_FOUR_POINT_MAX_BIAS_ERROR_DEG:.2f}deg, "
        f"max_eshape95={RID_FOUR_POINT_MAX_SHAPE_P95_DEG:.2f}deg, "
        f"sync={RID_ASSOC_SYNC_TOLERANCE_SECONDS:.2f}s, "
        f"render_delay={RID_UI_RENDER_DELAY_SECONDS:.2f}s, "
        f"max_prediction={RID_UI_MAX_PREDICTION_SECONDS:.2f}s, "
        f"display_tau={RID_UI_DISPLAY_TAU_SECONDS:.2f}s, "
        f"turn_reset={RID_UI_TURN_RESET_DEG:.1f}deg, "
        f"rid_distance_fresh={RID_DISTANCE_FRESH_SECONDS:.2f}s, "
        f"rid_ttl={RID_TRACK_TTL_SECONDS:.2f}s, "
        f"assoc_history={RID_ASSOC_HISTORY_SECONDS:.2f}s, "
        f"rid_delete_after={RID_TRACK_DELETE_AFTER_SECONDS:.2f}s"
    )
    print(
        "[Config] Special RID stable UI: "
        f"enabled={1 if SPECIAL_RID_IDENTITY_ENABLED else 0}, "
        f"ui_exclusive={1 if SPECIAL_RID_UI_EXCLUSIVE else 0}, "
        f"strike_ids={sorted(SPECIAL_RID_UI_IDS)}, "
        f"rate={SPECIAL_RID_UI_RATE_HZ:.1f}Hz, "
        f"predict={SPECIAL_RID_MAX_PREDICTION_SECONDS:.1f}s, "
        f"rid_fresh={SPECIAL_RID_FRESH_SECONDS:.1f}s, "
        f"sort_fresh={SPECIAL_RID_SORT_FRESH_SECONDS:.1f}s, "
        f"reacquire_delay={SPECIAL_RID_REACQUIRE_DELAY_SECONDS:.1f}s"
    )
    if ENABLE_STRIKE_SEND:
        print(
            f"[Strike] Enabled target UDP sender: {STRIKE_IP}:{STRIKE_PORT}, "
            f"hz={STRIKE_SEND_HZ:.1f}, window={STRIKE_WINDOW_SECONDS:.2f}s, "
            f"lead={STRIKE_LEAD_TIME:.2f}s, settled_ttl={STRIKE_SETTLED_EVENT_TTL:.2f}s"
        )
    if not USE_MOCK_GIMBAL:
        _validate_serial_port("GIMBAL_PORT", GIMBAL_PORT)
    if not USE_MOCK_LASER:
        _validate_serial_port("LASER_PORT", LASER_PORT)
    if ENABLE_GPS:
        _validate_serial_port("GPS_PORT", GPS_PORT)
    if ENABLE_RID:
        _validate_serial_port("RID_PORT", RID_PORT)

    if ENABLE_GPS:
        threading.Thread(
            target=gps_sender_thread,
            args=(sender, station_position),
            daemon=True,
        ).start()
    if USE_MOCK_GIMBAL:
        if MockGimbalAdapter is None:
            print("[Error] USE_MOCK_GIMBAL=True, but mock_gimbal.py import failed.")
            return
        print("[Init] Connecting to Mock Gimbal at MOCK_PORT...")
        gimbal = MockGimbalAdapter(port="MOCK_PORT", az_base=GIMBAL_AZ_BASE)
    else:
        print(f"[Init] Connecting to Gimbal at {GIMBAL_PORT}...")
        gimbal = GT06ZAdapter(port=GIMBAL_PORT)
    if not gimbal.connect():
        print("[Error] Failed to connect gimbal.")
        return
    
    if not gimbal.wait_ready():
        print("[Warning] Gimbal not ready instantly, wait...")

    laser = None
    laser_stop_event = None
    if USE_MOCK_LASER:
        print(
            "[Init] Legacy laser disabled by USE_MOCK_LASER=True; "
            "RID distance path is independent"
        )
    else:
        if SDDMLaser is None:
            print("[Laser][Warn] sddm_laser.py import failed; distance source remains mono.")
        else:
            try:
                laser = SDDMLaser(LASER_PORT)
                laser_stop_event = threading.Event()
                threading.Thread(
                    target=laser_reader_thread,
                    args=(laser, laser_stop_event),
                    daemon=True,
                ).start()
                print(f"[Init] Real laser read-only logging enabled on {LASER_PORT}")
            except Exception as e:
                print(f"[Laser][Warn] Init failed on {LASER_PORT}: {e}")
                laser = None

    # start background threads for network and gimbal control
    push_latest_gimbal_cmd({
        "cmd_id": 0,
        "track_id": -1,
        "az": GIMBAL_AZ_BASE,
        "el": GIMBAL_INIT_EL,
        "ts": time.time(),
    })
    print(
        f"[Init] Queue gimbal initial posture: "
        f"Az={GIMBAL_AZ_BASE:.2f}°, El={GIMBAL_INIT_EL:.2f}°"
    )
    threading.Thread(target=gimbal_control_thread, args=(gimbal,), daemon=True).start()
    threading.Thread(target=rk3588_thread, daemon=True).start()

    if TargetMeasurementRuntime is None:
        print(
            "[Ranging][Warn] target_measurement_runtime import failed; "
            "RID and visual distance disabled"
        )
    else:
        try:
            measurement_runtime = TargetMeasurementRuntime(
                config=build_target_measurement_config(),
                event_sink=measurement_runtime_event_sink,
            )
            measurement_runtime.start()
            print(
                "[Ranging] target_measurement_runtime started: "
                f"rid={1 if ENABLE_RID else 0}, "
                f"vision={1 if ENABLE_GIMBAL_VISION else 0}"
            )
        except Exception as e:
            measurement_runtime = None
            print(f"[Ranging][Warn] initialization failed: {e}")

    special_runtime_available = all((
        SpecialRidPredictor is not None,
        SpecialRidRegistry is not None,
        run_special_rid_ui_sender is not None,
        should_send_sort_through_ordinary_ui is not None,
        sort_observation_from_track is not None,
    ))
    if (
        SPECIAL_RID_IDENTITY_ENABLED
        and ENABLE_RID
        and measurement_runtime is not None
        and special_runtime_available
    ):
        special_rid_registry = SpecialRidRegistry(
            predictor_factory=lambda: SpecialRidPredictor(
                alpha=RID_UI_FILTER_ALPHA,
                beta=RID_UI_FILTER_BETA,
                max_speed_mps=RID_UI_MAX_SPEED_MPS,
                max_prediction_s=SPECIAL_RID_MAX_PREDICTION_SECONDS,
                fresh_s=SPECIAL_RID_FRESH_SECONDS,
            ),
            log_callback=field_log_special_rid_identity,
            camera_theta=DEVICE_THETA,
            sort_fresh_s=SPECIAL_RID_SORT_FRESH_SECONDS,
            sort_internal_s=TRACK_MAX_LOST_SECONDS,
            reacquire_delay_s=SPECIAL_RID_REACQUIRE_DELAY_SECONDS,
        )
        special_rid_stop_event = threading.Event()

        def special_rid_snapshot_provider():
            manager = getattr(measurement_runtime, "_rid_track_manager", None)
            if manager is None:
                return []
            return manager.snapshot(
                now_ts=time.time(),
                include_stale=True,
            )

        def special_station_snapshot_provider():
            return station_position.snapshot(time.time())

        special_rid_thread = threading.Thread(
            target=run_special_rid_ui_sender,
            kwargs={
                "registry": special_rid_registry,
                "rid_snapshot_provider": special_rid_snapshot_provider,
                "station_snapshot_provider": special_station_snapshot_provider,
                "sender": sender,
                "stop_event": special_rid_stop_event,
                "log_callback": field_log_special_rid_identity,
                "hz": SPECIAL_RID_UI_RATE_HZ,
            },
            name="special-rid-ui-sender",
            daemon=True,
        )
        special_rid_thread.start()
        print(
            "[SpecialRID] stable UI identity sender started: "
            f"hz={SPECIAL_RID_UI_RATE_HZ:.1f}, IDs=dynamic(1,2)"
        )
    elif SPECIAL_RID_IDENTITY_ENABLED and ENABLE_RID:
        print(
            "[SpecialRID][Warn] stable UI identity disabled because its "
            "runtime dependency is unavailable"
        )

    distance_mode = (
        "RID/visual runtime" if measurement_runtime is not None else "none"
    )
    print(
        f"[Init] UI/Strike distance source: {distance_mode} "
        f"(ttl={TRACK_DISTANCE_TTL:.1f}s)"
    )
    print("=== System V9.0 (Predictive Tracking & Scheduling) Running ===")

    # 初始化追踪大脑
    tracker = MultiTargetTracker(
        max_lost_frames=50,
        max_lost_seconds=TRACK_MAX_LOST_SECONDS,
        distance_threshold=1.2,
    )
    
    # 状态机与调度变量
    master_id = None
    master_epoch_ts = 0.0
    strike_track_id = None
    strike_target_id = None
    strike_challenger_id = None
    strike_challenger_since = 0.0
    PREDICT_DELAY = 0.3     # 系统与物理响应总延迟 (打提前量)
    CONFIRM_HITS = TRACK_CONFIRM_HITS  # internal track gate; UI/strike use stricter external gates
    MAX_DT = 0.25            # Clamp dt to avoid model divergence
    global_cmd_id = 0
    last_sent_ctrl_az = None
    last_sent_ctrl_el = None
    challenger_id = None
    challenger_since = 0.0
    angle_unsafe_frames = 0
    last_applied_vision_ts = {}
    last_logged_vision_frame_ts = 0.0
    strike_track_results_cache = {}
    strike_window = {
        "track_id": None,
        "distance": None,
        "source": "none",
        "valid_until": 0.0,
        "last_send_ts": 0.0,
        "last_consumed_settled_cmd_id": -1,
    }
    strike_coordinate_skip_track_id = None
    
    last_time = time.time()
    # 统计日志：接收坐标与UI发送ID
    recv_obj_total = 0
    recv_unique_boxes = set()  # {(x1,y1,x2,y2), ...}
    ui_send_total = 0
    ui_send_counter = Counter()  # {ui_id: send_count}
    reserved_special_ui_ids = (
        SPECIAL_RID_UI_IDS
        if special_rid_registry is not None
        else frozenset()
    )
    next_ui_id = next_unreserved_ui_id(1, reserved_special_ui_ids)
    track_to_ui_id = {}  # Allocate a stable UI ID when any valid track is first sent.
    latest_rid_bindings = {}
    last_applied_rid_measurement = {}
    last_selected_distance_source = {}
    last_distance_decision_key = {}
    rid_assoc_last_log_ts = 0.0
    rid_assoc_cycle = 0
    stats_last_print = last_time
    live_last_print = last_time
    live_packet_count = 0
    live_obj_count = 0
    live_last_packet_t = 0.0
    live_last_meas_count = 0
    live_last_track_count = 0
    live_last_valid_count = 0
    fusion_packet_buffer = []
    fusion_window_start_t = 0.0

    def get_or_assign_ui_id(track):
        nonlocal next_ui_id, track_to_ui_id
        internal_id = int(track.id)
        rid_binding = latest_rid_bindings.get(internal_id)
        if rid_binding is not None:
            return int(rid_binding["rid"]["ui_id"])
        if internal_id not in track_to_ui_id:
            assigned_ui_id = next_unreserved_ui_id(
                next_ui_id,
                reserved_special_ui_ids,
            )
            track_to_ui_id[internal_id] = assigned_ui_id
            next_ui_id = assigned_ui_id + 1
        return track_to_ui_id[internal_id]

    def maybe_print_live_status(now_t, meas_count=0, active_tracks=None, valid_tracks=None):
        nonlocal live_last_print, live_packet_count, live_obj_count
        nonlocal live_last_meas_count, live_last_track_count, live_last_valid_count
        if meas_count is not None:
            live_last_meas_count = meas_count
        if active_tracks is not None:
            live_last_track_count = len(active_tracks)
        if valid_tracks is not None:
            live_last_valid_count = len(valid_tracks)
        if (not PRINT_LIVE_STATUS) or ((now_t - live_last_print) < LIVE_STATUS_INTERVAL):
            return
        interval = max(now_t - live_last_print, 1e-6)
        udp_rate = live_packet_count / interval
        obj_rate = live_obj_count / interval
        last_udp_age = "" if live_last_packet_t <= 0 else f"{now_t - live_last_packet_t:.2f}s"
        print(
            f"[LIVE] udp={udp_rate:.1f}/s, objs={obj_rate:.1f}/s, "
            f"last_udp={last_udp_age}, meas={live_last_meas_count}, "
            f"tracks={live_last_track_count}, valid={live_last_valid_count}, master={master_id}"
        )
        live_last_print = now_t
        live_packet_count = 0
        live_obj_count = 0

    def summarize_window_field(pkgs, field):
        values = []
        for item in pkgs:
            value = item.get(field, "")
            if value in (None, ""):
                continue
            value = str(value)
            if value not in values:
                values.append(value)
        return ";".join(values)

    def packet_source_key(pkg):
        board = str(pkg.get("board", "Unknown"))
        try:
            cam = int(pkg.get("cam", 0))
        except (TypeError, ValueError):
            cam = str(pkg.get("cam", ""))
        return board, cam

    def keep_latest_packet_per_source(pkgs):
        """同一融合窗口内，每个物理摄像头只保留最新 UDP 包。

        这样融合窗口可以覆盖跨摄像头异步上报，同时避免同一摄像头
        连续两帧进入同一个 tracker.update()，造成重复观测/重复建轨。
        同一个最新包里的多个 objs 会全部保留，不影响同画面多目标。
        """
        latest_by_source = {}
        for pkg in pkgs:
            latest_by_source[packet_source_key(pkg)] = pkg
        latest_pkgs = sorted(
            latest_by_source.values(),
            key=lambda item: float(item.get("_recv_ts", 0.0) or 0.0),
        )
        return latest_pkgs, max(0, len(pkgs) - len(latest_pkgs))

    if strike_sender is not None:
        def log_strike_send(snapshot, packet, send_ts):
            if FIELD_LOGGER is not None:
                FIELD_LOGGER.write_strike_timing(snapshot, packet, send_ts)
            row = dict(snapshot["log_row"])
            row["timestamp"] = f"{send_ts:.6f}"
            row["reason"] = (
                f"settled_cmd_id={snapshot['settled_cmd_id']},"
                f"packet={packet.hex(' ')}"
            )
            field_log_event(row)

        def log_strike_send_error(snapshot, exc, send_ts):
            if PRINT_EVENT_LOGS:
                print(f"[Strike][Warn] send skipped: {exc}")
            row = dict(snapshot["error_log_row"])
            row["timestamp"] = f"{send_ts:.6f}"
            row["reason"] = str(exc)
            field_log_event(row)

        strike_send_worker = PeriodicStrikeSender(
            strike_sender,
            send_hz=STRIKE_SEND_HZ,
            hardware_state=shared_state,
            on_send=log_strike_send,
            on_error=log_strike_send_error,
        )
        strike_send_worker.start()

    while True:
        try:
            curr_time = time.time()
            # --- 1. 获取 UDP 数据：短时间窗内的分摄像头包合成一个逻辑帧 ---
            while packet_queue:
                pkg = packet_queue.popleft()
                if not fusion_packet_buffer:
                    fusion_window_start_t = float(pkg.get("_recv_ts", curr_time) or curr_time)
                fusion_packet_buffer.append(pkg)

                raw_objs = pkg.get("objs", [])
                live_packet_count += 1
                live_obj_count += len(raw_objs) if isinstance(raw_objs, list) else 0
                live_last_packet_t = curr_time

            if not fusion_packet_buffer:
                maybe_print_live_status(curr_time, meas_count=None)
                time.sleep(0.005) # 稍微让出 CPU
                continue

            if (curr_time - fusion_window_start_t) < MEAS_FUSION_WINDOW_SECONDS:
                maybe_print_live_status(curr_time, meas_count=None)
                time.sleep(0.005)
                continue

            window_pkgs = fusion_packet_buffer
            fusion_packet_buffer = []
            fusion_window_start_t = 0.0
            raw_window_packet_count = len(window_pkgs)
            window_pkgs, same_source_packet_drop_count = (
                keep_latest_packet_per_source(window_pkgs)
            )
            used_window_packet_count = len(window_pkgs)

            #计算两次逻辑观测帧的间隔时间
            dt = curr_time - last_time
            if dt <= 0:
                dt = 1.0 / 10.0
            #有可能两包数据间隔很久，为了防止追踪器模型发散，限制最大 dt
            if dt > MAX_DT:
                dt = MAX_DT
            last_time = curr_time

            frame_ref_pkg = window_pkgs[-1]
            board_str = frame_ref_pkg.get("board", "Unknown")
            try:
                cam_idx = int(frame_ref_pkg.get("cam", 0))
            except (TypeError, ValueError):
                cam_idx = 0
            sender_mode = summarize_window_field(window_pkgs, "mode")
            sender_seq = summarize_window_field(window_pkgs, "seq")

            # --- 2. 坐标解析为绝对角度 ---
            raw_measurements = []

            with shared_state.lock:
                shared_gimbal_az = shared_state.gimbal_az
                shared_gimbal_el = shared_state.gimbal_el
                active_gimbal_track_id = shared_state.active_track_id
                settled_cmd_id = shared_state.settled_cmd_id
                settled_track_id = shared_state.settled_track_id
                settled_ts = shared_state.settled_ts
                gimbal_is_settled = shared_state.is_settled
                gimbal_is_stationary = shared_state.is_stationary
                gimbal_stationary_ts = shared_state.stationary_ts

            for pkt in window_pkgs:
                pkt_board_str = pkt.get("board", "Unknown")
                try:
                    pkt_cam_idx = int(pkt.get("cam", 0))
                except (TypeError, ValueError):
                    pkt_cam_idx = 0
                pkt_mode = pkt.get("mode", "")
                pkt_seq = pkt.get("seq", "")
                parsed_objs = parse_udp_objects(pkt.get("objs", []))

                for obj_raw_idx, obj_item in enumerate(parsed_objs):
                    raw_rect = obj_item["box"]
                    mono_dist = obj_item["mono_dist"]
                    obj_board = obj_item.get("board") or pkt_board_str
                    obj_cam_raw = obj_item.get("cam")
                    try:
                        obj_cam_idx = int(obj_cam_raw if obj_cam_raw is not None else pkt_cam_idx)
                    except (TypeError, ValueError):
                        print(f"[Warning] 无效摄像头ID: board={obj_board}, cam={obj_cam_raw}")
                        continue
                    logic_id, cfg = get_camera_params(obj_board, obj_cam_idx)
                    if logic_id is None:
                        continue
                    recv_obj_total += 1
                    bbox_info, bbox_reject_reason = sanitize_bbox(
                        raw_rect,
                        image_w=UDP_DETECTION_W,
                        image_h=UDP_DETECTION_H,
                    )
                    if bbox_info is None:
                        try:
                            raw_log_values = [float(raw_rect[i]) for i in range(4)]
                        except (TypeError, ValueError, IndexError):
                            raw_log_values = ["", "", "", ""]
                        field_log_event({
                            "timestamp": f"{curr_time:.6f}",
                            "seq": pkt_seq,
                            "mode": pkt_mode,
                            "event": "BBOX_REJECT",
                            "meas_idx": obj_raw_idx,
                            "reason": bbox_reject_reason,
                            "raw_bbox_x1": "" if raw_log_values[0] == "" else f"{raw_log_values[0]:.3f}",
                            "raw_bbox_y1": "" if raw_log_values[1] == "" else f"{raw_log_values[1]:.3f}",
                            "raw_bbox_x2": "" if raw_log_values[2] == "" else f"{raw_log_values[2]:.3f}",
                            "raw_bbox_y2": "" if raw_log_values[3] == "" else f"{raw_log_values[3]:.3f}",
                        })
                        continue
                    rect = bbox_info["clipped"]
                    raw_rect = bbox_info["raw"]
                    recv_unique_boxes.add((
                        int(round(raw_rect[0])),
                        int(round(raw_rect[1])),
                        int(round(raw_rect[2])),
                        int(round(raw_rect[3])),
                    ))
                    if bbox_info["is_edge_bbox"]:
                        field_log_event({
                            "timestamp": f"{curr_time:.6f}",
                            "seq": pkt_seq,
                            "mode": pkt_mode,
                            "event": "BBOX_CLIPPED",
                            "meas_idx": obj_raw_idx,
                            "reason": "bbox_out_of_image_bounds",
                            "raw_bbox_x1": f"{raw_rect[0]:.3f}",
                            "raw_bbox_y1": f"{raw_rect[1]:.3f}",
                            "raw_bbox_x2": f"{raw_rect[2]:.3f}",
                            "raw_bbox_y2": f"{raw_rect[3]:.3f}",
                            "clipped_bbox_x1": f"{rect[0]:.3f}",
                            "clipped_bbox_y1": f"{rect[1]:.3f}",
                            "clipped_bbox_x2": f"{rect[2]:.3f}",
                            "clipped_bbox_y2": f"{rect[3]:.3f}",
                            "is_edge_bbox": 1,
                            "visible_ratio": f"{bbox_info['visible_ratio']:.6f}",
                        })

                    cx = (rect[0] + rect[2]) / 2.0
                    cy = (rect[1] + rect[3]) / 2.0

                    res = calculate_angles(
                        logic_id,
                        cx,
                        cy,
                        cfg,
                        image_w=UDP_DETECTION_W,
                        image_h=UDP_DETECTION_H,
                    )
                    if res:
                        ui_az, ui_el = res
                        d_az_to_target = angular_diff(ui_az, shared_gimbal_az)
                        d_el_to_target = ui_el - shared_gimbal_el
                        # turn_dir = get_turn_direction_label(d_az_to_target, d_el_to_target)
                        meas_idx = len(raw_measurements)
                        raw_measurements.append({
                            "az": ui_az,
                            "el": ui_el,
                            "mono_dist": mono_dist,
                            "board": obj_board,
                            "cam": obj_cam_idx,
                            "logic_id": logic_id,
                            "source_ts": float(pkt.get("_recv_ts", curr_time) or curr_time),
                            "raw_meas_idx": meas_idx,
                            "is_edge_bbox": bbox_info["is_edge_bbox"],
                            "visible_ratio": bbox_info["visible_ratio"],
                        })
                        if FIELD_LOGGER is not None:
                            FIELD_LOGGER.write_measurement({
                                "timestamp": f"{curr_time:.6f}",
                                "seq": pkt_seq,
                                "mode": pkt_mode,
                                "board": obj_board,
                                "cam": obj_cam_idx,
                                "logic_id": logic_id,
                                "meas_idx": meas_idx,
                                "raw_bbox_x1": f"{raw_rect[0]:.3f}",
                                "raw_bbox_y1": f"{raw_rect[1]:.3f}",
                                "raw_bbox_x2": f"{raw_rect[2]:.3f}",
                                "raw_bbox_y2": f"{raw_rect[3]:.3f}",
                                "raw_bbox_w": f"{bbox_info['raw_w']:.3f}",
                                "raw_bbox_h": f"{bbox_info['raw_h']:.3f}",
                                "clipped_bbox_x1": f"{rect[0]:.3f}",
                                "clipped_bbox_y1": f"{rect[1]:.3f}",
                                "clipped_bbox_x2": f"{rect[2]:.3f}",
                                "clipped_bbox_y2": f"{rect[3]:.3f}",
                                "clipped_bbox_w": f"{bbox_info['clipped_w']:.3f}",
                                "clipped_bbox_h": f"{bbox_info['clipped_h']:.3f}",
                                "bbox_x1": f"{rect[0]:.3f}",
                                "bbox_y1": f"{rect[1]:.3f}",
                                "bbox_x2": f"{rect[2]:.3f}",
                                "bbox_y2": f"{rect[3]:.3f}",
                                "bbox_cx": f"{cx:.3f}",
                                "bbox_cy": f"{cy:.3f}",
                                "bbox_w": f"{rect[2] - rect[0]:.3f}",
                                "bbox_h": f"{rect[3] - rect[1]:.3f}",
                                "is_edge_bbox": 1 if bbox_info["is_edge_bbox"] else 0,
                                "visible_ratio": f"{bbox_info['visible_ratio']:.6f}",
                                "mono_dist": "" if mono_dist is None else f"{mono_dist:.6f}",
                                "meas_az": f"{ui_az:.6f}",
                                "meas_el": f"{ui_el:.6f}",
                            })
                        if PRINT_PHASE_LOGS:
                            if mono_dist is not None:
                                print(
                                    f"\n[Phase 1: 视觉解析] 收到目标 cx={cx:.1f}, cy={cy:.1f}, mono={mono_dist:.1f}m"
                                    f" -> 解算绝对角度: Az={ui_az:.2f}°, El={ui_el:.2f}°"
                                )
                            else:
                                print(
                                    f"\n[Phase 1: 视觉解析] 收到目标 cx={cx:.1f}, cy={cy:.1f}"
                                    f" -> 解算绝对角度: Az={ui_az:.2f}°, El={ui_el:.2f}°"
                                )
                        # print(
                        #     f"[DirCheck] gimbal_ui=(Az={shared_gimbal_az:.2f}°, El={shared_gimbal_el:.2f}°) "
                        #     f"target_delta=(dAz={d_az_to_target:.2f}°, dEl={d_el_to_target:.2f}°) => turn={turn_dir}"
                        # )
            # --- 3. 喂给 Tracker 更新所有目标轨迹 ---
            current_measurements, fusion_groups = fuse_measurements_by_angle(
                raw_measurements,
                threshold_deg=MEAS_FUSION_THRESHOLD_DEG,
            )
            fusion_groups_text = format_fusion_groups(fusion_groups)
            debug_context = {
                "mode": sender_mode,
                "frame_id": sender_seq,
                "meas_count": len(current_measurements),
            }
            active_tracks = tracker.update(
                current_measurements,
                dt,
                now_t=curr_time,
                debug_context=debug_context,
            )

            # Control/strike freshness and UI display freshness are independent:
            # the UI may bridge a detector dropout after control has released it.
            valid_tracks = [
                t for t in active_tracks
                if t.confirmed
                and t.lost_seconds(curr_time) <= MAX_LOCK_LOST_SECONDS
            ]
            for t in valid_tracks:
                update_track_confirmation_flags(t)
            ui_tracks = select_ui_tracks_for_display(active_tracks, curr_time)
            # Cloud control requires both UI confirmation and the shorter
            # control-validity window; UI-only prediction bridging must not
            # drive the gimbal.
            gimbal_tracks = [t for t in ui_tracks if t in valid_tracks]
            strike_valid_tracks = [
                t for t in valid_tracks
                if getattr(t, "ui_confirmed", False)
                and getattr(t, "strike_confirmed", False)
            ]

            # --- 4. 状态机：调度决策 ---
            station_snapshot = station_position.snapshot(curr_time)
            if special_rid_registry is not None:
                try:
                    special_sort_observations = [
                        sort_observation_from_track(
                            track,
                            map_azimuth=relative_to_map_azimuth,
                        )
                        for track in active_tracks
                        if getattr(track, "ui_confirmed", False)
                    ]
                    special_rid_registry.observe_sorts(
                        special_sort_observations,
                        station_snapshot,
                        now_ts=curr_time,
                    )
                except Exception as e:
                    field_log_special_rid_identity({
                        "timestamp": f"{curr_time:.6f}",
                        "event": "UI_SKIP",
                        "send_result": 0,
                        "skip_reason": f"sort_observation_error:{e}",
                    })
            strike_valid_tracks = select_special_rid_strike_tracks(
                strike_valid_tracks,
                special_rid_registry,
            )
            master_track = next((t for t in gimbal_tracks if t.id == master_id), None)
            prev_master_id = master_id
            master_lost = (prev_master_id is not None and master_track is None)

            if master_lost:
                if PRINT_EVENT_LOGS:
                    print(
                        f"[TargetLost] master_id={prev_master_id} 不再满足锁定条件: "
                        f"gimbal_ids={[int(t.id) for t in gimbal_tracks]}"
                    )
                field_log_event({
                    "timestamp": f"{curr_time:.6f}",
                    "seq": sender_seq,
                    "mode": sender_mode,
                    "event": "TargetLost",
                    "track_id": int(prev_master_id),
                    "reason": "not_in_gimbal_tracks",
                })
                clear_strike_delivery(strike_window, strike_send_worker)
                master_id = None
                prev_master_id = None

            curr_gimbal_az = shared_gimbal_az
            curr_gimbal_el = shared_gimbal_el
            best_track, ranked_candidates = choose_master_track(
                gimbal_tracks,
                curr_gimbal_az,
                curr_gimbal_el,
                master_id=master_id,
            )
            selection_reason = None

            if master_track is None and best_track is not None:
                selection_reason = "master_lost" if master_lost else "initial_acquire"
                master_track = best_track
            elif master_track is not None and best_track is not None and best_track.id != master_track.id:
                eval_by_id = {
                    item["track_id"]: item for item in ranked_candidates
                }
                current_eval = eval_by_id.get(int(master_track.id))
                best_eval = eval_by_id.get(int(best_track.id))
                score_margin = (
                    best_eval["threat_score"] - current_eval["threat_score"]
                    if current_eval is not None and best_eval is not None
                    else -math.inf
                )
                if score_margin >= MASTER_SWITCH_SCORE_MARGIN:
                    if challenger_id != best_track.id:
                        challenger_id = best_track.id
                        challenger_since = curr_time
                    elif (curr_time - challenger_since) >= MASTER_SWITCH_CONFIRM_SECONDS:
                        selection_reason = "confirmed_higher_threat"
                        master_track = best_track
                else:
                    challenger_id = None
                    challenger_since = 0.0
            else:
                challenger_id = None
                challenger_since = 0.0

            if master_track is not None and (
                master_id is None or master_track.id != master_id
            ):
                old_master_id = master_id
                master_id = master_track.id
                master_epoch_ts = curr_time
                challenger_id = None
                challenger_since = 0.0
                angle_unsafe_frames = 0
                clear_strike_delivery(strike_window, strike_send_worker)
                candidates_text = format_selection_candidates(ranked_candidates)
                event_name = "TargetAcquire" if old_master_id is None else "TargetSwitch"
                if PRINT_EVENT_LOGS:
                    print(
                        f"[{event_name}] reason={selection_reason}, "
                        f"from={old_master_id}, to={master_id}, "
                        f"candidates={candidates_text}"
                    )
                field_log_event({
                    "timestamp": f"{curr_time:.6f}",
                    "seq": sender_seq,
                    "mode": sender_mode,
                    "event": event_name,
                    "track_id": int(master_id),
                    "reason": selection_reason,
                })
            track_ids = [int(t.id) for t in active_tracks]
            valid_ids = [int(t.id) for t in valid_tracks]
            ui_ids = [int(t.id) for t in ui_tracks]
            hit_values = [int(t.hit_streak) for t in active_tracks]
            lost_values = [int(t.time_since_update) for t in active_tracks]
            lost_seconds_values = [
                f"{t.lost_seconds(curr_time):.3f}" for t in active_tracks
            ]
            track_states = ";".join(
                f"{int(t.id)}:{t.state[0,0]:.4f},{t.state[1,0]:.4f},{t.state[2,0]:.4f},{t.state[3,0]:.4f}"
                for t in active_tracks
            )
            if FIELD_LOGGER is not None:
                FIELD_LOGGER.write_summary({
                    "timestamp": f"{curr_time:.6f}",
                    "seq": sender_seq,
                    "mode": sender_mode,
                    "dt": f"{dt:.6f}",
                    "window_packet_count": raw_window_packet_count,
                    "used_packet_count": used_window_packet_count,
                    "same_source_packet_drop_count": same_source_packet_drop_count,
                    "raw_meas_count": len(raw_measurements),
                    "fused_meas_count": len(current_measurements),
                    "fusion_groups": fusion_groups_text,
                    "meas_count": len(current_measurements),
                    "track_count": len(active_tracks),
                    "valid_count": len(valid_tracks),
                    "track_ids": ";".join(str(x) for x in track_ids),
                    "valid_ids": ";".join(str(x) for x in valid_ids),
                    "ui_ids": ";".join(str(x) for x in ui_ids),
                    "master_id": "" if master_id is None else int(master_id),
                    "hit_streaks": ";".join(str(x) for x in hit_values),
                    "time_since_updates": ";".join(str(x) for x in lost_values),
                    "lost_seconds": ";".join(lost_seconds_values),
                    "track_states": track_states,
                    "cmd_az": "" if last_sent_ctrl_az is None else f"{last_sent_ctrl_az:.6f}",
                    "cmd_el": "" if last_sent_ctrl_el is None else f"{last_sent_ctrl_el:.6f}",
                    "gimbal_ui_az": f"{shared_gimbal_az:.6f}",
                    "gimbal_ui_el": f"{shared_gimbal_el:.6f}",
                })
            if DEBUG_TRACKER:
                print(
                    f"[TRACK_SUMMARY] raw_meas={len(raw_measurements)}, "
                    f"fused_meas={len(current_measurements)}, "
                    f"pkts={used_window_packet_count}/{raw_window_packet_count}, "
                    f"same_src_drop={same_source_packet_drop_count}, "
                    f"tracks={len(active_tracks)}, "
                    f"valid={len(valid_tracks)}, "
                    f"ids={track_ids}, "
                    f"valid_ids={valid_ids}, "
                    f"master={master_id}, "
                    f"hits={hit_values}, "
                    f"lost={lost_values}, "
                    f"lost_s={lost_seconds_values}"
                )

            ranging_sort_tracks = []
            for track in ui_tracks:
                expected_delta_az = angular_diff(
                    track.state[0, 0], shared_gimbal_az
                )
                expected_delta_el = track.state[1, 0] - shared_gimbal_el
                source_board, source_cam = track_ui_source(
                    track, board_str, cam_idx
                )
                ranging_sort_tracks.append({
                    "track_id": int(track.id),
                    "track_created_ts": float(track.created_ts),
                    "ui_id": get_or_assign_ui_id(track),
                    "relative_az": float(track.state[0, 0]),
                    "map_az": relative_to_map_azimuth(track.state[0, 0]),
                    "elevation": float(track.state[1, 0]),
                    "source_board": source_board,
                    "source_cam": source_cam,
                    "logic_id": getattr(
                        track, "last_source_logic_id", ""
                    ),
                    "center": (
                        IMG_W / 2.0
                        + expected_delta_az / FOV_X * IMG_W,
                        IMG_H / 2.0
                        - expected_delta_el / FOV_Y * IMG_H,
                    ),
                    "lost_seconds": float(track.lost_seconds(curr_time)),
                })

            measurement_output = {
                "track_results": {},
                "rid_diagnostics": [],
                "vision_diagnostics": [],
                "events": [],
                "expired_rid_keys": [],
            }
            if measurement_runtime is not None:
                try:
                    measurement_output = measurement_runtime.update({
                        "timestamp": curr_time,
                        "sort_tracks": ranging_sort_tracks,
                        "master_track_id": master_id,
                        "station_position": station_snapshot,
                        "gimbal_state": {
                            "stationary": gimbal_is_stationary,
                            "stationary_ts": gimbal_stationary_ts,
                        },
                    })
                except Exception as e:
                    print(f"[Ranging][Warn] cycle failed: {e}")
                    field_log_event({
                        "timestamp": f"{curr_time:.6f}",
                        "event": "RANGING_RUNTIME_ERROR",
                        "reason": str(e),
                    })

            final_distance_by_track = {}
            current_vision_track_results = {}
            for track in ui_tracks:
                track_id = int(track.id)
                runtime_result = dict(
                    measurement_output.get("track_results", {}).get(
                        track_id, {}
                    )
                )
                runtime_result.setdefault("distance", float("nan"))
                runtime_result.setdefault("distance_valid", False)
                runtime_result.setdefault("distance_source", "none")
                runtime_result.setdefault("source_family", "none")
                runtime_result.setdefault("radial_velocity_mps", 0.0)
                runtime_result.setdefault("distance_uncertainty", math.inf)
                runtime_result.setdefault("replaced_target_id", 0)
                runtime_result.setdefault("notification_token", None)
                runtime_result.setdefault("ui_send_allowed", True)
                runtime_result["valid"] = bool(
                    runtime_result["distance_valid"]
                    and ui_distance_is_valid(runtime_result["distance"])
                )
                runtime_result["selected_source"] = (
                    f"{runtime_result['distance_source']}_smooth"
                    if runtime_result["valid"]
                    else "none"
                )
                final_distance_by_track[track_id] = runtime_result
                vision_candidate = runtime_result.get("vision_candidate")
                if vision_candidate:
                    current_vision_track_results[track_id] = vision_candidate

                if FIELD_LOGGER is not None:
                    FIELD_LOGGER.write_distance_arbitration({
                        "timestamp": f"{curr_time:.6f}",
                        "track_id": track_id,
                        "ui_id": get_or_assign_ui_id(track),
                        "master_id": (
                            "" if master_id is None else int(master_id)
                        ),
                        "is_master": 1 if track_id == master_id else 0,
                        "selected_family": runtime_result["source_family"],
                        "selected_source": runtime_result["selected_source"],
                        "selected_distance": (
                            f"{float(runtime_result['distance']):.6f}"
                            if runtime_result["valid"] else ""
                        ),
                        "selected_valid": 1 if runtime_result["valid"] else 0,
                        "measurement_ts": runtime_result.get(
                            "measurement_ts", 0.0
                        ),
                        "filter_applied": (
                            1 if runtime_result.get("filter_applied") else 0
                        ),
                        "source_switch": (
                            1 if runtime_result.get("source_switch") else 0
                        ),
                        "reason": "selected_by_target_measurement_runtime",
                    })

            if FIELD_LOGGER is not None:
                for diagnostic in measurement_output.get(
                    "vision_diagnostics", ()
                ):
                    FIELD_LOGGER.write_vision_association({
                        "timestamp": f"{curr_time:.6f}",
                        "event": "CANDIDATE",
                        "vision_frame_ts": diagnostic.get(
                            "vision_frame_ts", ""
                        ),
                        "association_policy": (
                            "hungarian_compensated_y_no_gate"
                        ),
                        "master_id": (
                            "" if master_id is None else int(master_id)
                        ),
                        "sort_track_id": diagnostic.get("track_id", ""),
                        "sort_source_board": diagnostic.get(
                            "sort_source_board", ""
                        ),
                        "sort_source_cam": diagnostic.get(
                            "sort_source_cam", ""
                        ),
                        "sort_source_logic_id": diagnostic.get(
                            "sort_source_logic_id", ""
                        ),
                        "sort_projected_x": (
                            diagnostic.get("sort_projected_center", ["", ""])[0]
                        ),
                        "sort_projected_y": (
                            diagnostic.get("sort_projected_center", ["", ""])[1]
                        ),
                        "sort_y_compensation_px": diagnostic.get(
                            "sort_y_compensation_px", ""
                        ),
                        "sort_compensated_y": (
                            diagnostic.get("sort_compensated_center", ["", ""])[1]
                        ),
                        "measurement_index": diagnostic.get(
                            "measurement_index", ""
                        ),
                        "detection_center_x": (
                            diagnostic.get("detection_center", ["", ""])[0]
                        ),
                        "detection_center_y": (
                            diagnostic.get("detection_center", ["", ""])[1]
                        ),
                        "association_dx_px": diagnostic.get(
                            "association_dx_px", ""
                        ),
                        "association_dy_px": diagnostic.get(
                            "association_dy_px", ""
                        ),
                        "association_error_px": diagnostic.get(
                            "association_error_px", ""
                        ),
                        "association_cost_y_px": diagnostic.get(
                            "association_cost_y_px", ""
                        ),
                        "selected": 1 if diagnostic.get("selected") else 0,
                    })
                for diagnostic in measurement_output.get(
                    "rid_diagnostics", ()
                ):
                    FIELD_LOGGER.write_rid_association({
                        "timestamp": f"{curr_time:.6f}",
                        "event": "RID_UI_FOUR_POINT_CANDIDATE",
                        "master_id": (
                            "" if master_id is None else int(master_id)
                        ),
                        **diagnostic,
                    })

            vision_summary = dict(
                measurement_output.get("vision_summary", {})
            )
            vision_result = {
                **vision_summary,
                "track_results": current_vision_track_results,
                "frame_ts": float(
                    vision_summary.get("frame_ts", 0.0) or 0.0
                ),
                "track_id": vision_summary.get("track_id"),
                "bbox": vision_summary.get("bbox"),
                "reposition_requested": bool(
                    vision_summary.get("reposition_requested", False)
                ),
            }

            if ENABLE_GIMBAL_VISION:
                current_track_results_for_strike = (
                    vision_result.get("track_results", {})
                    if isinstance(vision_result, dict)
                    else {}
                )
                track_results_for_strike = build_strike_track_results(
                    current_track_results_for_strike,
                    strike_track_results_cache,
                    curr_time,
                )
            else:
                # RID-only delivery must not depend on a vision bbox that is
                # intentionally unavailable when visual ranging is disabled.
                track_results_for_strike = final_distance_by_track
            current_strike_track = next(
                (t for t in strike_valid_tracks if t.id == strike_track_id),
                None,
            )
            proposed_strike_track, ranked_strike_candidates = choose_strike_target(
                strike_valid_tracks,
                curr_time,
                track_results_for_strike,
                strike_track_id=strike_track_id,
                require_vision_confirmation=ENABLE_GIMBAL_VISION,
            )
            strike_selection_reason = None
            if current_strike_track is None and proposed_strike_track is not None:
                strike_selection_reason = (
                    "strike_lost" if strike_track_id is not None else "initial_strike_acquire"
                )
                current_strike_track = proposed_strike_track
            elif (
                current_strike_track is not None
                and proposed_strike_track is not None
                and proposed_strike_track.id != current_strike_track.id
            ):
                strike_eval_by_id = {
                    item["track_id"]: item for item in ranked_strike_candidates
                }
                current_strike_eval = strike_eval_by_id.get(
                    int(current_strike_track.id)
                )
                proposed_strike_eval = strike_eval_by_id.get(
                    int(proposed_strike_track.id)
                )
                strike_score_margin = (
                    proposed_strike_eval["threat_score"]
                    - current_strike_eval["threat_score"]
                    if current_strike_eval is not None
                    and proposed_strike_eval is not None
                    else math.inf
                )
                if strike_score_margin >= STRIKE_TARGET_SWITCH_SCORE_MARGIN:
                    if strike_challenger_id != proposed_strike_track.id:
                        strike_challenger_id = proposed_strike_track.id
                        strike_challenger_since = curr_time
                    elif (
                        curr_time - strike_challenger_since
                    ) >= STRIKE_TARGET_SWITCH_CONFIRM_SECONDS:
                        strike_selection_reason = "confirmed_higher_strike_threat"
                        current_strike_track = proposed_strike_track
                else:
                    strike_challenger_id = None
                    strike_challenger_since = 0.0
            elif proposed_strike_track is None:
                current_strike_track = None
                strike_challenger_id = None
                strike_challenger_since = 0.0
            else:
                strike_challenger_id = None
                strike_challenger_since = 0.0

            if current_strike_track is None:
                if strike_track_id is not None:
                    field_log_event({
                        "timestamp": f"{curr_time:.6f}",
                        "seq": sender_seq,
                        "mode": sender_mode,
                        "event": "STRIKE_TARGET_LOST",
                        "track_id": int(strike_track_id),
                        "master_id": "" if master_id is None else int(master_id),
                        "reason": "no_fresh_safe_distance_candidate",
                    })
                strike_track_id = None
                clear_strike_delivery(strike_window, strike_send_worker)
            elif strike_track_id != current_strike_track.id:
                old_strike_track_id = strike_track_id
                strike_track_id = current_strike_track.id
                clear_strike_delivery(strike_window, strike_send_worker)
                strike_challenger_id = None
                strike_challenger_since = 0.0
                candidates_text = format_strike_candidates(ranked_strike_candidates)
                if PRINT_EVENT_LOGS:
                    print(
                        f"[StrikeTarget] reason={strike_selection_reason}, "
                        f"from={old_strike_track_id}, to={strike_track_id}, "
                        f"candidates={candidates_text}"
                    )
                field_log_event({
                    "timestamp": f"{curr_time:.6f}",
                    "seq": sender_seq,
                    "mode": sender_mode,
                    "event": "STRIKE_TARGET_SWITCH",
                    "track_id": int(strike_track_id),
                    "master_id": "" if master_id is None else int(master_id),
                    "reason": strike_selection_reason,
                })

            # --- 5. 状态机：物理执行与测距 (LOCKED) ---
            if master_track is not None:
                fut_az, fut_el = master_track.get_future_position(dt_delay=PREDICT_DELAY)
                ctrl_az, ctrl_el = ui_to_ctrl_angles(fut_az, fut_el)

                vision_frame_age = curr_time - float(
                    vision_result.get("frame_ts", 0.0)
                )
                vision_bbox_fresh = (
                    vision_result.get("track_id") == master_id
                    and vision_result.get("bbox") is not None
                    and 0.0 <= vision_frame_age <= GIMBAL_VISION_RESULT_TTL
                )
                reposition_reason = "none"
                if vision_bbox_fresh:
                    need_reposition = bool(
                        vision_result.get("reposition_requested", False)
                    )
                    angle_unsafe_frames = 0
                    if need_reposition:
                        reposition_reason = "vision_bbox_outside_safe_zone"
                else:
                    delta_az = abs(
                        angular_diff(
                            master_track.state[0, 0],
                            shared_gimbal_az,
                        )
                    )
                    delta_el = abs(
                        master_track.state[1, 0] - shared_gimbal_el
                    )
                    safe_half_az = FOV_X * GIMBAL_SAFE_FOV_RATIO_X / 2.0
                    safe_half_el = FOV_Y * GIMBAL_SAFE_FOV_RATIO_Y / 2.0
                    angle_safe = (
                        delta_az <= safe_half_az
                        and delta_el <= safe_half_el
                    )
                    angle_unsafe_frames = (
                        0 if angle_safe else angle_unsafe_frames + 1
                    )
                    need_reposition = angle_unsafe_frames >= 3
                    if need_reposition:
                        reposition_reason = "global_track_outside_safe_fov"

                # The target may move freely inside the central safe FOV. Only
                # reposition after three unsafe frames, and never continuously
                # preempt while the camera is physically moving. A target
                # switch may still replace the previous target command.
                can_issue_command = (
                    gimbal_is_stationary
                    or active_gimbal_track_id != master_id
                )
                need_send = need_reposition and can_issue_command
                if (
                    active_gimbal_track_id == master_id
                    and last_sent_ctrl_az is not None
                    and last_sent_ctrl_el is not None
                ):
                    d_az = abs(angular_diff(ctrl_az, last_sent_ctrl_az))
                    d_el = abs(ctrl_el - last_sent_ctrl_el)
                    if d_az < GIMBAL_CMD_DEADBAND_AZ and d_el < GIMBAL_CMD_DEADBAND_EL:
                        need_send = False

                if need_send:
                    global_cmd_id += 1
                    push_latest_gimbal_cmd({
                        "cmd_id": global_cmd_id,
                        "track_id": int(master_id) if master_id is not None else -1,
                        "az": ctrl_az,
                        "el": ctrl_el,
                        "ts": curr_time,
                    })
                    last_sent_ctrl_az = ctrl_az
                    last_sent_ctrl_el = ctrl_el
                    field_log_gimbal({
                        "timestamp": f"{curr_time:.6f}",
                        "event": "GIMBAL_CMD",
                        "cmd_id": int(global_cmd_id),
                        "track_id": "" if master_id is None else int(master_id),
                        "cmd_az": f"{ctrl_az:.6f}",
                        "cmd_el": f"{ctrl_el:.6f}",
                        "gimbal_ui_az": f"{shared_gimbal_az:.6f}",
                        "gimbal_ui_el": f"{shared_gimbal_el:.6f}",
                        "target_ctrl_az": f"{ctrl_az:.6f}",
                        "target_ctrl_el": f"{ctrl_el:.6f}",
                        "reason": reposition_reason,
                    })
                    angle_unsafe_frames = 0

                vision_distance_age = curr_time - float(
                    vision_result.get("frame_ts", 0.0)
                )
                strike_track = next(
                    (t for t in valid_tracks if t.id == strike_track_id),
                    None,
                )
                strike_track_result = (
                    track_results_for_strike.get(int(strike_track_id), {})
                    if strike_track_id is not None
                    else {}
                )
                strike_distance_result = (
                    final_distance_by_track.get(int(strike_track_id), {})
                    if strike_track_id is not None
                    else {}
                )
                strike_gate_result = (
                    strike_track_result
                    if ENABLE_GIMBAL_VISION
                    else strike_distance_result
                )
                strike_measurement_ready, _strike_result_age, _strike_ttl = (
                    strike_measurement_readiness(
                        strike_gate_result,
                        curr_time,
                        require_vision_confirmation=ENABLE_GIMBAL_VISION,
                    )
                )
                strike_base_ready = (
                    ENABLE_STRIKE_SEND
                    and gimbal_is_settled
                    and strike_track is not None
                    and strike_measurement_ready
                )
                strike_station_position = strike_station_coordinates(
                    station_position.snapshot(curr_time)
                )
                strike_delivery_ready = (
                    strike_base_ready and strike_station_position is not None
                )
                if strike_base_ready and strike_station_position is None:
                    current_skip_track_id = int(strike_track.id)
                    if strike_coordinate_skip_track_id != current_skip_track_id:
                        field_log_event({
                            "timestamp": f"{curr_time:.6f}",
                            "seq": sender_seq,
                            "mode": sender_mode,
                            "event": "STRIKE_SEND_SKIP",
                            "track_id": current_skip_track_id,
                            "master_id": "" if master_id is None else int(master_id),
                            "internal_track_id": current_skip_track_id,
                            "distance_source": str(
                                strike_distance_result.get(
                                    "distance_source", "none"
                                )
                            ),
                            "reason": "station_gps_coordinates_unavailable",
                        })
                    strike_coordinate_skip_track_id = current_skip_track_id
                else:
                    strike_coordinate_skip_track_id = None
                if strike_delivery_ready:
                    strike_dist = float(
                        strike_distance_result.get("distance", math.nan)
                    )
                    strike_dist_source = str(
                        strike_distance_result.get("distance_source", "none")
                    )
                    if (
                        strike_distance_result.get("distance_valid", False)
                        and ui_distance_is_valid(strike_dist)
                    ):
                        window_was_open = (
                            strike_window["track_id"] == strike_track_id
                            and curr_time < strike_window["valid_until"]
                        )
                        strike_window["track_id"] = strike_track_id
                        strike_window["distance"] = strike_dist
                        strike_window["source"] = strike_dist_source
                        strike_window["valid_until"] = curr_time + STRIKE_WINDOW_SECONDS
                        if PRINT_EVENT_LOGS and not window_was_open:
                            window_mode = (
                                "visual" if ENABLE_GIMBAL_VISION else "RID"
                            )
                            print(
                                f"[Strike] {window_mode} window open "
                                f"track_id={strike_track_id}, source={strike_dist_source}, "
                                f"dist={strike_dist:.2f}m, valid={STRIKE_WINDOW_SECONDS:.2f}s"
                            )
                        if not window_was_open:
                            field_log_event({
                                "timestamp": f"{curr_time:.6f}",
                                "seq": sender_seq,
                                "mode": sender_mode,
                                "event": "STRIKE_WINDOW_OPEN",
                                "track_id": int(strike_track_id),
                                "master_id": "" if master_id is None else int(master_id),
                                "is_master": 1 if strike_track_id == master_id else 0,
                                "distance": f"{strike_dist:.6f}",
                                "distance_source": strike_dist_source,
                                "reason": (
                                    "fresh_highest_threat_gimbal_yolo_distance"
                                    if ENABLE_GIMBAL_VISION
                                    else "fresh_highest_threat_rid_distance"
                                ),
                            })

                strike_window_valid = (
                    strike_delivery_ready
                    and strike_window["track_id"] == strike_track_id
                    and strike_track is not None
                    and strike_track.lost_seconds(curr_time)
                    <= MAX_LOCK_LOST_SECONDS
                    and curr_time < strike_window["valid_until"]
                )
                if strike_window_valid and strike_send_worker is not None:
                    try:
                        strike_ui_id = special_rid_ui_id_for_track(
                            strike_track,
                            special_rid_registry,
                        )
                        if strike_ui_id is None:
                            raise ValueError(
                                "strike target is not owned by special RID ID 1/2"
                            )

                        # 1. 角度预测：主打击模型采用 6D CA (常加速度外推)，同时生成 CV (常速度) 预测用于影子比对与日志记录
                        strike_rel_az_ca, strike_el_ca = strike_track.get_shadow_future_position_ca(STRIKE_LEAD_TIME)
                        strike_rel_az_cv, strike_el_cv = strike_track.get_future_position(STRIKE_LEAD_TIME)
                        strike_map_az_ca = relative_to_map_azimuth(strike_rel_az_ca)
                        strike_map_az_cv = relative_to_map_azimuth(strike_rel_az_cv)

                        strike_rel_az = strike_rel_az_ca
                        strike_el = strike_el_ca
                        strike_map_az = strike_map_az_ca

                        # 2. 距离处理：使用 Distance-KF 滤波器平滑去噪，但不作远期速度预测外推，若超过 3s 未更新则安全退回静态基准
                        # Distance uses only the current track's fresh
                        # final result from target_measurement_runtime.
                        strike_smooth_dist = float(
                            strike_distance_result.get("distance", math.nan)
                        )
                        dist_source = str(
                            strike_distance_result.get(
                                "distance_source", "none"
                            )
                        )
                        if not ui_distance_is_valid(strike_smooth_dist):
                            raise ValueError(
                                f"no fresh runtime distance, source={dist_source}"
                            )

                        # 3. 发布最新公开字段，由独立线程按10Hz发送。
                        strike_target_id = int(strike_ui_id)
                        strike_distance = float(strike_smooth_dist)
                        strike_azimuth = float(strike_map_az)
                        strike_elevation = float(strike_el)
                        strike_longitude, strike_latitude = strike_station_position

                        # 4. 发送线程在真实sendto成功后记录结构化日志。
                        strike_log_row = {
                            "seq": sender_seq,
                            "mode": sender_mode,
                            "event": "STRIKE_SEND",
                            "track_id": int(strike_track.id),
                            "pred_az": f"{strike_rel_az:.6f}",
                            "map_az": f"{strike_map_az:.6f}",
                            "pred_el": f"{strike_el:.6f}",
                            "pred_az_cv": f"{strike_rel_az_cv:.6f}",
                            "pred_el_cv": f"{strike_el_cv:.6f}",
                            "map_az_cv": f"{strike_map_az_cv:.6f}",
                            "pred_az_ca": f"{strike_rel_az_ca:.6f}",
                            "pred_el_ca": f"{strike_el_ca:.6f}",
                            "map_az_ca": f"{strike_map_az_ca:.6f}",
                            "master_id": "" if master_id is None else int(master_id),
                            "is_master": 1 if strike_track.id == master_id else 0,
                            "internal_track_id": int(strike_track.id),
                            "ui_id": int(strike_ui_id),
                            "distance": f"{strike_smooth_dist:.6f}",
                            "distance_source": dist_source,
                            "longitude": f"{strike_longitude:.8f}",
                            "latitude": f"{strike_latitude:.8f}",
                            "dist_uncertainty": f"{float(strike_distance_result.get('distance_uncertainty', math.nan)):.6f}",
                            "radial_velocity": f"{float(strike_distance_result.get('radial_velocity_mps', 0.0)):.6f}",
                        }
                        strike_error_log_row = {
                            "seq": sender_seq,
                            "mode": sender_mode,
                            "event": "STRIKE_SEND_SKIP",
                            "track_id": int(strike_track.id) if strike_track is not None else -1,
                            "master_id": "" if master_id is None else int(master_id),
                            "internal_track_id": int(strike_track.id) if strike_track is not None else -1,
                            "distance_source": strike_window["source"],
                        }
                        strike_send_worker.publish({
                            "internal_track_id": int(strike_track.id),
                            "target_id": strike_target_id,
                            "distance_m": strike_distance,
                            "azimuth_deg": strike_azimuth,
                            "elevation_deg": strike_elevation,
                            "longitude_deg": strike_longitude,
                            "latitude_deg": strike_latitude,
                            "valid_until": strike_window["valid_until"],
                            "log_row": strike_log_row,
                            "error_log_row": strike_error_log_row,
                        })
                    except Exception as e:
                        strike_send_worker.clear()
                        if PRINT_EVENT_LOGS:
                            print(f"[Strike][Warn] publish skipped: {e}")
                        field_log_event({
                            "timestamp": f"{curr_time:.6f}",
                            "seq": sender_seq,
                            "mode": sender_mode,
                            "event": "STRIKE_SEND_SKIP",
                            "track_id": int(strike_track.id) if strike_track is not None else -1,
                            "master_id": "" if master_id is None else int(master_id),
                            "internal_track_id": int(strike_track.id) if strike_track is not None else -1,
                            "distance_source": strike_window["source"],
                            "reason": str(e),
                        })
                elif strike_send_worker is not None:
                    strike_send_worker.clear()


                owned_special_generations = (
                    special_rid_registry.owned_sort_generations()
                    if special_rid_registry is not None
                    else frozenset()
                )
                ordinary_ui_tracks = select_ordinary_ui_tracks_for_output(
                    ui_tracks,
                    owned_special_generations,
                    special_rid_ui_exclusive=(
                        SPECIAL_RID_IDENTITY_ENABLED
                        and SPECIAL_RID_UI_EXCLUSIVE
                    ),
                )
                for t in ordinary_ui_tracks:
                    final_distance_state = final_distance_by_track.get(
                        int(t.id), {}
                    )
                    ui_target_id = get_or_assign_ui_id(t)
                    ui_azimuth = relative_to_map_azimuth(t.state[0, 0])
                    ui_elevation = float(t.state[1, 0])
                    ui_distance = float(
                        final_distance_state.get("distance", math.nan)
                    )
                    ui_threat_score = ui_threat_score_from_distance(
                        ui_distance
                    )
                    ui_replaced_target_id = int(
                        final_distance_state.get("replaced_target_id", 0)
                        or 0
                    )
                    ui_notification_token = final_distance_state.get(
                        "notification_token"
                    )
                    ui_send_allowed = bool(
                        final_distance_state.get("ui_send_allowed", True)
                    )
                    source_board, source_cam = track_ui_source(
                        t, board_str, cam_idx
                    )
                    send_decision = ui_status_send_decision(
                        ui_distance,
                        ui_send_allowed=ui_send_allowed,
                        replaced_target_id=ui_replaced_target_id,
                    )
                    if not send_decision["send"]:
                        field_log_event({
                            "timestamp": f"{curr_time:.6f}",
                            "seq": sender_seq,
                            "mode": sender_mode,
                            "event": "UI_STATUS_SKIP_NO_DISTANCE",
                            "track_id": int(t.id),
                            "ui_id": int(ui_target_id),
                            "distance_source": final_distance_state.get(
                                "selected_source", "none"
                            ),
                            "reason": send_decision["reason"],
                        })
                        continue
                    ui_replaced_target_id = int(
                        send_decision["replaced_target_id"]
                    )
                    ui_sent = sender.send_status(
                        board_str=source_board,
                        camera_id=source_cam,
                        target_id=ui_target_id,
                        azimuth=ui_azimuth,
                        elevation=ui_elevation,
                        distance=ui_distance,
                        threat_score=ui_threat_score,
                        replaced_target_id=ui_replaced_target_id,
                    )
                    if ui_sent and ui_notification_token and measurement_runtime is not None:
                        measurement_runtime.ack_ui_send(
                            ui_notification_token, success=True
                        )
                    if not ui_sent:
                        continue
                    ui_send_total += 1
                    ui_send_counter[int(ui_target_id)] += 1
                    field_log_event({
                        "timestamp": f"{curr_time:.6f}",
                        "seq": sender_seq,
                        "mode": sender_mode,
                        "event": "UI_STATUS_SEND",
                        "track_id": int(t.id),
                        "internal_track_id": int(t.id),
                        "ui_id": int(ui_target_id),
                        "master_id": "" if master_id is None else int(master_id),
                        "is_master": 1 if t.id == master_id else 0,
                        "distance": f"{ui_distance:.6f}",
                        "distance_source": final_distance_state.get(
                            "selected_source", "none"
                        ),
                        "threat_score": f"{ui_threat_score:.6f}",
                        "reason": (
                            f"source={source_board}/{source_cam},"
                            f"replaced_target_id={ui_replaced_target_id}"
                        ),
                    })
            else:
                clear_strike_delivery(strike_window, strike_send_worker)

            maybe_print_live_status(
                curr_time,
                meas_count=len(current_measurements),
                active_tracks=active_tracks,
                valid_tracks=valid_tracks,
            )

            if PRINT_STATS and (curr_time - stats_last_print) >= STATS_PRINT_INTERVAL:
                top_id_text = "none"
                if ui_send_counter:
                    top_id_text = ", ".join([f"{tid}:{cnt}" for tid, cnt in ui_send_counter.most_common(8)])
                print(
                    f"[Stats] recv_objs_total={recv_obj_total}, recv_unique_boxes={len(recv_unique_boxes)} | "
                    f"ui_send_total={ui_send_total}, ui_unique_ids={len(ui_send_counter)}, ui_id_counts={top_id_text}"
                )
                stats_last_print = curr_time

        except KeyboardInterrupt:
            break
        except Exception as e:
            print(f"Err: {e}")
            import traceback
            traceback.print_exc()

    if strike_send_worker is not None:
        strike_send_worker.stop()
    if laser_stop_event is not None:
        laser_stop_event.set()
        time.sleep(0.05)
    if laser is not None:
        laser.close()
    if special_rid_stop_event is not None:
        special_rid_stop_event.set()
    if special_rid_thread is not None:
        special_rid_thread.join(timeout=1.0)
    if measurement_runtime is not None:
        measurement_runtime.stop()
    if 'gimbal' in locals():
        gimbal.close()
    if FIELD_LOGGER is not None:
        logger = FIELD_LOGGER
        FIELD_LOGGER = None
        logger.close()
    final_id_text = "none"
    if ui_send_counter:
        final_id_text = ", ".join([f"{tid}:{cnt}" for tid, cnt in ui_send_counter.most_common()])
    print(
        f"[Stats][Final] recv_objs_total={recv_obj_total}, recv_unique_boxes={len(recv_unique_boxes)} | "
        f"ui_send_total={ui_send_total}, ui_unique_ids={len(ui_send_counter)}, ui_id_counts={final_id_text}"
    )

if __name__ == "__main__":
    main()
