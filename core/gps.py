import serial
import serial.tools.list_ports
import math
import os
import time
import pynmea2

DEFAULT_LONGITUDE = 104.091130
DEFAULT_LATITUDE = 30.405553
GPS_FIX_TIMEOUT_SECONDS = 60


def default_gps_port():
    env_port = os.getenv("GPS_PORT")
    if env_port:
        return env_port.strip()
    if os.name == "nt":
        return "COM8"
    return "/dev/serial/by-path/platform-xhci-hcd.4.auto-usb-0:1.2:1.0-port0"


def _normalize_windows_com_port(port_name):
    normalized = port_name.strip().upper()
    if normalized.startswith("\\\\.\\"):
        normalized = normalized[4:]
    return normalized


def _finite_float(value):
    try:
        result = float(value)
    except (TypeError, ValueError):
        return None
    return result if math.isfinite(result) else None


def gga_ellipsoid_height(msg):
    """Return WGS-84 ellipsoid height reconstructed from one NMEA GGA fix.

    GGA ``altitude`` is orthometric/MSL height and ``geo_sep`` is geoid
    separation. WGS-84 ellipsoid height is h = H + N.
    """
    altitude_msl = _finite_float(getattr(msg, "altitude", None))
    geoid_separation = _finite_float(getattr(msg, "geo_sep", None))
    if altitude_msl is None or geoid_separation is None:
        return None
    return altitude_msl + geoid_separation


def gga_msl_height(msg):
    """Return orthometric/mean-sea-level height from one NMEA GGA fix."""
    return _finite_float(getattr(msg, "altitude", None))


def wgs84_to_gcj02(lng, lat):
    """
    WGS84转GCJ02(火星坐标系)
    :param lng: WGS84坐标系的经度
    :param lat: WGS84坐标系的纬度
    :return: 转换后的(经度, 纬度)
    """

    # 判断是否在国内，不在国内不做偏移
    def out_of_china(lng, lat):
        return not (72.004 <= lng <= 137.8347 and 0.8293 <= lat <= 55.8271)

    if out_of_china(lng, lat):
        return lng, lat

    a = 6378245.0
    ee = 0.00669342162296594323
    pi = 3.1415926535897932384626

    def _transformlat(lng, lat):
        ret = -100.0 + 2.0 * lng + 3.0 * lat + 0.2 * lat * lat + 0.1 * lng * lat + 0.2 * math.sqrt(abs(lng))
        ret += (20.0 * math.sin(6.0 * lng * pi) + 20.0 * math.sin(2.0 * lng * pi)) * 2.0 / 3.0
        ret += (20.0 * math.sin(lat * pi) + 40.0 * math.sin(lat / 3.0 * pi)) * 2.0 / 3.0
        ret += (160.0 * math.sin(lat / 12.0 * pi) + 320 * math.sin(lat * pi / 30.0)) * 2.0 / 3.0
        return ret

    def _transformlng(lng, lat):
        ret = 300.0 + lng + 2.0 * lat + 0.1 * lng * lng + 0.1 * lng * lat + 0.1 * math.sqrt(abs(lng))
        ret += (20.0 * math.sin(6.0 * lng * pi) + 20.0 * math.sin(2.0 * lng * pi)) * 2.0 / 3.0
        ret += (20.0 * math.sin(lng * pi) + 40.0 * math.sin(lng / 3.0 * pi)) * 2.0 / 3.0
        ret += (150.0 * math.sin(lng / 12.0 * pi) + 300.0 * math.sin(lng / 30.0 * pi)) * 2.0 / 3.0
        return ret

    dlat = _transformlat(lng - 105.0, lat - 35.0)
    dlng = _transformlng(lng - 105.0, lat - 35.0)
    radlat = lat / 180.0 * pi
    magic = math.sin(radlat)
    magic = 1 - ee * magic * magic
    sqrtmagic = math.sqrt(magic)
    dlat = (dlat * 180.0) / ((a * (1 - ee)) / (magic * sqrtmagic) * pi)
    dlng = (dlng * 180.0) / (a / sqrtmagic * math.cos(radlat) * pi)

    return lng + dlng, lat + dlat


def read_gps_fix(
    port=None,
    baudrate=115200,
    timeout_seconds=GPS_FIX_TIMEOUT_SECONDS,
    print_raw=True,
    print_status=True,
    status_interval=5.0,
    coordinate_system="gcj02",
    include_altitude=False,
    altitude_reference="ellipsoid",
):
    altitude_reference = str(altitude_reference).strip().lower()
    if altitude_reference not in {"ellipsoid", "msl"}:
        raise ValueError(
            f"unsupported altitude_reference={altitude_reference!r}; "
            "expected 'ellipsoid' or 'msl'"
        )
    if port is None:
        port = default_gps_port()

    ports = [item.device for item in serial.tools.list_ports.comports()]
    if os.name == "nt":
        available_ports = {_normalize_windows_com_port(item) for item in ports}
        if _normalize_windows_com_port(port) not in available_ports:
            result = (None, None, None, "fallback")
            return result if include_altitude else (result[0], result[1], result[3])

    if not port:
        result = (None, None, None, "fallback")
        return result if include_altitude else (result[0], result[1], result[3])

    longitude = None
    latitude = None
    altitude = None

    try:
        with serial.Serial(port, baudrate, timeout=1) as ser:
            start_t = time.monotonic()
            deadline = time.monotonic() + timeout_seconds
            last_status_t = 0.0
            while time.monotonic() < deadline:
                line = ser.readline().decode("ascii", errors="ignore").strip()
                if print_raw:
                    print(f"[GPS] raw line: {line}", flush=True)
                if not line:
                    now_t = time.monotonic()
                    if print_status and (now_t - last_status_t) >= status_interval:
                        elapsed = now_t - start_t
                        print(
                            f"[GPS] Waiting for NMEA data on {port}, "
                            f"elapsed={elapsed:.1f}s/{timeout_seconds:.1f}s",
                            flush=True,
                        )
                        last_status_t = now_t
                    continue

                if line.startswith("$GPGGA") or line.startswith("$GNGGA"):
                    try:
                        msg = pynmea2.parse(line)
                    except pynmea2.ParseError:
                        continue

                    if print_status:
                        current_ellipsoid_height = gga_ellipsoid_height(msg)
                        print(
                            f"[GPS] GGA fix_quality={getattr(msg, 'gps_qual', '')}, "
                            f"num_sats={getattr(msg, 'num_sats', '')}, "
                            f"hdop={getattr(msg, 'horizontal_dil', '')}, "
                            f"lat={msg.latitude}, lon={msg.longitude}, "
                            f"alt_msl={getattr(msg, 'altitude', '')}, "
                            f"geoid_sep={getattr(msg, 'geo_sep', '')}, "
                            f"ellipsoid_h={current_ellipsoid_height}",
                            flush=True,
                        )

                    if str(getattr(msg, "gps_qual", "0")) == "0":
                        continue
                    if msg.longitude == 0 or msg.latitude == 0:
                        continue

                    longitude = float(msg.longitude)
                    latitude = float(msg.latitude)
                    altitude = (
                        gga_msl_height(msg)
                        if altitude_reference == "msl"
                        else gga_ellipsoid_height(msg)
                    )
                    break
    except serial.SerialException:
        result = (None, None, None, "serial_error")
        return result if include_altitude else (result[0], result[1], result[3])

    if longitude is None or latitude is None:
        result = (None, None, None, "no_fix")
        return result if include_altitude else (result[0], result[1], result[3])

    coordinate_system = str(coordinate_system).strip().lower()
    if coordinate_system == "wgs84":
        result = (longitude, latitude, altitude, port)
        return result if include_altitude else (result[0], result[1], result[3])
    if coordinate_system != "gcj02":
        raise ValueError(
            f"unsupported coordinate_system={coordinate_system!r}; "
            "expected 'wgs84' or 'gcj02'"
        )

    # Preserve the legacy UI result: the previous implementation rounded the
    # WGS-84 fix to four decimal places before converting it to GCJ-02. RID
    # geodesy requests WGS-84 explicitly above and therefore keeps full NMEA
    # precision.
    longitude, latitude = wgs84_to_gcj02(
        round(longitude, 4), round(latitude, 4)
    )
    result = (longitude, latitude, altitude, port)
    return result if include_altitude else (result[0], result[1], result[3])


def read_gps(port=None, baudrate=115200, timeout_seconds=GPS_FIX_TIMEOUT_SECONDS, print_raw=True):
    longitude, latitude, gps_source = read_gps_fix(
        port=port,
        baudrate=baudrate,
        timeout_seconds=timeout_seconds,
        print_raw=print_raw,
        print_status=print_raw,
    )
    if longitude is None or latitude is None:
        return DEFAULT_LONGITUDE, DEFAULT_LATITUDE, gps_source
    return longitude, latitude, gps_source


if __name__ == "__main__":
    longitude, latitude, gps_source = read_gps()
    print(f"source={gps_source} longitude={longitude:.6f} latitude={latitude:.6f}")
