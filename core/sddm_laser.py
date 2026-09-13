import struct

import serial


class SDDMLaser:
    """SDDM laser rangefinder UART driver."""

    START_BYTE = 0xFA
    DATA_BYTE = 0xFB
    CMD_MEASURE = 0x01
    MSG_DISTANCE = 0x03
    BROADCAST_ID = 0xFF
    PAYLOAD_LEN = 0x04

    def __init__(self, port, baudrate=115200, timeout=0.1):
        self.ser = None
        try:
            self.ser = serial.Serial(port, baudrate, timeout=timeout)
        except Exception as e:
            raise RuntimeError(f"无法打开激光串口 {port}: {e}")

    def close(self):
        if self.ser and self.ser.is_open:
            try:
                self.stop_measurement()
            except Exception:
                pass
            self.ser.close()

    def _calculate_crc(self, data: bytes) -> int:
        return sum(data) & 0xFF

    def _send_command(self, mea_type: int, mea_times: int):
        if not self.ser or not self.ser.is_open:
            raise RuntimeError("激光串口未打开")

        header = bytearray([
            self.START_BYTE,
            self.CMD_MEASURE,
            self.BROADCAST_ID,
            self.PAYLOAD_LEN,
        ])
        payload = struct.pack("<HH", int(mea_type), int(mea_times))
        frame_no_crc = header + payload
        crc = self._calculate_crc(frame_no_crc)
        self.ser.write(frame_no_crc + bytes([crc]))
        self.ser.flush()

    def start_measurement(self, continuous=True):
        # 手册: MeaType=1 为开始, MeaTimes=0 连续测量, 1 单次测量
        mea_times = 0x0000 if continuous else 0x0001
        self._send_command(mea_type=0x0001, mea_times=mea_times)
        mode = "continuous" if continuous else "single"
        print(f"[Laser] start_measurement mode={mode}")

    def stop_measurement(self):
        self._send_command(mea_type=0x0000, mea_times=0x0000)
        print("[Laser] stop_measurement")

    def read_distance(self, debug=False):
        """
        Return distance in meters.
        Return None when no valid frame is available.
        """
        if not self.ser or not self.ser.is_open:
            return None

        try:
            for _ in range(50):
                first = self.ser.read(1)
                if not first:
                    return None
                if first[0] != self.DATA_BYTE:
                    continue

                rest = self.ser.read(8)
                if len(rest) < 8:
                    return None

                frame = first + rest
                if debug:
                    print(f"[Laser RAW] {frame.hex()}")

                if frame[1] != self.MSG_DISTANCE or frame[3] != self.PAYLOAD_LEN:
                    continue
                if self._calculate_crc(frame[:-1]) != frame[-1]:
                    if debug:
                        print("[Laser] CRC mismatch")
                    continue

                valid_flag, distance_dm = struct.unpack("<HH", frame[4:8])
                if valid_flag != 1:
                    if debug:
                        print(f"[Laser] invalid flag={valid_flag}")
                    return None

                return distance_dm / 10.0

        except Exception as e:
            print(f"[Laser] Error: {e}")
            return None

        return None
