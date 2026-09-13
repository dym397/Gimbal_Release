import serial
import time
import struct
import os


def _env_flag(name, default=False):
    value = os.getenv(name)
    if value is None:
        return default
    return value.strip().lower() in {"1", "true", "yes", "on"}


class GT06ZGimbal:
    def __init__(self, port: str):
        self.port = port
        self.ser = None
        self.baudrate = 9600
        self.address = 0x00

        self.last_sent_el = None
        self.last_sent_az = None
        self.last_feedback_el = None
        self.last_feedback_az = None

        self.epsilon = 0.2
        self.min_interval = 0.1
        self.last_cmd_time = 0.0
        self.debug_serial = _env_flag("GT06Z_DEBUG_SERIAL", False)

    def open(self) -> bool:
        try:
            self.ser = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                bytesize=8,
                parity="N",
                stopbits=1,
                timeout=0.1,
            )

            if self.ser.is_open:
                self.ser.reset_input_buffer()
                print(f"[GT06Z] Driver Ready on {self.port}.")
            return self.ser.is_open
        except Exception as e:
            print(f"[GT06Z] Open fail: {e}")
            return False

    def close(self):
        if self.ser and self.ser.is_open:
            self.stop()
            self.ser.close()
            print("[GT06Z] Connection Closed.")

    def is_connected(self) -> bool:
        return self.ser is not None and self.ser.is_open

    def _calc_checksum(self, payload_without_head: bytes) -> int:
        return sum(payload_without_head) & 0xFF

    def _send_frame(self, cmd1, cmd2, data_val):
        """
        Standard 7-byte frame:
        FF [Addr] [Cmd1] [Cmd2] [DataH] [DataL] [Check]
        """
        if not self.is_connected():
            return

        data_val = int(data_val) & 0xFFFF
        data_h = (data_val >> 8) & 0xFF
        data_l = data_val & 0xFF

        payload = bytes([self.address, cmd1, cmd2, data_h, data_l])
        checksum = self._calc_checksum(payload)
        frame = bytes([0xFF]) + payload + bytes([checksum])
        if self.debug_serial:
            print(f"[GT06Z][TX] cmd2=0x{cmd2:02X}, data=0x{data_val:04X}, frame={frame.hex(' ').upper()}", flush=True)
        self.ser.write(frame)
        self.ser.flush()

    def stop(self):
        self._send_frame(0x00, 0x00, 0x0000)

    def _pulse_direction(self, cmd2: int, data_val: int, move_time_s: float = 0.02, stop_gap_s: float = 0.01):
        self._send_frame(0x00, cmd2, data_val)
        time.sleep(move_time_s)
        self.stop()
        time.sleep(stop_gap_s)

    def set_speed_single_axis(self, speed: int = 0x3F):
        """
        Configure speed using only the 4 single-axis direction commands.
        """
        if not self.is_connected():
            return

        val = max(1, min(63, int(speed)))
        print(f"[GT06Z] Configuring single-axis speed to: {val} (Range: 1-63)")

        data_pan_move = (val << 8) & 0xFF00
        data_tilt_move = val & 0xFF

        self._pulse_direction(0x02, data_pan_move)
        self._pulse_direction(0x04, data_pan_move)
        self._pulse_direction(0x08, data_tilt_move)
        self._pulse_direction(0x10, data_tilt_move)
        time.sleep(0.05)
        print("[GT06Z] Single-axis speed configuration done.")

    def set_speed_with_diagonal(self, speed: int = 0x3F):
        """
        Configure speed using both single-axis and diagonal direction commands.
        Useful if the firmware stores separate motion parameters for dual-axis moves.
        """
        if not self.is_connected():
            return

        val = max(1, min(63, int(speed)))
        print(f"[GT06Z] Configuring speed with diagonal commands to: {val} (Range: 1-63)")

        data_pan_move = (val << 8) & 0xFF00
        data_tilt_move = val & 0xFF
        data_diag_move = ((val & 0xFF) << 8) | (val & 0xFF)

        self._pulse_direction(0x02, data_pan_move)
        self._pulse_direction(0x04, data_pan_move)
        self._pulse_direction(0x08, data_tilt_move)
        self._pulse_direction(0x10, data_tilt_move)
        self._pulse_direction(0x0A, data_diag_move)
        self._pulse_direction(0x0C, data_diag_move)
        self._pulse_direction(0x12, data_diag_move)
        self._pulse_direction(0x14, data_diag_move)
        time.sleep(0.05)
        print("[GT06Z] Diagonal speed configuration done.")

    def set_speed(self, speed: int = 0x3F):
        """
        Backward-compatible speed configuration entry.
        Default behavior keeps the original single-axis-only strategy.
        """
        self.set_speed_single_axis(speed)

    def set_angles(self, elevation_deg=None, azimuth_deg=None, force: bool = False):
        """
        Absolute angle control using Item 8/9 from the protocol sheet.

        Either axis may be ``None`` so the control thread can retry only the
        axis that has not reached its target.  The return value is diagnostic;
        callers that do not need it may continue to ignore it.
        """
        status = {
            "el_requested": elevation_deg is not None,
            "az_requested": azimuth_deg is not None,
            "el_sent": False,
            "az_sent": False,
            "el_ack": False,
            "az_ack": False,
            "reason": "",
        }
        if not self.is_connected():
            status["reason"] = "not_connected"
            return status

        if elevation_deg is None and azimuth_deg is None:
            status["reason"] = "no_axis_requested"
            return status

        now = time.time()
        if (now - self.last_cmd_time) < self.min_interval:
            status["reason"] = "rate_limited"
            return status

        need_send_el = False
        if elevation_deg is not None and (
            force
            or self.last_sent_el is None
            or abs(elevation_deg - self.last_sent_el) > self.epsilon
        ):
            need_send_el = True

        need_send_az = False
        if azimuth_deg is not None:
            if force or self.last_sent_az is None:
                need_send_az = True
            else:
                diff = abs(azimuth_deg - self.last_sent_az)
                if diff > 180:
                    diff = 360 - diff
                if diff > self.epsilon:
                    need_send_az = True

        if not need_send_el and not need_send_az:
            status["reason"] = "deadband"
            return status

        self.ser.reset_input_buffer()

        if need_send_el:
            if elevation_deg > 0:
                el_cmd_val = 3600 - int(elevation_deg * 10)
            else:
                el_cmd_val = int(abs(elevation_deg) * 10)

            el_cmd_val = max(0, min(3600, el_cmd_val))
            self._send_frame(0x00, 0x4D, el_cmd_val)
            status["el_sent"] = True
            el_ack = self._read_specific_response(expected_cmd_byte=0x5E, retry=3)
            if el_ack is not None:
                self.last_sent_el = elevation_deg
                status["el_ack"] = True

            if need_send_az:
                time.sleep(0.04)

        if need_send_az:
            norm_az = azimuth_deg % 360.0
            az_cmd_val = int(norm_az * 10)
            az_cmd_val = max(0, min(3600, az_cmd_val))
            self._send_frame(0x00, 0x4B, az_cmd_val)
            status["az_sent"] = True
            az_ack = self._read_specific_response(expected_cmd_byte=0x5D, retry=3)
            if az_ack is not None:
                self.last_sent_az = azimuth_deg
                status["az_ack"] = True

        self.last_cmd_time = time.time()
        return status

    def _read_specific_response(self, expected_cmd_byte, retry=2):
        if not self.is_connected():
            return None

        for _ in range(retry):
            if self.ser.in_waiting < 7:
                time.sleep(0.02)

            data = self.ser.read(self.ser.in_waiting or 14)
            if self.debug_serial:
                print(f"[GT06Z][RX] expected=0x{expected_cmd_byte:02X}, len={len(data)}, data={data.hex(' ').upper() if data else '<empty>'}", flush=True)
            if len(data) < 7:
                continue

            for i in range(len(data) - 6):
                if data[i] != 0xFF:
                    continue
                frame = data[i : i + 7]

                if (sum(frame[1:6]) & 0xFF) != frame[6]:
                    continue

                if frame[3] == expected_cmd_byte:
                    return (frame[4] << 8) | frame[5]

        return None

    def query_angles(self):
        if not self.is_connected():
            return None

        self.ser.reset_input_buffer()

        self._send_frame(0x00, 0x53, 0)
        raw_el = self._read_specific_response(expected_cmd_byte=0x5B)
        if raw_el is not None:
            if raw_el <= 1800:
                self.last_feedback_el = -(raw_el / 10.0)
            else:
                self.last_feedback_el = (3600 - raw_el) / 10.0

        self._send_frame(0x00, 0x51, 0)
        raw_az = self._read_specific_response(expected_cmd_byte=0x59)
        if raw_az is not None:
            self.last_feedback_az = raw_az / 10.0

        if self.last_feedback_el is None or self.last_feedback_az is None:
            return None

        return (self.last_feedback_el, self.last_feedback_az)


# if __name__ == "__main__":
#     PORT_NAME = "COM3"  # Linux 示例可改为 "/dev/ttyUSB0"
#     gimbal = GT06ZGimbal(PORT_NAME)

#     if gimbal.open():
#         try:
#             print("\n=== Speed Setup Start ===")
#             print("Setting gimbal speed to maximum (63) using single-axis commands...")
#             gimbal.set_speed_single_axis(63)
#             print("Speed setup finished.")
#             print("\n=== Speed Setup Done ===")
#         except KeyboardInterrupt:
#             print("\nSpeed setup interrupted by user.")
#         finally:
#             gimbal.close()
#     else:
#         print("Error: Could not open serial port.")
