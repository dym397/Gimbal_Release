import serial
import threading
import struct
import os


def _default_imu_port():
    if os.name == "nt":
        return "COM5"
    return "/dev/serial/by-path/platform-xhci-hcd.4.auto-usb-0:1.4.2:1.0-port0"


class HWT905:

    def __init__(self, port=None, baudrate=9600):

        self.port = port if port is not None else _default_imu_port()
        self.baudrate = baudrate
        self.ser = None

        # 状态机变量
        self.FrameState = 0
        self.Bytenum = 0
        self.CheckSum = 0

        self.AccData = [0] * 8
        self.GyroData = [0] * 8
        self.AngleData = [0] * 8

        # 最新数据
        self.acc = (0.0, 0.0, 0.0)
        self.gyro = (0.0, 0.0, 0.0)
        self.angle = (0.0, 0.0, 0.0)

        self.running = False
        self.lock = threading.Lock()

    # ------------------------------------------------
    # 打开串口
    # ------------------------------------------------
    def open(self):

        self.ser = serial.Serial(self.port, self.baudrate, timeout=0.1)

        self.running = True

        threading.Thread(target=self._read_loop, daemon=True).start()

        print("[IMU] HWT905 started")

    # ------------------------------------------------
    # 串口读取线程
    # ------------------------------------------------
    def _read_loop(self):

        while self.running:

            data = self.ser.read(1)

            if data:
                self._parse_byte(data[0])

    # ------------------------------------------------
    # 状态机解析
    # ------------------------------------------------
    def _parse_byte(self, data):

        if self.FrameState == 0:

            if data == 0x55 and self.Bytenum == 0:
                self.CheckSum = data
                self.Bytenum = 1
                return

            elif data == 0x51 and self.Bytenum == 1:
                self.CheckSum += data
                self.FrameState = 1
                self.Bytenum = 2

            elif data == 0x52 and self.Bytenum == 1:
                self.CheckSum += data
                self.FrameState = 2
                self.Bytenum = 2

            elif data == 0x53 and self.Bytenum == 1:
                self.CheckSum += data
                self.FrameState = 3
                self.Bytenum = 2

        # ------------------------------------------------
        # 加速度
        # ------------------------------------------------
        elif self.FrameState == 1:

            if self.Bytenum < 10:
                self.AccData[self.Bytenum - 2] = data
                self.CheckSum += data
                self.Bytenum += 1

            else:
                if data == (self.CheckSum & 0xff):

                    acc = self._get_acc(self.AccData)

                    with self.lock:
                        self.acc = acc

                self._reset()

        # ------------------------------------------------
        # 角速度
        # ------------------------------------------------
        elif self.FrameState == 2:

            if self.Bytenum < 10:
                self.GyroData[self.Bytenum - 2] = data
                self.CheckSum += data
                self.Bytenum += 1

            else:
                if data == (self.CheckSum & 0xff):

                    gyro = self._get_gyro(self.GyroData)

                    with self.lock:
                        self.gyro = gyro

                self._reset()

        # ------------------------------------------------
        # 姿态角
        # ------------------------------------------------
        elif self.FrameState == 3:

            if self.Bytenum < 10:
                self.AngleData[self.Bytenum - 2] = data
                self.CheckSum += data
                self.Bytenum += 1

            else:
                if data == (self.CheckSum & 0xff):

                    angle = self._get_angle(self.AngleData)

                    with self.lock:
                        self.angle = angle

                self._reset()

    # ------------------------------------------------
    # 状态机重置
    # ------------------------------------------------
    def _reset(self):

        self.FrameState = 0
        self.Bytenum = 0
        self.CheckSum = 0

    # ------------------------------------------------
    # int16 补码解析
    # ------------------------------------------------
    def _int16(self, low, high):

        return struct.unpack('<h', bytes([low, high]))[0]

    # ------------------------------------------------
    # 加速度
    # ------------------------------------------------
    def _get_acc(self, data):

        ax_raw = self._int16(data[0], data[1])
        ay_raw = self._int16(data[2], data[3])
        az_raw = self._int16(data[4], data[5])

        ax = ax_raw / 32768.0 * 16
        ay = ay_raw / 32768.0 * 16
        az = az_raw / 32768.0 * 16

        return ax, ay, az

    # ------------------------------------------------
    # 角速度
    # ------------------------------------------------
    def _get_gyro(self, data):

        gx_raw = self._int16(data[0], data[1])
        gy_raw = self._int16(data[2], data[3])
        gz_raw = self._int16(data[4], data[5])

        gx = gx_raw / 32768.0 * 2000
        gy = gy_raw / 32768.0 * 2000
        gz = gz_raw / 32768.0 * 2000

        return gx, gy, gz

    # ------------------------------------------------
    # 姿态角
    # ------------------------------------------------
    def _get_angle(self, data):

        rx_raw = self._int16(data[0], data[1])
        ry_raw = self._int16(data[2], data[3])
        rz_raw = self._int16(data[4], data[5])

        rx = rx_raw / 32768.0 * 180
        ry = ry_raw / 32768.0 * 180
        rz = rz_raw / 32768.0 * 180

        return rx, ry, rz

    # ------------------------------------------------
    # 获取数据
    # ------------------------------------------------
    def get_acc(self):

        with self.lock:
            return self.acc

    def get_gyro(self):

        with self.lock:
            return self.gyro

    def get_angle(self):

        with self.lock:
            return self.angle

    def get_all(self):

        with self.lock:
            return self.acc, self.gyro, self.angle

    # ------------------------------------------------
    # 关闭串口
    # ------------------------------------------------
    def close(self):

        self.running = False

        if self.ser:
            self.ser.close()
