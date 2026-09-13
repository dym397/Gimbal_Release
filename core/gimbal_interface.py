from abc import ABC, abstractmethod
import time
from typing import Tuple, Optional

# ==========================================
# 1. 抽象基类 (标准接口定义)
# ==========================================
class GimbalBase(ABC):
    @abstractmethod
    def connect(self) -> bool:
        """连接设备"""
        pass

    @abstractmethod
    def wait_ready(self, timeout: float = 15.0) -> bool:
        """等待设备就绪"""
        pass

    @abstractmethod
    def set_attitude(self, elevation=None, azimuth=None, read_status: bool = False, force: bool = False):
        """
        核心控制函数：设置目标姿态
        :param elevation: 俯仰角 (度) +上 -下
        :param azimuth: 方位角 (度) 0-360 或 -180~180
        :param read_status: 是否在发送后尝试读取当前状态
        """
        pass

    @abstractmethod
    def get_attitude(self) -> Optional[Tuple[float, float, float]]:
        """
        获取当前真实姿态
        :return: (elevation, azimuth, other) 单位度
                 若无数据返回 None
        """
        pass

    @abstractmethod
    def close(self):
        """断开连接"""
        pass

# ==========================================
# 2. GT06Z 云台适配器 (对应您的新驱动)
# ==========================================
class GT06ZAdapter(GimbalBase):
    def __init__(self, port: str):
        self.port = port
        self.driver = None
        self._is_ready = False
        
        # 尝试导入您的驱动文件
        try:
            from GT06Z_gimbal import GT06ZGimbal
            self._driver_class = GT06ZGimbal
        except ImportError:
            self._driver_class = None
            print("[Error] 找不到 GT06Z_gimbal.py 驱动文件")

    def connect(self) -> bool:
        if self._driver_class is None:
            return False
            
        try:
            self.driver = self._driver_class(self.port)
            if self.driver.open():
                self._is_ready = True
                print(f"[Adapter] GT06Z 云台已连接: {self.port}")
                return True
        except Exception as e:
            print(f"[Adapter] 连接失败: {e}")
        
        return False

    def wait_ready(self, timeout: float = 5.0) -> bool:
        # 串口设备打开即视为就绪，这里简单休眠一下等待电路稳定
        if self._is_ready:
            time.sleep(1.0)
            return True
        return False

    def set_attitude(self, elevation=None, azimuth=None, read_status: bool = False, force: bool = False):
        if not self.driver:
            return None
        
        # 调用驱动的 set_angles
        # 注意：您的驱动内部已经处理了死区(epsilon)和频率限制(min_interval)
        # 直接透传即可
        status = self.driver.set_angles(
            elevation_deg=elevation,
            azimuth_deg=azimuth,
            force=force,
        )
        
        # 如果需要立即读取状态 (通常为了确认命令是否执行)
        # 注意：频繁读取会增加通信延迟
        if read_status:
            self.get_attitude()
        return status

    def get_attitude(self) -> Optional[Tuple[float, float, float]]:
        if not self.driver: return None
        
        # 调用驱动的 query_angles
        # 返回值是 (el, az)
        result = self.driver.query_angles()
        
        if result:
            el, az = result
            # 补齐第三个轴 (Roll/Other) 为 0.0，以符合接口标准
            return (el, az, 0.0)
        
        return None

    def close(self):
        if self.driver:
            self.driver.close()
            print("[Adapter] GT06Z 云台已关闭")
