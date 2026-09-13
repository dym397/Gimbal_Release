import os
import random
import time
from collections import deque
from typing import Optional, Tuple

from gimbal_interface import GimbalBase


MOCK_GIMBAL_MODES = {
    "fast": {"az_speed": 80.0, "el_speed": 50.0, "noise": 0.02},
    "normal": {"az_speed": 30.0, "el_speed": 20.0, "noise": 0.08},
    "slow": {"az_speed": 10.0, "el_speed": 8.0, "noise": 0.15},
}


def _env_float(name: str, default: float) -> float:
    value = os.getenv(name)
    if value is None or str(value).strip() == "":
        return float(default)
    try:
        return float(value)
    except ValueError:
        print(f"[MockGimbal][Warn] invalid {name}={value!r}, fallback to {default}")
        return float(default)


def _clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


class MockGimbalAdapter(GimbalBase):
    """
    Software gimbal model for SITL/closed-loop validation.

    It intentionally returns the simulated current attitude, not the target
    attitude, so the main control thread can exercise settle/timeout logic.
    """

    def __init__(
        self,
        port: str = "MOCK_PORT",
        slew_rate: Optional[float] = None,
        az_base: float = 90.0,
        az_slew_rate: Optional[float] = None,
        el_slew_rate: Optional[float] = None,
        repeatability_deg: float = 0.1,
        mode: Optional[str] = None,
        feedback_noise_deg: Optional[float] = None,
        az_min: float = 0.0,
        az_max: float = 350.0,
        el_min: float = -30.0,
        el_max: float = 90.0,
        cmd_deadband_deg: float = 0.0,
        feedback_delay_s: float = 0.0,
    ):
        self.port = port

        selected_mode = (mode or os.getenv("MOCK_GIMBAL_MODE", "normal")).strip().lower()
        if selected_mode not in MOCK_GIMBAL_MODES:
            print(f"[MockGimbal][Warn] unknown mode={selected_mode!r}, fallback to normal")
            selected_mode = "normal"
        mode_cfg = MOCK_GIMBAL_MODES[selected_mode]

        default_az_speed = mode_cfg["az_speed"] if az_slew_rate is None else float(az_slew_rate)
        default_el_speed = mode_cfg["el_speed"] if el_slew_rate is None else float(el_slew_rate)
        default_noise = mode_cfg["noise"] if feedback_noise_deg is None else float(feedback_noise_deg)

        # Backward compatibility: if the old single slew_rate is provided, use it for both axes.
        if slew_rate is not None:
            default_az_speed = float(slew_rate)
            default_el_speed = float(slew_rate)

        self.mode = selected_mode
        self.az_slew_rate = _env_float("MOCK_GIMBAL_AZ_SPEED", default_az_speed)
        self.el_slew_rate = _env_float("MOCK_GIMBAL_EL_SPEED", default_el_speed)
        self.feedback_noise_deg = max(0.0, _env_float("MOCK_GIMBAL_NOISE", default_noise))
        self.repeatability_deg = max(0.0, float(repeatability_deg))
        self.az_min = float(az_min)
        self.az_max = float(az_max)
        self.el_min = float(el_min)
        self.el_max = float(el_max)
        self.cmd_deadband_deg = max(0.0, _env_float("MOCK_GIMBAL_CMD_DEADBAND", cmd_deadband_deg))
        self.feedback_delay_s = max(0.0, _env_float("MOCK_GIMBAL_FEEDBACK_DELAY", feedback_delay_s))
        self.az_base = float(az_base)
        self._is_ready = False

        # Initial pose follows the system reference.
        self.curr_el = 0.0
        self.curr_az = _clamp(self.az_base, self.az_min, self.az_max)

        self.target_el = 0.0
        self.target_az = self.curr_az

        self.last_update_time = time.time()
        self._feedback_history = deque(maxlen=2000)
        self._feedback_history.append((self.last_update_time, self.curr_el, self.curr_az))

    def connect(self) -> bool:
        print(
            f"[MockGimbal] connected (port={self.port}, "
            f"mode={self.mode}, "
            f"az_rate={self.az_slew_rate:.1f}deg/s, "
            f"el_rate={self.el_slew_rate:.1f}deg/s, "
            f"noise={self.feedback_noise_deg:.2f}deg, "
            f"delay={self.feedback_delay_s:.3f}s, "
            f"az_range=[{self.az_min:.1f},{self.az_max:.1f}], "
            f"el_range=[{self.el_min:.1f},{self.el_max:.1f}], "
            f"repeatability={self.repeatability_deg:.2f}deg)"
        )
        self._is_ready = True
        self.last_update_time = time.time()
        return True

    def wait_ready(self, timeout: float = 5.0) -> bool:
        return self._is_ready

    def _snap_to_repeatability(self, angle: float) -> float:
        if self.repeatability_deg <= 0.0:
            return float(angle)
        return round(float(angle) / self.repeatability_deg) * self.repeatability_deg

    def set_attitude(
        self,
        elevation=None,
        azimuth=None,
        read_status: bool = False,
        force: bool = False,
    ):
        """Mirror the real adapter API, including independent-axis retries."""
        status = {
            "el_requested": elevation is not None,
            "az_requested": azimuth is not None,
            "el_sent": elevation is not None,
            "az_sent": azimuth is not None,
            "el_ack": elevation is not None,
            "az_ack": azimuth is not None,
            "reason": "mock",
        }
        if elevation is None and azimuth is None:
            status["reason"] = "no_axis_requested"
            return status

        new_el = self.target_el
        if elevation is not None:
            new_el = self._snap_to_repeatability(
                _clamp(float(elevation), self.el_min, self.el_max)
            )
        new_az = self.target_az
        if azimuth is not None:
            new_az = self._snap_to_repeatability(
                _clamp(float(azimuth), self.az_min, self.az_max)
            )

        if (
            abs(new_el - self.target_el) < self.cmd_deadband_deg
            and abs(new_az - self.target_az) < self.cmd_deadband_deg
        ):
            status["reason"] = "deadband"
            return status

        self.target_el = new_el
        self.target_az = new_az
        return status

    def get_attitude(self) -> Optional[Tuple[float, float, float]]:
        now = time.time()
        dt = now - self.last_update_time
        self.last_update_time = now

        step_el = self.el_slew_rate * dt
        step_az = self.az_slew_rate * dt

        if abs(self.target_el - self.curr_el) <= step_el:
            self.curr_el = self.target_el
        else:
            self.curr_el += step_el if self.target_el > self.curr_el else -step_el

        diff_az = self.target_az - self.curr_az
        if abs(diff_az) <= step_az:
            self.curr_az = self.target_az
        else:
            self.curr_az += step_az if diff_az > 0 else -step_az

        self.curr_el = self._snap_to_repeatability(_clamp(self.curr_el, self.el_min, self.el_max))
        self.curr_az = self._snap_to_repeatability(_clamp(self.curr_az, self.az_min, self.az_max))
        self._feedback_history.append((now, self.curr_el, self.curr_az))

        fb_el, fb_az = self._delayed_feedback(now)
        if self.feedback_noise_deg > 0.0:
            fb_el += random.gauss(0.0, self.feedback_noise_deg)
            fb_az += random.gauss(0.0, self.feedback_noise_deg)

        fb_el = _clamp(fb_el, self.el_min, self.el_max)
        fb_az = _clamp(fb_az, self.az_min, self.az_max)
        return (fb_el, fb_az, 0.0)

    def _delayed_feedback(self, now: float) -> Tuple[float, float]:
        if self.feedback_delay_s <= 0.0:
            return self.curr_el, self.curr_az

        cutoff = now - self.feedback_delay_s
        chosen = self._feedback_history[0]
        for item in self._feedback_history:
            if item[0] <= cutoff:
                chosen = item
            else:
                break
        return chosen[1], chosen[2]

    def close(self):
        print("[MockGimbal] closed")
        self._is_ready = False
