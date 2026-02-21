"""
Motor Inertia Identification Test

Applies a known torque step in torque mode and measures angular
acceleration to compute total rotational inertia (rotor + load):

    J = torque / angular_acceleration

The test captures velocity over time during a constant-torque burst.
A linear regression on the velocity ramp gives the acceleration, from
which inertia is derived.

Run as administrator: ADAPTER=\\Device\\NPF_{...} python test_inertia_identification.py
"""

import os
import sys
import time
import numpy as np
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent.parent))
from hdrive_etc import HDriveETC, Mode


class InertiaIdentificationTest:
    """Identify rotational inertia via a torque-step acceleration test."""

    TORQUE_MNM = int(os.environ.get("TORQUE_MNM", 200))
    CAPTURE_DURATION = float(os.environ.get("CAPTURE_DURATION_MS", 500)) / 1000.0
    SAMPLE_INTERVAL = 0.001  # 1 kHz

    def __init__(self, motor_or_adapter, output_file: str = None):
        if isinstance(motor_or_adapter, str):
            self.adapter = motor_or_adapter
            self.motor = None
            self._external_motor = False
        elif isinstance(motor_or_adapter, HDriveETC):
            self.adapter = None
            self.motor = motor_or_adapter
            self._external_motor = True
        else:
            self.adapter = None
            self.motor = None
            self._external_motor = False

        self.output_file = output_file
        self.timestamps = []
        self.velocities = []
        self.abort_event = None

    # ------------------------------------------------------------------
    # Connection helpers (standalone mode)
    # ------------------------------------------------------------------

    def setup(self) -> bool:
        if self._external_motor:
            return True
        try:
            self.motor = HDriveETC(
                adapter=self.adapter,
                slave_index=0,
                cycle_time_ms=0.02,
            )
            self.motor.connect()
            time.sleep(0.5)
            self.motor.set_mode(Mode.TORQUE)
            time.sleep(0.2)
            return True
        except Exception as exc:
            print(f"[ERROR] Setup failed: {exc}")
            return False

    def teardown(self):
        if self.motor and not self._external_motor:
            try:
                self.motor.set_mode(Mode.STOP)
                time.sleep(0.1)
                self.motor.disconnect()
            except Exception:
                pass

    # ------------------------------------------------------------------
    # Data capture
    # ------------------------------------------------------------------

    def capture_acceleration(self):
        """Apply a torque step and record velocity vs time."""
        self.timestamps = []
        self.velocities = []

        self.motor.set_torque(self.TORQUE_MNM)
        start = time.time()

        while (time.time() - start) < self.CAPTURE_DURATION:
            if self.abort_event and self.abort_event.is_set():
                print("[ABORT] Inertia test aborted")
                return
            elapsed = time.time() - start
            self.timestamps.append(elapsed)
            self.velocities.append(self.motor.get_velocity())
            time.sleep(self.SAMPLE_INTERVAL)

        self.motor.set_torque(0)

    # ------------------------------------------------------------------
    # Analysis
    # ------------------------------------------------------------------

    @staticmethod
    def _smooth(v, window=5):
        """Simple moving-average filter."""
        if len(v) < window:
            return v
        kernel = np.ones(window) / window
        return np.convolve(v, kernel, mode="valid")

    def analyze_inertia(self) -> dict | None:
        if len(self.timestamps) < 30:
            return None

        t = np.array(self.timestamps)
        v_rpm = np.array(self.velocities)
        v_rad = v_rpm * (2.0 * np.pi / 60.0)  # RPM -> rad/s

        # Smooth velocity to reduce noise
        win = min(7, max(3, len(v_rad) // 30))
        v_smooth = self._smooth(v_rad, win)
        t_smooth = self._smooth(t, win)

        # Detect ramp by velocity level: 10% .. 80% of peak velocity
        peak_v = np.max(np.abs(v_smooth))
        if peak_v < 0.1:
            return None
        sign = 1.0 if v_smooth[-1] >= 0 else -1.0
        v_signed = v_smooth * sign  # make ramp always positive

        lo = peak_v * 0.10
        hi = peak_v * 0.80

        above_lo = np.where(v_signed >= lo)[0]
        above_hi = np.where(v_signed >= hi)[0]
        if len(above_lo) == 0 or len(above_hi) == 0:
            return None

        i_ramp_start = above_lo[0]
        i_ramp_end = above_hi[0]
        if i_ramp_end <= i_ramp_start:
            i_ramp_end = min(i_ramp_start + 5, len(t_smooth) - 1)

        t_ramp_start = t_smooth[i_ramp_start]
        t_ramp_end = t_smooth[i_ramp_end]

        # Map back to original unsmoothed data for the linear fit
        orig_start = np.searchsorted(t, t_ramp_start)
        orig_end = np.searchsorted(t, t_ramp_end)
        orig_end = max(orig_end, orig_start + 3)
        orig_end = min(orig_end, len(t))
        t_fit = t[orig_start:orig_end]
        v_fit_data = v_rad[orig_start:orig_end]

        if len(t_fit) < 3:
            return None

        # Linear fit on the ramp region: v = slope*t + intercept
        coeffs = np.polyfit(t_fit, v_fit_data, 1)
        fit_slope = coeffs[0]  # rad/s²
        fit_intercept = coeffs[1]

        v_fit_line = np.polyval(coeffs, t_fit)
        ss_res = np.sum((v_fit_data - v_fit_line) ** 2)
        ss_tot = np.sum((v_fit_data - np.mean(v_fit_data)) ** 2)
        r_squared = 1.0 - (ss_res / ss_tot) if ss_tot > 0 else 0.0

        accel = fit_slope  # rad/s²

        # J = torque / alpha  (torque in N*m, alpha in rad/s²)
        torque_nm = self.TORQUE_MNM * 1e-3
        if abs(accel) < 1e-6:
            return None
        inertia_kgm2 = torque_nm / accel
        inertia_gcm2 = inertia_kgm2 * 1e7  # kg*m² -> g*cm²

        return {
            "inertia_gcm2": float(round(inertia_gcm2, 1)),
            "acceleration_rad_s2": float(round(accel, 2)),
            "torque_mNm": self.TORQUE_MNM,
            "r_squared": float(round(r_squared, 4)),
            "peak_velocity_rpm": float(round(np.max(np.abs(v_rpm)), 2)),
            "fit_slope": float(fit_slope),
            "fit_intercept": float(fit_intercept),
            "trim_start": float(t_ramp_start),
            "trim_end": float(t_ramp_end),
        }

    # ------------------------------------------------------------------
    # Standalone entry point
    # ------------------------------------------------------------------

    def run_all_tests(self) -> bool:
        if not self.setup():
            return False
        try:
            self.capture_acceleration()
            if not self.timestamps:
                print("[ERROR] No data captured")
                return False
            metrics = self.analyze_inertia()
            if metrics is None:
                print("[ERROR] Analysis failed")
                return False
            print(f"Inertia: {metrics['inertia_gcm2']:.1f} g\u00B7cm²")
            print(f"Accel:   {metrics['acceleration_rad_s2']:.2f} rad/s^2")
            print(f"R^2:     {metrics['r_squared']:.4f}")
            return True
        except Exception as exc:
            print(f"[ERROR] Test failed: {exc}")
            return False
        finally:
            self.teardown()


if __name__ == "__main__":
    adapter = os.environ.get("ADAPTER", None)
    tester = InertiaIdentificationTest(adapter)
    success = tester.run_all_tests()
    sys.exit(0 if success else 1)
