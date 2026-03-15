"""
control_core.py
ATLAS Control System — PID + LQR attitude and thrust vector control

Runs at 10 Hz (Python layer); sends GuidanceCommand to 1 kHz C++ loop.
All physical limits enforced from atlas_hardware.hpp constants (mirrored here).

Hardware limits (from datasheets):
  Merlin 1D throttle:     40% – 100%   (MERLIN1D::THROTTLE_MIN/MAX)
  TVC gimbal:             ±5°           (MERLIN1D::THRUST_VECTOR_MAX_DEG)
  Throttle slew:          200 ms 10→90% (MERLIN1D::THROTTLE_RESPONSE_TIME_MS)
  Grid fin slew:          400 °/s       (GRID_FIN::SLEW_RATE_DEG_S)
  RCS min pulse:          10 ms         (COLD_GAS_RCS::MIN_PULSE_WIDTH_MS)
"""

from __future__ import annotations

import math
import logging
from dataclasses import dataclass, field
from typing import Optional, Tuple

import numpy as np

logger = logging.getLogger(__name__)


# ─────────────────────────────────────────────────────────────────
#  Hardware limit constants (mirrors atlas_hardware.hpp)
# ─────────────────────────────────────────────────────────────────

THROTTLE_MIN_FRAC       = 0.40          # 40% — Merlin 1D minimum
THROTTLE_MAX_FRAC       = 1.00          # 100% maximum
THROTTLE_SLEW_PER_S     = 1.0 / 0.200  # 5.0/s — based on 200 ms response time
GIMBAL_MAX_RAD          = 5.0 * math.pi / 180.0   # ±5° in radians
GRID_FIN_SLEW_RAD_S     = 400.0 * math.pi / 180.0 # 400 °/s in rad/s
GRID_FIN_MAX_RAD        = 90.0 * math.pi / 180.0  # ±90° max
RCS_MIN_PULSE_S         = 0.010         # 10 ms minimum pulse
RCS_THRUST_N            = 22.0          # N per thruster (Moog DST-80 class)
CONTROL_DT_S            = 0.1          # 10 Hz Python control loop


# ─────────────────────────────────────────────────────────────────
#  PID Controller
# ─────────────────────────────────────────────────────────────────

@dataclass
class PIDGains:
    kp: float = 1.0
    ki: float = 0.0
    kd: float = 0.0
    integral_limit: float = 1.0   # anti-windup clamp


class PIDController:
    """
    Discrete PID controller with anti-windup, derivative filtering,
    and output clamping.

    Implements the velocity form to avoid integral windup on output saturation.
    """

    def __init__(self, gains: PIDGains, output_min: float = -1.0,
                 output_max: float = 1.0, dt: float = CONTROL_DT_S):
        self.gains       = gains
        self.output_min  = output_min
        self.output_max  = output_max
        self.dt          = dt

        self._integral   = 0.0
        self._prev_error = 0.0
        self._prev_deriv = 0.0
        self._deriv_alpha= 0.1    # low-pass filter on derivative term

    def reset(self) -> None:
        self._integral    = 0.0
        self._prev_error  = 0.0
        self._prev_deriv  = 0.0

    def update(self, error: float) -> float:
        """
        Compute PID output for current error.

        @param error  Setpoint − measured value
        @returns      Control output (clamped to [output_min, output_max])
        """
        # Proportional
        p_term = self.gains.kp * error

        # Integral with anti-windup (clamp then accumulate)
        self._integral += error * self.dt
        self._integral  = max(-self.gains.integral_limit,
                              min(self.gains.integral_limit, self._integral))
        i_term = self.gains.ki * self._integral

        # Derivative with low-pass filter to reduce noise amplification
        raw_deriv = (error - self._prev_error) / max(1e-6, self.dt)
        self._prev_deriv = (self._deriv_alpha * raw_deriv +
                            (1.0 - self._deriv_alpha) * self._prev_deriv)
        d_term = self.gains.kd * self._prev_deriv

        self._prev_error = error

        output = p_term + i_term + d_term
        return max(self.output_min, min(self.output_max, output))


# ─────────────────────────────────────────────────────────────────
#  Thrust Vector Control (TVC) — gimbal command generator
# ─────────────────────────────────────────────────────────────────

@dataclass
class AttitudeError:
    pitch_rad: float = 0.0
    yaw_rad:   float = 0.0
    roll_rad:  float = 0.0


class TVCController:
    """
    Thrust Vector Control — converts attitude error to gimbal commands.

    Uses separate PID loops for pitch and yaw axes.
    Roll is controlled by RCS only (engine gimbal has no roll authority).

    Gains tuned for Falcon 9 class first stage:
      - Mass ~500,000 kg, CG ~20 m from engine
      - Moment of inertia ~1e8 kg·m² (rough estimate)
      - Target bandwidth: 2 Hz (well below structural modes)
    """

    PITCH_GAINS = PIDGains(kp=0.8, ki=0.05, kd=0.15, integral_limit=GIMBAL_MAX_RAD)
    YAW_GAINS   = PIDGains(kp=0.8, ki=0.05, kd=0.15, integral_limit=GIMBAL_MAX_RAD)

    def __init__(self):
        self._pitch_pid = PIDController(
            self.PITCH_GAINS,
            output_min=-GIMBAL_MAX_RAD,
            output_max= GIMBAL_MAX_RAD
        )
        self._yaw_pid = PIDController(
            self.YAW_GAINS,
            output_min=-GIMBAL_MAX_RAD,
            output_max= GIMBAL_MAX_RAD
        )
        self._prev_pitch_cmd = 0.0
        self._prev_yaw_cmd   = 0.0

    def update(self, error: AttitudeError, dt: float = CONTROL_DT_S
               ) -> Tuple[float, float]:
        """
        Compute gimbal pitch and yaw commands.

        @param error  Attitude error (setpoint − current)
        @returns      (gimbal_pitch_rad, gimbal_yaw_rad) — both clamped to ±5°
        """
        raw_pitch = self._pitch_pid.update(error.pitch_rad)
        raw_yaw   = self._yaw_pid.update(error.yaw_rad)

        # Apply slew rate limiting (engine TVC cannot exceed physical limits)
        # Merlin 1D gimbal slew approximated at ~40 °/s (conservative estimate)
        gimbal_slew_rad_s = 40.0 * math.pi / 180.0
        max_delta = gimbal_slew_rad_s * dt

        pitch_delta = raw_pitch - self._prev_pitch_cmd
        yaw_delta   = raw_yaw   - self._prev_yaw_cmd

        if abs(pitch_delta) > max_delta:
            raw_pitch = self._prev_pitch_cmd + math.copysign(max_delta, pitch_delta)
        if abs(yaw_delta) > max_delta:
            raw_yaw = self._prev_yaw_cmd + math.copysign(max_delta, yaw_delta)

        self._prev_pitch_cmd = raw_pitch
        self._prev_yaw_cmd   = raw_yaw

        return raw_pitch, raw_yaw

    def reset(self) -> None:
        self._pitch_pid.reset()
        self._yaw_pid.reset()
        self._prev_pitch_cmd = 0.0
        self._prev_yaw_cmd   = 0.0


# ─────────────────────────────────────────────────────────────────
#  Throttle Controller — closed-loop thrust management
# ─────────────────────────────────────────────────────────────────

class ThrottleController:
    """
    Throttle management with:
      - Max-Q limiting (structural protection)
      - Minimum throttle enforcement (Merlin 1D: 40%)
      - Slew rate limiting (200 ms response time)
      - Propellant reserve protection (5% reserve non-negotiable)
    """

    MAX_Q_PA        = 35000.0   # Pa (Falcon 9 structural limit)
    MAX_Q_THROTTLE  = 0.72      # Throttle during max-Q region
    PROP_RESERVE    = 0.05      # 5% minimum propellant reserve

    def __init__(self):
        self._current_throttle = 0.0
        self._q_limiting       = False

    def compute(self, requested: float, q_pa: float,
                prop_fraction: float, dt: float = CONTROL_DT_S) -> float:
        """
        Compute final throttle command applying all physical constraints.

        @param requested      Guidance-requested throttle [0,1]
        @param q_pa           Current dynamic pressure (Pa)
        @param prop_fraction  Propellant remaining as fraction of total
        @param dt             Time step (s)
        @returns              Clamped, slew-limited throttle command [0,1]
        """
        target = requested

        # Max-Q protection: limit to 72% when q > 20,000 Pa (ascending toward peak)
        if q_pa > 20000.0 and q_pa < self.MAX_Q_PA:
            target = min(target, self.MAX_Q_THROTTLE)
            self._q_limiting = True
        else:
            self._q_limiting = False

        # Structural hard limit: cannot exceed throttle that produces q > MAX_Q
        if q_pa >= self.MAX_Q_PA:
            target = min(target, self.MAX_Q_THROTTLE * 0.9)
            logger.warning(f"MAX-Q EXCEEDED: q={q_pa:.0f} Pa — throttling to {target:.2%}")

        # Propellant reserve: cut engine if < 5% remaining
        if prop_fraction < self.PROP_RESERVE:
            target = 0.0
            logger.info("Propellant reserve reached — engine cutoff")

        # Minimum throttle when engine is running (Merlin 1D: 40%)
        if target > 0.0 and target < THROTTLE_MIN_FRAC:
            target = THROTTLE_MIN_FRAC

        # Hard maximum
        target = min(target, THROTTLE_MAX_FRAC)

        # Slew rate limiting (200 ms = 5.0/s)
        max_delta = THROTTLE_SLEW_PER_S * dt
        delta = target - self._current_throttle
        if abs(delta) > max_delta:
            target = self._current_throttle + math.copysign(max_delta, delta)

        self._current_throttle = target
        return target

    @property
    def is_q_limiting(self) -> bool:
        return self._q_limiting


# ─────────────────────────────────────────────────────────────────
#  Grid Fin Controller (Python layer — sends to C++ actuator)
# ─────────────────────────────────────────────────────────────────

class GridFinControllerPy:
    """
    Python-side grid fin command generator for atmospheric descent.
    Four fins in X configuration: [+pitch, +yaw, -pitch, -yaw] mapping.

    Produces normalized deflection commands [-1, +1] sent to C++ actuator.
    C++ layer applies slew rate limiting (400 °/s) and thermal checks.
    """

    FIN_MAP = {
        # fin_index: (pitch_sign, yaw_sign)
        0: (+1.0,  0.0),   # Fin 1: +pitch control
        1: ( 0.0, +1.0),   # Fin 2: +yaw control
        2: (-1.0,  0.0),   # Fin 3: -pitch control
        3: ( 0.0, -1.0),   # Fin 4: -yaw control
    }

    PITCH_GAINS = PIDGains(kp=0.5, ki=0.02, kd=0.10, integral_limit=0.5)
    YAW_GAINS   = PIDGains(kp=0.5, ki=0.02, kd=0.10, integral_limit=0.5)

    def __init__(self):
        self._pitch_pid = PIDController(self.PITCH_GAINS, -1.0, 1.0)
        self._yaw_pid   = PIDController(self.YAW_GAINS,   -1.0, 1.0)
        self._deployed  = False

    def deploy(self) -> None:
        self._deployed = True
        logger.info("Grid fins deployed")

    def compute(self, pitch_error_rad: float,
                yaw_error_rad: float) -> list[float]:
        """
        Compute four fin deflection commands.

        @returns  [fin0, fin1, fin2, fin3] in [-1, +1]
        """
        if not self._deployed:
            return [0.0, 0.0, 0.0, 0.0]

        pitch_cmd = self._pitch_pid.update(pitch_error_rad)
        yaw_cmd   = self._yaw_pid.update(yaw_error_rad)

        cmds = []
        for i in range(4):
            p_sign, y_sign = self.FIN_MAP[i]
            cmd = p_sign * pitch_cmd + y_sign * yaw_cmd
            cmd = max(-1.0, min(1.0, cmd))
            cmds.append(cmd)
        return cmds


# ─────────────────────────────────────────────────────────────────
#  RCS Controller (Python layer)
# ─────────────────────────────────────────────────────────────────

class RCSControllerPy:
    """
    Python-side RCS thruster selector.
    12 thrusters in 4 groups of 3:
      0–2:  +pitch  (fire for nose-up torque)
      3–5:  -pitch  (fire for nose-down torque)
      6–8:  +yaw    (fire for +yaw torque)
      9–11: -yaw    (fire for -yaw torque)

    Roll axis: cross-coupled pairs (0,6) for +roll, (3,9) for -roll.
    Minimum pulse: 10 ms enforced in C++ layer.
    """

    DEADBAND_RAD = 0.005  # 0.3° deadband — below this, no RCS fire

    def compute(self, error: AttitudeError) -> list[bool]:
        """
        @returns  12-element bool array (True = fire)
        """
        cmds = [False] * 12

        # Pitch axis
        if error.pitch_rad >  self.DEADBAND_RAD:
            cmds[0] = cmds[1] = cmds[2] = True   # +pitch
        elif error.pitch_rad < -self.DEADBAND_RAD:
            cmds[3] = cmds[4] = cmds[5] = True   # -pitch

        # Yaw axis
        if error.yaw_rad >  self.DEADBAND_RAD:
            cmds[6] = cmds[7] = cmds[8] = True   # +yaw
        elif error.yaw_rad < -self.DEADBAND_RAD:
            cmds[9] = cmds[10]= cmds[11]= True   # -yaw

        # Roll: use cross-coupled pairs
        if error.roll_rad >  self.DEADBAND_RAD:
            cmds[0] = cmds[6] = True
        elif error.roll_rad < -self.DEADBAND_RAD:
            cmds[3] = cmds[9] = True

        return cmds


# ─────────────────────────────────────────────────────────────────
#  Integrated Control Core
# ─────────────────────────────────────────────────────────────────

class ControlCore:
    """
    Integrated control system combining TVC, throttle, grid fins, and RCS.
    All outputs go into a GuidanceCommand dict for the ATLAS main loop.
    """

    def __init__(self):
        self.tvc       = TVCController()
        self.throttle  = ThrottleController()
        self.grid_fins = GridFinControllerPy()
        self.rcs       = RCSControllerPy()
        self.logger    = logging.getLogger("ControlCore")

    def compute_command(
        self,
        attitude_error: AttitudeError,
        throttle_request: float,
        q_pa: float,
        prop_fraction: float,
        altitude_m: float,
        dt: float = CONTROL_DT_S
    ) -> dict:
        """
        Compute complete actuator command package.

        @returns dict with keys matching GuidanceCommand fields
        """
        # TVC gimbal
        pitch_cmd, yaw_cmd = self.tvc.update(attitude_error, dt)

        # Throttle
        throttle_cmd = self.throttle.compute(throttle_request, q_pa, prop_fraction, dt)

        # Grid fins (descent only — below 70 km altitude)
        fin_cmds = [0.0, 0.0, 0.0, 0.0]
        if altitude_m < 70000.0 and self.grid_fins._deployed:
            fin_cmds = self.grid_fins.compute(
                attitude_error.pitch_rad, attitude_error.yaw_rad)

        # RCS (attitude control when TVC authority is low)
        rcs_cmds = [False] * 12
        if throttle_cmd < 0.05 or q_pa < 500.0:
            # Low dynamic pressure — TVC ineffective, use RCS
            rcs_cmds = self.rcs.compute(attitude_error)

        return {
            "throttle_cmd":        throttle_cmd,
            "gimbal_pitch_cmd_rad": pitch_cmd,
            "gimbal_yaw_cmd_rad":   yaw_cmd,
            "grid_fin_1_cmd":       fin_cmds[0],
            "grid_fin_2_cmd":       fin_cmds[1],
            "grid_fin_3_cmd":       fin_cmds[2],
            "grid_fin_4_cmd":       fin_cmds[3],
            "rcs_cmd":              rcs_cmds,
        }

    def reset(self) -> None:
        self.tvc.reset()
        self.throttle._current_throttle = 0.0
