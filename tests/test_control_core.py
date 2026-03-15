"""
test_control_core.py
Unit tests for ATLAS control system — PID, TVC, throttle, RCS.

Tests verify:
  - PID output within bounds
  - Slew rate limiting enforced (Merlin 1D: 200 ms response)
  - Minimum throttle enforcement (40% when engine running)
  - Max-Q throttle reduction (72% at q > 20,000 Pa)
  - Gimbal limit enforcement (±5° from datasheet)
"""

import pytest
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent.parent / "src/python"))

from atlas.control.control_core import (
    PIDController, PIDGains, TVCController, ThrottleController,
    RCSControllerPy, ControlCore, AttitudeError,
    GIMBAL_MAX_RAD, THROTTLE_MIN_FRAC, THROTTLE_MAX_FRAC,
    THROTTLE_SLEW_PER_S, CONTROL_DT_S
)


class TestPIDController:

    def test_proportional_output(self):
        pid = PIDController(PIDGains(kp=2.0, ki=0.0, kd=0.0), -10.0, 10.0)
        out = pid.update(1.0)
        assert abs(out - 2.0) < 1e-9

    def test_output_clamped_to_bounds(self):
        pid = PIDController(PIDGains(kp=100.0, ki=0.0, kd=0.0), -5.0, 5.0)
        out = pid.update(1.0)
        assert out == pytest.approx(5.0)

    def test_integral_anti_windup(self):
        """Integral should not grow beyond integral_limit."""
        pid = PIDController(PIDGains(kp=0.0, ki=1.0, kd=0.0,
                                      integral_limit=2.0), -10.0, 10.0)
        for _ in range(1000):
            pid.update(1.0)   # large positive error repeatedly
        assert pid._integral <= 2.0 + 1e-6

    def test_reset_clears_state(self):
        pid = PIDController(PIDGains(kp=1.0, ki=1.0, kd=1.0), -10.0, 10.0)
        for _ in range(20):
            pid.update(0.5)
        pid.reset()
        assert pid._integral == 0.0
        assert pid._prev_error == 0.0


class TestTVCController:

    def test_gimbal_within_physical_limit(self):
        """Gimbal output must stay within ±5° (Merlin 1D THRUST_VECTOR_MAX_DEG)."""
        tvc = TVCController()
        # Very large attitude error — should be clamped
        error = AttitudeError(pitch_rad=1.0, yaw_rad=-1.0)
        pitch_cmd, yaw_cmd = tvc.update(error)
        assert abs(pitch_cmd) <= GIMBAL_MAX_RAD + 1e-9
        assert abs(yaw_cmd)   <= GIMBAL_MAX_RAD + 1e-9

    def test_slew_rate_limit_applied(self):
        """Gimbal cannot jump from 0 to max in one step."""
        tvc = TVCController()
        error = AttitudeError(pitch_rad=0.5)   # large step
        pitch_cmd, _ = tvc.update(error, dt=CONTROL_DT_S)
        # At 40 °/s slew, max delta in 0.1s = 4° = 0.0698 rad
        max_delta = 40.0 * math.pi / 180.0 * CONTROL_DT_S
        assert abs(pitch_cmd) <= max_delta + 1e-6

    def test_zero_error_gives_zero_cmd(self):
        tvc = TVCController()
        p, y = tvc.update(AttitudeError(0.0, 0.0, 0.0))
        assert abs(p) < 1e-9
        assert abs(y) < 1e-9


class TestThrottleController:

    def test_minimum_throttle_enforced(self):
        """Throttle below 40% when requested > 0 should be bumped to 40%."""
        ctrl = ThrottleController()
        ctrl._current_throttle = 0.5  # engine running
        out = ctrl.compute(requested=0.1, q_pa=0.0, prop_fraction=1.0)
        assert out >= THROTTLE_MIN_FRAC - 1e-9

    def test_max_q_throttle_reduction(self):
        """At q > 20,000 Pa, throttle limited to 72%."""
        ctrl = ThrottleController()
        ctrl._current_throttle = 1.0
        # Must also check slew — start at 1.0, one tick
        out = ctrl.compute(requested=1.0, q_pa=25000.0, prop_fraction=1.0,
                           dt=10.0)  # large dt to bypass slew
        assert out <= ThrottleController.MAX_Q_THROTTLE + 1e-9

    def test_propellant_reserve_triggers_cutoff(self):
        """When prop_fraction < 5%, throttle should become 0."""
        ctrl = ThrottleController()
        ctrl._current_throttle = 1.0
        out = ctrl.compute(requested=1.0, q_pa=0.0, prop_fraction=0.03,
                           dt=10.0)
        assert out == pytest.approx(0.0)

    def test_slew_rate_limit(self):
        """Throttle cannot ramp faster than THROTTLE_SLEW_PER_S × dt."""
        ctrl = ThrottleController()
        ctrl._current_throttle = 0.0
        max_expected = THROTTLE_SLEW_PER_S * CONTROL_DT_S
        out = ctrl.compute(requested=1.0, q_pa=0.0, prop_fraction=1.0,
                           dt=CONTROL_DT_S)
        # THROTTLE_MIN is applied after slew, so compare against that
        assert out <= max(THROTTLE_MIN_FRAC, max_expected) + 1e-9

    def test_no_overshoot_above_100(self):
        ctrl = ThrottleController()
        ctrl._current_throttle = 1.0
        out = ctrl.compute(requested=2.0, q_pa=0.0, prop_fraction=1.0, dt=1.0)
        assert out <= THROTTLE_MAX_FRAC + 1e-9


class TestRCSController:

    def test_pitch_up_fires_correct_thrusters(self):
        rcs = RCSControllerPy()
        error = AttitudeError(pitch_rad=0.1, yaw_rad=0.0, roll_rad=0.0)
        cmds = rcs.compute(error)
        assert len(cmds) == 12
        # +pitch thrusters (0,1,2) should fire
        assert cmds[0] and cmds[1] and cmds[2]
        # -pitch thrusters (3,4,5) should NOT fire
        assert not cmds[3] and not cmds[4] and not cmds[5]

    def test_deadband_no_fire(self):
        rcs = RCSControllerPy()
        error = AttitudeError(pitch_rad=0.001, yaw_rad=0.001, roll_rad=0.001)
        cmds = rcs.compute(error)
        assert not any(cmds)

    def test_yaw_fires_correct_thrusters(self):
        rcs = RCSControllerPy()
        error = AttitudeError(pitch_rad=0.0, yaw_rad=-0.1, roll_rad=0.0)
        cmds = rcs.compute(error)
        # -yaw (thrusters 9-11)
        assert cmds[9] and cmds[10] and cmds[11]
        assert not cmds[6] and not cmds[7] and not cmds[8]


class TestControlCore:

    def test_command_package_has_all_keys(self):
        ctrl = ControlCore()
        cmd = ctrl.compute_command(
            AttitudeError(), 0.8, 5000.0, 1.0, 10000.0
        )
        required = [
            "throttle_cmd", "gimbal_pitch_cmd_rad", "gimbal_yaw_cmd_rad",
            "grid_fin_1_cmd", "grid_fin_2_cmd", "grid_fin_3_cmd", "grid_fin_4_cmd",
            "rcs_cmd"
        ]
        for key in required:
            assert key in cmd, f"Missing key: {key}"

    def test_throttle_clamped_in_command(self):
        ctrl = ControlCore()
        # Force throttle controller to have high current throttle
        ctrl.throttle._current_throttle = 1.0
        cmd = ctrl.compute_command(
            AttitudeError(), 2.0, 0.0, 1.0, 0.0, dt=10.0
        )
        assert cmd["throttle_cmd"] <= THROTTLE_MAX_FRAC + 1e-9

    def test_rcs_activates_at_low_throttle(self):
        ctrl = ControlCore()
        error = AttitudeError(pitch_rad=0.1)
        cmd = ctrl.compute_command(error, 0.0, 0.0, 1.0, 0.0)  # zero throttle
        assert any(cmd["rcs_cmd"]), "RCS should fire when throttle is 0"
