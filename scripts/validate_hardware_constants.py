#!/usr/bin/env python3
"""
validate_hardware_constants.py
CI script — verifies that atlas_hardware.hpp constants match hardware_specs.md.

Parses both files and cross-checks key values to catch copy/paste errors
or drift between documentation and code.

Exit code 0 = all constants match.
Exit code 1 = one or more mismatches found.
"""

import sys
import re
from pathlib import Path

ROOT = Path(__file__).parent.parent

CHECKS = [
    # (description, hpp_pattern, expected_value, tolerance)
    ("HG4930 gyro bias stability [°/hr]",
     r"GYRO_BIAS_STABILITY_DEG_HR\s*=\s*([0-9.]+)", 0.005, 1e-9),
    ("HG4930 gyro ARW [°/√hr]",
     r"GYRO_ARW_DEG_SQRTHR\s*=\s*([0-9.]+)", 0.004, 1e-9),
    ("HG4930 output rate [Hz]",
     r"OUTPUT_RATE_HZ\s*=\s*([0-9.]+)", 400.0, 1e-6),
    ("ICM-42688-P gyro NSD [°/s/√Hz]",
     r"GYRO_NOISE_DENSITY_DPS_SQRTHZ\s*=\s*([0-9.]+)", 0.0028, 1e-9),
    ("ZED-F9P RTK horizontal CEP [m]",
     r"RTK_HORIZONTAL_CEP_M\s*=\s*([0-9.]+)", 0.010, 1e-9),
    ("ZED-F9P velocity accuracy [m/s]",
     r"VELOCITY_ACCURACY_MPS\s*=\s*([0-9.]+)", 0.050, 1e-9),
    ("ZED-F9P max nav rate [Hz]",
     r"MAX_NAV_RATE_HZ\s*=\s*([0-9.]+)", 20.0, 1e-6),
    ("MS5611 altitude resolution [cm]",
     r"ALTITUDE_RESOLUTION_CM\s*=\s*([0-9.]+)", 10.0, 1e-6),
    ("Merlin 1D thrust SL [kN]",
     r"THRUST_SL_KN\s*=\s*([0-9.]+)", 845.0, 1e-6),
    ("Merlin 1D min throttle [%]",
     r"THROTTLE_MIN_PERCENT\s*=\s*([0-9.]+)", 40.0, 1e-6),
    ("Merlin 1D TVC gimbal max [°]",
     r"THRUST_VECTOR_MAX_DEG\s*=\s*([0-9.]+)", 5.0, 1e-6),
    ("Merlin 1D Isp SL [s]",
     r"ISP_SL_S\s*=\s*([0-9.]+)", 282.0, 1e-6),
    ("Grid fin slew rate [°/s]",
     r"SLEW_RATE_DEG_S\s*=\s*([0-9.]+)", 400.0, 1e-6),
    ("Grid fin max deflection [°]",
     r"DEFLECTION_MAX_DEG\s*=\s*([0-9.]+)", 90.0, 1e-6),
    ("RCS thrust per thruster [N]",
     r"THRUST_N\s*=\s*([0-9.]+)", 22.0, 1e-6),
    ("RCS min pulse width [ms]",
     r"MIN_PULSE_WIDTH_MS\s*=\s*([0-9.]+)", 10.0, 1e-6),
    ("Watchdog timeout [ms]",
     r"WATCHDOG_TIMEOUT_MS\s*=\s*([0-9.]+)", 50.0, 1e-6),
    ("Control loop period [ms]",
     r"CONTROL_LOOP_PERIOD_MS\s*=\s*([0-9.]+)", 1.0, 1e-9),
]


def main() -> int:
    hpp_path = ROOT / "src/cpp/include/atlas_hardware.hpp"
    if not hpp_path.exists():
        print(f"ERROR: {hpp_path} not found")
        return 1

    hpp_text = hpp_path.read_text()
    failures = []
    passed   = 0

    for description, pattern, expected, tol in CHECKS:
        match = re.search(pattern, hpp_text)
        if not match:
            failures.append(f"  NOT FOUND: {description!r} (pattern: {pattern})")
            continue

        actual = float(match.group(1))
        if abs(actual - expected) > tol:
            failures.append(
                f"  MISMATCH: {description}\n"
                f"    Expected: {expected}\n"
                f"    Actual:   {actual}"
            )
        else:
            passed += 1

    print(f"Hardware constant validation: {passed}/{len(CHECKS)} passed")
    if failures:
        print("\nFAILURES:")
        for f in failures:
            print(f)
        return 1

    print("All hardware constants verified against datasheets.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
