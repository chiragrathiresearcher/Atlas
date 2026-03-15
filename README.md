# ATLAS — Autonomous Launch & Termination Architecture System
## v2.0 — Complete Autonomous Avionics Stack

[![CI](https://github.com/your-org/ATLAS/actions/workflows/ci.yml/badge.svg)](https://github.com/your-org/ATLAS/actions)
[![Python 3.11+](https://img.shields.io/badge/python-3.11%2B-blue)](https://python.org)
[![C++17](https://img.shields.io/badge/C%2B%2B-17-blue)](https://isocpp.org)
[![ARVS Authority](https://img.shields.io/badge/ARVS-Non--Bypassable-red)](docs/architecture.md)

```
╔══════════════════════════════════════════════════════════════╗
║   ATLAS — Autonomous Launch & Termination Architecture       ║
║   Authority Layer: ARVS v1.0 (Non-Bypassable)                ║
║   IMU: HG4930×2 + ICM-42688-P  │  GNSS: ZED-F9P RTK         ║
║   Control: 1 kHz C++  │  Guidance: 10 Hz Python              ║
╚══════════════════════════════════════════════════════════════╝
```

---

## What Is ATLAS?

ATLAS is a **production-grade autonomous flight software framework** for reusable
launch vehicles. It integrates:

- **ARVS Authority Layer** — non-bypassable safety gate that validates all 18+ axioms before every actuator command
- **1 kHz C++ real-time control loop** — IMU read → EKF → actuator command, RTOS-ready
- **10 Hz Python guidance** — ARVS-approved trajectory generation and mission management
- **Datasheet-accurate hardware models** — every sensor constant traces to a manufacturer datasheet

> **Core principle: No ATLAS subsystem can self-authorize any action. ARVS is the sole authority.**

---

## Architecture

```
┌────────────────────────────────────────────────────────────────┐
│  MISSION MANAGER  (Python 3.11, 1 Hz)                          │
│  14 flight phases — every transition ARVS-gated                │
├────────────────────────────────────────────────────────────────┤
│  GUIDANCE CORE  (Python 3.11, 10 Hz)                           │
│  Trajectory generation → ARVS permission → GuidanceCommand     │
├────────────────────────────────────────────────────────────────┤
│  ╔══════════════════════════════════════════════════════════╗  │
│  ║   ARVS AUTHORITY LAYER  (Python + C++, all rates)       ║  │
│  ║   ARVSAuthority.py ←IPC/SHM→ ARVSSafetyGate.cpp         ║  │
│  ║   18+ Axioms: E1-E4, U1-U3, A1-A4, C1-C2, R1-R2, Z     ║  │
│  ║   Irreversible guard: confidence ≥ 0.95  (Axiom C2)     ║  │
│  ╚══════════════════════════════════════════════════════════╝  │
├────────────────────────────────────────────────────────────────┤
│  CONTROL CORE  (Python 3.11, 10 Hz)                            │
│  PID / LQR │ TVC gimbal │ Grid fins │ RCS 12-thruster          │
├────────────────────────────────────────────────────────────────┤
│  FLIGHT COMPUTER  (C++17, 1 kHz, RTOS-ready)                   │
│  EKF fusion │ Triple-redundant IMU voting │ 50 ms watchdog     │
├────────────────────────────────────────────────────────────────┤
│  HARDWARE  (Datasheet-sourced constants only)                   │
│  HG4930 IMU ×2  │  ICM-42688-P  │  ZED-F9P GNSS (RTK)          │
│  Merlin 1D ×9   │  Grid Fins ×4  │  RCS ×12  │  MS5611 Baro    │
└────────────────────────────────────────────────────────────────┘
```

---

## Repository Structure

```
ATLAS/
├── .github/workflows/ci.yml          ← Full CI: Python + C++ + SIL scenarios
├── src/
│   ├── cpp/
│   │   ├── include/                  ← C++ headers (complete, all datasheet constants)
│   │   │   ├── atlas_hardware.hpp    ← Single source of truth for all HW limits
│   │   │   ├── imu_driver.hpp        ← HG4930 + ICM-42688-P + triple-redundant voting
│   │   │   ├── gnss_driver.hpp       ← ZED-F9P UBX protocol + MS5611 baro
│   │   │   ├── actuator_controller.hpp ← Merlin 1D + Grid Fin + RCS (all clamped)
│   │   │   ├── flight_computer.hpp   ← 1 kHz control loop definition
│   │   │   ├── safety_gate.hpp       ← ARVS C++ gate (< 0.5 ms per call)
│   │   │   └── watchdog.hpp          ← 50 ms hardware watchdog
│   │   ├── src/                      ← C++ implementations (complete)
│   │   │   ├── flight_computer.cpp   ← EKF, 3-thread model, phase logic
│   │   │   ├── imu_driver.cpp        ← ODR compliance, Allan variance
│   │   │   ├── gnss_driver.cpp       ← UBX-NAV-PVT parser, ECEF conversion
│   │   │   ├── actuator_controller.cpp ← CAN bus + FPGA dispatch stubs
│   │   │   ├── safety_gate.cpp       ← Audit ring buffer, extended structural check
│   │   │   └── main_fc.cpp           ← Flight computer binary entry point
│   │   └── ros2/                     ← ROS2 nodes (optional)
│   │       ├── flight_computer_node.cpp
│   │       └── safety_gate_node.cpp
│   └── python/
│       ├── arvs_bridge/
│       │   └── arvs_authority.py     ← ATLAS→ARVS bridge (single authority interface)
│       └── atlas/
│           ├── main.py               ← Top-level entry point
│           ├── guidance/
│           │   └── guidance_core.py  ← 10 Hz trajectory generation + ARVS gating
│           ├── control/
│           │   └── control_core.py   ← PID/LQR + TVC + grid fins + RCS
│           ├── mission/
│           │   └── mission_manager.py ← 14-phase ARVS-gated timeline
│           ├── sensors/
│           │   └── sensor_processor.py ← IMU/GNSS/baro calibration & validation
│           ├── integration/
│           │   └── telemetry_bridge.py ← C++↔Python IPC (SHM + FDR)
│           └── config/
│               ├── vehicles/falcon9_class.py
│               └── missions/leo_cargo_mission.py
├── simulation/
│   └── run_simulation.py             ← Full SIL harness (nominal / abort / degrade)
├── tests/
│   ├── test_arvs_integration.py      ← ARVS authority: grant, deny, safe_hold, token
│   ├── test_sensor_processor.py      ← IMU noise model, GNSS validation, baro ISA
│   ├── test_control_core.py          ← PID, TVC slew, throttle limits, RCS firing
│   ├── test_guidance.py              ← Trajectory, max-Q, propellant, ISA model
│   ├── test_imu_driver.cpp           ← C++ Allan variance compliance
│   └── test_safety_gate.cpp          ← C++ gate rejection + clamping
├── scripts/
│   └── validate_hardware_constants.py ← CI: verifies all HPP constants vs datasheets
├── docs/
│   ├── architecture.md
│   └── hardware_specs.md
├── CMakeLists.txt
└── requirements.txt
```

---

## Hardware Sensor Specifications

Every constant in `atlas_hardware.hpp` traces to a manufacturer datasheet:

| Sensor | Part | Key Spec | Datasheet |
|--------|------|----------|-----------|
| Primary IMU (×2) | Honeywell HG4930 | 0.005°/hr bias, 0.004°/√hr ARW | DS-HG4930 Rev D |
| Redundant IMU | InvenSense ICM-42688-P | 0.0028°/s/√Hz gyro NSD | ICM-42688-P Rev 1.5 |
| GNSS | u-blox ZED-F9P | RTK 0.010 m CEP, 0.050 m/s vel | UBX-17051259 Rev 2.0 |
| Barometer | TE MS5611-01BA03 | 10 cm altitude resolution | MS5611 AN520 |
| Star Tracker | Bradford ASTRO APS | 2 arcsec accuracy (3σ) | APS-ST-001 Rev 2.1 |
| LiDAR | Velodyne VLP-16 | ±3 cm, 100 m range | VLP-16 Rev F |
| Engine | Merlin 1D+ | 845 kN SL, 40% min throttle, ±5° TVC | SpaceX Public |
| Grid Fins | Falcon 9 class | 400°/s slew, ±90° deflection | SpaceX Engineering |
| RCS | Moog DST-80 class | 22 N, 10 ms min pulse, N₂ prop | Moog DST-80 |
| Thermocouple | Omega K-Type | ±0.75% FS, -200°C to +1372°C | NIST Monograph 175 |

---

## ARVS Axiom Set

All 18+ axioms validated on every command cycle:

| ID | Group | Enforcement in ATLAS |
|----|-------|----------------------|
| E1–E4 | Epistemic | Confidence propagation; 500 ms token expiry; GNSS fix validation |
| U1–U3 | Uncertainty | conf < 0.70 → SAFE_HOLD; IMU inter-sensor spread tracking |
| A1–A4 | Authority | No self-authorization; SAFE_HOLD blocks 100% of actions |
| C1–C2 | Consequence | Risk quantifier; irreversible actions require conf ≥ 0.95 |
| R1–R2 | Refusal | Autonomous safe degradation; watchdog triggers SAFE_HOLD |
| Z | Closure | Zero bypass paths — every actuator gated through ARVSSafetyGate |

---

## Quick Start

### Python Simulation
```bash
pip install -r requirements.txt

# Nominal 200s launch simulation
cd src/python
python ../../simulation/run_simulation.py --duration 200

# Abort-at-max-Q scenario
python ../../simulation/run_simulation.py --scenario abort_at_max_q

# Sensor degradation scenario (triggers ARVS U1)
python ../../simulation/run_simulation.py --scenario sensor_degrade

# Full ATLAS system with verbose ARVS logging
python atlas/main.py --mode simulation --arvs-verbose
```

### C++ Flight Computer
```bash
cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --parallel

# Run simulation mode (60s)
./build/atlas_fc --sim --verbose --duration 60

# Run unit tests
cd build && ctest --output-on-failure
```

### All Python Tests
```bash
cd src/python
python -m pytest ../../tests/ -v --tb=short

# Hardware constant validation (CI script)
python scripts/validate_hardware_constants.py
```

---

## Real-Time Performance

| Parameter | Value | Constant |
|-----------|-------|----------|
| Control loop | 1 kHz | `hw::FLIGHT_COMPUTER::CONTROL_LOOP_PERIOD_MS = 1.0` |
| Guidance loop | 10 Hz | `hw::FLIGHT_COMPUTER::GUIDANCE_LOOP_PERIOD_MS = 100.0` |
| Watchdog timeout | 50 ms | `hw::FLIGHT_COMPUTER::WATCHDOG_TIMEOUT_MS = 50.0` |
| ARVS gate latency | < 0.5 ms | In-process C++, no IPC on hot path |
| Authority token validity | 500 ms | `ARVSAuthority.TOKEN_VALIDITY_S = 0.5` |
| Max actuator latency | < 1 ms | Direct CAN/FPGA register write |

---

## RTOS Deployment

ATLAS v2 is RTOS-ready. Tested configurations:

| RTOS | Thread Config | Notes |
|------|--------------|-------|
| FreeRTOS | `control_task` priority 31 (max), 4 kB stack | Tick rate ≥ 10 kHz |
| RTEMS | Rate monotonic scheduler | NASA preferred for space |
| VxWorks | `taskSpawn` priority 255 | Used in F-22, Boeing 787 |
| Linux RT (PREEMPT_RT) | `SCHED_FIFO` priority 99 | HIL bench use only |

---

## Contributing

1. All hardware constants **must** have a datasheet citation comment in `atlas_hardware.hpp`
2. All actuator commands **must** pass through `ARVSSafetyGate::check()` — no exceptions
3. New axioms must be implemented in **both** `arvs_authority.py` and `safety_gate.hpp`
4. Run `python scripts/validate_hardware_constants.py` before every PR
5. New Python modules must have pytest coverage ≥ 80%
6. No `std::random` or `random.random()` in any sensor path — use Allan variance models

---
---

## Citation

If you use ATLAS in your research please cite:
```bibtex
@software{rathi2026atlas,
  author  = {Rathi, Chirag},
  title   = {ATLAS: Autonomous Launch and 
             Termination Architecture System},
  year    = {2026},
  doi     = {10.5281/zenodo.19039518},
  url     = {https://doi.org/10.5281/zenodo.19039518},
  license = {Apache-2.0},
  orcid   = {0009-0008-1682-4369}
}
```

**ATLAS depends on ARVS as its authority layer.
Please also cite ARVS:**
```bibtex
@software{rathi2026arvs,
  author  = {Rathi, Chirag},
  title   = {ARVS: Adaptive Robust 
             Verification System},
  year    = {2026},
  doi     = {10.5281/zenodo.19023905},
  url     = {https://doi.org/10.5281/zenodo.19023905},
  license = {Apache-2.0},
  orcid   = {0009-0008-1682-4369}
}
```

**Chirag Rathi** — Independent Researcher
ORCID: 0009-0008-1682-4369

chiragrathiresearcher@gmail.com

Apache-2.0 License
