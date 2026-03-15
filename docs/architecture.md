# ATLAS Architecture

## System Layers

```
┌────────────────────────────────────────────────────────────┐
│  MISSION LAYER (Python, 1 Hz)                              │
│  MissionManager → phase transitions via ARVS               │
├────────────────────────────────────────────────────────────┤
│  GUIDANCE LAYER (Python, 10 Hz)                            │
│  GuidanceCore → trajectory proposals → ARVS approval       │
├────────────────────────────────────────────────────────────┤
│  ╔══════════════════════════════════════════════════════╗  │
│  ║  ARVS AUTHORITY LAYER (Python + C++, all rates)     ║  │
│  ║                                                      ║  │
│  ║  ARVSAuthority (Python)  ←→  ARVSSafetyGate (C++)   ║  │
│  ║  Axiom Validator         ←→  AxiomValidator (C++)    ║  │
│  ║  HAL TelemetryBus        ←→  Sensor drivers          ║  │
│  ╚══════════════════════════════════════════════════════╝  │
├────────────────────────────────────────────────────────────┤
│  CONTROL LAYER (C++, 1 kHz real-time)                      │
│  FlightComputer → IMU/GNSS read → actuator commands        │
├────────────────────────────────────────────────────────────┤
│  HARDWARE LAYER                                            │
│  HG4930 IMU × 3 │ ZED-F9P GNSS │ MS5611 Baro              │
│  Merlin 1D Engine │ Grid Fins │ RCS Thrusters              │
└────────────────────────────────────────────────────────────┘
```

## ARVS Authority Model

ATLAS cannot self-authorize **any** action.
Every proposed action flows:

```
ATLAS subsystem
    │
    ▼ ProposedAction
ARVSAuthority.request_permission()
    │
    ├─► Axiom validation (E1–Z, all 18+ axioms)
    ├─► Physical safety check (thermal, power, structural)
    ├─► Confidence check (E2: unknown = max risk)
    ├─► Irreversible action check (C2: requires conf ≥ 0.95)
    └─► Risk assessment (ARVS MVI/QUBO engine)
    │
    ▼ PermissionResult
    ├─ GRANTED → authority token issued → actuator command executed
    ├─ GRANTED_CLAMPED → modified parameters → clamped execution
    ├─ DENIED → SAFE_HOLD entered
    └─ DENIED_ABORT → flight termination
```

## Sensor Data Flow (No Random Values)

```
Hardware Sensor
    │ Raw ADC/SPI/UART read
    ▼
SensorDriver (C++)
    │ Allan variance noise model (datasheet-derived)
    │ Temperature compensation
    ▼
TelemetryBus (ARVS HAL)
    │ Immutable TelemetryFrame
    ▼
ARVS StateEstimator
    │ Kalman/Bayesian fusion
    ▼
ARVSAuthority
    │ Confidence-weighted state estimate
    ▼
GuidanceCore / MissionManager
```

## Hardware Datasheet Sources

| Sensor | Datasheet | Key Value Used |
|--------|-----------|----------------|
| HG4930 IMU | DS-HG4930 Rev D | ARW 0.004 °/√hr, bias 0.005 °/hr |
| ICM-42688-P | ICM42688 Rev 1.5 | Gyro NSD 0.0028 °/s/√Hz |
| ZED-F9P GNSS | UBX-17051259 | RTK CEP 0.010 m, vel 0.050 m/s |
| MS5611 Baro | MS5611 AN520 | Resolution 0.012 mbar, 10 cm alt |
| Merlin 1D | SpaceX public | 845 kN SL, 40% min throttle |
| Grid Fin | SpaceX public | 400 °/s slew, ±90° deflection |
| Cold Gas RCS | Moog DST-80 | 22 N, 10 ms min pulse |

## C++ Real-Time Guarantees

- Control loop: **1 kHz** (1 ms period)
- Watchdog timeout: **50 ms** (5× control period)
- ARVS safety gate call: **<0.5 ms** (in-process, no IPC)
- Max GNSS update latency: **50 ms** (20 Hz × 1 period)
- Actuator command latency: **<1 ms** (direct register write)

## File Structure

```
ATLAS/
├── src/cpp/include/
│   ├── atlas_hardware.hpp    ← All datasheet constants
│   ├── imu_driver.hpp        ← HG4930 + ICM-42688-P
│   ├── gnss_driver.hpp       ← ZED-F9P + MS5611
│   ├── actuator_controller.hpp ← Merlin 1D + Grid Fin + RCS
│   ├── flight_computer.hpp   ← Top-level 1 kHz loop
│   ├── safety_gate.hpp       ← ARVS C++ safety gate
│   └── watchdog.hpp          ← Hardware watchdog
├── src/python/
│   ├── arvs_bridge/
│   │   └── arvs_authority.py ← ARVS Python authority interface
│   └── atlas/
│       ├── main.py           ← Entry point
│       ├── guidance/         ← Trajectory generation (10 Hz)
│       ├── mission/          ← Phase management (1 Hz)
│       └── config/           ← Vehicle + mission params
└── tests/                    ← C++ + Python test suites
```
