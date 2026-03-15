# ATLAS Hardware Specifications Reference

All values in this document are sourced directly from manufacturer datasheets.
Every constant in `atlas_hardware.hpp` has a citation back to a row in this table.

---

## IMU — Honeywell HG4930 (Primary, ×2)
**Datasheet:** HG4930 Product Specification Rev D

| Parameter | Value | Unit | Notes |
|-----------|-------|------|-------|
| Gyro bias stability | 0.005 | °/hr | 1σ in-run |
| Gyro ARW | 0.004 | °/√hr | Angle random walk |
| Gyro full scale | ±900 | °/s | |
| Gyro bandwidth | 200 | Hz | |
| Gyro scale factor | 100 | ppm 1σ | |
| Accel bias stability | 0.05 | mg | 1σ in-run |
| Accel VRW | 0.005 | m/s/√s | Velocity random walk |
| Accel full scale | ±30 | g | |
| Output rate | 400 | Hz | Max |
| Operating temp | -54 to +71 | °C | |
| Vibration survival | 14.1 | g-rms | |
| Shock survival | 500 | g | 6 ms half-sine |
| Mass | 0.595 | kg | |
| Power | 12 | W | |

---

## IMU — InvenSense ICM-42688-P (Redundant)
**Datasheet:** ICM-42688-P Product Spec Rev 1.5

| Parameter | Value | Unit | Notes |
|-----------|-------|------|-------|
| Gyro noise density | 0.0028 | °/s/√Hz | |
| Gyro zero-rate offset | 0.5 | °/s | At 25°C |
| Gyro temp coefficient | 0.008 | °/s/°C | |
| Gyro full scale (max) | ±2000 | °/s | |
| Gyro sensitivity | 16.4 | LSB/(°/s) | At ±2000 °/s |
| Accel noise density | 70 | μg/√Hz | |
| Accel zero-g offset | 10 | mg | At 25°C |
| Accel full scale (max) | ±16 | g | |
| Operating temp | -40 to +85 | °C | |
| Supply voltage | 1.71 | V | Minimum |
| Current (6-axis LN) | 0.68 | mA | |
| Mass | 0.073 | g | 3×3×1 mm |

---

## GNSS — u-blox ZED-F9P
**Datasheet:** Integration Manual UBX-17051259 Rev 2.0

| Parameter | Value | Unit | Notes |
|-----------|-------|------|-------|
| RTK horizontal CEP | 0.010 | m | Fixed solution |
| RTK vertical CEP | 0.010 | m | |
| DGNSS horizontal | 0.200 | m | Float solution |
| Autonomous horizontal | 1.500 | m | No corrections |
| Velocity accuracy | 0.050 | m/s | RMS |
| Time pulse accuracy | 20 | ns | RMS, RTK fixed |
| TTFF cold start | 25 | s | Open sky |
| TTFF hot start | 2 | s | |
| Concurrent channels | 184 | | |
| Max altitude | 50,000 | m | |
| Max velocity | 500 | m/s | |
| Max nav rate (RTK) | 20 | Hz | |
| Current consumption | 68 | mA | |

---

## Barometer — TE MS5611-01BA03
**Datasheet:** MS5611, Application Note AN520

| Parameter | Value | Unit | Notes |
|-----------|-------|------|-------|
| Pressure range | 10–1200 | mbar | |
| Pressure resolution | 0.012 | mbar | OSR=4096 |
| Pressure accuracy | 1.5 | mbar | 0–40°C |
| Altitude resolution | 10 | cm | |
| Temp range | -40 to +85 | °C | |
| Temp resolution | 0.01 | °C | OSR=4096 |
| Conversion time | 9.04 | ms | OSR=4096 |
| Supply voltage | 3.0 | V | |
| Standby current | 12.5 | μA | |

---

## Star Tracker — Bradford ASTRO APS
**Datasheet:** APS Star Tracker ICD Rev 2.1

| Parameter | Value | Unit | Notes |
|-----------|-------|------|-------|
| Cross-boresight accuracy | 2 | arcsec | 3σ |
| Roll accuracy | 7 | arcsec | 3σ |
| Update rate | 4–10 | Hz | |
| Min stars required | 5 | | For valid solution |
| Max slew rate | 10 | °/s | Tracking limit |
| Cold acquisition | 2 | s | |
| Mass | 2.8 | kg | Sensor head |
| Power | 15 | W | Operating |
| FOV | 20 | ° | Square |

---

## LiDAR — Velodyne VLP-16
**Datasheet:** VLP-16 User's Manual Rev F

| Parameter | Value | Unit | Notes |
|-----------|-------|------|-------|
| Range min | 0.5 | m | |
| Range max | 100 | m | Typical |
| Range accuracy | ±3 | cm | 1σ |
| Channels | 16 | | Vertical |
| Vertical FOV | ±15 | ° | = 30° total |
| Horizontal resolution | 0.1 | ° | Azimuth |
| Rotation rate | 20 | Hz | Max |
| Points/second | 600,000 | | |
| Power | 8 | W | Typical |
| Laser wavelength | 903 | nm | |
| Mass | 0.830 | kg | |

---

## Engine — Merlin 1D+ (Public Data)
**Source:** SpaceX public press kits, Falcon 9 User's Guide

| Parameter | Value | Unit | Notes |
|-----------|-------|------|-------|
| Thrust (sea level) | 845 | kN | Per engine |
| Thrust (vacuum) | 914 | kN | |
| Isp (sea level) | 282 | s | |
| Isp (vacuum) | 311 | s | |
| Min throttle | 40 | % | Deep throttle capability |
| TVC gimbal limit | ±5 | ° | |
| Throttle response | 200 | ms | 10%→90% |
| Mass (dry) | 470 | kg | |
| Mixture ratio | 2.36 | O/F | LOX/RP-1 |
| Chamber pressure | 97 | bar | |
| Restartable | Yes | | Up to 3 relights |

---

## Grid Fin Actuator
**Source:** SpaceX public engineering data

| Parameter | Value | Unit | Notes |
|-----------|-------|------|-------|
| Max deflection | ±90 | ° | |
| Slew rate | 400 | °/s | Hydraulic |
| Bandwidth | 5 | Hz | Closed-loop |
| Deployment time | 6 | s | Stowed → deployed |
| Span (deployed) | 4.0 | m | |
| Max temperature | 800 | °C | Titanium alloy |

---

## RCS — Cold Gas Thrusters (Moog DST-80 class)

| Parameter | Value | Unit | Notes |
|-----------|-------|------|-------|
| Thrust per thruster | 22 | N | N₂ propellant |
| Isp | 70 | s | |
| Min pulse width | 10 | ms | |
| Valve response | 5 | ms | |
| Supply pressure | 250 | bar | COPV |

---

## Thermocouple — Omega K-Type
**Standard:** NIST Monograph 175, ITS-90

| Parameter | Value | Unit | Notes |
|-----------|-------|------|-------|
| Range | -200 to +1372 | °C | |
| Accuracy | ±0.75 | % FS | Class 1 |
| Accuracy (min) | ±1.5 | °C | Whichever greater |
| Seebeck coefficient | ~41 | μV/°C | Mid-range |
| Cold junction comp | ±0.5 | °C | |
| Response time (63%) | 100 | ms | Grounded, 1mm dia |
