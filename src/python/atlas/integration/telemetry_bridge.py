"""
telemetry_bridge.py
ATLAS Telemetry Bridge — C++ ↔ Python IPC Layer

Bridges the 1 kHz C++ flight computer state to the 10 Hz Python guidance layer.
Uses POSIX shared memory for low-latency, lock-free state transfer.

Architecture:
    C++ FlightComputer (1 kHz)
        │ writes VehicleState to shared memory ring buffer
        ▼
    TelemetryBridge.read_latest_state()
        │ returns most recent VehicleState as Python dict
        ▼
    GuidanceCore / ARVSAuthority (10 Hz)

For the Python guidance → C++ control path, TelemetryBridge serializes
GuidanceCommand and writes it to a command shared memory segment,
which the C++ guidance_thread polls at 10 Hz.
"""

from __future__ import annotations

import time
import struct
import logging
import json
from dataclasses import dataclass, asdict
from typing import Optional, Dict, Any

import numpy as np

logger = logging.getLogger(__name__)


# ─────────────────────────────────────────────────────────────────
#  Shared memory layout (matches C++ VehicleState struct)
#
#  Offset  Size  Field
#  0       8     mission_time_s (double)
#  8       24    position_m (3× double: x,y,z)
#  32      24    velocity_ms (3× double: x,y,z)
#  56      32    attitude (4× double: w,x,y,z)
#  88      24    angular_rate_rads (3× double)
#  112     8     total_mass_kg (double)
#  120     8     propellant_remaining_kg (double)
#  128     8     current_q_pa (double)
#  136     8     engine_temp_k (double)
#  144     8     battery_soc (double)
#  152     8     nav_confidence (double)
#  160     1     phase (uint8)
#  161     1     arvs_mode (uint8)
#  162     2     padding
#  164     4     checksum (uint32)
#  = 168 bytes total
# ─────────────────────────────────────────────────────────────────

VEHICLE_STATE_FORMAT = "=d3d3d4d3dddddddBBHI"  # 168 bytes
VEHICLE_STATE_SIZE   = struct.calcsize(VEHICLE_STATE_FORMAT)

GUIDANCE_CMD_FORMAT  = "=d ddd dddd 12? B? d Q"
# Fields: timestamp, throttle, gimbal_pitch, gimbal_yaw,
#         fin1..fin4, rcs0..rcs11, requested_phase, phase_change, valid_for, token


class TelemetryBridge:
    """
    IPC bridge between C++ flight computer and Python guidance layer.

    In simulation: uses an in-process dict (no shared memory needed).
    In HIL/production: uses POSIX shared memory via /dev/shm.
    """

    SHM_STATE_NAME   = "/atlas_vehicle_state"
    SHM_CMD_NAME     = "/atlas_guidance_cmd"
    SHM_SIZE         = 4096   # 1 page

    def __init__(self, sim_mode: bool = True):
        self._sim_mode = sim_mode
        self._sim_state: Dict[str, Any] = self._default_state()
        self._cmd_count  = 0
        self._read_count = 0
        logger.info(f"TelemetryBridge init — mode={'simulation' if sim_mode else 'hardware'}")

    @staticmethod
    def _default_state() -> Dict[str, Any]:
        """Return a default ground state for simulation initialization."""
        return {
            "timestamp":          time.time(),
            "mission_time_s":     0.0,
            "position":           [0.0, 0.0, 0.0],
            "velocity":           [0.0, 0.0, 0.0],
            "orientation":        [1.0, 0.0, 0.0, 0.0],
            "angular_velocity":   [0.0, 0.0, 0.0],
            "altitude_m":         0.0,
            "velocity_ms":        0.0,
            "total_mass_kg":      549054.0,
            "propellant_kg":      488370.0,
            "q_pa":               0.0,
            "temperature_k":      300.0,
            "battery_soc":        1.0,
            "nav_confidence":     1.0,
            "flight_phase":       0,     # PRE_LAUNCH
            "arvs_mode":          0,     # NORMAL
            "prop_fraction":      1.0,
            "power_consumption":  100.0,
        }

    def update_sim_state(self, state: Dict[str, Any]) -> None:
        """Called by GuidanceCore to update simulated vehicle state."""
        self._sim_state.update(state)
        self._sim_state["timestamp"] = time.time()

    def read_latest_state(self) -> Dict[str, Any]:
        """
        Read the most recent vehicle state.
        Simulation: returns in-process state dict.
        Hardware: deserializes from POSIX shared memory.
        """
        self._read_count += 1

        if self._sim_mode:
            return dict(self._sim_state)

        # Hardware path: read from shared memory
        try:
            return self._read_shm_state()
        except Exception as e:
            logger.error(f"SHM read failed: {e}")
            return dict(self._sim_state)

    def send_guidance_command(self, cmd: Dict[str, Any]) -> bool:
        """
        Send guidance command to C++ flight computer.
        Simulation: stores in local buffer for inspection.
        Hardware: writes to shared memory command segment.
        """
        self._cmd_count += 1

        if self._sim_mode:
            # Validate command structure
            required = ["throttle_cmd", "gimbal_pitch_cmd_rad", "gimbal_yaw_cmd_rad"]
            missing = [k for k in required if k not in cmd]
            if missing:
                logger.error(f"Guidance command missing fields: {missing}")
                return False
            return True

        # Hardware path: write to shared memory
        try:
            return self._write_shm_cmd(cmd)
        except Exception as e:
            logger.error(f"SHM write failed: {e}")
            return False

    def _read_shm_state(self) -> Dict[str, Any]:
        """Read VehicleState from POSIX shared memory (/dev/shm/atlas_vehicle_state)."""
        shm_path = f"/dev/shm{self.SHM_STATE_NAME}"
        try:
            with open(shm_path, "rb") as f:
                data = f.read(VEHICLE_STATE_SIZE)
            if len(data) < VEHICLE_STATE_SIZE:
                raise ValueError(f"SHM too short: {len(data)} < {VEHICLE_STATE_SIZE}")

            fields = struct.unpack(VEHICLE_STATE_FORMAT, data[:VEHICLE_STATE_SIZE])
            return {
                "mission_time_s":    fields[0],
                "position":          list(fields[1:4]),
                "velocity":          list(fields[4:7]),
                "orientation":       list(fields[7:11]),
                "angular_velocity":  list(fields[11:14]),
                "total_mass_kg":     fields[14],
                "propellant_kg":     fields[15],
                "q_pa":              fields[16],
                "temperature_k":     fields[17],
                "battery_soc":       fields[18],
                "nav_confidence":    fields[19],
                "flight_phase":      fields[20],
                "arvs_mode":         fields[21],
                "timestamp":         time.time(),
            }
        except FileNotFoundError:
            logger.warning("SHM state file not found — C++ FC not running?")
            return self._default_state()

    def _write_shm_cmd(self, cmd: Dict[str, Any]) -> bool:
        """Write GuidanceCommand to POSIX shared memory."""
        shm_path = f"/dev/shm{self.SHM_CMD_NAME}"
        payload = json.dumps(cmd).encode("utf-8")
        try:
            with open(shm_path, "wb") as f:
                f.write(struct.pack("=I", len(payload)))
                f.write(payload)
            return True
        except OSError as e:
            logger.error(f"Cannot write guidance command: {e}")
            return False

    def get_statistics(self) -> Dict[str, Any]:
        return {
            "cmd_count":  self._cmd_count,
            "read_count": self._read_count,
            "mode":       "simulation" if self._sim_mode else "hardware",
        }


# ─────────────────────────────────────────────────────────────────
#  Flight Data Recorder — logs all state transitions to file
# ─────────────────────────────────────────────────────────────────

class FlightDataRecorder:
    """
    Records VehicleState at each guidance tick to a JSON-lines file.
    Enables post-flight analysis and anomaly investigation.
    """

    def __init__(self, output_path: str = "/tmp/atlas_fdr.jsonl"):
        self._path     = output_path
        self._file     = None
        self._count    = 0
        self._start_t  = time.time()

    def start(self) -> None:
        import os
        os.makedirs(os.path.dirname(self._path) or ".", exist_ok=True)
        self._file = open(self._path, "w")
        logger.info(f"FDR recording to {self._path}")

    def record(self, state: Dict[str, Any], phase: str,
               arvs_stats: Optional[Dict] = None) -> None:
        if not self._file:
            return

        record = {
            "t":     state.get("mission_time_s", 0.0),
            "phase": phase,
            "alt":   state.get("altitude_m", state.get("position", [0,0,0])[2] if "position" in state else 0),
            "vel":   state.get("velocity_ms", 0.0),
            "prop":  state.get("propellant_kg", 0.0),
            "q":     state.get("q_pa", 0.0),
            "conf":  state.get("nav_confidence", 0.0),
            "bat":   state.get("battery_soc", 0.0),
        }
        if arvs_stats:
            record["arvs_deny_rate"] = arvs_stats.get("deny_rate", 0.0)

        self._file.write(json.dumps(record) + "\n")
        self._count += 1

        # Flush every 100 records
        if self._count % 100 == 0:
            self._file.flush()

    def stop(self) -> None:
        if self._file:
            self._file.flush()
            self._file.close()
            self._file = None
        logger.info(f"FDR stopped — {self._count} records written to {self._path}")
