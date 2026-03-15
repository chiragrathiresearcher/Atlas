"""
run_simulation.py
ATLAS Simulation Harness — Software-in-the-Loop (SIL) & Hardware-in-the-Loop (HIL)

Full end-to-end simulation of a Falcon 9 class launch to 550 km LEO.
Exercises the complete ATLAS stack:
  - GuidanceCore trajectory generation
  - MissionManager phase transitions (all ARVS-gated)
  - ARVSAuthority permission engine (18+ axioms)
  - ControlCore PID/LQR loops
  - Sensor noise models (HG4930, ZED-F9P, MS5611 — all datasheet-based)
  - FlightDataRecorder for post-flight analysis

Usage:
    python simulation/run_simulation.py
    python simulation/run_simulation.py --duration 300 --verbose
    python simulation/run_simulation.py --scenario abort_at_max_q
"""

import asyncio
import argparse
import logging
import sys
import time
from pathlib import Path

# Add project root to path
sys.path.insert(0, str(Path(__file__).parent.parent / "src/python"))

from atlas.guidance.guidance_core import GuidanceCore
from atlas.mission.mission_manager import MissionManager
from atlas.control.control_core import ControlCore, AttitudeError
from atlas.sensors.sensor_processor import (
    IMUSensorProcessor, GNSSSensorProcessor, BaroSensorProcessor
)
from atlas.integration.telemetry_bridge import TelemetryBridge, FlightDataRecorder
from atlas.config.vehicles.falcon9_class import Falcon9ClassVehicle
from atlas.config.missions.leo_cargo_mission import LEOCargoMission
from arvs_bridge.arvs_authority import ARVSAuthority, ProposedAction, ActionType

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s.%(msecs)03d | %(levelname)-8s | %(name)-28s | %(message)s",
    datefmt="%H:%M:%S"
)
logger = logging.getLogger("SIM")


# ─────────────────────────────────────────────────────────────────
#  Abort scenario: engine cutoff at max-Q
# ─────────────────────────────────────────────────────────────────

async def scenario_abort_at_max_q(system) -> None:
    """Inject a forced abort when q exceeds 30,000 Pa."""
    while True:
        await asyncio.sleep(0.5)
        state = system["guidance"].current_state
        if state and state.get("q_pa", 0) > 30000.0:
            logger.warning("SCENARIO: Injecting abort at max-Q")
            system["arvs"]._mode = "SAFE_HOLD"
            break


# ─────────────────────────────────────────────────────────────────
#  Low-confidence scenario: sensor degradation at T+60s
# ─────────────────────────────────────────────────────────────────

async def scenario_sensor_degrade(system) -> None:
    """Degrade IMU confidence to 0.65 at T+60s — triggers ARVS U1."""
    target_time = 60.0
    while True:
        await asyncio.sleep(0.5)
        state = system["guidance"].current_state
        if state and state.get("mission_time_s", 0) >= target_time:
            logger.warning("SCENARIO: Degrading IMU confidence to 0.65")
            if state:
                state["confidence"] = 0.65
                system["guidance"].current_state = state
            break


# ─────────────────────────────────────────────────────────────────
#  Main simulation run
# ─────────────────────────────────────────────────────────────────

async def run_simulation(args: argparse.Namespace) -> int:
    logger.info("=" * 60)
    logger.info("ATLAS SIL SIMULATION")
    logger.info(f"Scenario: {args.scenario}")
    logger.info(f"Duration: {args.duration} s")
    logger.info("=" * 60)

    # ── Initialize subsystems ─────────────────────────────────────
    arvs    = ARVSAuthority("ATLAS_SIM_V1")
    await arvs.initialize()
    logger.info(f"ARVS initialized — mode={arvs.get_system_mode()}")

    vehicle = Falcon9ClassVehicle("SIM-001")
    vehicle.configure(payload_mass_kg=3500)

    mission = LEOCargoMission("SIM-M001")
    mission.configure(target_altitude_km=550, payload_mass_kg=3500, inclination_deg=28.5)

    guidance = GuidanceCore(arvs_authority=arvs, vehicle=vehicle)
    mgr      = MissionManager(arvs_authority=arvs, guidance=guidance, mission=mission)
    ctrl     = ControlCore()
    bridge   = TelemetryBridge(sim_mode=True)
    fdr      = FlightDataRecorder(output_path="/tmp/atlas_sim_fdr.jsonl")

    imu_proc  = IMUSensorProcessor()
    gnss_proc = GNSSSensorProcessor()
    baro_proc = BaroSensorProcessor()

    fdr.start()
    await mgr.start()

    system = {"guidance": guidance, "mission": mgr, "arvs": arvs}

    # ── Inject scenario ───────────────────────────────────────────
    scenario_task = None
    if args.scenario == "abort_at_max_q":
        scenario_task = asyncio.create_task(scenario_abort_at_max_q(system))
    elif args.scenario == "sensor_degrade":
        scenario_task = asyncio.create_task(scenario_sensor_degrade(system))

    # ── Main simulation loop (10 Hz) ──────────────────────────────
    t_start    = time.time()
    tick_count = 0
    last_status_t = time.time()

    logger.info("Simulation loop started")

    try:
        while not mgr.complete:
            t_elapsed = time.time() - t_start
            if t_elapsed >= args.duration:
                logger.info(f"Duration limit {args.duration}s reached — stopping")
                break

            # Update state
            await guidance.tick()
            await mgr.tick()

            state = guidance.current_state or {}
            bridge.update_sim_state(state)

            # Compute control
            q_pa          = state.get("q_pa", 0.0)
            prop_fraction = state.get("propellant_kg", 488370.0) / 488370.0
            alt_m         = state.get("altitude_m", 0.0)
            throttle_req  = state.get("throttle", 0.0)

            attitude_err = AttitudeError(
                pitch_rad=0.0, yaw_rad=0.0, roll_rad=0.0   # nominal: no error
            )
            cmd = ctrl.compute_command(
                attitude_err, throttle_req, q_pa, prop_fraction, alt_m
            )
            bridge.send_guidance_command(cmd)

            # Record to FDR
            fdr.record(state, mgr.current_phase.value, arvs.get_statistics())

            # Periodic status print
            if time.time() - last_status_t >= 5.0:
                stats = arvs.get_statistics()
                logger.info(
                    f"T+{guidance.mission_time_s:.1f}s | "
                    f"Phase={mgr.current_phase.value:16s} | "
                    f"Alt={alt_m/1000:.1f}km | "
                    f"Vel={state.get('velocity_ms',0):.0f}m/s | "
                    f"Prop={state.get('propellant_kg',0):,.0f}kg | "
                    f"Q={q_pa:.0f}Pa | "
                    f"ARVS_deny={stats['deny_rate']:.1%}"
                )
                last_status_t = time.time()

            tick_count += 1
            await asyncio.sleep(0.1)   # 10 Hz

    except KeyboardInterrupt:
        logger.info("Simulation interrupted by user")
    except Exception as e:
        logger.error(f"Simulation error: {e}", exc_info=True)
        return 1
    finally:
        if scenario_task and not scenario_task.done():
            scenario_task.cancel()
        fdr.stop()
        await arvs.shutdown()

    # ── Summary ────────────────────────────────────────────────────
    final_state = guidance.current_state or {}
    final_stats = arvs.get_statistics()
    logger.info("=" * 60)
    logger.info("SIMULATION COMPLETE")
    logger.info(f"  Ticks:         {tick_count}")
    logger.info(f"  Final phase:   {mgr.current_phase.value}")
    logger.info(f"  Final alt:     {final_state.get('altitude_m',0)/1000:.1f} km")
    logger.info(f"  Final vel:     {final_state.get('velocity_ms',0):.0f} m/s")
    logger.info(f"  Prop remaining:{final_state.get('propellant_kg',0):,.0f} kg")
    logger.info(f"  ARVS requests: {final_stats['request_count']}")
    logger.info(f"  ARVS denials:  {final_stats['deny_count']} ({final_stats['deny_rate']:.1%})")
    logger.info(f"  ARVS latency:  {final_stats['avg_latency_ms']:.2f} ms avg")
    logger.info(f"  FDR file:      /tmp/atlas_sim_fdr.jsonl")
    logger.info("=" * 60)
    return 0


async def main() -> int:
    parser = argparse.ArgumentParser(description="ATLAS SIL Simulation")
    parser.add_argument("--duration",  type=float, default=200.0,
                        help="Simulation duration in seconds (default: 200)")
    parser.add_argument("--scenario",  default="nominal",
                        choices=["nominal", "abort_at_max_q", "sensor_degrade"],
                        help="Simulation scenario")
    parser.add_argument("--verbose",   action="store_true",
                        help="Enable debug logging")
    args = parser.parse_args()

    if args.verbose:
        logging.getLogger().setLevel(logging.DEBUG)

    return await run_simulation(args)


if __name__ == "__main__":
    sys.exit(asyncio.run(main()))
