#!/usr/bin/env python3
"""
ATLAS — Autonomous Launch & Termination Architecture System
Main entry point.

Integrates:
  - ARVS authority layer (non-bypassable)
  - C++ real-time flight computer (1 kHz)
  - Python guidance and mission planning (10 Hz)
  - Hardware sensors (HG4930 IMU, ZED-F9P GNSS, MS5611 baro)

Usage:
    python main.py --mode simulation
    python main.py --mode hil --vehicle falcon9_class
    python main.py --mode hil --arvs-verbose
"""

import asyncio
import argparse
import logging
import signal
import sys
import time
from pathlib import Path

# Add project root to path
sys.path.insert(0, str(Path(__file__).parent.parent.parent.parent))

from atlas.guidance.guidance_core import GuidanceCore
from atlas.mission.mission_manager import MissionManager
from atlas.config.vehicles.falcon9_class import Falcon9ClassVehicle
from atlas.config.missions.leo_cargo_mission import LEOCargoMission
from arvs_bridge.arvs_authority import ARVSAuthority, ProposedAction, ActionType

# Configure structured logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s.%(msecs)03d | %(levelname)-8s | %(name)-30s | %(message)s',
    datefmt='%H:%M:%S'
)
logger = logging.getLogger("ATLAS.MAIN")


class ATLASSystem:
    """
    Top-level ATLAS system orchestrator.
    Owns all subsystems and enforces ARVS authority at every step.
    """

    BANNER = """
╔══════════════════════════════════════════════════════════════╗
║   ATLAS — Autonomous Launch & Termination Architecture       ║
║   Authority Layer: ARVS v1.0                                 ║
║   Hardware: HG4930 IMU | ZED-F9P GNSS | MS5611 Baro         ║
╚══════════════════════════════════════════════════════════════╝
"""

    def __init__(self, args: argparse.Namespace):
        self.args = args
        self.shutdown_requested = False

        # ARVS authority (initialized first — nothing moves without it)
        self.arvs = ARVSAuthority(robot_id="ATLAS_V1")

        # Guidance and mission (initialized after ARVS)
        self.guidance: GuidanceCore | None = None
        self.mission:  MissionManager | None = None

        # Signal handlers
        signal.signal(signal.SIGINT,  self._handle_signal)
        signal.signal(signal.SIGTERM, self._handle_signal)

    def _handle_signal(self, sig, _frame):
        logger.info(f"Signal {sig} received — requesting shutdown")
        self.shutdown_requested = True

    async def initialize(self) -> bool:
        logger.info(self.BANNER)
        logger.info("=" * 60)
        logger.info("PHASE: SYSTEM INITIALIZATION")
        logger.info("=" * 60)

        # ── Step 1: Initialize ARVS (MUST succeed before anything else) ──
        logger.info("Initializing ARVS authority layer...")
        arvs_ok = await self.arvs.initialize()
        if not arvs_ok:
            logger.critical("ARVS failed to initialize — SAFE HOLD: no operations permitted")
            return False
        logger.info(f"✓ ARVS authority established | mode={self.arvs.get_system_mode()}")

        # ── Step 2: Vehicle configuration ─────────────────────────────────
        vehicle = Falcon9ClassVehicle("ATLAS-V1-001")
        vehicle.configure()
        logger.info(f"✓ Vehicle: {vehicle.name}")
        logger.info(f"  Liftoff mass:    {vehicle.liftoff_mass_kg:,.0f} kg")
        logger.info(f"  Payload to LEO:  {vehicle.payload_leo_kg:,} kg")
        logger.info(f"  Engines:         {vehicle.engine_count}× {vehicle.engine_model}")
        logger.info(f"  Stages:          {vehicle.stage_count}")

        # ── Step 3: Mission ────────────────────────────────────────────────
        mission = LEOCargoMission("M-001")
        mission.configure(
            target_altitude_km=550,
            payload_mass_kg=3500,
            inclination_deg=28.5
        )
        logger.info(f"✓ Mission: {mission.mission_id}")
        logger.info(f"  Target orbit:    {mission.target_altitude_km} km × {mission.inclination_deg}°")
        logger.info(f"  Payload:         {mission.payload_mass_kg} kg")

        # ── Step 4: Guidance and mission manager ───────────────────────────
        self.guidance = GuidanceCore(arvs_authority=self.arvs, vehicle=vehicle)
        self.mission  = MissionManager(arvs_authority=self.arvs,
                                       guidance=self.guidance,
                                       mission=mission)

        logger.info("✓ Guidance and mission layers initialized")
        logger.info("=" * 60)
        logger.info("System initialization COMPLETE — ARVS authority active")
        logger.info("=" * 60)
        return True

    async def run(self) -> int:
        """Main event loop. Runs until shutdown or abort."""
        if not self.guidance or not self.mission:
            logger.error("Not initialized")
            return 1

        logger.info("Starting ATLAS operations")
        await self.mission.start()

        last_status_t = time.time()
        status_interval_s = 5.0

        try:
            while not self.shutdown_requested and not self.mission.complete:
                # Print status periodically
                if time.time() - last_status_t >= status_interval_s:
                    await self._print_status()
                    last_status_t = time.time()

                # Guidance loop iteration (10 Hz)
                await self.guidance.tick()

                # Mission manager iteration
                await self.mission.tick()

                await asyncio.sleep(0.1)  # 10 Hz

        except asyncio.CancelledError:
            logger.info("Main loop cancelled")

        except Exception as exc:
            logger.error(f"Fatal error in main loop: {exc}", exc_info=True)
            return 1

        finally:
            await self.shutdown()

        return 0

    async def _print_status(self):
        state  = self.guidance.current_state
        phase  = self.mission.current_phase
        arvs_s = self.arvs.get_statistics()
        if not state:
            logger.info(f"[STATUS] Phase={phase} | Guidance not yet initialized")
            return

        logger.info(
            f"[STATUS] Phase={phase:18s} | "
            f"Alt={state.get('altitude_m', 0)/1000:.1f}km | "
            f"Vel={state.get('velocity_ms', 0):.0f}m/s | "
            f"Conf={state.get('confidence', 0):.3f} | "
            f"ARVS deny_rate={arvs_s['deny_rate']:.2%} | "
            f"Prop={state.get('propellant_kg', 0):,.0f}kg"
        )

    async def shutdown(self):
        logger.info("Initiating ATLAS shutdown...")
        if self.mission:
            await self.mission.stop()
        await self.arvs.shutdown()
        logger.info("ATLAS shutdown complete")


async def main():
    parser = argparse.ArgumentParser(description="ATLAS Flight System")
    parser.add_argument("--mode",        choices=["simulation", "hil"], default="simulation")
    parser.add_argument("--vehicle",     default="falcon9_class")
    parser.add_argument("--arvs-verbose",action="store_true")
    args = parser.parse_args()

    if args.arvs_verbose:
        logging.getLogger("ARVSAuthority").setLevel(logging.DEBUG)
        logging.getLogger("ARVS").setLevel(logging.DEBUG)

    system = ATLASSystem(args)

    if not await system.initialize():
        logger.critical("Initialization failed — aborting")
        return 1

    return await system.run()


if __name__ == "__main__":
    sys.exit(asyncio.run(main()))
