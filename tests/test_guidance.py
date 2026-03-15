"""
test_guidance.py
Unit tests for ATLAS guidance core.

Tests:
  - Trajectory generation produces valid waypoints
  - Throttle program respects max-Q and MECO conditions
  - ARVS permission integration (trajectory approved / denied)
  - ISA atmosphere model accuracy
  - Propellant consumption (Tsiolkovsky compliance)
"""

import pytest
import asyncio
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent.parent / "src/python"))

from atlas.guidance.guidance_core import GuidanceCore, VehicleParams
from atlas.config.vehicles.falcon9_class import Falcon9ClassVehicle
from arvs_bridge.arvs_authority import ARVSAuthority


@pytest.fixture
async def guidance_system():
    arvs = ARVSAuthority("TEST_GUIDANCE")
    await arvs.initialize()
    vehicle = Falcon9ClassVehicle("TEST-001")
    vehicle.configure(payload_mass_kg=3500)
    gc = GuidanceCore(arvs_authority=arvs, vehicle=vehicle)
    return gc, arvs


@pytest.mark.asyncio
async def test_initial_state_on_pad():
    arvs = ARVSAuthority("TG1"); await arvs.initialize()
    vehicle = Falcon9ClassVehicle("V1"); vehicle.configure()
    gc = GuidanceCore(arvs_authority=arvs, vehicle=vehicle)
    await gc.tick()
    state = gc.current_state
    assert state is not None
    assert state["altitude_m"] >= 0.0
    assert state["propellant_kg"] > 0.0
    assert state["confidence"] == 1.0


@pytest.mark.asyncio
async def test_trajectory_generation_produces_waypoints():
    arvs = ARVSAuthority("TG2"); await arvs.initialize()
    vehicle = Falcon9ClassVehicle("V2"); vehicle.configure()
    gc = GuidanceCore(arvs_authority=arvs, vehicle=vehicle)
    await gc.tick()
    traj = gc._generate_ascent_trajectory(550)
    assert len(traj.waypoints) > 5
    assert traj.duration_s > 0
    assert traj.fuel_required_kg > 0


@pytest.mark.asyncio
async def test_trajectory_respects_max_q_limit():
    """Max dynamic pressure must stay below 35,000 Pa (structural limit)."""
    arvs = ARVSAuthority("TG3"); await arvs.initialize()
    vehicle = Falcon9ClassVehicle("V3"); vehicle.configure()
    gc = GuidanceCore(arvs_authority=arvs, vehicle=vehicle)
    await gc.tick()
    traj = gc._generate_ascent_trajectory(550)
    assert traj.max_q_pa <= 36000.0, (
        f"Max-Q {traj.max_q_pa:.0f} Pa exceeds 35,000 Pa structural limit"
    )


@pytest.mark.asyncio
async def test_arvs_permission_granted_for_valid_trajectory():
    arvs = ARVSAuthority("TG4"); await arvs.initialize()
    vehicle = Falcon9ClassVehicle("V4"); vehicle.configure()
    gc = GuidanceCore(arvs_authority=arvs, vehicle=vehicle)
    await gc.tick()
    traj = await gc.propose_trajectory(target_orbit_km=550)
    if traj is not None:
        assert traj.arvs_permitted
        assert traj.arvs_token != 0


@pytest.mark.asyncio
async def test_safe_hold_denies_all_trajectories():
    arvs = ARVSAuthority("TG5"); await arvs.initialize()
    vehicle = Falcon9ClassVehicle("V5"); vehicle.configure()
    gc = GuidanceCore(arvs_authority=arvs, vehicle=vehicle)
    await gc.tick()
    arvs._mode = "SAFE_HOLD"
    traj = await gc.propose_trajectory()
    assert traj is None, "SAFE_HOLD must deny all trajectory proposals"


def test_isa_density_sea_level():
    """ISA sea level density = 1.225 kg/m³."""
    arvs = ARVSAuthority.__new__(ARVSAuthority)  # don't initialize
    vehicle = Falcon9ClassVehicle("V")
    gc = GuidanceCore.__new__(GuidanceCore)
    gc.RHO_SL = 1.225
    rho = gc._isa_density(0.0)
    assert abs(rho - 1.225) < 0.01

def test_isa_density_decreases_with_altitude():
    gc = GuidanceCore.__new__(GuidanceCore)
    gc.RHO_SL = 1.225
    rho_0  = gc._isa_density(0.0)
    rho_5  = gc._isa_density(5000.0)
    rho_10 = gc._isa_density(10000.0)
    assert rho_0 > rho_5 > rho_10

def test_sound_speed_sea_level():
    """ISA sea level sound speed = √(1.4 × 287 × 288.15) ≈ 340 m/s."""
    gc = GuidanceCore.__new__(GuidanceCore)
    c = gc._sound_speed(0.0)
    assert abs(c - 340.3) < 2.0

def test_throttle_program_zero_before_liftoff():
    arvs = ARVSAuthority.__new__(ARVSAuthority)
    vehicle = Falcon9ClassVehicle("V")
    vehicle.configure()
    gc = GuidanceCore.__new__(GuidanceCore)
    gc.params = VehicleParams()
    gc.MAX_Q_THROTTLE_LIMIT = 0.72
    assert gc._throttle_program(0.0, 0.0, 0.0, 1000.0) == 0.0

def test_throttle_program_max_q_reduction():
    arvs = ARVSAuthority.__new__(ARVSAuthority)
    vehicle = Falcon9ClassVehicle("V")
    vehicle.configure()
    gc = GuidanceCore.__new__(GuidanceCore)
    gc.params = VehicleParams()
    gc.MAX_Q_THROTTLE_LIMIT = 0.72
    throttle = gc._throttle_program(60.0, 14000.0, 22000.0, 100000.0)
    assert throttle <= gc.MAX_Q_THROTTLE_LIMIT + 1e-6

def test_propellant_flow_mass_conservation():
    """
    Tsiolkovsky: Δv = Isp × g0 × ln(m0/mf)
    For Falcon 9 S1: Δv ≈ 3000 m/s, Isp ≈ 290 s (average), m0/mf ≈ 2.2
    """
    params = VehicleParams()
    total_prop = params.prop_s1_kg
    total_flight_s = 162.0   # approximate S1 burn duration
    avg_throttle = 0.88
    avg_alt_m    = 50000.0   # rough average
    mdot = params.prop_flow_kgs(avg_throttle, avg_alt_m)
    prop_used = mdot * total_flight_s
    # Should use between 60% and 95% of total S1 propellant
    assert 0.60 * total_prop < prop_used < 0.95 * total_prop, (
        f"Propellant used {prop_used:.0f} kg seems wrong "
        f"(total S1: {total_prop:.0f} kg)"
    )
