"""
test_arvs_integration.py
Integration tests — ATLAS guidance + ARVS authority layer.
"""

import asyncio
import pytest
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent.parent / "src/python"))

from arvs_bridge.arvs_authority import (
    ARVSAuthority, ProposedAction, ActionType, PermissionStatus
)


@pytest.fixture
async def authority():
    auth = ARVSAuthority("TEST_VEHICLE")
    await auth.initialize()
    return auth


@pytest.mark.asyncio
async def test_normal_engine_command_granted():
    auth = ARVSAuthority("TEST_V1")
    await auth.initialize()

    action = ProposedAction(
        action_type=ActionType.ENGINE_THROTTLE,
        parameters={"throttle": 0.8, "max_torque": 7607000.0,
                    "power_required": 100.0, "duration": 0.1},
        mission_phase="LIFTOFF"
    )
    state = {
        "temperature": 300.0, "battery_level": 0.85,
        "confidence": 0.98, "power_consumption": 100.0,
        "position": [0, 0, 0], "velocity": [0, 0, 0],
        "orientation": [1, 0, 0, 0], "angular_velocity": [0, 0, 0]
    }
    result = await auth.request_permission(action, state)
    assert result.permitted, f"Should be granted, got: {result.denial_reason}"


@pytest.mark.asyncio
async def test_irreversible_denied_low_confidence():
    auth = ARVSAuthority("TEST_V2")
    await auth.initialize()

    action = ProposedAction(
        action_type=ActionType.STAGE_SEPARATION,
        parameters={"duration": 1.0},
        is_irreversible=True,
        mission_phase="MECO"
    )
    state = {
        "temperature": 300.0, "battery_level": 0.85,
        "confidence": 0.80,   # Below 0.95 required for irreversible
        "power_consumption": 100.0,
        "position": [0, 0, 0], "velocity": [0, 0, 100],
        "orientation": [1, 0, 0, 0], "angular_velocity": [0, 0, 0]
    }
    result = await auth.request_permission(action, state)
    assert not result.permitted, "Irreversible action with conf=0.80 should be denied"
    assert "C2" in result.denial_reason or "irreversible" in result.denial_reason.lower()


@pytest.mark.asyncio
async def test_safe_hold_blocks_all_actions():
    auth = ARVSAuthority("TEST_V3")
    await auth.initialize()
    auth._mode = "SAFE_HOLD"

    action = ProposedAction(
        action_type=ActionType.ENGINE_THROTTLE,
        parameters={"throttle": 0.5},
        mission_phase="LIFTOFF"
    )
    result = await auth.request_permission(action)
    assert not result.permitted, "SAFE_HOLD must block all actions"
    assert result.status == PermissionStatus.DENIED


@pytest.mark.asyncio
async def test_authority_token_issued_on_grant():
    auth = ARVSAuthority("TEST_V4")
    await auth.initialize()

    action = ProposedAction(
        action_type=ActionType.ENGINE_THROTTLE,
        parameters={"throttle": 0.7, "max_torque": 1e6,
                    "power_required": 100.0, "duration": 0.1},
    )
    state = {
        "temperature": 295.0, "battery_level": 0.9,
        "confidence": 0.99, "power_consumption": 80.0,
        "position": [0, 0, 1000], "velocity": [0, 100, 200],
        "orientation": [1, 0, 0, 0], "angular_velocity": [0, 0, 0]
    }
    result = await auth.request_permission(action, state)
    if result.permitted:
        assert result.authority_token != 0, "Granted permission must include authority token"
        assert result.valid_until > 0, "Token must have expiry"


@pytest.mark.asyncio
async def test_statistics_tracked():
    auth = ARVSAuthority("TEST_V5")
    await auth.initialize()

    action_ok = ProposedAction(
        action_type=ActionType.ENGINE_THROTTLE,
        parameters={"throttle": 0.5, "max_torque": 1e6,
                    "power_required": 50.0, "duration": 0.1},
    )
    good_state = {
        "temperature": 290.0, "battery_level": 0.9,
        "confidence": 0.99, "power_consumption": 50.0,
        "position": [0, 0, 0], "velocity": [0, 0, 0],
        "orientation": [1, 0, 0, 0], "angular_velocity": [0, 0, 0]
    }
    # Force safe hold to get denials
    auth._mode = "SAFE_HOLD"
    for _ in range(3):
        await auth.request_permission(action_ok, good_state)

    stats = auth.get_statistics()
    assert stats["request_count"] == 3
    assert stats["deny_count"] == 3
    assert abs(stats["deny_rate"] - 1.0) < 0.01
