"""
mission_manager.py
ATLAS Mission Manager — Phase transitions gated by ARVS authority.

All phase transitions (LIFTOFF, MECO, STAGE SEP, PAYLOAD DEPLOY)
require explicit ARVS permission. Irreversible actions require
confidence ≥ 0.95 per ARVS Axiom C2.
"""

from __future__ import annotations

import asyncio
import logging
import time
from dataclasses import dataclass, field
from enum import Enum, auto
from typing import Any, Dict, List, Optional

from arvs_bridge.arvs_authority import (
    ARVSAuthority, ProposedAction, ActionType, PermissionStatus
)

logger = logging.getLogger("MissionManager")


class FlightPhase(Enum):
    PRE_LAUNCH      = "PRE_LAUNCH"
    IGNITION        = "IGNITION"
    LIFTOFF         = "LIFTOFF"
    MAX_Q           = "MAX_Q"
    MECO            = "MECO"
    STAGE_SEP       = "STAGE_SEP"
    UPPER_STAGE     = "UPPER_STAGE"
    PAYLOAD_DEPLOY  = "PAYLOAD_DEPLOY"
    BOOSTBACK       = "BOOSTBACK"
    ENTRY           = "ENTRY"
    LANDING_BURN    = "LANDING_BURN"
    TOUCHDOWN       = "TOUCHDOWN"
    SAFE_HOLD       = "SAFE_HOLD"
    ABORT           = "ABORT"
    COMPLETE        = "COMPLETE"


@dataclass
class PhaseTransition:
    from_phase:     FlightPhase
    to_phase:       FlightPhase
    trigger_time_s: float          # nominal time trigger
    is_irreversible: bool = False  # stage sep, payload deploy = irreversible
    condition:      str = ""       # human-readable condition description


# Nominal Falcon 9 class phase timeline (public data)
NOMINAL_TIMELINE: List[PhaseTransition] = [
    PhaseTransition(FlightPhase.PRE_LAUNCH, FlightPhase.IGNITION,    t:= -3.0,  False, "Engine startup sequence"),
    PhaseTransition(FlightPhase.IGNITION,   FlightPhase.LIFTOFF,        0.0,    False, "T+0 liftoff"),
    PhaseTransition(FlightPhase.LIFTOFF,    FlightPhase.MAX_Q,          60.0,   False, "Approaching max-Q"),
    PhaseTransition(FlightPhase.MAX_Q,      FlightPhase.MECO,           150.0,  False, "Main engine cut-off"),
    PhaseTransition(FlightPhase.MECO,       FlightPhase.STAGE_SEP,      153.0,  True,  "Stage separation (IRREVERSIBLE)"),
    PhaseTransition(FlightPhase.STAGE_SEP,  FlightPhase.UPPER_STAGE,    160.0,  False, "Upper stage ignition"),
    PhaseTransition(FlightPhase.UPPER_STAGE,FlightPhase.PAYLOAD_DEPLOY, 480.0,  True,  "Payload deploy (IRREVERSIBLE)"),
    PhaseTransition(FlightPhase.PAYLOAD_DEPLOY, FlightPhase.COMPLETE,   490.0,  False, "Mission complete"),
]


class MissionManager:
    """
    Manages flight phase progression with ARVS authority gating.
    Every phase transition — especially irreversible ones — is ARVS-approved.
    """

    def __init__(self, arvs_authority: ARVSAuthority, guidance: Any, mission: Any):
        self.arvs     = arvs_authority
        self.guidance = guidance
        self.mission  = mission

        self.current_phase  = FlightPhase.PRE_LAUNCH
        self.mission_time_s = 0.0
        self.phase_start_t  = time.time()
        self.complete       = False

        self._phase_history: List[Dict] = []

    @property
    def current_phase_name(self) -> str:
        return self.current_phase.value

    async def start(self) -> None:
        logger.info("Mission manager started")
        # Request ARVS permission to begin mission
        result = await self.arvs.request_permission(
            ProposedAction(
                action_type=ActionType.PHASE_TRANSITION,
                parameters={"to_phase": "IGNITION"},
                mission_phase="PRE_LAUNCH",
                rationale="Begin launch sequence"
            )
        )
        if result.permitted:
            logger.info("ARVS granted mission start authority")
        else:
            logger.critical(f"ARVS denied mission start: {result.denial_reason}")
            self.current_phase = FlightPhase.SAFE_HOLD

    async def tick(self) -> None:
        """Called at 10 Hz to check phase transition conditions."""
        self.mission_time_s += 0.1

        # Find applicable transition
        for transition in NOMINAL_TIMELINE:
            if (transition.from_phase == self.current_phase and
                    self.mission_time_s >= transition.trigger_time_s):

                await self._request_transition(transition)
                break

        # Check complete
        if self.current_phase == FlightPhase.COMPLETE:
            self.complete = True

    async def _request_transition(self, transition: PhaseTransition) -> None:
        """Request phase transition from ARVS."""
        is_irrev = transition.is_irreversible

        logger.info(
            f"Requesting phase transition: {transition.from_phase.value} → "
            f"{transition.to_phase.value} "
            f"{'[IRREVERSIBLE]' if is_irrev else ''}"
        )

        result = await self.arvs.request_permission(
            ProposedAction(
                action_type=ActionType.PHASE_TRANSITION,
                parameters={
                    "from_phase":   transition.from_phase.value,
                    "to_phase":     transition.to_phase.value,
                    "mission_time": self.mission_time_s,
                },
                is_irreversible=is_irrev,
                mission_phase=transition.from_phase.value,
                rationale=transition.condition
            ),
            vehicle_state=self.guidance.current_state
        )

        if result.permitted:
            old = self.current_phase
            self.current_phase = transition.to_phase
            self._phase_history.append({
                "time_s":    self.mission_time_s,
                "from":      old.value,
                "to":        self.current_phase.value,
                "token":     hex(result.authority_token),
                "risk":      result.risk_level,
                "confidence":result.confidence,
            })
            logger.info(
                f"Phase → {self.current_phase.value} | "
                f"risk={result.risk_level:.3f} conf={result.confidence:.3f}"
            )
        else:
            logger.warning(
                f"ARVS denied phase transition to {transition.to_phase.value}: "
                f"{result.denial_reason}"
            )
            # For irreversible transitions, SAFE_HOLD on denial
            if is_irrev:
                self.current_phase = FlightPhase.SAFE_HOLD
                logger.error("SAFE_HOLD: Irreversible transition denied by ARVS")

    async def stop(self) -> None:
        logger.info(f"Mission stopped at phase={self.current_phase.value}, "
                    f"T+{self.mission_time_s:.1f}s")
        logger.info(f"Phase history: {len(self._phase_history)} transitions")
