"""
arvs_authority.py
ATLAS → ARVS Authority Bridge

This is the ONLY interface between ATLAS Python guidance/mission layers
and the ARVS authority system. ALL decisions route through here.

Architecture:
    ATLAS Guidance → ARVSAuthority.request_permission() → ARVS Core
    ARVS Core validates all 18 axioms → returns permit/deny
    ATLAS executes ONLY on permit + ARVS authority token

ARVS Authority Model:
    ARVS is not just a safety filter — it is the AUTHORITY.
    ATLAS cannot self-authorize any action. ARVS grants authority.
    If ARVS is unreachable → SAFE_HOLD (ARVS Axiom A2: no authority = no action).
"""

from __future__ import annotations

import time
import logging
import asyncio
import uuid
from dataclasses import dataclass, field
from enum import Enum, auto
from typing import Any, Dict, List, Optional

import numpy as np

# ARVS core imports
try:
    from ARVS.system.arvs_core import ARVSCore, ARVSConfig
    from ARVS.hardware.hal import HardwareAbstractionLayer, SensorType, TelemetryFrame
    from ARVS.safety.safety_gate import SafetyGate
    from ARVS.core.axioms import ARVSAxiomBase, AxiomSeverity
    from ARVS.core.constants import (
        RISK_THRESHOLD_SAFE, RISK_THRESHOLD_CRITICAL,
        IRREVERSIBLE_CONF_THR, SAFETY_MARGIN_STRUCTURAL,
        SAFETY_MARGIN_THERMAL, SAFETY_MARGIN_POWER
    )
    from ARVS.core.data_types import RobotState, SystemMode
    from ARVS.audit.logger import AuditLogger
    _ARVS_AVAILABLE = True
except ImportError:
    _ARVS_AVAILABLE = False
    logging.warning("ARVS modules not found — running in simulation-only mode")

logger = logging.getLogger(__name__)


# ─────────────────────────────────────────────────────────────────
#  Permission result — ARVS decision to ATLAS
# ─────────────────────────────────────────────────────────────────

class PermissionStatus(Enum):
    GRANTED         = auto()   # Action approved, proceed
    GRANTED_CLAMPED = auto()   # Approved with constraint modifications
    DENIED          = auto()   # Action denied — safe hold
    DENIED_ABORT    = auto()   # Fatal — initiate flight termination
    PENDING         = auto()   # Awaiting ARVS response (non-blocking flow)


@dataclass
class PermissionResult:
    """ARVS authority decision for a proposed ATLAS action."""
    status:          PermissionStatus
    authority_token: int = 0           # Used by C++ safety gate to verify ARVS approval

    # Modified parameters (on GRANTED_CLAMPED)
    modified_params: Dict[str, Any] = field(default_factory=dict)

    # ARVS assessment
    risk_level:      float = 0.0       # 0–1 (ARVS risk score)
    confidence:      float = 0.0       # ARVS state confidence
    violated_axioms: List[str] = field(default_factory=list)
    denial_reason:   str = ""

    # Timing
    latency_ms:      float = 0.0
    timestamp:       float = field(default_factory=time.time)

    # Validity window — token expires
    valid_until:     float = 0.0       # UNIX timestamp

    @property
    def permitted(self) -> bool:
        return self.status in (PermissionStatus.GRANTED,
                               PermissionStatus.GRANTED_CLAMPED)

    @property
    def expired(self) -> bool:
        return time.time() > self.valid_until


# ─────────────────────────────────────────────────────────────────
#  Proposed action types
# ─────────────────────────────────────────────────────────────────

class ActionType(Enum):
    ENGINE_THROTTLE     = "engine_throttle"
    GIMBAL_COMMAND      = "gimbal_command"
    GRID_FIN_COMMAND    = "grid_fin_command"
    RCS_FIRE            = "rcs_fire"
    STAGE_SEPARATION    = "stage_separation"    # IRREVERSIBLE
    PAYLOAD_DEPLOY      = "payload_deploy"       # IRREVERSIBLE
    FLIGHT_ABORT        = "flight_abort"         # IRREVERSIBLE
    LANDING_BURN_INIT   = "landing_burn_init"
    PHASE_TRANSITION    = "phase_transition"


@dataclass
class ProposedAction:
    """An action ATLAS wants to take, pending ARVS approval."""
    action_type:    ActionType
    parameters:     Dict[str, Any]
    is_irreversible: bool = False
    priority:       int = 0            # Higher = more urgent
    requestor:      str = "guidance"   # Which ATLAS subsystem is requesting

    # Context for ARVS decision
    mission_phase:  str = ""
    rationale:      str = ""           # Why this action is needed

    def __post_init__(self):
        # Mark irreversible actions correctly
        if self.action_type in (ActionType.STAGE_SEPARATION,
                                ActionType.PAYLOAD_DEPLOY,
                                ActionType.FLIGHT_ABORT):
            self.is_irreversible = True


# ─────────────────────────────────────────────────────────────────
#  ARVS Authority Interface
#  ATLAS's single connection point to ARVS
# ─────────────────────────────────────────────────────────────────

class ARVSAuthority:
    """
    ATLAS's interface to the ARVS authority system.

    Every action ATLAS wants to take MUST be submitted here.
    ARVS checks:
      1. All 18+ axioms (epistemic, uncertainty, authority, consequence, etc.)
      2. Physical safety limits (structural, thermal, power)
      3. Mission phase validity
      4. Risk level (MVI optimization result)

    If ARVS is unavailable → SAFE_HOLD (Axiom A2: no authority → no action)
    """

    # Token validity window — authority expires after this (requires re-check)
    TOKEN_VALIDITY_S = 0.5   # 500 ms max command age (matches guidance loop period × 5)

    def __init__(self, robot_id: str = "ATLAS_V1"):
        self.robot_id = robot_id
        self.logger   = logging.getLogger(f"ARVSAuthority.{robot_id}")

        # ARVS core (if available)
        self._arvs: Optional[ARVSCore]  = None
        self._hal:  Optional[HardwareAbstractionLayer] = None

        # Latest HAL state (updated from C++ telemetry bus via bridge)
        self._latest_robot_state: Optional[RobotState] = None

        # Performance tracking
        self._request_count  = 0
        self._deny_count     = 0
        self._latency_history: List[float] = []

        # System mode
        self._mode = "NORMAL"   # NORMAL | DEGRADED | SAFE_HOLD | EMERGENCY

        # Token counter (monotonically increasing)
        self._token_counter = 0

    async def initialize(self) -> bool:
        """Initialize ARVS connection. Returns False → enter safe hold."""
        try:
            if not _ARVS_AVAILABLE:
                self.logger.warning("ARVS not available — simulation mode only")
                self._mode = "DEGRADED"
                return True  # Allow simulation to continue

            # Build ARVS configuration
            config = ARVSConfig(
                robot_id=self.robot_id,
                initial_mode=SystemMode.NORMAL,
                enable_learning=True,
                enable_audit_logging=True,
                safety_margins={
                    'torque':      0.80,
                    'thermal':     0.90,
                    'structural':  0.70,
                    'velocity':    0.80,
                    'battery':     0.30,
                }
            )

            # Initialize ARVS hardware abstraction layer
            self._hal = HardwareAbstractionLayer(use_mock_sensors=True)
            self._hal.start()

            # Initialize ARVS core
            self._arvs = ARVSCore(config)

            self.logger.info(f"ARVS authority established for {self.robot_id}")
            self._mode = "NORMAL"
            return True

        except Exception as exc:
            self.logger.error(f"ARVS initialization failed: {exc}")
            self._mode = "SAFE_HOLD"
            return False

    async def request_permission(
            self,
            action: ProposedAction,
            vehicle_state: Optional[Dict[str, Any]] = None
    ) -> PermissionResult:
        """
        Submit an action to ARVS for authority approval.

        This is the ONLY way ATLAS gets permission to act.
        Synchronous equivalent: request_permission_sync()

        @param action         The proposed action with all parameters
        @param vehicle_state  Current vehicle state (if None, uses HAL state)
        @returns              PermissionResult with GRANTED/DENIED decision
        """
        t_start = time.monotonic()
        self._request_count += 1

        # ── Build ARVS robot state from vehicle telemetry ─────────
        rs = self._build_robot_state(vehicle_state)

        # ── ARVS unavailable → SAFE_HOLD (Axiom A2) ──────────────
        if self._mode == "SAFE_HOLD":
            self._deny_count += 1
            return PermissionResult(
                status=PermissionStatus.DENIED,
                denial_reason="ARVS A2: System in SAFE_HOLD — no authority granted",
                latency_ms=(time.monotonic() - t_start) * 1000,
                timestamp=time.time()
            )

        # ── Simulate ARVS if full system not available ────────────
        if not _ARVS_AVAILABLE or self._arvs is None:
            return await self._simulated_permission(action, rs, t_start)

        # ── Full ARVS evaluation ──────────────────────────────────
        return await self._full_arvs_evaluation(action, rs, t_start)

    async def _full_arvs_evaluation(
            self,
            action: ProposedAction,
            rs: RobotState,
            t_start: float
    ) -> PermissionResult:
        """Run complete ARVS evaluation pipeline."""
        try:
            # 1. Update ARVS state from latest HAL telemetry
            hal_state = self._hal.latest_state_estimate()
            rs = self._hal_state_to_robot_state(hal_state)

            # 2. Check ARVS axioms
            from ARVS.core.axioms import ARVSAxiomValidator
            axiom_validator = ARVSAxiomValidator()
            axiom_result = axiom_validator.validate_all(rs)

            if not axiom_result.authority_valid:
                self._deny_count += 1
                violated = [r.axiom_id for r in axiom_result.checks[:axiom_result.n_checked]
                           if not r.passed]
                return PermissionResult(
                    status=PermissionStatus.DENIED,
                    violated_axioms=violated,
                    denial_reason=f"Axiom violation: {violated}",
                    latency_ms=(time.monotonic() - t_start) * 1000,
                    timestamp=time.time()
                )

            # 3. Build ARVS action for safety gate
            arvs_action = self._to_arvs_action(action)

            # 4. Run ARVS safety gate
            safety_gate = SafetyGate()
            safety_result = safety_gate.evaluate(rs, arvs_action)

            if not safety_result.allowed:
                self._deny_count += 1
                return PermissionResult(
                    status=PermissionStatus.DENIED,
                    risk_level=safety_result.risk_level,
                    confidence=rs.confidence,
                    denial_reason=f"Safety gate denied: {safety_result.constraints}",
                    latency_ms=(time.monotonic() - t_start) * 1000,
                    timestamp=time.time()
                )

            # 5. Check irreversible action confidence threshold (Axiom C2)
            if action.is_irreversible and rs.confidence < 0.95:
                return PermissionResult(
                    status=PermissionStatus.DENIED,
                    confidence=rs.confidence,
                    denial_reason=f"C2: Irreversible action confidence {rs.confidence:.3f} < 0.95",
                    latency_ms=(time.monotonic() - t_start) * 1000,
                    timestamp=time.time()
                )

            # 6. Issue authority token
            token = self._issue_token()
            latency = (time.monotonic() - t_start) * 1000
            self._latency_history.append(latency)

            return PermissionResult(
                status=PermissionStatus.GRANTED,
                authority_token=token,
                risk_level=safety_result.risk_level,
                confidence=rs.confidence,
                latency_ms=latency,
                timestamp=time.time(),
                valid_until=time.time() + self.TOKEN_VALIDITY_S
            )

        except Exception as exc:
            self.logger.error(f"ARVS evaluation error: {exc}")
            # Fail-safe: deny on any exception
            return PermissionResult(
                status=PermissionStatus.DENIED,
                denial_reason=f"ARVS evaluation exception: {exc}",
                latency_ms=(time.monotonic() - t_start) * 1000
            )

    async def _simulated_permission(
            self,
            action: ProposedAction,
            rs: RobotState,
            t_start: float
    ) -> PermissionResult:
        """
        Simulation-mode ARVS evaluation.
        Implements the same logic as ARVS but without the full Python module.
        Uses datasheet-based thresholds.
        """
        denied_reasons = []

        # ── Epistemic check (E1/E2): confidence ──────────────────
        if rs.confidence < 0.70:
            denied_reasons.append(f"E2: confidence {rs.confidence:.3f} < 0.70")

        # ── Thermal check ─────────────────────────────────────────
        from ARVS.core.constants import DEFAULT_MAX_TEMPERATURE
        thermal_limit = DEFAULT_MAX_TEMPERATURE * SAFETY_MARGIN_THERMAL
        if rs.temperature > thermal_limit:
            denied_reasons.append(f"THERMAL: {rs.temperature:.1f}K > limit {thermal_limit:.1f}K")

        # ── Power check ───────────────────────────────────────────
        from ARVS.core.constants import FAULT_THRESHOLD_BATTERY
        if rs.battery_level < FAULT_THRESHOLD_BATTERY:
            denied_reasons.append(f"POWER: battery {rs.battery_level:.1%} < {FAULT_THRESHOLD_BATTERY:.1%}")

        # ── Irreversible action check (C2) ────────────────────────
        if action.is_irreversible and rs.confidence < 0.95:
            denied_reasons.append(f"C2: irreversible action, confidence {rs.confidence:.3f} < 0.95")

        # ── Risk assessment (simplified) ──────────────────────────
        risk = self._estimate_risk(action, rs)
        if risk > RISK_THRESHOLD_CRITICAL:
            denied_reasons.append(f"RISK: {risk:.3f} > critical threshold {RISK_THRESHOLD_CRITICAL}")

        # ── Decision ──────────────────────────────────────────────
        latency = (time.monotonic() - t_start) * 1000
        self._latency_history.append(latency)

        if denied_reasons:
            self._deny_count += 1
            return PermissionResult(
                status=PermissionStatus.DENIED,
                risk_level=risk,
                confidence=rs.confidence,
                violated_axioms=denied_reasons,
                denial_reason="; ".join(denied_reasons),
                latency_ms=latency,
                timestamp=time.time()
            )

        token = self._issue_token()
        return PermissionResult(
            status=PermissionStatus.GRANTED,
            authority_token=token,
            risk_level=risk,
            confidence=rs.confidence,
            latency_ms=latency,
            timestamp=time.time(),
            valid_until=time.time() + self.TOKEN_VALIDITY_S
        )

    def _build_robot_state(self, vehicle_state: Optional[Dict]) -> RobotState:
        """Convert ATLAS vehicle dict to ARVS RobotState."""
        if self._latest_robot_state and not vehicle_state:
            return self._latest_robot_state

        if vehicle_state:
            return self._hal_state_to_robot_state(vehicle_state)

        # Fallback: use HAL state if available
        if self._hal:
            return self._hal_state_to_robot_state(self._hal.latest_state_estimate())

        # Last resort: minimal default state (high uncertainty)
        import numpy as np
        rs = RobotState(
            robot_id=self.robot_id,
            timestamp=time.time(),
            position=np.zeros(3),
            velocity=np.zeros(3),
            orientation=np.array([1.0, 0.0, 0.0, 0.0]),
            angular_velocity=np.zeros(3),
            temperature=300.0,
            battery_level=0.85,
            power_consumption=100.0,
            confidence=0.5   # Low confidence — ARVS will be cautious
        )
        return rs

    @staticmethod
    def _hal_state_to_robot_state(hal: Dict) -> RobotState:
        """Convert HAL state dict to ARVS RobotState dataclass."""
        import numpy as np
        return RobotState(
            robot_id="ATLAS_V1",
            timestamp=hal.get("timestamp", time.time()),
            position=np.array(hal.get("position", [0,0,0])),
            velocity=np.array(hal.get("velocity", [0,0,0])),
            orientation=np.array(hal.get("orientation", [1,0,0,0])),
            angular_velocity=np.array(hal.get("angular_velocity", [0,0,0])),
            temperature=hal.get("temperature", 300.0),
            battery_level=hal.get("battery_level", 0.85),
            power_consumption=hal.get("power_consumption", 100.0),
            confidence=hal.get("confidence", 1.0)
        )

    @staticmethod
    def _to_arvs_action(action: ProposedAction) -> Dict:
        """Convert ProposedAction to ARVS action format."""
        params = action.parameters
        return {
            "action_id":      str(uuid.uuid4())[:16],
            "action_type":    action.action_type.value,
            "duration":       params.get("duration", 0.1),
            "max_torque":     params.get("max_torque", -1.0),
            "max_velocity":   params.get("max_velocity", -1.0),
            "thermal_load":   params.get("thermal_load", 0.0),
            "power_required": params.get("power_required", 0.0),
            "is_reversible":  not action.is_irreversible,
            "priority":       action.priority,
        }

    @staticmethod
    def _estimate_risk(action: ProposedAction, rs: RobotState) -> float:
        """
        Simplified risk estimate for simulation mode.
        Real system: uses ARVS RiskQuantifier (QUBO-based).
        """
        base_risk = 0.1

        # Irreversible actions carry higher base risk
        if action.is_irreversible:
            base_risk += 0.15

        # Low confidence → higher risk
        confidence_penalty = (1.0 - rs.confidence) * 0.3
        base_risk += confidence_penalty

        # Thermal proximity to limit
        thermal_margin = 1.0 - rs.temperature / 373.0
        if thermal_margin < 0.1:
            base_risk += 0.2

        return min(1.0, base_risk)

    def _issue_token(self) -> int:
        """Issue a monotonically increasing authority token."""
        self._token_counter += 1
        # XOR with timestamp bits for collision resistance
        t_bits = int(time.time() * 1000) & 0xFFFFFFFF
        return (self._token_counter << 32) | t_bits

    def update_vehicle_state(self, state: Dict[str, Any]) -> None:
        """Called by ATLAS telemetry bridge to update ARVS with current state."""
        self._latest_robot_state = self._hal_state_to_robot_state(state)

    def get_system_mode(self) -> str:
        return self._mode

    def get_statistics(self) -> Dict[str, Any]:
        avg_latency = (sum(self._latency_history[-100:]) /
                       max(1, len(self._latency_history[-100:])))
        return {
            "request_count":    self._request_count,
            "deny_count":       self._deny_count,
            "deny_rate":        self._deny_count / max(1, self._request_count),
            "avg_latency_ms":   avg_latency,
            "mode":             self._mode,
            "arvs_available":   _ARVS_AVAILABLE,
        }

    async def shutdown(self) -> None:
        if self._hal:
            self._hal.stop()
        self.logger.info("ARVS authority shut down")
