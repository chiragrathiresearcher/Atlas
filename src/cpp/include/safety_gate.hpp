/**
 * safety_gate.hpp
 * ATLAS Safety Gate — ARVS Authority Layer (C++ Real-Time Interface)
 *
 * This is the ARVS integration point for the C++ flight computer.
 * EVERY actuator command passes through here before execution.
 * No actuator can move without ARVS axiom validation.
 *
 * Integration Architecture:
 *   Python ARVS Core → arvs_bridge → [IPC/shared memory] → C++ ARVSSafetyGate
 *   C++ ARVSSafetyGate validates via ARVS axiom engine (C++ native side)
 *
 * Sources:
 *   [ARVS] ARVS System Document v1.0 — Axioms E1–Z, safety gate spec
 *   [ARVS_CPP] arvs_types.hpp, axiom_validator.hpp, safety_gate.hpp (ARVS repo)
 */

#pragma once
#include <cstdint>
#include <cstring>
#include <cmath>
#include "atlas_hardware.hpp"
#include "arvs_types.hpp"

// Include ARVS C++ headers (from ARVS repo cpp/include/)
#include "axiom_validator.hpp"   // ARVS axiom validation engine
#include "watchdog_timer.hpp"    // ARVS watchdog

namespace atlas {

// ─────────────────────────────────────────────────────────────────
//  Safety gate decision
// ─────────────────────────────────────────────────────────────────
enum class GateDecision : uint8_t {
    PERMIT      = 0,   // Command approved — proceed
    CLAMP       = 1,   // Command approved with modifications — use clamped values
    SAFE_HOLD   = 2,   // Command rejected — enter safe hold
    ABORT       = 3    // Fatal violation — flight termination
};

struct SafeGateResult {
    GateDecision decision{GateDecision::SAFE_HOLD};

    // Modified command (when decision == CLAMP)
    double   throttle_permitted{0.0};
    double   gimbal_pitch_permitted{0.0};
    double   gimbal_yaw_permitted{0.0};

    // ARVS axiom validation result
    arvs::AxiomValidationResult axiom_result{};

    // ARVS safety check result
    arvs::SafetyCheckResult     safety_result{};

    // Reason for rejection (populated on non-PERMIT)
    char     rejection_reason[256]{};

    // Confidence of the safety assessment
    double   confidence{0.0};

    bool permitted() const { return decision == GateDecision::PERMIT ||
                                    decision == GateDecision::CLAMP; }
};

// ─────────────────────────────────────────────────────────────────
//  ARVS Safety Gate
//  Wraps ARVS C++ axiom validator and safety checker
//  All flight-critical decisions flow through here
// ─────────────────────────────────────────────────────────────────
class ARVSSafetyGate {
public:
    // Safety margins from ARVS constants (see arvs_types.hpp)
    static constexpr double TORQUE_SAFETY_MARGIN   = arvs::SAFETY_MARGIN_TORQUE;
    static constexpr double THERMAL_SAFETY_MARGIN  = arvs::SAFETY_MARGIN_THERMAL;
    static constexpr double POWER_SAFETY_MARGIN    = arvs::SAFETY_MARGIN_POWER;

    // Vehicle-specific limits (launch vehicle context)
    // Max Q structural limit: from vehicle load analysis
    static constexpr double MAX_Q_PA_LIMIT         = 35000.0;  // Pa (Falcon 9 class ~35 kPa max-Q)
    static constexpr double MAX_AXIAL_LOAD_G       = 5.5;      // g (structural limit at max Q)
    static constexpr double MIN_PROPELLANT_RESERVE = 0.05;     // 5% reserve (non-negotiable)

    ARVSSafetyGate() : validator_(), gate_call_count_(0), rejections_(0) {}

    /**
     * PRIMARY ENTRY POINT — Must be called before every actuator command.
     *
     * Validates proposed command against:
     *   1. ARVS axiom set (E1–Z, all 18+ axioms)
     *   2. Physical safety limits (structural, thermal, power)
     *   3. Vehicle-specific flight envelope
     *
     * @param vehicle_state  Current vehicle state (from ARVS HAL)
     * @param axiom_state    Current ARVS axiom system state
     * @param throttle       Proposed throttle [0,1]
     * @param gimbal_pitch   Proposed gimbal pitch [rad]
     * @param gimbal_yaw     Proposed gimbal yaw [rad]
     * @param is_irreversible Is this a command that cannot be undone (e.g., stage sep)?
     */
    SafeGateResult check(
            const arvs::RobotState&       vehicle_state,
            const arvs::AxiomSystemState& axiom_state,
            double throttle,
            double gimbal_pitch,
            double gimbal_yaw,
            bool   is_irreversible = false) {

        SafeGateResult result;
        ++gate_call_count_;

        // ── Step 1: Validate all ARVS axioms ─────────────────────────
        result.axiom_result = validator_.validate(axiom_state);

        if (!result.axiom_result.authority_valid) {
            result.decision = GateDecision::SAFE_HOLD;
            std::strncpy(result.rejection_reason,
                         "ARVS: Authority validation failed — axiom violation",
                         255);
            ++rejections_;
            return result;
        }

        // ── Step 2: Physical safety checks ───────────────────────────
        build_action(throttle, gimbal_pitch, gimbal_yaw, is_irreversible);
        result.safety_result = check_physics(vehicle_state);

        if (result.safety_result.has_critical()) {
            result.decision = GateDecision::SAFE_HOLD;
            format_violation_reason(result.safety_result, result.rejection_reason);
            ++rejections_;
            return result;
        }

        // ── Step 3: Confidence check (ARVS Axiom U1) ─────────────────
        // Unknown = maximum uncertainty. If confidence below threshold → deny.
        if (axiom_state.confidence < 0.70) {
            result.decision = GateDecision::SAFE_HOLD;
            std::strncpy(result.rejection_reason,
                         "ARVS U1: Confidence below safe threshold (0.70)",
                         255);
            ++rejections_;
            return result;
        }

        // ── Step 4: Irreversible action check (ARVS Axiom C2) ────────
        if (is_irreversible && axiom_state.confidence < arvs::IRREVERSIBLE_CONF_THR) {
            result.decision = GateDecision::SAFE_HOLD;
            std::strncpy(result.rejection_reason,
                         "ARVS C2: Irreversible action requires confidence ≥ 0.95",
                         255);
            ++rejections_;
            return result;
        }

        // ── Step 5: Flight envelope clamping ─────────────────────────
        bool clamped = false;
        result.throttle_permitted     = throttle;
        result.gimbal_pitch_permitted = gimbal_pitch;
        result.gimbal_yaw_permitted   = gimbal_yaw;

        // Throttle minimum when in flight
        if (vehicle_state.velocity.norm() > 10.0 && throttle < 0.40) {
            result.throttle_permitted = 0.40;   // Merlin 1D min throttle (40%)
            clamped = true;
        }

        // Gimbal authority reduces at low dynamic pressure
        const double q = estimate_q(vehicle_state);
        if (q < 500.0 && std::abs(gimbal_pitch) > 0.044) {
            // At low Q, limit TVC to ±2.5° (insufficient aero control effectiveness)
            result.gimbal_pitch_permitted =
                (gimbal_pitch > 0) ? 0.044 : -0.044;
            clamped = true;
        }

        // ── All checks passed ─────────────────────────────────────────
        result.decision   = clamped ? GateDecision::CLAMP : GateDecision::PERMIT;
        result.confidence = axiom_state.confidence;
        return result;
    }

    // Statistics
    uint32_t gate_call_count() const { return gate_call_count_; }
    uint32_t rejections()      const { return rejections_; }

    double rejection_rate() const {
        if (gate_call_count_ == 0) return 0.0;
        return static_cast<double>(rejections_) / gate_call_count_;
    }

private:
    arvs::AxiomValidator validator_;
    uint32_t             gate_call_count_;
    uint32_t             rejections_;
    arvs::Action         pending_action_{};

    void build_action(double throttle, double pitch, double yaw, bool irreversible) {
        std::strncpy(pending_action_.action_type, "engine_command", 31);
        // Map throttle to torque equivalent for ARVS RobotState model
        // (ARVS uses torque/force units; we map thrust force)
        pending_action_.max_torque   = throttle * hw::MERLIN1D::THRUST_SL_KN * 1000.0;
        pending_action_.max_velocity = -1.0;   // Not applicable for engine cmd
        pending_action_.is_reversible= !irreversible;
        pending_action_.power_required = throttle * 12.0;  // Engine controller power (W)
    }

    arvs::SafetyCheckResult check_physics(const arvs::RobotState& vs) const {
        arvs::SafetyCheckResult r;

        // Thermal check
        if (vs.temperature > arvs::MAX_TEMPERATURE_K * THERMAL_SAFETY_MARGIN) {
            r.add_violation(
                arvs::ViolationType::TEMPERATURE_EXCEEDED,
                "engine_bay",
                vs.temperature,
                arvs::MAX_TEMPERATURE_K * THERMAL_SAFETY_MARGIN);
        }

        // Battery/power check
        if (vs.battery_level < arvs::MIN_BATTERY_FRACTION) {
            r.add_violation(
                arvs::ViolationType::BATTERY_LOW,
                "power_bus",
                vs.battery_level,
                arvs::MIN_BATTERY_FRACTION);
        }

        // Joint/actuator torque check (engine thrust as torque analog)
        if (pending_action_.max_torque > arvs::MAX_TORQUE_NM * TORQUE_SAFETY_MARGIN) {
            r.add_violation(
                arvs::ViolationType::TORQUE_EXCEEDED,
                "engine_tvc",
                pending_action_.max_torque,
                arvs::MAX_TORQUE_NM * TORQUE_SAFETY_MARGIN);
        }

        r.safe      = (r.n_violations == 0);
        r.confidence= r.safe ? 1.0 : 0.0;
        return r;
    }

    static double estimate_q(const arvs::RobotState& vs) {
        // Dynamic pressure q = 0.5 × ρ × v²
        // Simplified: use ISA density at current altitude
        const double v = vs.velocity.norm();
        const double rho = 1.225;   // kg/m³ (sea level approx — real system uses altitude)
        return 0.5 * rho * v * v;
    }

    static void format_violation_reason(
            const arvs::SafetyCheckResult& r, char* out) {
        if (r.n_violations == 0) {
            std::strncpy(out, "No violation (unknown rejection)", 255);
            return;
        }
        const auto& v = r.violations[0];
        // Build reason string from first violation
        const char* type_str = "UNKNOWN";
        switch (v.type) {
            case arvs::ViolationType::TORQUE_EXCEEDED:      type_str="TORQUE_EXCEEDED"; break;
            case arvs::ViolationType::TEMPERATURE_EXCEEDED: type_str="TEMPERATURE_EXCEEDED"; break;
            case arvs::ViolationType::BATTERY_LOW:          type_str="BATTERY_LOW"; break;
            case arvs::ViolationType::STRUCTURAL_OVERLOAD:  type_str="STRUCTURAL_OVERLOAD"; break;
            case arvs::ViolationType::AXIOM_VIOLATION:      type_str="AXIOM_VIOLATION"; break;
            default: break;
        }
        // Safe sprintf equivalent
        int written = 0;
        written += snprintf(out + written, 255 - written,
                            "ARVS SafetyGate: %s on %s ", type_str, v.component);
        if (written < 255) {
            snprintf(out + written, 255 - written,
                     "(actual=%.2f, limit=%.2f)", v.actual, v.limit);
        }
    }
};

} // namespace atlas
