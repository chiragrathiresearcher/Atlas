/**
 * safety_gate.cpp
 * ATLAS ARVS Safety Gate — Implementation
 *
 * Out-of-line definitions for ARVSSafetyGate.
 * The gate must complete in < 0.5 ms to stay within 1 kHz budget.
 *
 * Benchmark target (ARM Cortex-A72 @ 1.8 GHz, -O2):
 *   validate_all(18 axioms):  ~120 µs
 *   check_physics:            ~  8 µs
 *   confidence checks:        ~  2 µs
 *   Total:                    ~130 µs  (well within 500 µs budget)
 */

#include "safety_gate.hpp"
#include <cstring>
#include <cstdio>
#include <cmath>

namespace atlas {

// Static constexpr ODR compliance
constexpr double ARVSSafetyGate::TORQUE_SAFETY_MARGIN;
constexpr double ARVSSafetyGate::THERMAL_SAFETY_MARGIN;
constexpr double ARVSSafetyGate::POWER_SAFETY_MARGIN;
constexpr double ARVSSafetyGate::MAX_Q_PA_LIMIT;
constexpr double ARVSSafetyGate::MAX_AXIAL_LOAD_G;
constexpr double ARVSSafetyGate::MIN_PROPELLANT_RESERVE;

// ─────────────────────────────────────────────────────────────────
//  Audit log entry (ring buffer — no heap allocation)
// ─────────────────────────────────────────────────────────────────

struct AuditEntry {
    double   timestamp_s{0.0};
    GateDecision decision{GateDecision::SAFE_HOLD};
    char     reason[128]{};
    double   confidence{0.0};
    uint32_t gate_call_id{0};
};

static AuditEntry audit_ring[256];
static uint32_t audit_head = 0;

static void audit_log(double ts, GateDecision d, const char* reason,
                      double conf, uint32_t call_id) {
    AuditEntry& e = audit_ring[audit_head & 0xFF];
    e.timestamp_s  = ts;
    e.decision     = d;
    e.confidence   = conf;
    e.gate_call_id = call_id;
    std::strncpy(e.reason, reason, 127);
    audit_head++;
}

// ─────────────────────────────────────────────────────────────────
//  Structural overload check
//  Called from check_physics() — additional launch-vehicle checks
// ─────────────────────────────────────────────────────────────────

static bool check_structural_limits(const arvs::RobotState& vs,
                                     double q_pa, double axial_g,
                                     char* out_reason) {
    // Dynamic pressure structural limit
    if (q_pa > ARVSSafetyGate::MAX_Q_PA_LIMIT) {
        snprintf(out_reason, 255,
                 "STRUCTURAL: q=%.0f Pa > limit %.0f Pa",
                 q_pa, ARVSSafetyGate::MAX_Q_PA_LIMIT);
        return false;
    }
    // Axial load factor
    if (axial_g > ARVSSafetyGate::MAX_AXIAL_LOAD_G) {
        snprintf(out_reason, 255,
                 "STRUCTURAL: axial_g=%.2f > limit %.2f g",
                 axial_g, ARVSSafetyGate::MAX_AXIAL_LOAD_G);
        return false;
    }
    (void)vs;
    return true;
}

// ─────────────────────────────────────────────────────────────────
//  Extended check() — adds structural + audit logging
//  Overloads the header's inline version with additional checks.
// ─────────────────────────────────────────────────────────────────

SafeGateResult ARVSSafetyGate::check_extended(
    const arvs::RobotState&       vehicle_state,
    const arvs::AxiomSystemState& axiom_state,
    double throttle,
    double gimbal_pitch,
    double gimbal_yaw,
    bool   is_irreversible,
    double q_pa,
    double axial_g)
{
    SafeGateResult result = check(vehicle_state, axiom_state,
                                   throttle, gimbal_pitch, gimbal_yaw,
                                   is_irreversible);

    // Additional structural check (not in base check())
    if (result.permitted()) {
        char struct_reason[256]{};
        if (!check_structural_limits(vehicle_state, q_pa, axial_g, struct_reason)) {
            result.decision = GateDecision::SAFE_HOLD;
            std::strncpy(result.rejection_reason, struct_reason, 255);
            ++rejections_;
        }
    }

    // Audit log every call
    audit_log(vehicle_state.timestamp, result.decision,
              result.rejection_reason, result.confidence,
              gate_call_count_);

    return result;
}

// ─────────────────────────────────────────────────────────────────
//  Statistics dump — for telemetry and ground display
// ─────────────────────────────────────────────────────────────────

void ARVSSafetyGate::print_statistics() const {
    std::printf("[ARVS_GATE] Calls: %u  Rejections: %u  Rate: %.3f%%\n",
                gate_call_count_, rejections_,
                100.0 * rejection_rate());

    // Print last 5 audit entries
    const uint32_t n = std::min(5u, audit_head);
    for (uint32_t i = 0; i < n; ++i) {
        const auto& e = audit_ring[(audit_head - 1 - i) & 0xFF];
        std::printf("  [%u] T=%.3f d=%u conf=%.3f '%s'\n",
                    e.gate_call_id,
                    e.timestamp_s,
                    static_cast<unsigned>(e.decision),
                    e.confidence,
                    e.reason);
    }
}

} // namespace atlas
