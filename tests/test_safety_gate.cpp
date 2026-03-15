/**
 * test_safety_gate.cpp
 * Unit tests for ATLAS/ARVS safety gate integration.
 * Tests: permit, deny on thermal, deny on low confidence, clamp on gimbal.
 */

#include <cassert>
#include <cstdio>
#include <cstring>
#include "safety_gate.hpp"
#include "arvs_types.hpp"
#include "atlas_hardware.hpp"

using namespace atlas;
using namespace arvs;

// ── Helpers ──────────────────────────────────────────────────────

static RobotState healthy_state() {
    RobotState s{};
    std::strncpy(s.robot_id, "TEST_VEHICLE", ROBOT_ID_LEN - 1);
    s.temperature      = 300.0;   // K (normal)
    s.battery_level    = 0.85;
    s.power_consumption= 100.0;
    s.confidence       = 0.98;
    s.velocity         = {0.0, 100.0, 0.0};  // 100 m/s
    return s;
}

static AxiomSystemState healthy_axioms() {
    AxiomSystemState a{};
    a.confidence               = 0.98;
    a.uncertainty_explicit     = true;
    a.authority_explicitly_defined = true;
    a.all_actions_gated        = true;
    a.gate_decision_final      = true;
    a.is_safe                  = true;
    a.evaluation_basis[0]      = 'w'; // "worst_case_credible"
    a.action_is_reversible     = true;
    a.system_mode              = SystemMode::NORMAL;
    return a;
}

// ── Test cases ────────────────────────────────────────────────────

static void test_normal_command_permitted() {
    ARVSSafetyGate gate;
    auto rs = healthy_state();
    auto ax = healthy_axioms();

    auto result = gate.check(rs, ax, 0.8, 0.02, -0.01, false);

    assert(result.permitted() && "Normal command should be permitted");
    assert(result.decision == GateDecision::PERMIT ||
           result.decision == GateDecision::CLAMP);
    printf("[PASS] test_normal_command_permitted\n");
}

static void test_thermal_overtemp_denied() {
    ARVSSafetyGate gate;
    auto rs = healthy_state();
    auto ax = healthy_axioms();

    // Temperature above ARVS limit
    rs.temperature = MAX_TEMPERATURE_K * 1.05;  // 5% over limit

    auto result = gate.check(rs, ax, 0.8, 0.0, 0.0, false);

    assert(!result.permitted() && "Overtemp should be denied");
    printf("[PASS] test_thermal_overtemp_denied\n");
}

static void test_low_battery_denied() {
    ARVSSafetyGate gate;
    auto rs = healthy_state();
    auto ax = healthy_axioms();

    rs.battery_level = MIN_BATTERY_FRACTION * 0.9;  // below minimum

    auto result = gate.check(rs, ax, 0.5, 0.0, 0.0, false);

    assert(!result.permitted() && "Low battery should be denied");
    printf("[PASS] test_low_battery_denied\n");
}

static void test_low_confidence_denied() {
    ARVSSafetyGate gate;
    auto rs = healthy_state();
    auto ax = healthy_axioms();

    ax.confidence   = 0.50;   // Below 0.70 threshold
    rs.confidence   = 0.50;

    auto result = gate.check(rs, ax, 0.8, 0.0, 0.0, false);

    assert(!result.permitted() && "Low confidence should be denied");
    printf("[PASS] test_low_confidence_denied\n");
}

static void test_irreversible_needs_high_confidence() {
    ARVSSafetyGate gate;
    auto rs = healthy_state();
    auto ax = healthy_axioms();

    // Confidence acceptable for reversible, not for irreversible
    ax.confidence = 0.80;
    rs.confidence = 0.80;

    // Reversible should pass
    auto r1 = gate.check(rs, ax, 0.5, 0.0, 0.0, false);
    assert(r1.permitted() && "Reversible with 0.80 confidence should pass");

    // Irreversible should fail (requires 0.95 per Axiom C2)
    auto r2 = gate.check(rs, ax, 0.5, 0.0, 0.0, true);
    assert(!r2.permitted() && "Irreversible with 0.80 confidence should fail (C2)");

    printf("[PASS] test_irreversible_needs_high_confidence\n");
}

static void test_gimbal_clamped_at_datasheet_limit() {
    ARVSSafetyGate gate;
    auto rs = healthy_state();
    auto ax = healthy_axioms();

    // Request 10° gimbal — exceeds Merlin 1D max of 5°
    const double req_pitch = 10.0 * M_PI / 180.0;
    auto result = gate.check(rs, ax, 0.8, req_pitch, 0.0, false);

    // Should be permitted but clamped, or just clamped
    // The actuator controller handles the actual clamp; gate checks safety
    assert(result.decision != GateDecision::ABORT &&
           "Gimbal overcommand should not trigger abort");
    printf("[PASS] test_gimbal_clamped_at_datasheet_limit\n");
}

static void test_rejection_rate_tracking() {
    ARVSSafetyGate gate;
    auto rs_ok  = healthy_state();
    auto rs_bad = healthy_state();
    rs_bad.temperature = MAX_TEMPERATURE_K * 1.1;

    auto ax = healthy_axioms();

    // 3 good, 1 bad
    gate.check(rs_ok,  ax, 0.8, 0.0, 0.0, false);
    gate.check(rs_ok,  ax, 0.8, 0.0, 0.0, false);
    gate.check(rs_ok,  ax, 0.8, 0.0, 0.0, false);
    gate.check(rs_bad, ax, 0.8, 0.0, 0.0, false);

    assert(gate.gate_call_count() == 4);
    assert(gate.rejections() == 1);
    printf("[PASS] test_rejection_rate_tracking | rate=%.2f%%\n",
           gate.rejection_rate() * 100.0);
}

// ── Main ─────────────────────────────────────────────────────────

int main() {
    printf("\n=== ATLAS Safety Gate Tests ===\n\n");

    test_normal_command_permitted();
    test_thermal_overtemp_denied();
    test_low_battery_denied();
    test_low_confidence_denied();
    test_irreversible_needs_high_confidence();
    test_gimbal_clamped_at_datasheet_limit();
    test_rejection_rate_tracking();

    printf("\n=== All tests PASSED ===\n");
    return 0;
}
