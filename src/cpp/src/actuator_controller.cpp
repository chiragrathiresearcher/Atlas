/**
 * actuator_controller.cpp
 * ATLAS Actuator Controller — Implementation
 *
 * Out-of-line constexpr definitions and hardware interface stubs.
 * All limits from atlas_hardware.hpp (manufacturer datasheets).
 *
 * On real hardware:
 *   Engine gimbal  → CAN bus commands to TVC servo controller
 *   Grid fins      → CAN bus to hydraulic actuator ECU
 *   RCS thrusters  → PWM solenoid valve drivers via FPGA
 *   Landing legs   → Pyrotechnic deployment + pneumatic lockout
 */

#include "actuator_controller.hpp"
#include <cmath>
#include <cstring>
#include <iostream>

namespace atlas {

// ─────────────────────────────────────────────────────────────────
//  ODR compliance — static constexpr out-of-line definitions
// ─────────────────────────────────────────────────────────────────

constexpr double EngineController::THROTTLE_MIN;
constexpr double EngineController::THROTTLE_MAX;
constexpr double EngineController::GIMBAL_MAX_RAD;
constexpr double EngineController::THRUST_SL_KN;
constexpr double EngineController::THRUST_VAC_KN;
constexpr double EngineController::ISP_SL_S;
constexpr double EngineController::ISP_VAC_S;
constexpr double EngineController::THROTTLE_SLEW_PER_S;

constexpr int    GridFinController::NUM_FINS;
constexpr double GridFinController::MAX_DEFLECTION_RAD;
constexpr double GridFinController::SLEW_RATE_RAD_S;
constexpr double GridFinController::BANDWIDTH_HZ;
constexpr double GridFinController::MAX_TEMP_K;

constexpr int    RCSController::NUM_THRUSTERS;
constexpr double RCSController::THRUST_N;
constexpr double RCSController::MIN_PULSE_S;
constexpr double RCSController::VALVE_OPEN_S;
constexpr double RCSController::ISP_S;

// ─────────────────────────────────────────────────────────────────
//  CAN bus stubs (replace with real driver on flight hardware)
// ─────────────────────────────────────────────────────────────────

namespace {

/**
 * Send throttle + gimbal command over CAN bus to Merlin 1D controller.
 * CAN ID: 0x100 (engine controller node)
 * Frame: [throttle_u16, pitch_i16, yaw_i16, checksum_u16]
 *
 * throttle_u16 = throttle × 65535
 * pitch_i16    = pitch_rad × (32767 / GIMBAL_MAX_RAD)
 * yaw_i16      = yaw_rad   × (32767 / GIMBAL_MAX_RAD)
 */
void can_send_engine_command(double throttle, double pitch_rad, double yaw_rad) {
    (void)throttle; (void)pitch_rad; (void)yaw_rad;
    // On hardware: fill CAN frame and call can_write(0x100, frame, 8)
    // Placeholder for HIL/flight hardware path
}

/**
 * Send grid fin deflection command.
 * CAN ID: 0x200 + fin_idx
 * Frame: [deflection_i16, temp_u16_K, status_u8, padding_u8, checksum_u16]
 */
void can_send_fin_command(int fin_idx, double deflection_rad, double temp_k) {
    (void)fin_idx; (void)deflection_rad; (void)temp_k;
    // On hardware: fill CAN frame and call can_write(0x200 + fin_idx, frame, 8)
}

/**
 * Fire RCS thruster via FPGA PWM output.
 * GPIO pin mapped from thruster index (0-11):
 *   0–3:  pitch control pair (fore)
 *   4–7:  pitch control pair (aft)
 *   8–11: yaw/roll control
 * Minimum pulse: 10 ms (from Moog DST-80 spec)
 */
void fpga_fire_rcs(int thruster_idx, bool on, double duration_ms) {
    (void)thruster_idx; (void)on; (void)duration_ms;
    // On hardware: write to FPGA register-mapped PWM channel
    // mmio_write32(FPGA_BASE + 0x1000 + thruster_idx * 4, on ? pulse_cycles : 0)
}

} // anonymous namespace

// ─────────────────────────────────────────────────────────────────
//  ActuatorController — apply_command (hardware path override)
//  When not in simulation, calls CAN/FPGA stubs above.
// ─────────────────────────────────────────────────────────────────

// The main apply_command logic is inline in the header.
// This file provides the hardware dispatch layer.
void ActuatorController::dispatch_to_hardware(
    double throttle, double pitch, double yaw,
    const double grid_fins[4], const bool rcs[12]) const
{
    can_send_engine_command(throttle, pitch, yaw);
    for (int i = 0; i < 4; ++i) {
        can_send_fin_command(i, grid_fins[i] * GridFinController::MAX_DEFLECTION_RAD, 300.0);
    }
    for (int i = 0; i < 12; ++i) {
        if (rcs[i]) {
            fpga_fire_rcs(i, true, RCSController::MIN_PULSE_S * 1000.0);
        }
    }
}

// ─────────────────────────────────────────────────────────────────
//  Emergency shutdown — called on ABORT or SAFE_HOLD
//  Ensures engine is off and fins neutralized within one cycle.
// ─────────────────────────────────────────────────────────────────
void ActuatorController::emergency_shutdown() {
    // Engine off
    engine_.set_ignited(false);
    engine_.command(0.0, 0.0, 0.0, 0.0);
    can_send_engine_command(0.0, 0.0, 0.0);

    // Grid fins to zero
    double zero_fins[4] = {0.0, 0.0, 0.0, 0.0};
    for (int i = 0; i < 4; ++i) {
        grid_fins_.command(i, 0.0, 0.001);
        can_send_fin_command(i, 0.0, 300.0);
    }

    // RCS off
    for (int i = 0; i < 12; ++i) {
        fpga_fire_rcs(i, false, 0.0);
    }

    std::cout << "[ACTUATOR] Emergency shutdown complete\n";
}

} // namespace atlas
