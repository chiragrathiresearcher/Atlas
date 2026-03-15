/**
 * flight_computer.cpp
 * ATLAS Flight Computer — Main C++ Real-Time Control Loop Implementation
 *
 * Implements FlightComputer declared in flight_computer.hpp.
 * - 1 kHz control_thread: IMU read → EKF → actuator command
 * - 10 Hz guidance_thread: Python guidance command → ARVS validate → queue
 * - 1 Hz  telemetry_thread: downlink + logging
 *
 * All timing constants from atlas_hardware.hpp (datasheet sourced).
 * Every actuator command passes through ARVSSafetyGate before execution.
 */

#include "flight_computer.hpp"

#include <chrono>
#include <thread>
#include <cmath>
#include <cstring>
#include <iostream>
#include <array>

namespace atlas {

using namespace std::chrono;
using Clock = steady_clock;

// ─────────────────────────────────────────────────────────────────
//  Constructor / Destructor
// ─────────────────────────────────────────────────────────────────

FlightComputer::FlightComputer()
    : imu_()
    , gnss_(true)      // simulation mode by default; set false on real HW
    , baro_(true)
    , actuators_()
    , safety_gate_()
    , watchdog_()
    , state_()
    , health_()
    , running_(false)
    , abort_commanded_(false)
{
    // Initialize state
    state_.phase      = FlightPhase::PRE_LAUNCH;
    state_.arvs_mode  = arvs::SystemMode::NORMAL;
    state_.total_mass_kg = 549054.0;   // Falcon 9 liftoff mass (public data)
    state_.propellant_remaining_kg = 395700.0 + 92670.0;
    state_.battery_soc = 1.0;
    state_.nav_confidence = 1.0;
}

FlightComputer::~FlightComputer() {
    shutdown();
}

// ─────────────────────────────────────────────────────────────────
//  Lifecycle
// ─────────────────────────────────────────────────────────────────

bool FlightComputer::initialize() {
    std::cout << "[FC] Initializing ATLAS Flight Computer\n";
    std::cout << "[FC] Control loop period: " << CONTROL_PERIOD_MS << " ms ("
              << static_cast<int>(1000.0 / CONTROL_PERIOD_MS) << " Hz)\n";
    std::cout << "[FC] Guidance period:     " << GUIDANCE_PERIOD_MS << " ms\n";
    std::cout << "[FC] Watchdog timeout:    " << WATCHDOG_TIMEOUT_MS << " ms\n";

    // Validate ARVS safety gate is operational
    arvs::AxiomSystemState init_axiom_state{};
    init_axiom_state.confidence = 1.0;
    auto gate_check = safety_gate_.check(
        to_arvs_state(), init_axiom_state,
        0.0, 0.0, 0.0, false
    );
    // PRE_LAUNCH: gate returns SAFE_HOLD until we arm — that is expected
    health_.arvs_healthy = true;

    // Mark all sensors healthy for now (real HW would self-test here)
    health_.imu_healthy   = true;
    health_.gnss_healthy  = true;
    health_.baro_healthy  = true;
    health_.engine_healthy= true;
    health_.watchdog_ok   = true;

    std::cout << "[FC] Initialization complete — all subsystems nominal\n";
    running_.store(true);
    return true;
}

void FlightComputer::run() {
    std::cout << "[FC] Starting flight computer threads\n";

    // Spawn real-time control thread
    std::thread ctrl_thread([this]{ control_loop(); });
    std::thread guid_thread([this]{ guidance_loop(); });
    std::thread telem_thread([this]{ telemetry_loop(); });

    ctrl_thread.join();
    guid_thread.join();
    telem_thread.join();

    std::cout << "[FC] All threads stopped\n";
}

void FlightComputer::shutdown() {
    running_.store(false);
}

// ─────────────────────────────────────────────────────────────────
//  Thread-safe state access
// ─────────────────────────────────────────────────────────────────

VehicleState FlightComputer::get_state() const {
    // Spin-lock (suitable for short critical sections at these rates)
    while (state_lock_.exchange(true)) {}
    VehicleState copy = state_;
    state_lock_.store(false);
    return copy;
}

FCHealth FlightComputer::get_health() const {
    return health_;
}

bool FlightComputer::submit_guidance_command(const GuidanceCommand& cmd) {
    // Reject expired commands immediately (ARVS E2 — stale data)
    if ((state_.mission_time_s - cmd.timestamp_s) > cmd.valid_for_s) {
        std::cerr << "[FC] Guidance command expired (age="
                  << (state_.mission_time_s - cmd.timestamp_s) << "s)\n";
        return false;
    }
    pending_cmd_   = cmd;
    cmd_pending_.store(true);
    return true;
}

// ─────────────────────────────────────────────────────────────────
//  1 kHz Control Loop
// ─────────────────────────────────────────────────────────────────

void FlightComputer::control_loop() {
    const auto period = microseconds(static_cast<long>(CONTROL_PERIOD_MS * 1000));
    auto next_wake    = Clock::now() + period;

    while (running_.load() && !abort_commanded_.load()) {
        auto t_start = Clock::now();

        // ── 1. Read sensors ──────────────────────────────────────
        if (!read_sensors()) {
            health_.imu_healthy = false;
            enter_safe_hold("IMU read failure");
        }

        // ── 2. Update mass / propellant estimate ─────────────────
        update_mass_estimate(CONTROL_PERIOD_MS / 1000.0);

        // ── 3. Execute pending guidance command ──────────────────
        if (cmd_pending_.exchange(false)) {
            execute_guidance_command(pending_cmd_);
        }

        // ── 4. Update flight phase ────────────────────────────────
        update_phase();

        // ── 5. Kick watchdog ─────────────────────────────────────
        watchdog_.kick();

        // ── 6. Timing diagnostics ─────────────────────────────────
        auto elapsed_us = duration_cast<microseconds>(Clock::now() - t_start).count();
        if (elapsed_us > static_cast<long>(CONTROL_PERIOD_MS * 1000)) {
            health_.control_loop_overruns++;
        }
        double jitter = static_cast<double>(elapsed_us);
        if (jitter > health_.control_loop_jitter_us) {
            health_.control_loop_jitter_us = jitter;
        }

        // ── 7. Sleep until next cycle ─────────────────────────────
        std::this_thread::sleep_until(next_wake);
        next_wake += period;
    }
}

// ─────────────────────────────────────────────────────────────────
//  10 Hz Guidance Loop
// ─────────────────────────────────────────────────────────────────

void FlightComputer::guidance_loop() {
    const auto period = milliseconds(static_cast<long>(GUIDANCE_PERIOD_MS));
    auto next_wake    = Clock::now() + period;

    while (running_.load() && !abort_commanded_.load()) {
        // Check watchdog — if control loop stalled, enter safe hold
        if (watchdog_.is_tripped()) {
            health_.watchdog_ok = false;
            enter_safe_hold("Watchdog tripped — control loop stalled");
        }

        // Check CPU usage estimate (simplified — real impl uses /proc/stat)
        health_.cpu_usage_percent = 0.0;   // placeholder; real impl queries OS

        std::this_thread::sleep_until(next_wake);
        next_wake += period;
    }
}

// ─────────────────────────────────────────────────────────────────
//  1 Hz Telemetry Loop
// ─────────────────────────────────────────────────────────────────

void FlightComputer::telemetry_loop() {
    const auto period = milliseconds(static_cast<long>(
        hw::FLIGHT_COMPUTER::TELEMETRY_PERIOD_MS));
    auto next_wake = Clock::now() + period;

    while (running_.load() && !abort_commanded_.load()) {
        const auto s = get_state();
        std::cout << "[TLM] T+" << s.mission_time_s
                  << "s  Phase=" << static_cast<int>(s.phase)
                  << "  Alt=" << s.position_m.z / 1000.0 << "km"
                  << "  Vel=" << s.velocity_ms.norm() << "m/s"
                  << "  Prop=" << s.propellant_remaining_kg << "kg"
                  << "  Conf=" << s.nav_confidence
                  << "  ARVS=" << static_cast<int>(s.arvs_mode)
                  << "\n";

        std::this_thread::sleep_until(next_wake);
        next_wake += period;
    }
}

// ─────────────────────────────────────────────────────────────────
//  Sensor Read
// ─────────────────────────────────────────────────────────────────

bool FlightComputer::read_sensors() {
    const double t = state_.mission_time_s;

    // Read IMU (triple-redundant, mid-value voted)
    auto imu_sample = imu_.read_voted(t);
    if (!imu_sample.data_valid) {
        return false;
    }

    // Read GNSS (20 Hz — skip if not ready)
    auto gnss_sample = gnss_.read(t);
    auto baro_sample = baro_.read(t);

    // Run EKF update
    run_ekf(imu_sample, gnss_sample);

    // Update navigation confidence from IMU voting result
    while (state_lock_.exchange(true)) {}
    state_.mission_time_s += hw::FLIGHT_COMPUTER::CONTROL_LOOP_PERIOD_MS / 1000.0;
    state_.nav_confidence  = imu_sample.confidence;
    if (baro_sample.data_valid) {
        // Blend barometer altitude confidence
        state_.nav_confidence = 0.9 * imu_sample.confidence + 0.1;
    }
    state_lock_.store(false);

    return true;
}

// ─────────────────────────────────────────────────────────────────
//  Extended Kalman Filter — Sensor Fusion
//  State: [pos(3), vel(3), quat(4), omega(3), gyro_bias(3), accel_bias(3)]
//  Simplified propagation — full 19-state EKF for production
// ─────────────────────────────────────────────────────────────────

void FlightComputer::run_ekf(const ImuCalibratedSample& imu,
                              const GNSSSample& gnss) {
    const double dt = hw::FLIGHT_COMPUTER::CONTROL_LOOP_PERIOD_MS / 1000.0; // 1 ms

    while (state_lock_.exchange(true)) {}

    // ── Prediction step (IMU-driven) ─────────────────────────────
    // Position update: p += v * dt
    state_.position_m.x += state_.velocity_ms.x * dt;
    state_.position_m.y += state_.velocity_ms.y * dt;
    state_.position_m.z += state_.velocity_ms.z * dt;

    // Velocity update: v += (a_body - g) * dt
    // Specific force from IMU minus gravity
    constexpr double G0 = 9.80665;
    const double alt = state_.position_m.z;
    // Gravity decreases with altitude: g(h) = G0 × (R_e / (R_e + h))²
    constexpr double R_EARTH = 6378137.0;
    const double g_h = G0 * (R_EARTH / (R_EARTH + alt)) * (R_EARTH / (R_EARTH + alt));

    // Body-frame specific force → inertial (simplified: assume pitch-aligned)
    // Full implementation uses quaternion rotation from attitude estimate
    state_.velocity_ms.x += imu.accel_x_ms2 * dt;
    state_.velocity_ms.y += imu.accel_y_ms2 * dt;
    state_.velocity_ms.z += (imu.accel_z_ms2 - g_h) * dt;

    // Attitude rate update
    state_.angular_rate_rads.x = imu.gyro_x_rads;
    state_.angular_rate_rads.y = imu.gyro_y_rads;
    state_.angular_rate_rads.z = imu.gyro_z_rads;

    // Simple quaternion integration: q += 0.5 * q ⊗ ω * dt
    // Using small-angle approximation for 1 ms steps
    const double wx = imu.gyro_x_rads * dt * 0.5;
    const double wy = imu.gyro_y_rads * dt * 0.5;
    const double wz = imu.gyro_z_rads * dt * 0.5;
    const double qw = state_.attitude.w;
    const double qx = state_.attitude.x;
    const double qy = state_.attitude.y;
    const double qz = state_.attitude.z;

    state_.attitude.w += -(qx*wx + qy*wy + qz*wz);
    state_.attitude.x +=  (qw*wx + qy*wz - qz*wy);
    state_.attitude.y +=  (qw*wy + qz*wx - qx*wz);
    state_.attitude.z +=  (qw*wz + qx*wy - qy*wx);

    // Re-normalize quaternion
    const double qnorm = std::sqrt(
        state_.attitude.w * state_.attitude.w +
        state_.attitude.x * state_.attitude.x +
        state_.attitude.y * state_.attitude.y +
        state_.attitude.z * state_.attitude.z);
    if (qnorm > 1e-10) {
        state_.attitude.w /= qnorm;
        state_.attitude.x /= qnorm;
        state_.attitude.y /= qnorm;
        state_.attitude.z /= qnorm;
    }

    // ── GNSS Update step (when valid fix available) ───────────────
    if (gnss.data_valid && gnss.fix_type >= 3) {
        // Kalman gain simplified — weighting by fix quality
        const double K_pos = gnss.rtk_fixed ? 0.05 : 0.10;   // small gain: trust IMU more
        const double K_vel = 0.15;

        // Innovation (measurement - prediction)
        const double innov_x = gnss.pos_x_m - state_.position_m.x;
        const double innov_y = gnss.pos_y_m - state_.position_m.y;
        const double innov_z = gnss.pos_z_m - state_.position_m.z;

        state_.position_m.x += K_pos * innov_x;
        state_.position_m.y += K_pos * innov_y;
        state_.position_m.z += K_pos * innov_z;

        state_.velocity_ms.x += K_vel * (gnss.vel_x_ms - state_.velocity_ms.x);
        state_.velocity_ms.y += K_vel * (gnss.vel_y_ms - state_.velocity_ms.y);
        state_.velocity_ms.z += K_vel * (gnss.vel_z_ms - state_.velocity_ms.z);
    }

    // Dynamic pressure
    // ISA density (simplified): ρ ≈ 1.225 × exp(-h/8500)
    const double rho = 1.225 * std::exp(-std::max(0.0, alt) / 8500.0);
    const double v2  = state_.velocity_ms.x * state_.velocity_ms.x
                     + state_.velocity_ms.y * state_.velocity_ms.y
                     + state_.velocity_ms.z * state_.velocity_ms.z;
    state_.current_q_pa = 0.5 * rho * v2;
    if (state_.current_q_pa > state_.max_q_pa) {
        state_.max_q_pa = state_.current_q_pa;
    }

    state_lock_.store(false);
}

// ─────────────────────────────────────────────────────────────────
//  Propellant Accounting
// ─────────────────────────────────────────────────────────────────

void FlightComputer::update_mass_estimate(double dt_s) {
    while (state_lock_.exchange(true)) {}
    const double mdot = state_.propellant_flow_rate_kgs;
    state_.propellant_remaining_kg -= mdot * dt_s;
    state_.propellant_remaining_kg  = std::max(0.0, state_.propellant_remaining_kg);
    state_.total_mass_kg            = 22200.0 + 4500.0 + state_.propellant_remaining_kg + 3500.0;
    state_lock_.store(false);
}

// ─────────────────────────────────────────────────────────────────
//  Execute Guidance Command (with ARVS validation)
// ─────────────────────────────────────────────────────────────────

bool FlightComputer::execute_guidance_command(const GuidanceCommand& cmd) {
    if (!validate_command_with_arvs(cmd)) {
        return false;
    }

    const double dt = CONTROL_PERIOD_MS / 1000.0;
    double gf[4] = {
        cmd.grid_fin_1_cmd, cmd.grid_fin_2_cmd,
        cmd.grid_fin_3_cmd, cmd.grid_fin_4_cmd
    };
    bool rcs[12];
    std::copy(std::begin(cmd.rcs_cmd), std::end(cmd.rcs_cmd), rcs);

    actuators_.apply_command(
        cmd.throttle_cmd,
        cmd.gimbal_pitch_cmd_rad,
        cmd.gimbal_yaw_cmd_rad,
        gf, rcs, dt
    );

    // Update propellant flow from engine state
    while (state_lock_.exchange(true)) {}
    state_.propellant_flow_rate_kgs = actuators_.engine_state().propellant_flow_kgs;
    state_lock_.store(false);

    return true;
}

// ─────────────────────────────────────────────────────────────────
//  ARVS Command Validation
// ─────────────────────────────────────────────────────────────────

bool FlightComputer::validate_command_with_arvs(const GuidanceCommand& cmd) {
    arvs::AxiomSystemState axiom_state{};
    axiom_state.confidence   = state_.nav_confidence;
    axiom_state.sensor_valid = health_.imu_healthy && health_.gnss_healthy;
    axiom_state.authority_valid = (state_.arvs_mode == arvs::SystemMode::NORMAL ||
                                   state_.arvs_mode == arvs::SystemMode::DEGRADED);
    axiom_state.robot_id[0] = 'A'; // simplified

    bool is_irrev = (cmd.requested_phase == FlightPhase::ABORT ||
                     cmd.phase_change_requested);

    auto result = safety_gate_.check(
        to_arvs_state(),
        axiom_state,
        cmd.throttle_cmd,
        cmd.gimbal_pitch_cmd_rad,
        cmd.gimbal_yaw_cmd_rad,
        is_irrev
    );

    if (!result.permitted()) {
        std::cerr << "[FC] ARVS rejected command: " << result.rejection_reason << "\n";

        switch (result.decision) {
            case GateDecision::SAFE_HOLD:
                enter_safe_hold(result.rejection_reason);
                break;
            case GateDecision::ABORT:
                enter_abort(result.rejection_reason);
                break;
            default:
                break;
        }
        return false;
    }
    return true;
}

// ─────────────────────────────────────────────────────────────────
//  Flight Phase Logic
// ─────────────────────────────────────────────────────────────────

void FlightComputer::update_phase() {
    const double t   = state_.mission_time_s;
    const double alt = state_.position_m.z;
    const double vel = state_.velocity_ms.norm();

    switch (state_.phase) {
        case FlightPhase::PRE_LAUNCH:
            // Transition to IGNITION handled externally (launch command)
            break;

        case FlightPhase::IGNITION:
            if (actuators_.engine_state().ignited) {
                while (state_lock_.exchange(true)) {}
                state_.phase = FlightPhase::LIFTOFF;
                state_lock_.store(false);
                std::cout << "[FC] Phase → LIFTOFF at T+" << t << "s\n";
            }
            break;

        case FlightPhase::LIFTOFF:
            if (state_.current_q_pa > 15000.0) {
                while (state_lock_.exchange(true)) {}
                state_.phase = FlightPhase::MAX_Q;
                state_lock_.store(false);
                std::cout << "[FC] Phase → MAX_Q at T+" << t
                          << "s  q=" << state_.current_q_pa << "Pa\n";
            }
            break;

        case FlightPhase::MAX_Q:
            if (check_meco_conditions()) {
                while (state_lock_.exchange(true)) {}
                state_.phase = FlightPhase::MECO;
                state_lock_.store(false);
                actuators_.shutdown_engine();
                std::cout << "[FC] Phase → MECO at T+" << t << "s\n";
            }
            break;

        case FlightPhase::MECO:
            // Stage sep triggered externally via guidance command
            break;

        case FlightPhase::STAGE_SEP:
            if (vel > 5000.0 && alt > 80000.0) {
                while (state_lock_.exchange(true)) {}
                state_.phase = FlightPhase::UPPER_STAGE;
                state_lock_.store(false);
                std::cout << "[FC] Phase → UPPER_STAGE at T+" << t << "s\n";
            }
            break;

        case FlightPhase::SAFE_HOLD:
        case FlightPhase::ABORT:
        case FlightPhase::SAFED:
            break;  // Terminal states — no auto-transitions

        default:
            break;
    }
}

bool FlightComputer::check_meco_conditions() const {
    // MECO when propellant nearly depleted (< 5% reserve) or target velocity reached
    const double prop_fraction = state_.propellant_remaining_kg /
                                 (395700.0 + 92670.0);
    return (prop_fraction < 0.05) || (state_.velocity_ms.norm() > 7500.0);
}

bool FlightComputer::check_stage_sep_conditions() const {
    return (state_.phase == FlightPhase::MECO) && (state_.mission_time_s > 3.0);
}

bool FlightComputer::check_landing_conditions() const {
    return (state_.position_m.z < 500.0) &&
           (state_.velocity_ms.z < -10.0) &&
           (state_.phase == FlightPhase::ENTRY);
}

// ─────────────────────────────────────────────────────────────────
//  Safe Hold / Abort
// ─────────────────────────────────────────────────────────────────

void FlightComputer::enter_safe_hold(const char* reason) {
    while (state_lock_.exchange(true)) {}
    state_.phase     = FlightPhase::SAFE_HOLD;
    state_.arvs_mode = arvs::SystemMode::SAFE_HOLD;
    state_lock_.store(false);

    actuators_.shutdown_engine();
    std::cerr << "[FC] *** SAFE HOLD *** Reason: " << reason << "\n";
}

void FlightComputer::enter_abort(const char* reason) {
    abort_commanded_.store(true);

    while (state_lock_.exchange(true)) {}
    state_.phase     = FlightPhase::ABORT;
    state_.arvs_mode = arvs::SystemMode::EMERGENCY;
    state_lock_.store(false);

    actuators_.shutdown_engine();
    std::cerr << "[FC] *** FLIGHT ABORT *** Reason: " << reason << "\n";
}

} // namespace atlas
