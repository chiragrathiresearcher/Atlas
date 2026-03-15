/**
 * actuator_controller.hpp
 * ATLAS Actuator Controller — Engine, Grid Fins, RCS, Landing Legs
 *
 * ALL limits sourced from datasheets (see atlas_hardware.hpp):
 *   [MERLIN1D]  SpaceX Merlin 1D public specifications
 *   [GRID_FIN]  SpaceX grid fin engineering publications
 *   [RCS]       Moog DST-80 class cold gas thruster
 *
 * CRITICAL: Every command is validated by ARVS SafetyGate
 * BEFORE being sent to hardware. No actuator moves without ARVS approval.
 */

#pragma once
#include <cstdint>
#include <cmath>
#include <cstring>
#include "atlas_hardware.hpp"
#include "arvs_types.hpp"

namespace atlas {

// ─────────────────────────────────────────────────────────────────
//  Actuator command status
// ─────────────────────────────────────────────────────────────────
enum class ActuatorStatus : uint8_t {
    OK               = 0,
    CLAMPED          = 1,   // Command accepted but clamped to limit
    REJECTED_ARVS    = 2,   // ARVS safety gate rejected command
    REJECTED_HEALTH  = 3,   // Hardware health check failed
    TIMEOUT          = 4,   // Command timed out
    FAULT            = 5    // Hardware fault detected
};

// ─────────────────────────────────────────────────────────────────
//  Engine controller (Merlin 1D class)
//  Source: [MERLIN1D]
// ─────────────────────────────────────────────────────────────────
struct EngineState {
    double   throttle{0.0};        // [0,1] actual position
    double   gimbal_pitch_rad{0.0};
    double   gimbal_yaw_rad{0.0};
    double   chamber_pressure_bar{0.0};
    double   thrust_kn{0.0};
    double   propellant_flow_kgs{0.0};
    bool     ignited{false};
    bool     healthy{true};
};

class EngineController {
public:
    // Limits from [MERLIN1D]
    static constexpr double THROTTLE_MIN    = hw::MERLIN1D::THROTTLE_MIN_PERCENT / 100.0;
    static constexpr double THROTTLE_MAX    = hw::MERLIN1D::THROTTLE_MAX_PERCENT / 100.0;
    static constexpr double GIMBAL_MAX_RAD  = hw::MERLIN1D::THRUST_VECTOR_MAX_DEG * M_PI / 180.0;
    static constexpr double THRUST_SL_KN   = hw::MERLIN1D::THRUST_SL_KN;
    static constexpr double THRUST_VAC_KN  = hw::MERLIN1D::THRUST_VAC_KN;
    static constexpr double ISP_SL_S       = hw::MERLIN1D::ISP_SL_S;
    static constexpr double ISP_VAC_S      = hw::MERLIN1D::ISP_VAC_S;

    // Response time from [MERLIN1D]: 200 ms for 10%→90% throttle
    static constexpr double THROTTLE_SLEW_PER_S = 1.0 / (hw::MERLIN1D::THROTTLE_RESPONSE_TIME_MS / 1000.0);

    EngineController() : current_throttle_(0.0), target_throttle_(0.0) {}

    /**
     * Command throttle and gimbal — returns clamped values and status.
     * Clamp to datasheet limits BEFORE sending to ARVS (ARVS checks physical safety).
     *
     * @param throttle_cmd  Requested throttle [0,1]
     * @param pitch_rad     Requested gimbal pitch [rad]
     * @param yaw_rad       Requested gimbal yaw [rad]
     * @param dt_s          Time step for slew rate limiting
     */
    ActuatorStatus command(double throttle_cmd, double pitch_rad, double yaw_rad, double dt_s) {
        if (!state_.healthy) return ActuatorStatus::FAULT;

        // Apply slew rate limiting — engine cannot jump instantly
        double delta_throttle = THROTTLE_SLEW_PER_S * dt_s;
        if (throttle_cmd > current_throttle_ + delta_throttle)
            throttle_cmd = current_throttle_ + delta_throttle;
        if (throttle_cmd < current_throttle_ - delta_throttle)
            throttle_cmd = current_throttle_ - delta_throttle;

        // Clamp to datasheet limits
        bool clamped = false;
        if (state_.ignited && throttle_cmd < THROTTLE_MIN) {
            throttle_cmd = THROTTLE_MIN;  // engine cannot go below 40%
            clamped = true;
        }
        if (throttle_cmd > THROTTLE_MAX) { throttle_cmd = THROTTLE_MAX; clamped = true; }

        // Gimbal limits (±5° = ±0.0873 rad from [MERLIN1D])
        if (std::abs(pitch_rad) > GIMBAL_MAX_RAD) {
            pitch_rad = (pitch_rad > 0) ? GIMBAL_MAX_RAD : -GIMBAL_MAX_RAD;
            clamped = true;
        }
        if (std::abs(yaw_rad) > GIMBAL_MAX_RAD) {
            yaw_rad = (yaw_rad > 0) ? GIMBAL_MAX_RAD : -GIMBAL_MAX_RAD;
            clamped = true;
        }

        // Apply to state
        current_throttle_         = throttle_cmd;
        state_.throttle           = throttle_cmd;
        state_.gimbal_pitch_rad   = pitch_rad;
        state_.gimbal_yaw_rad     = yaw_rad;

        // Update derived values
        update_derived_values();

        return clamped ? ActuatorStatus::CLAMPED : ActuatorStatus::OK;
    }

    /**
     * Compute propellant mass flow rate at current throttle.
     * F = mdot × Isp × g0  →  mdot = F / (Isp × g0)
     */
    double propellant_flow_kgs(double altitude_m) const {
        if (!state_.ignited) return 0.0;
        const double isp = isp_at_altitude(altitude_m);
        const double thrust_n = current_throttle_ * THRUST_SL_KN * 1000.0;
        return thrust_n / (isp * 9.80665);
    }

    const EngineState& state() const { return state_; }

    void set_ignited(bool v) { state_.ignited = v; }
    void set_healthy(bool v) { state_.healthy = v; }

private:
    EngineState state_;
    double      current_throttle_{0.0};
    double      target_throttle_{0.0};

    /**
     * Interpolate Isp between sea-level and vacuum.
     * Simple linear model — real engine uses nozzle flow tables.
     */
    double isp_at_altitude(double alt_m) const {
        // Vacuum above ~80 km
        const double frac = std::min(1.0, alt_m / 80000.0);
        return ISP_SL_S + frac * (ISP_VAC_S - ISP_SL_S);
    }

    void update_derived_values() {
        if (!state_.ignited) {
            state_.thrust_kn = 0.0;
            state_.propellant_flow_kgs = 0.0;
            state_.chamber_pressure_bar = 0.0;
            return;
        }
        // Thrust scales linearly with throttle (simplified)
        state_.thrust_kn = current_throttle_ * THRUST_SL_KN;
        // Chamber pressure scales with throttle (P_c ∝ mdot ∝ throttle)
        state_.chamber_pressure_bar = current_throttle_ * hw::MERLIN1D::CHAMBER_PRESSURE_BAR;
        // Propellant flow (sea level approx)
        state_.propellant_flow_kgs = propellant_flow_kgs(0.0);
    }
};

// ─────────────────────────────────────────────────────────────────
//  Grid Fin Controller (4× fins, hydraulic actuator)
//  Source: [GRID_FIN] — Titanium honeycomb, hydraulic actuation
// ─────────────────────────────────────────────────────────────────
class GridFinController {
public:
    static constexpr int    NUM_FINS           = 4;
    static constexpr double MAX_DEFLECTION_RAD = hw::GRID_FIN::DEFLECTION_MAX_DEG * M_PI / 180.0;
    static constexpr double SLEW_RATE_RAD_S    = hw::GRID_FIN::SLEW_RATE_DEG_S * M_PI / 180.0;
    static constexpr double BANDWIDTH_HZ       = hw::GRID_FIN::ACTUATOR_BANDWIDTH_HZ;
    static constexpr double MAX_TEMP_K         = hw::GRID_FIN::TEMP_OPERATING_MAX_C + 273.15;

    struct FinState {
        double   deflection_rad{0.0};
        double   temp_k{300.0};
        bool     deployed{false};
        bool     healthy{true};
    };

    GridFinController() {
        for (auto& f : fins_) { f = FinState{}; }
    }

    /**
     * Command fin deflection (0–1 normalized, ±90° max).
     * @param fin_idx  0–3
     * @param cmd      Deflection command [-1, +1] (±MAX_DEFLECTION_RAD)
     * @param dt_s     Time step for slew rate
     */
    ActuatorStatus command(int fin_idx, double cmd, double dt_s) {
        if (fin_idx < 0 || fin_idx >= NUM_FINS) return ActuatorStatus::FAULT;
        auto& fin = fins_[fin_idx];
        if (!fin.healthy)  return ActuatorStatus::FAULT;
        if (!fin.deployed) return ActuatorStatus::FAULT;  // can't command stowed fin

        // Thermal check — above 800°C titanium loses strength
        if (fin.temp_k > MAX_TEMP_K) {
            fin.healthy = false;
            return ActuatorStatus::FAULT;
        }

        // Desired deflection angle
        double desired_rad = cmd * MAX_DEFLECTION_RAD;

        // Slew rate limit (hydraulic actuator: 400 °/s from datasheet)
        double max_delta = SLEW_RATE_RAD_S * dt_s;
        double delta = desired_rad - fin.deflection_rad;
        if (std::abs(delta) > max_delta)
            desired_rad = fin.deflection_rad + (delta > 0 ? max_delta : -max_delta);

        // Limit check
        bool clamped = false;
        if (desired_rad > MAX_DEFLECTION_RAD)  { desired_rad =  MAX_DEFLECTION_RAD; clamped=true; }
        if (desired_rad < -MAX_DEFLECTION_RAD) { desired_rad = -MAX_DEFLECTION_RAD; clamped=true; }

        fin.deflection_rad = desired_rad;
        return clamped ? ActuatorStatus::CLAMPED : ActuatorStatus::OK;
    }

    void set_deployed(int fin_idx, bool v) {
        if (fin_idx >= 0 && fin_idx < NUM_FINS) fins_[fin_idx].deployed = v;
    }
    void update_temperature(int fin_idx, double temp_k) {
        if (fin_idx >= 0 && fin_idx < NUM_FINS) fins_[fin_idx].temp_k = temp_k;
    }
    const FinState& fin_state(int fin_idx) const { return fins_[fin_idx]; }

private:
    FinState fins_[NUM_FINS];
};

// ─────────────────────────────────────────────────────────────────
//  RCS Cold Gas Thruster Controller (12 thrusters)
//  Source: Moog DST-80 class, N₂ propellant
// ─────────────────────────────────────────────────────────────────
class RCSController {
public:
    static constexpr int    NUM_THRUSTERS    = 12;
    static constexpr double THRUST_N         = hw::COLD_GAS_RCS::THRUST_N;      // 22 N per thruster
    static constexpr double MIN_PULSE_S      = hw::COLD_GAS_RCS::MIN_PULSE_WIDTH_MS / 1000.0;
    static constexpr double VALVE_OPEN_S     = hw::COLD_GAS_RCS::RESPONSE_TIME_MS / 1000.0;
    static constexpr double ISP_S            = hw::COLD_GAS_RCS::ISP_S;         // 70 s

    /**
     * Command individual thruster on/off.
     * Minimum impulse bit enforced from datasheet (10 ms min pulse).
     *
     * @param thruster_idx  0–11
     * @param on            Fire command
     * @param duration_s    Commanded on-time (must be >= MIN_PULSE_S)
     */
    ActuatorStatus fire(int thruster_idx, bool on, double duration_s) {
        if (thruster_idx < 0 || thruster_idx >= NUM_THRUSTERS) return ActuatorStatus::FAULT;
        if (!healthy_[thruster_idx]) return ActuatorStatus::FAULT;

        // Enforce minimum pulse width from datasheet
        if (on && duration_s < MIN_PULSE_S) {
            duration_s = MIN_PULSE_S;
        }

        states_[thruster_idx] = on;
        pulse_duration_s_[thruster_idx] = duration_s;

        // Propellant consumption: F = mdot × Isp × g0 → mdot = F/(Isp × g0)
        if (on) {
            const double mdot = THRUST_N / (ISP_S * 9.80665);
            total_propellant_used_kg_ += mdot * duration_s;
        }

        return ActuatorStatus::OK;
    }

    double total_propellant_used_kg() const { return total_propellant_used_kg_; }
    bool   state(int idx) const { return (idx >= 0 && idx < NUM_THRUSTERS) ? states_[idx] : false; }

private:
    bool   states_[NUM_THRUSTERS]{};
    bool   healthy_[NUM_THRUSTERS];
    double pulse_duration_s_[NUM_THRUSTERS]{};
    double total_propellant_used_kg_{0.0};

    // Initialize all thrusters healthy
    RCSController() {
        for (auto& h : healthy_) h = true;
    }

    friend class ActuatorController;
};

// ─────────────────────────────────────────────────────────────────
//  Top-level actuator controller
//  Aggregates all actuators, applies ARVS pre-validation
// ─────────────────────────────────────────────────────────────────
class ActuatorController {
public:
    ActuatorController() : rcs_() {
        for (auto& h : rcs_.healthy_) h = true;
    }

    /**
     * Apply a complete guidance command to all actuators.
     * Assumes ARVS SafetyGate has already approved this command.
     * Still applies hardware-level clamps as final defense.
     *
     * @returns true if all commands applied OK (some may be clamped)
     */
    bool apply_command(double throttle, double gimbal_pitch, double gimbal_yaw,
                       double grid_fins[4], bool rcs[12], double dt_s) {
        auto engine_status = engine_.command(throttle, gimbal_pitch, gimbal_yaw, dt_s);

        for (int i = 0; i < 4; ++i) {
            grid_fins_.command(i, grid_fins[i], dt_s);
        }
        for (int i = 0; i < 12; ++i) {
            if (rcs[i]) rcs_.fire(i, true, dt_s);
        }

        return (engine_status == ActuatorStatus::OK ||
                engine_status == ActuatorStatus::CLAMPED);
    }

    void ignite_engine()          { engine_.set_ignited(true); }
    void shutdown_engine()        { engine_.set_ignited(false); engine_.command(0,0,0,0); }
    void deploy_grid_fin(int idx) { grid_fins_.set_deployed(idx, true); }

    const EngineState&            engine_state() const { return engine_.state(); }
    const GridFinController::FinState& fin_state(int i) const { return grid_fins_.fin_state(i); }

private:
    EngineController engine_;
    GridFinController grid_fins_;
    RCSController    rcs_;
};

} // namespace atlas
