/**
 * flight_computer.hpp
 * ATLAS Flight Computer — Main C++ Real-Time Control Layer
 *
 * Responsibilities:
 *   1. 1 kHz sensor acquisition (IMU, GNSS, baro)
 *   2. 10 Hz ARVS safety-gated guidance command execution
 *   3. 1 kHz actuator commands (engine throttle, gimbal, RCS, grid fins)
 *   4. Hardware watchdog management
 *   5. ARVS axiom validation on every control cycle
 *
 * Thread model:
 *   - control_thread  : 1 kHz RT thread — IMU read + actuator command
 *   - guidance_thread : 10 Hz — receive guidance from Python layer, validate via ARVS
 *   - telemetry_thread: 1 Hz  — downlink + logging
 *
 * ARVS Integration:
 *   Every actuator command is validated by ARVS SafetyGate BEFORE execution.
 *   If ARVS rejects a command, the flight computer enters SAFE_HOLD.
 */

#pragma once
#include <cstdint>
#include <atomic>
#include <array>
#include "atlas_hardware.hpp"
#include "arvs_types.hpp"
#include "imu_driver.hpp"
#include "gnss_driver.hpp"
#include "actuator_controller.hpp"
#include "safety_gate.hpp"
#include "watchdog.hpp"

namespace atlas {

// ─────────────────────────────────────────────────────────────────
//  Flight phase enumeration
// ─────────────────────────────────────────────────────────────────
enum class FlightPhase : uint8_t {
    PRE_LAUNCH        = 0,
    IGNITION          = 1,
    LIFTOFF           = 2,
    MAX_Q             = 3,
    MECO              = 4,   // Main Engine Cut-Off
    STAGE_SEP         = 5,
    UPPER_STAGE       = 6,
    PAYLOAD_DEPLOY    = 7,
    BOOSTBACK         = 8,   // Reusable — boostback burn
    ENTRY             = 9,   // Reusable — atmospheric entry
    LANDING_BURN      = 10,  // Reusable — landing burn
    TOUCHDOWN         = 11,
    SAFE_HOLD         = 253, // ARVS-commanded safe state
    ABORT             = 254, // Flight termination
    SAFED             = 255  // Post-abort / post-flight safe state
};

// ─────────────────────────────────────────────────────────────────
//  Guidance command (from Python guidance layer → C++ executor)
//  Validated by ARVS before execution
// ─────────────────────────────────────────────────────────────────
struct GuidanceCommand {
    double   timestamp_s{0.0};

    // Engine commands
    double   throttle_cmd{0.0};         // 0–1 (normalized)
    double   gimbal_pitch_cmd_rad{0.0}; // ±5° = ±0.0873 rad
    double   gimbal_yaw_cmd_rad{0.0};

    // Grid fin commands (0–1 normalized deflection)
    double   grid_fin_1_cmd{0.0};
    double   grid_fin_2_cmd{0.0};
    double   grid_fin_3_cmd{0.0};
    double   grid_fin_4_cmd{0.0};

    // RCS commands (thrusters 0–11 on/off duty cycle)
    bool     rcs_cmd[12]{};

    // Phase transition request
    FlightPhase requested_phase{FlightPhase::PRE_LAUNCH};
    bool        phase_change_requested{false};

    // Validity window — command expires after this duration
    double   valid_for_s{0.2};   // 200 ms max command age

    // ARVS authority token (set by ARVS adapter, checked by safety gate)
    uint64_t arvs_authority_token{0};
};

// ─────────────────────────────────────────────────────────────────
//  Vehicle state (updated at 1 kHz, read by guidance at 10 Hz)
// ─────────────────────────────────────────────────────────────────
struct VehicleState {
    double   mission_time_s{0.0};

    // Navigation state (from GNSS + IMU fusion)
    arvs::Vec3  position_m{};         // m, launch site frame
    arvs::Vec3  velocity_ms{};        // m/s
    arvs::Quat  attitude{};           // quaternion
    arvs::Vec3  angular_rate_rads{};  // rad/s

    // Mass / propellant (from fuel flow integrator)
    double   total_mass_kg{0.0};
    double   propellant_remaining_kg{0.0};
    double   propellant_flow_rate_kgs{0.0};

    // Structural
    double   max_q_pa{0.0};             // max dynamic pressure seen
    double   current_q_pa{0.0};         // current dynamic pressure
    double   axial_load_factor_g{0.0};

    // Thermal
    double   engine_temp_k{0.0};        // from thermocouple
    double   avionics_temp_c{0.0};      // avionics bay
    double   tank_ullage_temp_k{0.0};

    // Power
    double   battery_soc{1.0};          // state of charge [0,1]
    double   power_consumption_w{0.0};

    // Confidence (from ARVS HAL)
    double   nav_confidence{1.0};       // 0–1

    // Flight phase
    FlightPhase phase{FlightPhase::PRE_LAUNCH};

    // ARVS system mode
    arvs::SystemMode arvs_mode{arvs::SystemMode::NORMAL};
};

// ─────────────────────────────────────────────────────────────────
//  Flight computer health diagnostics
// ─────────────────────────────────────────────────────────────────
struct FCHealth {
    bool     imu_healthy{false};
    bool     gnss_healthy{false};
    bool     baro_healthy{false};
    bool     engine_healthy{false};
    bool     arvs_healthy{false};
    bool     watchdog_ok{false};
    uint32_t control_loop_overruns{0};
    double   control_loop_jitter_us{0.0};  // max jitter seen, microseconds
    double   cpu_usage_percent{0.0};
};

// ─────────────────────────────────────────────────────────────────
//  ATLAS Flight Computer
// ─────────────────────────────────────────────────────────────────
class FlightComputer {
public:
    // Control loop timing from hardware constants
    static constexpr double CONTROL_PERIOD_MS  = hw::FLIGHT_COMPUTER::CONTROL_LOOP_PERIOD_MS;
    static constexpr double GUIDANCE_PERIOD_MS = hw::FLIGHT_COMPUTER::GUIDANCE_LOOP_PERIOD_MS;
    static constexpr double WATCHDOG_TIMEOUT_MS= hw::FLIGHT_COMPUTER::WATCHDOG_TIMEOUT_MS;

    FlightComputer();
    ~FlightComputer();

    // Lifecycle
    bool initialize();
    void run();        // Blocking — runs until shutdown or abort
    void shutdown();

    // State access (thread-safe, for guidance/telemetry threads)
    VehicleState get_state() const;
    FCHealth     get_health() const;

    // Guidance command injection (from Python guidance layer via shared memory / IPC)
    bool submit_guidance_command(const GuidanceCommand& cmd);

    // ARVS interface (used by safety gate to query current state)
    arvs::RobotState to_arvs_state() const;

private:
    // ── Hardware drivers ─────────────────────────────────────────
    TripleRedundantIMU  imu_;
    GNSSDriver          gnss_;
    BarometerDriver     baro_;
    ActuatorController  actuators_;

    // ── ARVS safety layer ────────────────────────────────────────
    ARVSSafetyGate      safety_gate_;
    WatchdogTimer       watchdog_;

    // ── State (atomically updated) ───────────────────────────────
    mutable std::atomic<bool>  state_lock_{false};
    VehicleState               state_{};
    FCHealth                   health_{};

    // ── Pending guidance command ─────────────────────────────────
    mutable std::atomic<bool>  cmd_pending_{false};
    GuidanceCommand            pending_cmd_{};

    // ── Control loop ─────────────────────────────────────────────
    void control_loop();     // 1 kHz
    void guidance_loop();    // 10 Hz
    void telemetry_loop();   // 1 Hz

    bool read_sensors();
    bool execute_guidance_command(const GuidanceCommand& cmd);
    bool validate_command_with_arvs(const GuidanceCommand& cmd);
    void update_phase();
    void enter_safe_hold(const char* reason);
    void enter_abort(const char* reason);

    // ── Navigation ───────────────────────────────────────────────
    void run_ekf(const ImuCalibratedSample& imu, const GNSSSample& gnss);

    // ── Propellant accounting ────────────────────────────────────
    void update_mass_estimate(double dt_s);

    // ── Phase transition logic ───────────────────────────────────
    bool check_meco_conditions() const;
    bool check_stage_sep_conditions() const;
    bool check_landing_conditions() const;

    std::atomic<bool> running_{false};
    std::atomic<bool> abort_commanded_{false};
};

// ─────────────────────────────────────────────────────────────────
//  Inline: ARVS state conversion
// ─────────────────────────────────────────────────────────────────
inline arvs::RobotState FlightComputer::to_arvs_state() const {
    const auto s = get_state();
    arvs::RobotState rs;

    // Robot ID
    std::strncpy(rs.robot_id, "ATLAS_V1", arvs::ROBOT_ID_LEN - 1);

    rs.timestamp          = s.mission_time_s;
    rs.position           = s.position_m;
    rs.velocity           = s.velocity_ms;
    rs.orientation        = s.attitude;
    rs.angular_velocity   = s.angular_rate_rads;
    rs.temperature        = s.engine_temp_k;
    rs.battery_level      = s.battery_soc;
    rs.power_consumption  = s.power_consumption_w;
    rs.confidence         = s.nav_confidence;

    // Map joints to actuator states (gimbal as joint angles)
    rs.n_joints = 2;
    std::strncpy(rs.joints[0].name, "gimbal_pitch", 31);
    rs.joints[0].position = 0.0;   // populated by actuator controller
    std::strncpy(rs.joints[1].name, "gimbal_yaw", 31);
    rs.joints[1].position = 0.0;

    return rs;
}

} // namespace atlas
