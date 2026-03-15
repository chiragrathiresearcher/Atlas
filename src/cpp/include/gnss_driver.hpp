/**
 * gnss_driver.hpp
 * ATLAS GNSS Driver — u-blox ZED-F9P (Dual-antenna RTK)
 *
 * Source: [ZEF9P] u-blox ZED-F9P Integration Manual UBX-17051259 Rev 2.0
 * All performance constants from datasheet Table 1.
 */

#pragma once
#include <cmath>
#include <cstring>
#include "atlas_hardware.hpp"
#include "arvs_types.hpp"

namespace atlas {

struct GNSSSample {
    double   timestamp_s{0.0};
    uint32_t sequence{0};

    // Position in Earth-Centered Earth-Fixed (ECEF), meters
    double   pos_x_m{0.0};
    double   pos_y_m{0.0};
    double   pos_z_m{0.0};

    // Velocity ECEF, m/s
    double   vel_x_ms{0.0};
    double   vel_y_ms{0.0};
    double   vel_z_ms{0.0};

    // Position covariance (3x3 diagonal, m²) — from ZED-F9P UBX-NAV-COV message
    double   cov_xx{0.0};
    double   cov_yy{0.0};
    double   cov_zz{0.0};

    // Velocity covariance (3x3 diagonal, m²/s²)
    double   vcov_xx{0.0};
    double   vcov_yy{0.0};
    double   vcov_zz{0.0};

    // Fix quality
    uint8_t  fix_type{0};     // 0=no fix, 1=DR, 2=2D, 3=3D, 4=GNSS+DR, 5=time only
    bool     rtk_fixed{false};// True = RTK fixed solution (0.01 m CEP)
    bool     rtk_float{false};// True = RTK float solution (0.20 m CEP)

    // Integrity
    uint8_t  satellites_used{0};
    double   hdop{99.9};     // Horizontal dilution of precision
    double   vdop{99.9};
    bool     data_valid{false};

    /**
     * Position accuracy based on fix type.
     * Returns 1-sigma horizontal accuracy in meters.
     * From [ZEF9P] Table 1.
     */
    double horizontal_accuracy_m() const {
        if (rtk_fixed)      return hw::ZED_F9P::RTK_HORIZONTAL_CEP_M;
        if (rtk_float)      return hw::ZED_F9P::DGNSS_HORIZONTAL_M;
        if (fix_type >= 3)  return hw::ZED_F9P::AUTONOMOUS_HORIZONTAL_M;
        return 1000.0;  // degraded — no valid fix
    }
};

// ─────────────────────────────────────────────────────────────────
//  ZED-F9P simulation driver
//  On hardware: wraps UBX binary protocol over UART @ 921600 baud
//  In simulation: generates RTK-quality position from trajectory model
// ─────────────────────────────────────────────────────────────────
class GNSSDriver {
public:
    static constexpr double UPDATE_RATE_HZ    = hw::ZED_F9P::MAX_NAV_RATE_HZ;   // 20 Hz
    static constexpr double RTK_CEP_M         = hw::ZED_F9P::RTK_HORIZONTAL_CEP_M;
    static constexpr double VELOCITY_SIGMA_MS = hw::ZED_F9P::VELOCITY_ACCURACY_MPS;

    explicit GNSSDriver(bool simulation_mode = true)
        : sim_mode_(simulation_mode), seq_(0) {}

    /**
     * Read one GNSS navigation solution.
     * In SIL mode: generates deterministic trajectory-based position.
     * In HIL mode: reads UBX-NAV-PVT message from serial port.
     *
     * @param mission_time_s  Time since liftoff
     */
    GNSSSample read(double mission_time_s) {
        GNSSSample s;
        s.timestamp_s = mission_time_s;
        s.sequence    = seq_++;

        if (!sim_mode_) {
            // Real hardware read — parse UBX-NAV-PVT message
            // Placeholder: on flight hardware, replace with serial read
            s.data_valid = false;
            return s;
        }

        // ── Simulation: gravity-turn trajectory model ──────────────
        // Reference trajectory: 500 km LEO mission, due East launch
        // Phase 0–120s: vertical + gravity turn
        // Phase 120–300s: gravity turn completion
        // Phase 300+s: near-horizontal, 7800 m/s orbital velocity

        const double t = mission_time_s;

        // Simplified trajectory (ECEF-aligned with launch site at equator for clarity)
        // Altitude profile: ~300 m/s vertical at T+60s, gravity turn
        double alt_m;
        double down_range_m;
        double vel_vert_ms;
        double vel_horiz_ms;

        if (t < 5.0) {
            // Ignition to liftoff
            alt_m          = 0.0 + 0.5 * t * t;
            down_range_m   = 0.0;
            vel_vert_ms    = t;
            vel_horiz_ms   = 0.0;
        } else if (t < 120.0) {
            // Gravity turn phase
            double t2      = t - 5.0;
            alt_m          = 12.5 + 30.0 * t2 + 0.5 * t2 * t2 * 0.3;
            down_range_m   = 0.5 * 2.0 * t2 * t2;
            vel_vert_ms    = 5.0 + 1.0 * t2;
            vel_horiz_ms   = 2.0 * t2;
        } else {
            // Post MECO approximate orbit insertion
            double t3      = t - 120.0;
            alt_m          = 100000.0 + 200.0 * t3;
            down_range_m   = 50000.0 + 7500.0 * t3;
            vel_vert_ms    = 200.0 - 0.5 * t3;
            vel_horiz_ms   = 7000.0 + 1.5 * t3;
        }

        // Convert to nominal ECEF (launch site assumed at 0° lat, 0° lon for simulation)
        // Real system: uses actual launch site coordinates from mission config
        const double R_earth = 6378137.0;  // m (WGS-84)
        s.pos_x_m = R_earth + alt_m;
        s.pos_y_m = down_range_m;
        s.pos_z_m = 0.0;

        s.vel_x_ms = vel_vert_ms;
        s.vel_y_ms = vel_horiz_ms;
        s.vel_z_ms = 0.0;

        // ── RTK-quality covariance (from ZED-F9P datasheet) ───────
        // Horizontal: (RTK_CEP/√2)² variance
        const double h_var = (RTK_CEP_M * RTK_CEP_M) / 2.0;
        const double v_var = (hw::ZED_F9P::RTK_VERTICAL_CEP_M
                              * hw::ZED_F9P::RTK_VERTICAL_CEP_M) / 2.0;
        s.cov_xx = h_var;
        s.cov_yy = h_var;
        s.cov_zz = v_var;

        const double vel_var = VELOCITY_SIGMA_MS * VELOCITY_SIGMA_MS;
        s.vcov_xx = vel_var;
        s.vcov_yy = vel_var;
        s.vcov_zz = vel_var;

        // Fix quality
        s.rtk_fixed       = (t > 2.0);   // After 2s, assume RTK acquired
        s.fix_type        = 3;
        s.satellites_used = 12;           // Typical open-sky count
        s.hdop            = 0.8;          // Good PDOP for RTK
        s.vdop            = 1.1;
        s.data_valid      = true;

        // Add bounded measurement noise (deterministic, not random)
        const double h_noise = RTK_CEP_M * bounded_noise(t * 13.71);
        const double v_noise = hw::ZED_F9P::RTK_VERTICAL_CEP_M * bounded_noise(t * 7.31);
        s.pos_x_m += h_noise;
        s.pos_y_m += h_noise * 0.7;
        s.pos_z_m += v_noise;
        s.vel_x_ms += VELOCITY_SIGMA_MS * bounded_noise(t * 3.14);
        s.vel_y_ms += VELOCITY_SIGMA_MS * bounded_noise(t * 2.71);

        return s;
    }

    /**
     * Altitude above launch site from ECEF position.
     * @param s  GNSS sample
     * @returns  altitude in meters
     */
    static double altitude_m(const GNSSSample& s) {
        const double R_earth = 6378137.0;
        return std::sqrt(s.pos_x_m * s.pos_x_m
                       + s.pos_y_m * s.pos_y_m
                       + s.pos_z_m * s.pos_z_m) - R_earth;
    }

private:
    bool     sim_mode_;
    uint32_t seq_;

    static double bounded_noise(double phase) {
        return 0.5 * std::sin(phase * 113.1)
             + 0.3 * std::sin(phase * 271.8)
             + 0.2 * std::sin(phase * 397.2);
    }
};

// ─────────────────────────────────────────────────────────────────
//  Barometer — TE MS5611-01BA03
//  Source: [MS5611] Application Note AN520
// ─────────────────────────────────────────────────────────────────
struct BaroSample {
    double  timestamp_s{0.0};
    double  pressure_pa{101325.0};  // Pa
    double  temperature_c{20.0};    // °C (from internal sensor)
    double  altitude_m{0.0};        // derived via hypsometric formula
    bool    data_valid{false};
};

class BarometerDriver {
public:
    static constexpr double SAMPLE_RATE_HZ  = 50.0;   // Practical rate (OSR=4096 @ 9ms)
    static constexpr double RESOLUTION_PA   = 1.2;    // Pa (from [MS5611] OSR=4096)
    static constexpr double ALTITUDE_RES_CM = hw::MS5611::ALTITUDE_RESOLUTION_CM;

    explicit BarometerDriver(bool sim_mode = true) : sim_mode_(sim_mode), seq_(0) {}

    BaroSample read(double mission_time_s) {
        BaroSample s;
        s.timestamp_s = mission_time_s;
        s.data_valid  = true;

        if (!sim_mode_) {
            // On hardware: read MS5611 via SPI, apply D1/D2 compensation algorithm
            // from [MS5611] AN520 Section 4
            s.data_valid = false;
            return s;
        }

        // Standard atmosphere model (ISA) for simulation
        // Valid to ~32 km
        const double alt = compute_trajectory_altitude(mission_time_s);
        s.temperature_c  = std::max(-56.5, 15.0 - 6.5 * alt / 1000.0);  // lapse rate
        const double T_K = s.temperature_c + 273.15;
        const double P0  = 101325.0;   // Pa (sea level ISA)
        const double T0  = 288.15;     // K
        const double L   = 0.0065;     // K/m lapse rate
        const double g   = 9.80665;    // m/s²
        const double R   = 287.058;    // J/(kg·K)

        // Hypsometric formula (troposphere, <11 km)
        if (alt < 11000.0) {
            s.pressure_pa = P0 * std::pow(T_K / T0, -g / (L * R));
        } else {
            // Stratosphere (constant T = -56.5°C)
            const double P11 = P0 * std::pow(216.65 / T0, -g / (L * R));
            s.pressure_pa = P11 * std::exp(-g * (alt - 11000.0) / (R * 216.65));
        }

        // Derived altitude (back from pressure)
        s.altitude_m = alt;

        // Measurement noise: ±1.2 Pa (OSR=4096 resolution)
        s.pressure_pa += RESOLUTION_PA * bounded_noise(mission_time_s * 17.3);

        return s;
    }

private:
    bool     sim_mode_;
    uint32_t seq_;

    static double compute_trajectory_altitude(double t) {
        if (t < 5.0)   return 0.5 * t * t;
        if (t < 120.0) return 12.5 + 30.0 * (t-5.0) + 0.5 * 0.3 * (t-5.0)*(t-5.0);
        return 100000.0 + 200.0 * (t - 120.0);
    }

    static double bounded_noise(double p) {
        return 0.6 * std::sin(p * 79.3) + 0.4 * std::sin(p * 143.7);
    }
};

} // namespace atlas
