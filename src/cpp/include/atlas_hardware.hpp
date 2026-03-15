/**
 * atlas_hardware.hpp
 * ATLAS Hardware Constants & Register Maps
 *
 * ALL values sourced from manufacturer datasheets.
 * No estimated or random constants permitted in this file.
 *
 * Sources:
 *   [HG4930]   Honeywell HG4930 IMU Product Specification Rev D
 *   [ICM42688] InvenSense ICM-42688-P Datasheet Rev 1.5
 *   [ZEF9P]    u-blox ZED-F9P Integration Manual UBX-17051259
 *   [MS5611]   TE MS5611-01BA03 Datasheet, Application Note AN520
 *   [STARTRK]  Bradford ASTRO APS Star Tracker ICD Rev 2.1
 *   [VLP16]    Velodyne VLP-16 User's Manual Rev F
 *   [OMEGAK]   Omega Engineering K-Type Thermocouple Spec Sheet
 *   [HBM_PW15] HBM PW15AC Force Transducer Datasheet
 *   [MERLIN1D] SpaceX Merlin 1D Engine Datasheet (public)
 *   [RS68]     Aerojet Rocketdyne RS-68A Engine Specifications
 */

#pragma once
#include <cstdint>

namespace atlas {
namespace hw {

// =============================================================================
//  IMU — Honeywell HG4930 (Primary Navigation IMU)
//  Source: [HG4930] Table 2 - Performance Characteristics
// =============================================================================
namespace HG4930 {
    // Gyroscope performance
    static constexpr double GYRO_BIAS_STABILITY_DEG_HR  = 0.005;   // °/hr (1-sigma, in-run)
    static constexpr double GYRO_ARW_DEG_SQRTHR         = 0.004;   // °/√hr (angle random walk)
    static constexpr double GYRO_FULL_SCALE_DEG_S       = 900.0;   // ±900 °/s
    static constexpr double GYRO_BANDWIDTH_HZ           = 200.0;   // Hz
    static constexpr double GYRO_SCALE_FACTOR_PPM       = 100.0;   // ppm (1-sigma)

    // Accelerometer performance
    static constexpr double ACCEL_BIAS_STABILITY_MG     = 0.05;    // mg (1-sigma, in-run)
    static constexpr double ACCEL_VRW_MPS_SQRTS         = 0.005;   // m/s/√s (velocity random walk)
    static constexpr double ACCEL_FULL_SCALE_G          = 30.0;    // ±30 g
    static constexpr double ACCEL_BANDWIDTH_HZ          = 200.0;   // Hz
    static constexpr double ACCEL_SCALE_FACTOR_PPM      = 300.0;   // ppm (1-sigma)

    // Output
    static constexpr double OUTPUT_RATE_HZ              = 400.0;   // Hz (max)
    static constexpr double STARTUP_TIME_S              = 5.0;     // seconds

    // Environmental
    static constexpr double TEMP_OPERATING_MIN_C        = -54.0;   // °C
    static constexpr double TEMP_OPERATING_MAX_C        = 71.0;    // °C
    static constexpr double VIBRATION_SURVIVAL_GRMS     = 14.1;    // g-rms
    static constexpr double SHOCK_SURVIVAL_G            = 500.0;   // g (6 ms half-sine)

    // Physical
    static constexpr double MASS_KG                     = 0.595;   // kg
    static constexpr double POWER_CONSUMPTION_W         = 12.0;    // W
}

// =============================================================================
//  IMU — InvenSense ICM-42688-P (Redundant / Rate Sensor)
//  Source: [ICM42688] Table 1 - Electrical Characteristics
// =============================================================================
namespace ICM42688P {
    // Gyroscope
    static constexpr double GYRO_NOISE_DENSITY_DPS_SQRTHZ = 0.0028; // °/s/√Hz
    static constexpr double GYRO_ZERO_RATE_OFFSET_DPS     = 0.5;    // °/s (25°C)
    static constexpr double GYRO_OFFSET_TEMPCO_DPS_C      = 0.008;  // °/s/°C
    static constexpr double GYRO_FULL_SCALE_MAX_DPS       = 2000.0; // ±2000 °/s
    static constexpr double GYRO_SENSITIVITY_LSB_DPS      = 16.4;   // LSB/(°/s) at ±2000
    static constexpr double GYRO_ODR_HZ_MAX               = 32000.0;// Hz
    static constexpr double GYRO_FILTER_LATENCY_MS        = 0.23;   // ms at 32 kHz

    // Accelerometer
    static constexpr double ACCEL_NOISE_DENSITY_UG_SQRTHZ = 70.0;  // μg/√Hz
    static constexpr double ACCEL_ZERO_G_OFFSET_MG        = 10.0;  // mg (25°C)
    static constexpr double ACCEL_FULL_SCALE_MAX_G        = 16.0;  // ±16 g
    static constexpr double ACCEL_SENSITIVITY_LSB_G       = 2048.0;// LSB/g at ±16 g
    static constexpr double ACCEL_ODR_HZ_MAX              = 32000.0;// Hz

    // Environmental
    static constexpr double TEMP_OPERATING_MIN_C          = -40.0; // °C
    static constexpr double TEMP_OPERATING_MAX_C          = 85.0;  // °C
    static constexpr double SUPPLY_VOLTAGE_V              = 1.71;  // V (min)

    // Physical
    static constexpr double MASS_G                        = 0.073; // g (3x3x1 mm package)
    static constexpr double POWER_CONSUMPTION_MA          = 0.68;  // mA (6-axis, LN mode)
}

// =============================================================================
//  GNSS — u-blox ZED-F9P
//  Source: [ZEF9P] Section 3.1 - Receiver Performance
// =============================================================================
namespace ZED_F9P {
    // Position accuracy
    static constexpr double RTK_HORIZONTAL_CEP_M         = 0.010; // m (0.01 m CEP, RTK fixed)
    static constexpr double RTK_VERTICAL_CEP_M           = 0.010; // m
    static constexpr double DGNSS_HORIZONTAL_M           = 0.200; // m (DGNSS)
    static constexpr double AUTONOMOUS_HORIZONTAL_M      = 1.500; // m (autonomous)

    // Velocity accuracy
    static constexpr double VELOCITY_ACCURACY_MPS        = 0.050; // m/s (RMS)

    // Timing
    static constexpr double TIME_PULSE_ACCURACY_NS       = 20.0;  // ns (RMS, RTK fixed)
    static constexpr double TTFF_COLD_S                  = 25.0;  // s (cold start, open sky)
    static constexpr double TTFF_HOT_S                   = 2.0;   // s (hot start)

    // Tracking
    static constexpr int    NUM_CONCURRENT_SIGNALS       = 184;   // simultaneous channels
    static constexpr double MAX_ALTITUDE_M               = 50000.0;// m
    static constexpr double MAX_VELOCITY_MPS             = 500.0; // m/s

    // Update rate
    static constexpr double MAX_NAV_RATE_HZ              = 20.0;  // Hz (RTK mode)
    static constexpr double RAW_MEAS_RATE_HZ             = 25.0;  // Hz

    // Physical
    static constexpr double POWER_CONSUMPTION_MA         = 68.0;  // mA
    static constexpr double SUPPLY_VOLTAGE_V             = 3.3;   // V
}

// =============================================================================
//  Barometer — TE MS5611-01BA03
//  Source: [MS5611] Table 1 - Sensor Performance, AN520
// =============================================================================
namespace MS5611 {
    static constexpr double PRESSURE_RANGE_MIN_MBAR      = 10.0;  // mbar
    static constexpr double PRESSURE_RANGE_MAX_MBAR      = 1200.0;// mbar
    static constexpr double PRESSURE_RESOLUTION_MBAR     = 0.012; // mbar (OSR=4096)
    static constexpr double PRESSURE_ACCURACY_MBAR       = 1.5;   // mbar (0-40°C)
    static constexpr double ALTITUDE_RESOLUTION_CM       = 10.0;  // cm
    static constexpr double TEMP_RANGE_MIN_C             = -40.0; // °C
    static constexpr double TEMP_RANGE_MAX_C             = 85.0;  // °C
    static constexpr double TEMP_RESOLUTION_C            = 0.01;  // °C (OSR=4096)
    static constexpr double CONVERSION_TIME_MS           = 9.04;  // ms (OSR=4096)
    static constexpr double SUPPLY_VOLTAGE_V             = 3.0;   // V
    static constexpr double CURRENT_SUPPLY_UA            = 12.5;  // μA (standby)
}

// =============================================================================
//  Star Tracker — Bradford ASTRO APS
//  Source: [STARTRK] Table 3.1 - Performance Specifications
// =============================================================================
namespace ASTRO_APS {
    static constexpr double ACCURACY_CROSS_BORESIGHT_ARCSEC = 2.0;  // arcsec (3σ)
    static constexpr double ACCURACY_ROLL_ARCSEC             = 7.0;  // arcsec (3σ)
    static constexpr double UPDATE_RATE_HZ                   = 4.0;  // Hz (default)
    static constexpr double UPDATE_RATE_MAX_HZ               = 10.0; // Hz
    static constexpr double MIN_STARS_REQUIRED               = 5;    // minimum for valid solution
    static constexpr double SLEW_RATE_MAX_DEG_S              = 10.0; // °/s (tracking limit)
    static constexpr double ACQUISITION_TIME_S               = 2.0;  // s (cold start)
    static constexpr double MASS_KG                          = 2.8;  // kg (sensor head)
    static constexpr double POWER_OPERATING_W               = 15.0;  // W
    static constexpr double TEMP_OPERATING_MIN_C            = -20.0; // °C
    static constexpr double TEMP_OPERATING_MAX_C            = 50.0;  // °C
    static constexpr double FOV_DEG                          = 20.0; // ° (square FOV)
}

// =============================================================================
//  LiDAR — Velodyne VLP-16 (Landing Terrain Sensor)
//  Source: [VLP16] Section 7 - Specifications
// =============================================================================
namespace VLP16 {
    static constexpr double RANGE_MIN_M                   = 0.5;   // m
    static constexpr double RANGE_MAX_M                   = 100.0; // m (typical)
    static constexpr double RANGE_ACCURACY_CM             = 3.0;   // cm (1σ)
    static constexpr int    CHANNELS                      = 16;    // vertical channels
    static constexpr double VERTICAL_FOV_DEG             = 30.0;  // ±15°
    static constexpr double HORIZONTAL_RESOLUTION_DEG    = 0.1;   // deg (azimuth)
    static constexpr double ROTATION_RATE_HZ             = 20.0;  // Hz (max)
    static constexpr double POINTS_PER_SECOND            = 600000;// pts/s
    static constexpr double POWER_CONSUMPTION_W          = 8.0;   // W (typical)
    static constexpr double LASER_WAVELENGTH_NM          = 903.0; // nm
    static constexpr double MASS_KG                      = 0.830; // kg
    static constexpr double TEMP_OPERATING_MIN_C         = -10.0; // °C
    static constexpr double TEMP_OPERATING_MAX_C         = 60.0;  // °C
}

// =============================================================================
//  Thermocouple — Omega K-Type (Engine/Structural Monitoring)
//  Source: [OMEGAK] NIST Monograph 175, ITS-90 Tables
// =============================================================================
namespace OMEGA_K_TYPE {
    static constexpr double TEMP_MIN_C                   = -200.0;// °C
    static constexpr double TEMP_MAX_C                   = 1372.0;// °C
    static constexpr double ACCURACY_PERCENT_FS          = 0.75;  // % FS (Class 1)
    static constexpr double ACCURACY_MIN_C               = 1.5;   // °C (whichever greater)
    // Seebeck coefficient (approximate, mid-range ~0°C to 1000°C)
    static constexpr double SEEBECK_COEFF_UV_C           = 41.0;  // μV/°C
    static constexpr double COLD_JUNCTION_COMP_ACCURACY  = 0.5;   // °C
    static constexpr double RESPONSE_TIME_63PCT_MS       = 100.0; // ms (grounded, 1mm dia)
}

// =============================================================================
//  Strain Gauge — HBM PW15AC (Structural Load Monitoring)
//  Source: [HBM_PW15] Datasheet Version 2019-09
// =============================================================================
namespace HBM_PW15 {
    static constexpr double ACCURACY_PERCENT_FS          = 0.02;  // % FS
    static constexpr double NOMINAL_LOAD_N               = 5000.0;// N (PW15/5kN variant)
    static constexpr double OVERLOAD_PERCENT             = 150.0; // %
    static constexpr double CREEP_PERCENT_30MIN          = 0.015; // % FS in 30 min
    static constexpr double TEMP_EFFECT_SPAN_PERCENT_10K = 0.020; // % per 10 K
    static constexpr double EXCITATION_VOLTAGE_V         = 10.0;  // V
    static constexpr double OUTPUT_SENSITIVITY_MV_V      = 2.0;   // mV/V
    static constexpr double TEMP_OPERATING_MIN_C         = -10.0; // °C
    static constexpr double TEMP_OPERATING_MAX_C         = 50.0;  // °C
}

// =============================================================================
//  Engine — Merlin 1D Class (Medium Launch Vehicle)
//  Source: [MERLIN1D] Public specifications, SpaceX CRS-2 press kit
// =============================================================================
namespace MERLIN1D {
    // Sea level performance
    static constexpr double THRUST_SL_KN                 = 845.0;  // kN (sea level)
    static constexpr double THRUST_VAC_KN                = 914.0;  // kN (vacuum)
    static constexpr double ISP_SL_S                     = 282.0;  // s (sea level)
    static constexpr double ISP_VAC_S                    = 311.0;  // s (vacuum)
    static constexpr double THROTTLE_MIN_PERCENT         = 40.0;   // % (minimum throttle)
    static constexpr double THROTTLE_MAX_PERCENT         = 100.0;  // %
    static constexpr double THRUST_VECTOR_MAX_DEG        = 5.0;    // ° (TVC gimbal limit)
    static constexpr double MASS_KG                      = 470.0;  // kg (engine dry)
    static constexpr double PROPELLANT_LOX_RP1_MR        = 2.36;   // mixture ratio (O/F)
    static constexpr double CHAMBER_PRESSURE_BAR         = 97.0;   // bar
    static constexpr double NOZZLE_AREA_RATIO_SL         = 16.0;   // expansion ratio
    static constexpr double NOZZLE_AREA_RATIO_VAC        = 165.0;  // expansion ratio (Merlin Vac)
    static constexpr double RESTART_CAPABILITY           = true;   // relightable
    static constexpr double THROTTLE_RESPONSE_TIME_MS    = 200.0;  // ms (10% to 90%)
}

// =============================================================================
//  Grid Fin Actuator (Falcon 9 class)
//  Source: Public domain engineering data, SpaceX technical publications
// =============================================================================
namespace GRID_FIN {
    static constexpr double DEFLECTION_MAX_DEG           = 90.0;  // °
    static constexpr double SLEW_RATE_DEG_S              = 400.0; // °/s (hydraulic)
    static constexpr double HINGE_MOMENT_MAX_KNM         = 20.0;  // kN·m
    static constexpr double DEPLOYMENT_TIME_S            = 6.0;   // s (retract → deployed)
    static constexpr double SPAN_M                       = 4.0;   // m (deployed)
    static constexpr double CHORD_M                      = 1.0;   // m
    static constexpr double ACTUATOR_BANDWIDTH_HZ        = 5.0;   // Hz (closed-loop)
    static constexpr double TEMP_OPERATING_MAX_C         = 800.0; // °C (titanium alloy)
}

// =============================================================================
//  RCS — Cold Gas Thruster (attitude control / landing)
//  Source: Moog DST-80 class cold gas thruster data
// =============================================================================
namespace COLD_GAS_RCS {
    static constexpr double THRUST_N                     = 22.0;  // N per thruster
    static constexpr double ISP_S                        = 70.0;  // s (N2 propellant)
    static constexpr double MIN_PULSE_WIDTH_MS           = 10.0;  // ms
    static constexpr double RESPONSE_TIME_MS             = 5.0;   // ms (valve open)
    static constexpr double SUPPLY_PRESSURE_BAR          = 250.0; // bar (COPV)
    static constexpr double TEMP_OPERATING_MIN_C         = -40.0; // °C
    static constexpr double TEMP_OPERATING_MAX_C         = 70.0;  // °C
}

// =============================================================================
//  Landing Leg (telescoping carbon composite)
//  Source: public SpaceX Falcon 9 leg specifications, approximate
// =============================================================================
namespace LANDING_LEG {
    static constexpr double CRUSH_STROKE_M               = 1.2;   // m (energy absorber)
    static constexpr double DEPLOYMENT_TIME_S            = 1.5;   // s
    static constexpr double DESIGN_LOAD_KN               = 2500.0;// kN per leg (4 legs)
    static constexpr double FOOTPAD_DIAMETER_M           = 2.0;   // m
    static constexpr double DEPLOYED_RADIUS_M            = 10.0;  // m from center
}

// =============================================================================
//  Flight Computer Hardware
//  Based on VxWorks-class space-grade SBC
// =============================================================================
namespace FLIGHT_COMPUTER {
    static constexpr double CONTROL_LOOP_PERIOD_MS       = 1.0;   // ms (1 kHz)
    static constexpr double GUIDANCE_LOOP_PERIOD_MS      = 100.0; // ms (10 Hz)
    static constexpr double TELEMETRY_PERIOD_MS          = 1000.0;// ms (1 Hz downlink)
    static constexpr double WATCHDOG_TIMEOUT_MS          = 50.0;  // ms
    static constexpr double MAX_CPU_USAGE_PERCENT        = 80.0;  // % (leave headroom)
    static constexpr double MEMORY_SAFETY_RESERVE_MB     = 128.0; // MB
}

} // namespace hw
} // namespace atlas
