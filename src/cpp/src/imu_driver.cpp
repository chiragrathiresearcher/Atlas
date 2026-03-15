/**
 * imu_driver.cpp
 * ATLAS IMU Driver — HG4930 + ICM-42688-P Implementation
 *
 * On hardware: wraps SPI register reads at 400 Hz (HG4930) / 32 kHz (ICM).
 * In simulation: uses Allan variance–derived deterministic noise model.
 *
 * Allan variance model sources:
 *   [HG4930]   ARW = 0.004 °/√hr → σ_sample = ARW × √(rate_hz) × π/180/60
 *   [ICM42688] NSD = 0.0028 °/s/√Hz → σ_sample = NSD × √(ODR) × π/180
 *
 * Gauss-Markov bias drift (1st order):
 *   bias(k+1) = exp(-dt/τ) × bias(k) + σ_b × √(1 - exp(-2dt/τ)) × n(k)
 *   τ = 3600 s, σ_b = bias_stability / 3600 [rad/s]
 */

#include "imu_driver.hpp"
#include <cmath>
#include <algorithm>

namespace atlas {

// All methods are inline/in-header for the simulation driver.
// This .cpp exists for on-hardware register read implementations
// and any additional link-time only definitions.

// ─────────────────────────────────────────────────────────────────
//  ImuNoiseModel — static factory methods (out-of-line definitions)
// ─────────────────────────────────────────────────────────────────

// Declared static constexpr in header, definitions here for ODR compliance
constexpr double ImuNoiseModel::HG4930_GYRO_ARW_DEG_SQRTHR;
constexpr double ImuNoiseModel::HG4930_GYRO_BIAS_INST_DEG_HR;
constexpr double ImuNoiseModel::HG4930_ACCEL_VRW_MPS_SQRTS;
constexpr double ImuNoiseModel::HG4930_ACCEL_BIAS_INST_MG;
constexpr double ImuNoiseModel::ICM_GYRO_NSD_DPS_SQRTHZ;
constexpr double ImuNoiseModel::ICM_ACCEL_NSD_UG_SQRTHZ;

// ─────────────────────────────────────────────────────────────────
//  TripleRedundantIMU — voting thresholds (out-of-line)
// ─────────────────────────────────────────────────────────────────

constexpr double TripleRedundantIMU::GYRO_VOTE_THRESHOLD_RADS;
constexpr double TripleRedundantIMU::ACCEL_VOTE_THRESHOLD_MS2;
constexpr int    TripleRedundantIMU::NUM_IMUS;

} // namespace atlas
