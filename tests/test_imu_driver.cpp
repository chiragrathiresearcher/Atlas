/**
 * test_imu_driver.cpp
 * Unit tests for ATLAS IMU driver.
 * Validates: noise bounds, voting, calibration, datasheet limits.
 */

#include <cassert>
#include <cstdio>
#include <cmath>
#include "imu_driver.hpp"
#include "atlas_hardware.hpp"

using namespace atlas;

static void test_hg4930_noise_within_datasheet_bounds() {
    // HG4930 gyro ARW: 0.004 °/√hr at 400 Hz
    // Expected 1-sigma: ARW_rad_sqrts × √(rate_hz)
    const double sigma = ImuNoiseModel::hg4930_gyro_sigma_rads(400.0);
    const double expected_max = 0.001;  // rad/s — generous bound

    assert(sigma < expected_max &&
           "HG4930 gyro sigma exceeds expected datasheet bound");
    printf("[PASS] test_hg4930_noise_within_datasheet_bounds | sigma=%.6f rad/s\n", sigma);
}

static void test_icm42688_noise_within_datasheet_bounds() {
    // ICM-42688-P gyro NSD: 0.0028 °/s/√Hz
    // At 400 Hz bandwidth: sigma = NSD × √BW × π/180
    const double sigma = ImuNoiseModel::icm_gyro_sigma_rads(400.0);
    const double spec   = hw::ICM42688P::GYRO_NOISE_DENSITY_DPS_SQRTHZ
                          * std::sqrt(400.0) * (M_PI / 180.0);

    assert(std::abs(sigma - spec) < 1e-9 && "ICM sigma must match datasheet formula");
    printf("[PASS] test_icm42688_noise_within_datasheet_bounds | sigma=%.6f rad/s\n", sigma);
}

static void test_triple_imu_voting_healthy() {
    TripleRedundantIMU imu;
    auto sample = imu.read_voted(10.0);  // T+10s

    assert(sample.data_valid && "Voted sample must be valid with all IMUs healthy");
    assert(sample.confidence >= 0.95 &&
           "Confidence must be high when all IMUs agree");
    printf("[PASS] test_triple_imu_voting_healthy | conf=%.3f\n", sample.confidence);
}

static void test_temperature_within_operating_range() {
    ImuCalibration cal = TripleRedundantIMU::default_calibration(0);
    HG4930Driver driver(0, cal);

    // Read at different mission times
    for (double t : {0.0, 30.0, 60.0, 120.0}) {
        auto raw = driver.read(t);
        assert(raw.temperature_c >= hw::HG4930::TEMP_OPERATING_MIN_C &&
               raw.temperature_c <= hw::HG4930::TEMP_OPERATING_MAX_C &&
               "Temperature must stay within HG4930 operating range");
    }
    printf("[PASS] test_temperature_within_operating_range\n");
}

static void test_calibration_removes_bias() {
    ImuCalibration cal = TripleRedundantIMU::default_calibration(0);
    // Inject a known bias
    cal.gyro_bias_x_rads = 0.01;  // 10 mrad/s bias
    cal.gyro_bias_y_rads = 0.005;
    cal.valid = true;

    HG4930Driver driver(0, cal);
    auto raw = driver.read(0.0);
    auto calib = driver.calibrate(raw);

    // Calibrated value should be approximately raw − bias
    double expected_x = raw.gyro_x_rads - 0.01;
    assert(std::abs(calib.gyro_x_rads - expected_x) < 1e-9 &&
           "Calibration must remove injected bias");
    printf("[PASS] test_calibration_removes_bias\n");
}

static void test_gyro_full_scale_not_exceeded() {
    ImuCalibration cal = TripleRedundantIMU::default_calibration(0);
    HG4930Driver driver(0, cal);

    // Read 100 samples during high-dynamic ascent phase
    double max_rate = 0.0;
    for (int i = 0; i < 100; ++i) {
        auto raw = driver.read(double(i) * 0.01);
        double rate = std::abs(raw.gyro_x_rads) + std::abs(raw.gyro_y_rads)
                    + std::abs(raw.gyro_z_rads);
        if (rate > max_rate) max_rate = rate;
    }

    const double full_scale_rads = hw::HG4930::GYRO_FULL_SCALE_DEG_S * (M_PI / 180.0);
    assert(max_rate < full_scale_rads &&
           "Simulated trajectory must stay within HG4930 full-scale range");
    printf("[PASS] test_gyro_full_scale_not_exceeded | max_rate=%.3f rad/s (limit=%.0f)\n",
           max_rate, full_scale_rads);
}

int main() {
    printf("\n=== ATLAS IMU Driver Tests ===\n\n");

    test_hg4930_noise_within_datasheet_bounds();
    test_icm42688_noise_within_datasheet_bounds();
    test_triple_imu_voting_healthy();
    test_temperature_within_operating_range();
    test_calibration_removes_bias();
    test_gyro_full_scale_not_exceeded();

    printf("\n=== All tests PASSED ===\n");
    return 0;
}
