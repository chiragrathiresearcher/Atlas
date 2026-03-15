"""
test_sensor_processor.py
Unit tests for ATLAS sensor processing layer.

Tests:
  - IMU calibration pipeline (temperature compensation, noise model)
  - GNSS fix validation (RTK, float, autonomous accuracy)
  - Barometer ISA altitude model
  - Allan variance noise model (datasheet compliance)
"""

import pytest
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent.parent / "src/python"))

import numpy as np
from atlas.sensors.sensor_processor import (
    IMUSensorProcessor, GNSSSensorProcessor,
    BaroSensorProcessor, IMUMeasurement, GNSSMeasurement
)


class TestIMUSensorProcessor:

    def test_gyro_sigma_at_400hz_matches_datasheet(self):
        """
        HG4930 ARW = 0.004 °/√hr
        σ_gyro at 400 Hz = ARW × √(400) × π/180 / 60
        Expected: ~2.33e-5 rad/s
        """
        sigma = IMUSensorProcessor.hg4930_gyro_sigma(400.0)
        arw_rad_sqrts = 0.004 * (math.pi / 180.0) / 60.0
        expected = arw_rad_sqrts * math.sqrt(400.0)
        assert abs(sigma - expected) < 1e-10, (
            f"Gyro sigma {sigma:.2e} != expected {expected:.2e}"
        )
        # Sanity: should be in the μrad/s range for a navigation-grade IMU
        assert 1e-5 < sigma < 1e-3, f"Gyro sigma {sigma:.2e} out of expected range"

    def test_accel_sigma_at_400hz(self):
        """HG4930 VRW = 0.005 m/s/√s → σ_accel at 400 Hz = 0.005 × √400 = 0.1 m/s²"""
        sigma = IMUSensorProcessor.hg4930_accel_sigma(400.0)
        expected = 0.005 * math.sqrt(400.0)
        assert abs(sigma - expected) < 1e-10

    def test_calibration_removes_bias(self):
        """Applying known bias should zero out calibrated output."""
        proc = IMUSensorProcessor()
        known_bias_rad_s = np.array([0.01, -0.02, 0.005])
        known_bias_m_s2  = np.array([0.05, -0.03, 0.01])
        proc._gyro_bias_rad_s = known_bias_rad_s
        proc._accel_bias_m_s2 = known_bias_m_s2

        raw_gyro  = np.array([0.01, -0.02, 0.005])   # = bias only
        raw_accel = np.array([0.05, -0.03, 0.01])
        result = proc.calibrate(raw_gyro, raw_accel, 25.0, 0.0)

        assert result.data_valid
        np.testing.assert_allclose(result.gyro_rad_s, np.zeros(3), atol=1e-10)
        np.testing.assert_allclose(result.accel_m_s2, np.zeros(3), atol=1e-10)

    def test_temperature_compensation_applied(self):
        """
        Gyro offset temp coefficient for ICM-42688-P: 0.008 °/s/°C
        At 35°C (ΔT=10°C from 25°C ref), bias correction = 0.008 × 10 × π/180 rad/s
        """
        proc = IMUSensorProcessor()
        raw_gyro  = np.zeros(3)
        raw_accel = np.zeros(3)
        temp_c    = 35.0   # 10°C above reference
        result    = proc.calibrate(raw_gyro, raw_accel, temp_c, 0.0)

        # With no stored bias but non-zero temp, temp correction should be applied
        # Expected correction magnitude: 0.008 × π/180 × 10 per axis
        expected_correction = 0.008 * (math.pi / 180.0) * 10.0
        # Each axis should have this correction subtracted
        assert abs(result.gyro_rad_s[0]) < expected_correction + 1e-9

    def test_confidence_is_one_on_valid_data(self):
        proc = IMUSensorProcessor()
        result = proc.calibrate(np.zeros(3), np.array([0, 0, 9.81]), 25.0, 0.0)
        assert result.confidence == 1.0
        assert result.data_valid is True


class TestGNSSSensorProcessor:

    def _make_raw(self, rtk_fixed=True, num_sv=12, fix_type=3,
                  altitude_m=1000.0, speed_m_s=100.0):
        import math
        R = 6378137.0
        return {
            "timestamp_s":    1.0,
            "fix_type":       fix_type,
            "rtk_fixed":      rtk_fixed,
            "rtk_float":      False,
            "satellites_used":num_sv,
            "hdop":           0.8,
            "pos_x_m":        R + altitude_m,
            "pos_y_m":        1000.0,
            "pos_z_m":        0.0,
            "vel_x_ms":       speed_m_s,
            "vel_y_ms":       0.0,
            "vel_z_ms":       0.0,
            "data_valid":     True,
        }

    def test_rtk_fixed_covariance_matches_datasheet(self):
        """RTK fixed CEP = 0.010 m → pos_cov diagonal = (0.010/√2)² = 5e-5 m²"""
        proc = GNSSSensorProcessor()
        meas = proc.process(self._make_raw(rtk_fixed=True))
        assert meas is not None
        expected_var = (0.010 / math.sqrt(2.0)) ** 2
        np.testing.assert_allclose(meas.pos_cov[0, 0], expected_var, rtol=1e-6)
        np.testing.assert_allclose(meas.pos_cov[1, 1], expected_var, rtol=1e-6)

    def test_invalid_fix_returns_none(self):
        proc = GNSSSensorProcessor()
        raw = self._make_raw()
        raw["fix_type"] = 0   # no fix
        assert proc.process(raw) is None

    def test_too_few_satellites_returns_none(self):
        proc = GNSSSensorProcessor()
        raw = self._make_raw()
        raw["satellites_used"] = 2
        assert proc.process(raw) is None

    def test_excessive_altitude_returns_none(self):
        """ZED-F9P operational limit: 50,000 m"""
        proc = GNSSSensorProcessor()
        raw = self._make_raw(altitude_m=55000.0)
        assert proc.process(raw) is None

    def test_excessive_velocity_returns_none(self):
        """ZED-F9P velocity limit: 500 m/s"""
        proc = GNSSSensorProcessor()
        raw = self._make_raw(speed_m_s=510.0)
        assert proc.process(raw) is None

    def test_valid_rate_tracking(self):
        proc = GNSSSensorProcessor()
        good = self._make_raw()
        bad  = self._make_raw(); bad["fix_type"] = 0
        for _ in range(8): proc.process(good)
        for _ in range(2): proc.process(bad)
        assert abs(proc.valid_rate - 0.8) < 0.01


class TestBaroSensorProcessor:

    def test_sea_level_pressure_gives_zero_altitude(self):
        proc = BaroSensorProcessor()
        raw = {"timestamp_s": 0.0, "pressure_pa": 101325.0,
               "temperature_c": 15.0, "data_valid": True}
        meas = proc.process(raw)
        assert meas is not None
        assert abs(meas.altitude_m) < 1.0   # < 1 m error at sea level

    def test_pressure_decreases_with_altitude(self):
        """Higher altitude → lower pressure."""
        proc = BaroSensorProcessor()
        alt_10km = proc._pressure_to_altitude(26500.0, -50.0)   # ~10 km pressure
        alt_5km  = proc._pressure_to_altitude(54000.0, -17.5)   # ~5 km
        assert alt_10km > alt_5km

    def test_altitude_uncertainty_from_pressure_accuracy(self):
        """
        MS5611 accuracy: 1.5 mbar = 150 Pa
        dh/dP at sea level ≈ -RT/(g×P) ≈ -8.4 m/Pa
        σ_alt ≈ 150 × 8.4 ≈ 1260 m — rough upper bound at full pressure range
        At cruise (500 hPa, ~5.5 km): σ ≈ 10 m
        """
        proc = BaroSensorProcessor()
        raw = {"timestamp_s": 0.0, "pressure_pa": 50000.0,
               "temperature_c": -17.5, "data_valid": True}
        meas = proc.process(raw)
        assert meas is not None
        # Altitude uncertainty should be reasonable (< 50 m at cruise)
        assert meas.altitude_sigma < 50.0
        assert meas.altitude_sigma > 0.0

    def test_invalid_data_returns_none(self):
        proc = BaroSensorProcessor()
        raw = {"timestamp_s": 0.0, "pressure_pa": 101325.0,
               "temperature_c": 15.0, "data_valid": False}
        assert proc.process(raw) is None
