/**
 * gnss_driver.cpp
 * ATLAS GNSS Driver — u-blox ZED-F9P Implementation
 *
 * On hardware: parses UBX binary protocol (UBX-NAV-PVT, UBX-NAV-COV)
 * over UART at 921600 baud, 20 Hz navigation rate.
 *
 * UBX-NAV-PVT message (0x01 0x07) gives:
 *   - iTOW, lat, lon, height (mm), hMSL (mm)
 *   - velN, velE, velD (mm/s)
 *   - hAcc (mm), vAcc (mm), sAcc (mm/s)
 *   - fixType, flags (carrSoln for RTK fixed/float)
 *   - numSV (satellites used)
 *
 * UBX-NAV-COV message (0x01 0x36) gives full 6×6 position+velocity covariance.
 *
 * Source: [ZEF9P] u-blox ZED-F9P Integration Manual UBX-17051259 Rev 2.0
 *         Section 3 — Communication Interface, Section 10 — UBX Protocol
 */

#include "gnss_driver.hpp"
#include <cmath>
#include <algorithm>

namespace atlas {

// Static constexpr ODR compliance
constexpr double GNSSDriver::UPDATE_RATE_HZ;
constexpr double GNSSDriver::RTK_CEP_M;
constexpr double GNSSDriver::VELOCITY_SIGMA_MS;

constexpr double BarometerDriver::SAMPLE_RATE_HZ;
constexpr double BarometerDriver::RESOLUTION_PA;
constexpr double BarometerDriver::ALTITUDE_RES_CM;

// ─────────────────────────────────────────────────────────────────
//  Hardware read stubs (UBX protocol parsing)
//  These replace the simulation branch when sim_mode_ == false.
//
//  On real hardware, call:
//    GNSSDriver gnss(false);   // real hardware mode
//    gnss.configure_ubx_port("/dev/ttyS1", 921600);
// ─────────────────────────────────────────────────────────────────

/**
 * Configure ZED-F9P via UBX-CFG-PRT and UBX-CFG-RATE messages.
 * - Set UART baud rate to 921600
 * - Set navigation rate to 20 Hz (min measurement period = 50 ms)
 * - Enable UBX-NAV-PVT and UBX-NAV-COV output
 * - Enable RTK (requires RTCM3 corrections on UART2)
 *
 * UBX-CFG-RATE payload:
 *   measRate:  50   (ms — 20 Hz)
 *   navRate:   1    (nav solutions per measurement)
 *   timeRef:   1    (GPS time)
 */
bool configure_zed_f9p_hardware() {
    // UBX sync chars + class/id + payload
    // UBX-CFG-RATE (0x06 0x08): 50 ms measurement period, GPS time reference
    static const uint8_t UBX_CFG_RATE[] = {
        0xB5, 0x62,          // sync chars
        0x06, 0x08,          // class: CFG, id: RATE
        0x06, 0x00,          // length = 6 bytes
        0x32, 0x00,          // measRate = 50 ms (20 Hz)
        0x01, 0x00,          // navRate = 1
        0x01, 0x00,          // timeRef = 1 (GPS time)
        0x48, 0xE7           // checksum CK_A, CK_B
    };
    (void)UBX_CFG_RATE;  // suppress warning — used in real HW path

    // UBX-CFG-MSG (0x06 0x01): Enable UBX-NAV-PVT (0x01 0x07) on UART1
    static const uint8_t UBX_CFG_MSG_NAV_PVT[] = {
        0xB5, 0x62,
        0x06, 0x01,
        0x08, 0x00,
        0x01, 0x07,          // UBX-NAV-PVT
        0x00, 0x01, 0x00, 0x00, 0x00, 0x00,  // enable on UART1 only
        0x18, 0xE1
    };
    (void)UBX_CFG_MSG_NAV_PVT;

    // On real hardware: write these via open("/dev/ttyS1") and write()
    // Return true when ACK-ACK (0x05 0x01) received
    return true;  // simulation: always succeed
}

/**
 * Parse UBX-NAV-PVT message (92-byte payload).
 * Converts from ZED-F9P native units to SI for ATLAS use.
 *
 * Key fields (byte offsets from start of payload):
 *  [0]  iTOW     : uint32  ms      GPS time of week
 *  [20] lat      : int32   1e-7°   WGS-84 latitude
 *  [24] lon      : int32   1e-7°   WGS-84 longitude
 *  [28] height   : int32   mm      height above ellipsoid
 *  [32] hMSL     : int32   mm      height above MSL
 *  [36] hAcc     : uint32  mm      horizontal accuracy
 *  [40] vAcc     : uint32  mm      vertical accuracy
 *  [48] velN     : int32   mm/s    velocity North
 *  [52] velE     : int32   mm/s    velocity East
 *  [56] velD     : int32   mm/s    velocity Down
 *  [66] sAcc     : uint32  mm/s    speed accuracy
 *  [20] fixType  : uint8           0=no fix, 3=3D, 4=GNSS+DR, 5=time
 *  [21] flags    : uint8           bit 0: gnssFixOK; bits 6-7: carrSoln (RTK)
 *  [23] numSV    : uint8           satellites used
 */
GNSSSample parse_ubx_nav_pvt(const uint8_t* payload, double timestamp_s) {
    GNSSSample s;
    s.timestamp_s = timestamp_s;

    // Parse little-endian fields
    auto read_i32 = [&](int offset) -> int32_t {
        return static_cast<int32_t>(
            (static_cast<uint32_t>(payload[offset])      |
            (static_cast<uint32_t>(payload[offset+1])<<8) |
            (static_cast<uint32_t>(payload[offset+2])<<16)|
            (static_cast<uint32_t>(payload[offset+3])<<24)));
    };
    auto read_u32 = [&](int offset) -> uint32_t {
        return (static_cast<uint32_t>(payload[offset])      |
               (static_cast<uint32_t>(payload[offset+1])<<8) |
               (static_cast<uint32_t>(payload[offset+2])<<16)|
               (static_cast<uint32_t>(payload[offset+3])<<24));
    };

    const uint8_t fix_type  = payload[20];
    const uint8_t flags     = payload[21];
    const uint8_t num_sv    = payload[23];
    const uint8_t carr_soln = (flags >> 6) & 0x03;  // 0=none, 1=float, 2=fixed

    const double lat_deg = read_i32(20) * 1e-7;
    const double lon_deg = read_i32(24) * 1e-7;
    const double height_mm = read_i32(28);
    const double vel_n_mms = read_i32(48);
    const double vel_e_mms = read_i32(52);
    const double vel_d_mms = read_i32(56);
    const double h_acc_mm  = read_u32(36);
    const double v_acc_mm  = read_u32(40);

    // Convert to ECEF (simplified — real system uses proper WGS-84 transform)
    constexpr double R_EARTH  = 6378137.0;  // WGS-84 semi-major axis
    const double lat_rad = lat_deg * M_PI / 180.0;
    const double lon_rad = lon_deg * M_PI / 180.0;
    const double alt_m   = height_mm / 1000.0;
    const double N = R_EARTH; // simplified: assume spherical

    s.pos_x_m = (N + alt_m) * std::cos(lat_rad) * std::cos(lon_rad);
    s.pos_y_m = (N + alt_m) * std::cos(lat_rad) * std::sin(lon_rad);
    s.pos_z_m = (N + alt_m) * std::sin(lat_rad);

    // Velocity: NED → ECEF
    s.vel_x_ms = -std::sin(lat_rad)*std::cos(lon_rad) * (vel_n_mms/1000.0)
               -  std::sin(lon_rad)                   * (vel_e_mms/1000.0)
               +  std::cos(lat_rad)*std::cos(lon_rad) * (-vel_d_mms/1000.0);
    s.vel_y_ms = -std::sin(lat_rad)*std::sin(lon_rad) * (vel_n_mms/1000.0)
               +  std::cos(lon_rad)                   * (vel_e_mms/1000.0)
               +  std::cos(lat_rad)*std::sin(lon_rad) * (-vel_d_mms/1000.0);
    s.vel_z_ms =  std::cos(lat_rad) * (vel_n_mms/1000.0)
               +  std::sin(lat_rad) * (-vel_d_mms/1000.0);

    // Covariance from accuracy estimates
    s.cov_xx = (h_acc_mm/1000.0) * (h_acc_mm/1000.0);
    s.cov_yy = s.cov_xx;
    s.cov_zz = (v_acc_mm/1000.0) * (v_acc_mm/1000.0);

    s.fix_type        = fix_type;
    s.rtk_fixed       = (carr_soln == 2);
    s.rtk_float       = (carr_soln == 1);
    s.satellites_used = num_sv;
    s.hdop            = h_acc_mm / 1000.0 / 5.0;  // approximate
    s.data_valid      = (fix_type >= 3) && (flags & 0x01);

    return s;
}

} // namespace atlas
