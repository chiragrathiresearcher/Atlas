/**
 * watchdog.hpp
 * ATLAS Hardware Watchdog Timer
 *
 * If the control loop fails to kick the watchdog within WATCHDOG_TIMEOUT_MS,
 * the system automatically enters SAFE_HOLD (engine shutdown + telemetry distress).
 *
 * Timeout: 50 ms — from hw::FLIGHT_COMPUTER::WATCHDOG_TIMEOUT_MS
 */

#pragma once
#include <atomic>
#include <chrono>
#include <cstdint>
#include "atlas_hardware.hpp"

namespace atlas {

class WatchdogTimer {
public:
    static constexpr double TIMEOUT_MS = hw::FLIGHT_COMPUTER::WATCHDOG_TIMEOUT_MS;

    WatchdogTimer() : last_kick_(std::chrono::steady_clock::now()), tripped_(false) {}

    /** Called by control loop every cycle to indicate system is healthy. */
    void kick() {
        last_kick_ = std::chrono::steady_clock::now();
        tripped_.store(false);
    }

    /** Returns true if watchdog has tripped (control loop stalled). */
    bool is_tripped() {
        auto now   = std::chrono::steady_clock::now();
        auto delta = std::chrono::duration<double, std::milli>(now - last_kick_).count();
        if (delta > TIMEOUT_MS) {
            tripped_.store(true);
        }
        return tripped_.load();
    }

    double elapsed_ms() const {
        auto now = std::chrono::steady_clock::now();
        return std::chrono::duration<double, std::milli>(now - last_kick_).count();
    }

    void reset() {
        kick();
        tripped_.store(false);
    }

private:
    std::chrono::steady_clock::time_point last_kick_;
    std::atomic<bool> tripped_;
};

} // namespace atlas
