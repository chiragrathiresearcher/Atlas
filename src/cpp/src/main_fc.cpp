/**
 * main_fc.cpp
 * ATLAS Flight Computer — C++ Entry Point
 *
 * Standalone binary for the real-time C++ flight computer.
 * In production: this runs on the flight computer SBC under RTOS.
 * In HIL: this links against sensor emulator stubs.
 *
 * Usage:
 *   ./atlas_fc [--sim] [--verbose] [--duration <seconds>]
 *
 * The Python guidance layer connects via shared memory / POSIX IPC.
 */

#include "flight_computer.hpp"
#include "watchdog.hpp"
#include <iostream>
#include <csignal>
#include <cstring>
#include <thread>
#include <chrono>
#include <atomic>
#include <argparse/argparse.hpp>

static std::atomic<bool> g_shutdown_requested{false};

static void signal_handler(int sig) {
    std::cerr << "\n[FC_MAIN] Signal " << sig << " — requesting shutdown\n";
    g_shutdown_requested.store(true);
}

static void print_banner() {
    std::cout <<
        "╔══════════════════════════════════════════════════════════╗\n"
        "║  ATLAS Flight Computer  v2.0                             ║\n"
        "║  ARVS Authority Layer: ACTIVE                            ║\n"
        "║  IMU: HG4930×2 + ICM-42688-P  |  GNSS: ZED-F9P RTK      ║\n"
        "║  Control loop: 1 kHz  |  Watchdog: 50 ms                ║\n"
        "╚══════════════════════════════════════════════════════════╝\n";
}

int main(int argc, char* argv[]) {
    // Parse arguments
    bool verbose  = false;
    bool sim_mode = true;
    double max_duration_s = 300.0;

    for (int i = 1; i < argc; ++i) {
        if (std::strcmp(argv[i], "--verbose") == 0) verbose  = true;
        if (std::strcmp(argv[i], "--hw")      == 0) sim_mode = false;
        if (std::strcmp(argv[i], "--duration") == 0 && i+1 < argc) {
            max_duration_s = std::stod(argv[++i]);
        }
    }

    print_banner();
    std::cout << "[FC_MAIN] Mode: " << (sim_mode ? "SIMULATION" : "HARDWARE") << "\n";
    std::cout << "[FC_MAIN] Verbose: " << (verbose ? "ON" : "OFF") << "\n";
    std::cout << "[FC_MAIN] Max duration: " << max_duration_s << "s\n";

    // Install signal handlers
    std::signal(SIGINT,  signal_handler);
    std::signal(SIGTERM, signal_handler);

    // Create and initialize flight computer
    atlas::FlightComputer fc;
    if (!fc.initialize()) {
        std::cerr << "[FC_MAIN] FATAL: Initialization failed — aborting\n";
        return 1;
    }

    std::cout << "[FC_MAIN] Flight computer initialized — starting main loop\n";
    std::cout << "[FC_MAIN] Press Ctrl+C to shutdown\n\n";

    // Run in a thread so we can monitor shutdown signal
    std::thread fc_thread([&fc]{ fc.run(); });

    // Wait for shutdown signal or duration
    const auto t_start = std::chrono::steady_clock::now();
    while (!g_shutdown_requested.load()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        auto elapsed = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - t_start).count();
        if (elapsed >= max_duration_s) {
            std::cout << "[FC_MAIN] Duration limit reached — shutting down\n";
            break;
        }
    }

    // Graceful shutdown
    fc.shutdown();
    fc_thread.join();

    const auto& health = fc.get_health();
    std::cout << "\n[FC_MAIN] Shutdown complete\n";
    std::cout << "[FC_MAIN] Control loop overruns: " << health.control_loop_overruns << "\n";
    std::cout << "[FC_MAIN] Max jitter: " << health.control_loop_jitter_us << " µs\n";

    return 0;
}
