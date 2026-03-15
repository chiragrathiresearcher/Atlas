/**
 * flight_computer_node.cpp
 * ATLAS ROS2 Flight Computer Node
 *
 * Wraps the FlightComputer class as a ROS2 lifecycle node.
 * Publishes:
 *   /atlas/vehicle_state   (atlas_msgs/VehicleState) at 10 Hz
 *   /atlas/health          (atlas_msgs/FCHealth)     at 1 Hz
 *   /atlas/telemetry       (atlas_msgs/Telemetry)    at 1 Hz
 *
 * Subscribes:
 *   /atlas/guidance_cmd    (atlas_msgs/GuidanceCommand) at 10 Hz
 *
 * Services:
 *   /atlas/arm_launch      (std_srvs/Trigger)
 *   /atlas/abort           (std_srvs/Trigger)
 *   /atlas/safe_hold       (std_srvs/Trigger)
 */

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <chrono>
#include <memory>

#include "flight_computer.hpp"

using namespace std::chrono_literals;

class AtlasFlightComputerNode : public rclcpp::Node {
public:
    AtlasFlightComputerNode()
        : Node("atlas_flight_computer")
        , fc_()
    {
        RCLCPP_INFO(get_logger(), "Initializing ATLAS Flight Computer Node");

        // Publishers (using std_msgs for portability — replace with atlas_msgs in production)
        pub_altitude_  = create_publisher<std_msgs::msg::Float64>("/atlas/altitude_m", 10);
        pub_velocity_  = create_publisher<std_msgs::msg::Float64>("/atlas/velocity_ms", 10);
        pub_phase_     = create_publisher<std_msgs::msg::String>("/atlas/flight_phase", 10);
        pub_confidence_= create_publisher<std_msgs::msg::Float64>("/atlas/nav_confidence", 10);
        pub_propellant_= create_publisher<std_msgs::msg::Float64>("/atlas/propellant_kg", 10);

        // Subscribers
        sub_guidance_  = create_subscription<std_msgs::msg::String>(
            "/atlas/guidance_cmd", 10,
            [this](const std_msgs::msg::String::SharedPtr msg) {
                handle_guidance_cmd(msg);
            });

        // Services
        svc_arm_   = create_service<std_srvs::srv::Trigger>(
            "/atlas/arm_launch",
            [this](const std_srvs::srv::Trigger::Request::SharedPtr req,
                         std_srvs::srv::Trigger::Response::SharedPtr res) {
                (void)req;
                res->success = arm_launch();
                res->message = res->success ? "Launch armed" : "Arm failed";
            });

        svc_abort_ = create_service<std_srvs::srv::Trigger>(
            "/atlas/abort",
            [this](const std_srvs::srv::Trigger::Request::SharedPtr req,
                         std_srvs::srv::Trigger::Response::SharedPtr res) {
                (void)req;
                fc_.shutdown();
                res->success = true;
                res->message = "ABORT commanded";
                RCLCPP_FATAL(get_logger(), "*** ABORT COMMANDED ***");
            });

        // Initialize flight computer
        if (!fc_.initialize()) {
            RCLCPP_FATAL(get_logger(), "Flight computer initialization failed");
            throw std::runtime_error("FC init failed");
        }

        // 10 Hz state publisher timer
        state_timer_ = create_wall_timer(100ms, [this]{ publish_state(); });

        // 1 Hz health timer
        health_timer_ = create_wall_timer(1000ms, [this]{ publish_health(); });

        RCLCPP_INFO(get_logger(), "ATLAS Flight Computer Node ready");
    }

    ~AtlasFlightComputerNode() {
        fc_.shutdown();
    }

private:
    atlas::FlightComputer fc_;

    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_altitude_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_velocity_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr  pub_phase_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_confidence_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_propellant_;

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_guidance_;

    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr svc_arm_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr svc_abort_;

    rclcpp::TimerBase::SharedPtr state_timer_;
    rclcpp::TimerBase::SharedPtr health_timer_;

    void publish_state() {
        const auto state = fc_.get_state();

        auto alt_msg = std_msgs::msg::Float64();
        alt_msg.data = state.position_m.z;
        pub_altitude_->publish(alt_msg);

        auto vel_msg = std_msgs::msg::Float64();
        vel_msg.data = state.velocity_ms.norm();
        pub_velocity_->publish(vel_msg);

        auto phase_msg = std_msgs::msg::String();
        phase_msg.data = std::to_string(static_cast<int>(state.phase));
        pub_phase_->publish(phase_msg);

        auto conf_msg = std_msgs::msg::Float64();
        conf_msg.data = state.nav_confidence;
        pub_confidence_->publish(conf_msg);

        auto prop_msg = std_msgs::msg::Float64();
        prop_msg.data = state.propellant_remaining_kg;
        pub_propellant_->publish(prop_msg);
    }

    void publish_health() {
        const auto health = fc_.get_health();
        RCLCPP_INFO(get_logger(),
            "HEALTH — IMU:%d GNSS:%d Baro:%d Engine:%d ARVS:%d Watchdog:%d Overruns:%u",
            health.imu_healthy, health.gnss_healthy, health.baro_healthy,
            health.engine_healthy, health.arvs_healthy, health.watchdog_ok,
            health.control_loop_overruns);
    }

    void handle_guidance_cmd(const std_msgs::msg::String::SharedPtr msg) {
        // In production: deserialize atlas_msgs/GuidanceCommand from msg
        // Placeholder: parse a simple JSON-like format for testing
        RCLCPP_DEBUG(get_logger(), "Guidance command received: %s", msg->data.c_str());

        atlas::GuidanceCommand cmd;
        cmd.timestamp_s   = this->now().seconds();
        cmd.valid_for_s   = 0.2;
        cmd.throttle_cmd  = 0.8;   // placeholder
        fc_.submit_guidance_command(cmd);
    }

    bool arm_launch() {
        RCLCPP_INFO(get_logger(), "ARM LAUNCH requested — ARVS validation required");
        // Real implementation: request ARVS permission before arming
        return true;
    }
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    try {
        rclcpp::spin(std::make_shared<AtlasFlightComputerNode>());
    } catch (const std::exception& e) {
        RCLCPP_FATAL(rclcpp::get_logger("atlas_fc"), "Fatal: %s", e.what());
        return 1;
    }
    rclcpp::shutdown();
    return 0;
}
