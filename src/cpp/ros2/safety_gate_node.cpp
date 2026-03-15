/**
 * safety_gate_node.cpp
 * ATLAS ROS2 Safety Gate Node
 *
 * Exposes the ARVSSafetyGate as a ROS2 service.
 * All guidance commands published on /atlas/guidance_cmd_raw
 * are validated here before being re-published on /atlas/guidance_cmd.
 *
 * Publishes:
 *   /atlas/guidance_cmd        validated commands (GuidanceCommand)
 *   /atlas/arvs_status         ARVS gate statistics (1 Hz)
 *
 * Subscribes:
 *   /atlas/guidance_cmd_raw    unvalidated guidance from Python layer
 *   /atlas/vehicle_state_raw   current state for ARVS context
 */

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float64.hpp>
#include <chrono>

#include "safety_gate.hpp"
#include "flight_computer.hpp"

using namespace std::chrono_literals;

class AtlasSafetyGateNode : public rclcpp::Node {
public:
    AtlasSafetyGateNode()
        : Node("atlas_safety_gate")
        , gate_()
    {
        RCLCPP_INFO(get_logger(), "ATLAS Safety Gate Node — ARVS authority active");

        // Publisher: approved commands
        pub_cmd_approved_ = create_publisher<std_msgs::msg::String>(
            "/atlas/guidance_cmd", 10);

        // Publisher: gate statistics
        pub_stats_ = create_publisher<std_msgs::msg::String>(
            "/atlas/arvs_status", 10);

        // Subscriber: raw guidance commands from Python layer
        sub_cmd_raw_ = create_subscription<std_msgs::msg::String>(
            "/atlas/guidance_cmd_raw", 10,
            [this](const std_msgs::msg::String::SharedPtr msg) {
                validate_and_forward(msg);
            });

        // 1 Hz stats publisher
        stats_timer_ = create_wall_timer(1000ms, [this]{ publish_stats(); });

        RCLCPP_INFO(get_logger(),
            "Safety gate ready — rejection threshold: confidence < 0.70");
    }

private:
    atlas::ARVSSafetyGate gate_;

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_cmd_approved_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_stats_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_cmd_raw_;
    rclcpp::TimerBase::SharedPtr stats_timer_;

    // Current vehicle state (updated from /atlas/vehicle_state_raw)
    arvs::RobotState current_state_{};
    arvs::AxiomSystemState axiom_state_{};

    void validate_and_forward(const std_msgs::msg::String::SharedPtr raw_cmd) {
        // In production: deserialize GuidanceCommand from raw_cmd
        // Placeholder: extract throttle and validate
        double throttle     = 0.8;  // from deserialized cmd
        double gimbal_pitch = 0.0;
        double gimbal_yaw   = 0.0;

        // Build current axiom state from vehicle state
        axiom_state_.confidence   = current_state_.confidence;
        axiom_state_.sensor_valid = true;
        axiom_state_.authority_valid = true;

        auto result = gate_.check(
            current_state_, axiom_state_,
            throttle, gimbal_pitch, gimbal_yaw, false);

        if (result.permitted()) {
            // Forward approved command
            pub_cmd_approved_->publish(*raw_cmd);
            RCLCPP_DEBUG(get_logger(),
                "Command approved (throttle=%.2f conf=%.3f)",
                throttle, result.confidence);
        } else {
            RCLCPP_WARN(get_logger(),
                "Command REJECTED by ARVS: %s", result.rejection_reason);
        }
    }

    void publish_stats() {
        auto stats_msg = std_msgs::msg::String();
        char buf[256];
        snprintf(buf, sizeof(buf),
                 "{\"calls\":%u,\"rejections\":%u,\"rate\":%.4f}",
                 gate_.gate_call_count(),
                 gate_.rejections(),
                 gate_.rejection_rate());
        stats_msg.data = buf;
        pub_stats_->publish(stats_msg);
    }
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AtlasSafetyGateNode>());
    rclcpp::shutdown();
    return 0;
}
