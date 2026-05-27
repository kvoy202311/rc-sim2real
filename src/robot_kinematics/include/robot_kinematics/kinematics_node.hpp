#pragma once
#include <rclcpp/rclcpp.hpp>
#include <kvoy_msgs/msg/motor_cmd.hpp>
#include "robot_kinematics/joint_config.hpp"
#include <array>
#include <functional>

namespace kvoy {

enum class KinematicsPhase { STANDUP, LIEDOWN, DONE };

// Publishes interpolated joint position commands for stand-up / lie-down.
// Caller sets the target pose and duration, then calls step() each control tick.
// When done, invokes the on_done callback.
class KinematicsNode : public rclcpp::Node
{
public:
    explicit KinematicsNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

    // Start a stand-up sequence from current joint positions.
    void start_standup(const std::array<float, NUM_JOINTS>& current_pos);

    // Start a lie-down sequence from current joint positions.
    void start_liedown(const std::array<float, NUM_JOINTS>& current_pos);

    // Register callback invoked when the motion completes.
    void set_done_callback(std::function<void(KinematicsPhase)> cb) { done_cb_ = cb; }

private:
    struct MotionPlan {
        std::array<float, NUM_JOINTS> waypoint{};
        std::array<float, NUM_JOINTS> target{};
        double first_stage_ratio{0.5};
    };

    void load_pose_parameter(
        const std::string& name,
        const std::array<float, NUM_JOINTS>& fallback,
        std::array<float, NUM_JOINTS>& target);
    static float smoothstep(float t);
    std::array<float, NUM_JOINTS> sample_motion_plan(double elapsed_s) const;
    void timer_callback();
    void publish_cmd(const std::array<float, NUM_JOINTS>& pos);

    rclcpp::Publisher<kvoy_msgs::msg::MotorCmd>::SharedPtr   cmd_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::array<float, NUM_JOINTS> start_pos_{};
    std::array<float, NUM_JOINTS> stand_pose_{STAND_POS};
    std::array<float, NUM_JOINTS> lie_pose_{LIE_POS};
    std::array<float, NUM_JOINTS> stand_transition_pose_{STAND_POS};
    std::array<float, NUM_JOINTS> lie_transition_pose_{LIE_POS};
    MotionPlan active_plan_{};

    KinematicsPhase phase_{KinematicsPhase::DONE};
    bool running_{false};
    double duration_s_{5.0};
    double elapsed_s_{0.0};
    double dt_s_{0.02};

    std::function<void(KinematicsPhase)> done_cb_;
};

} // namespace kvoy
