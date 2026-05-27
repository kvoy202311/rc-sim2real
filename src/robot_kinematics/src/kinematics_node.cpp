#include "robot_kinematics/kinematics_node.hpp"
#include <algorithm>

namespace kvoy {

float KinematicsNode::smoothstep(float t)
{
    return t * t * (3.0f - 2.0f * t);
}

void KinematicsNode::load_pose_parameter(
    const std::string& name,
    const std::array<float, NUM_JOINTS>& fallback,
    std::array<float, NUM_JOINTS>& target)
{
    std::vector<double> fallback_vec(fallback.begin(), fallback.end());
    declare_parameter(name, fallback_vec);
    const auto values = get_parameter(name).as_double_array();
    if (values.size() != NUM_JOINTS) {
        throw std::runtime_error(name + " must contain exactly 12 joint values");
    }
    for (int i = 0; i < NUM_JOINTS; ++i) {
        target[i] = static_cast<float>(values[static_cast<std::size_t>(i)]);
    }
}

KinematicsNode::KinematicsNode(const rclcpp::NodeOptions& options)
: Node("kinematics_node", options)
{
    declare_parameter("standup_duration_s", 5.0);
    declare_parameter("liedown_duration_s", 5.0);
    declare_parameter("control_rate_hz",    50.0);

    load_pose_parameter("stand_pose", STAND_POS, stand_pose_);
    load_pose_parameter("lie_pose", LIE_POS, lie_pose_);
    load_pose_parameter("stand_transition_pose", STAND_POS, stand_transition_pose_);
    load_pose_parameter("lie_transition_pose", LIE_POS, lie_transition_pose_);
    duration_s_ = get_parameter("standup_duration_s").as_double();
    dt_s_       = 1.0 / get_parameter("control_rate_hz").as_double();

    cmd_pub_   = create_publisher<kvoy_msgs::msg::MotorCmd>("/motor_cmd", 10);

    auto period = std::chrono::duration<double>(dt_s_);
    timer_ = create_wall_timer(period, std::bind(&KinematicsNode::timer_callback, this));
}

void KinematicsNode::start_standup(const std::array<float, NUM_JOINTS>& current_pos)
{
    start_pos_  = current_pos;
    active_plan_.waypoint = stand_transition_pose_;
    active_plan_.target = stand_pose_;
    active_plan_.first_stage_ratio = 0.5;
    phase_      = KinematicsPhase::STANDUP;
    duration_s_ = get_parameter("standup_duration_s").as_double();
    elapsed_s_  = 0.0;
    running_    = true;
    RCLCPP_INFO(get_logger(), "Kinematics: starting stand-up (%.1fs)", duration_s_);
}

void KinematicsNode::start_liedown(const std::array<float, NUM_JOINTS>& current_pos)
{
    start_pos_  = current_pos;
    active_plan_.waypoint = lie_transition_pose_;
    active_plan_.target = lie_pose_;
    active_plan_.first_stage_ratio = 0.5;
    phase_      = KinematicsPhase::LIEDOWN;
    duration_s_ = get_parameter("liedown_duration_s").as_double();
    elapsed_s_  = 0.0;
    running_    = true;
    RCLCPP_INFO(get_logger(), "Kinematics: starting lie-down (%.1fs)", duration_s_);
}

std::array<float, NUM_JOINTS> KinematicsNode::sample_motion_plan(double elapsed_s) const
{
    const double clamped_ratio = std::max(0.05, std::min(0.95, active_plan_.first_stage_ratio));
    const double first_stage_duration = duration_s_ * clamped_ratio;

    if (elapsed_s <= first_stage_duration) {
        const float alpha = static_cast<float>(
            std::min(elapsed_s / first_stage_duration, 1.0));
        return lerp_joints(start_pos_, active_plan_.waypoint, smoothstep(alpha));
    }

    const double second_stage_duration = std::max(duration_s_ - first_stage_duration, 1e-6);
    const float alpha = static_cast<float>(
        std::min((elapsed_s - first_stage_duration) / second_stage_duration, 1.0));
    return lerp_joints(active_plan_.waypoint, active_plan_.target, smoothstep(alpha));
}

void KinematicsNode::timer_callback()
{
    if (!running_) return;

    elapsed_s_ += dt_s_;
    const float alpha = static_cast<float>(std::min(elapsed_s_ / duration_s_, 1.0));
    auto pos = sample_motion_plan(elapsed_s_);
    publish_cmd(pos);

    if (alpha >= 1.0f) {
        running_ = false;
        RCLCPP_INFO(get_logger(), "Kinematics: motion complete");
        if (done_cb_) done_cb_(phase_);
        phase_ = KinematicsPhase::DONE;
    }
}

void KinematicsNode::publish_cmd(const std::array<float, NUM_JOINTS>& pos)
{
    auto msg = kvoy_msgs::msg::MotorCmd();
    msg.header.stamp = now();
    for (int i = 0; i < NUM_JOINTS; ++i) {
        msg.position[i] = pos[i];
        msg.velocity[i] = 0.0f;
        msg.torque[i]   = 0.0f;
    }
    cmd_pub_->publish(msg);
}

} // namespace kvoy
