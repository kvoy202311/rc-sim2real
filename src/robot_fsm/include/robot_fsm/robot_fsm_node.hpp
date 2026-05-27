#pragma once
#include <rclcpp/rclcpp.hpp>
#include <kvoy_msgs/msg/motor_state.hpp>
#include <kvoy_msgs/msg/robot_state.hpp>
#include <kvoy_msgs/msg/gamepad_cmd.hpp>
#include <rclcpp/parameter.hpp>
#include "robot_kinematics/kinematics_node.hpp"
#include "robot_kinematics/joint_config.hpp"
#include "rl_controller/rl_controller_node.hpp"

namespace kvoy {

// FSM states
enum class FsmState {
    WAITING  = 0,
    STANDING = 1,
    RUNNING  = 2,
    STANDUP  = 3,
    LIEDOWN  = 4,
    ESTOP    = 5,
};

// Top-level state machine node.
// Owns KinematicsNode and RLControllerNode as composable components.
// Transitions:
//   WAITING  --[btn_standup]--> STANDUP
//   STANDUP  --[motion done]--> STANDING
//   STANDING --[btn_standup]--> RUNNING
//   RUNNING  --[btn_standup]--> STANDING
//   STANDING --[btn_liedown && zero velocity command]--> LIEDOWN
//   LIEDOWN  --[motion done]--> WAITING
//   ESTOP    --[btn_standup]--> STANDUP  (re-arm)
//   any      --[btn_estop]  --> ESTOP

class RobotFsmNode : public rclcpp::Node
{
public:
    explicit RobotFsmNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

    std::shared_ptr<KinematicsNode>   kinematics()    const { return kinematics_; }
    std::shared_ptr<RLControllerNode> rl_controller() const { return rl_controller_; }

private:
    rclcpp::NodeOptions make_kinematics_options();
    rclcpp::NodeOptions make_rl_options();
    void on_gamepad(const kvoy_msgs::msg::GamepadCmd::SharedPtr msg);
    void on_motor_state(const kvoy_msgs::msg::MotorState::SharedPtr msg);
    void on_kinematics_done(KinematicsPhase phase);
    void transition_to(FsmState next);
    void publish_robot_state();
    bool is_zero_velocity_command(const kvoy_msgs::msg::GamepadCmd& msg) const;

    // Sub-nodes (composable, not separate processes)
    std::shared_ptr<KinematicsNode>   kinematics_;
    std::shared_ptr<RLControllerNode> rl_controller_;

    rclcpp::Subscription<kvoy_msgs::msg::GamepadCmd>::SharedPtr  gamepad_sub_;
    rclcpp::Subscription<kvoy_msgs::msg::MotorState>::SharedPtr  state_sub_;
    rclcpp::Publisher<kvoy_msgs::msg::RobotState>::SharedPtr     robot_state_pub_;
    rclcpp::TimerBase::SharedPtr                                 robot_state_timer_;

    FsmState fsm_state_{FsmState::WAITING};
    std::array<float, NUM_JOINTS> current_joint_pos_{};
    bool has_motor_state_{false};

    // Button edge detection (fire on press, not hold)
    bool prev_btn_standup_{false};
    bool prev_btn_liedown_{false};
    bool prev_btn_estop_{false};
};

} // namespace kvoy
