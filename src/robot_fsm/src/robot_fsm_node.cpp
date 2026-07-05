#include "robot_fsm/robot_fsm_node.hpp"

namespace kvoy {

namespace {

constexpr float kZeroCmdThreshold = 0.05f;
constexpr double kRobotStatePublishRateHz = 20.0;

std::vector<std::string> get_string_array_param(
    rclcpp::Node& node, const std::string& name, const std::vector<std::string>& default_value)
{
    node.declare_parameter(name, default_value);
    return node.get_parameter(name).as_string_array();
}

std::string get_string_param(
    rclcpp::Node& node, const std::string& name, const std::string& default_value)
{
    node.declare_parameter(name, default_value);
    return node.get_parameter(name).as_string();
}

bool get_bool_param(rclcpp::Node& node, const std::string& name, bool default_value)
{
    node.declare_parameter(name, default_value);
    return node.get_parameter(name).as_bool();
}

std::vector<double> get_double_array_param(
    rclcpp::Node& node, const std::string& name, const std::vector<double>& default_value)
{
    node.declare_parameter(name, default_value);
    return node.get_parameter(name).as_double_array();
}

std::vector<double> identity_rotation()
{
    return {
        1.0, 0.0, 0.0,
        0.0, 1.0, 0.0,
        0.0, 0.0, 1.0};
}

double get_double_param(rclcpp::Node& node, const std::string& name, double default_value)
{
    node.declare_parameter(name, default_value);
    return node.get_parameter(name).as_double();
}

int64_t get_int_param(rclcpp::Node& node, const std::string& name, int64_t default_value)
{
    node.declare_parameter(name, default_value);
    return node.get_parameter(name).as_int();
}

} // namespace

RobotFsmNode::RobotFsmNode(const rclcpp::NodeOptions& options)
: Node("robot_fsm_node", options)
{
    // Create sub-nodes
    kinematics_ = std::make_shared<KinematicsNode>(make_kinematics_options());
    rl_controller_ = std::make_shared<RLControllerNode>(make_rl_options());

    kinematics_->set_done_callback(
        std::bind(&RobotFsmNode::on_kinematics_done, this, std::placeholders::_1));

    // Subscribers
    gamepad_sub_ = create_subscription<kvoy_msgs::msg::GamepadCmd>(
        "/gamepad_cmd", 10,
        std::bind(&RobotFsmNode::on_gamepad, this, std::placeholders::_1));
    state_sub_ = create_subscription<kvoy_msgs::msg::MotorState>(
        "/motor_state", 10,
        std::bind(&RobotFsmNode::on_motor_state, this, std::placeholders::_1));

    auto robot_state_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable();
    robot_state_pub_ = create_publisher<kvoy_msgs::msg::RobotState>(
        "/robot_state", robot_state_qos);
    robot_state_timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::duration<double>(1.0 / kRobotStatePublishRateHz)),
        std::bind(&RobotFsmNode::publish_robot_state, this));

    RCLCPP_INFO(get_logger(), "robot_fsm_node started — state: WAITING");
    RCLCPP_INFO(get_logger(), "  LT+LB+Y    -> wait->stand / stand->run / run->stand");
    RCLCPP_INFO(get_logger(), "  RT+RB+X    -> stand->lie (only when cmd velocity is zero)");
    RCLCPP_INFO(get_logger(), "  LB+RB      -> emergency stop");
    RCLCPP_INFO(get_logger(), "  RT+RB+X    -> estop->wait/damping");

    publish_robot_state();
}

rclcpp::NodeOptions RobotFsmNode::make_kinematics_options()
{
    return rclcpp::NodeOptions()
        .arguments({"--ros-args", "-r", "__node:=kinematics_node"})
        .parameter_overrides({
            rclcpp::Parameter("standup_duration_s", get_double_param(*this, "standup_duration_s", 5.0)),
            rclcpp::Parameter("liedown_duration_s", get_double_param(*this, "liedown_duration_s", 5.0)),
            rclcpp::Parameter("control_rate_hz", get_double_param(*this, "kinematics_control_rate_hz", 50.0)),
            rclcpp::Parameter(
                "stand_pose",
                get_double_array_param(
                    *this, "stand_pose", std::vector<double>(STAND_POS.begin(), STAND_POS.end()))),
            rclcpp::Parameter(
                "stand_transition_pose",
                get_double_array_param(
                    *this, "stand_transition_pose", std::vector<double>(STAND_POS.begin(), STAND_POS.end()))),
            rclcpp::Parameter(
                "lie_pose",
                get_double_array_param(
                    *this, "lie_pose", std::vector<double>(LIE_POS.begin(), LIE_POS.end()))),
            rclcpp::Parameter(
                "lie_transition_pose",
                get_double_array_param(
                    *this, "lie_transition_pose", std::vector<double>(LIE_POS.begin(), LIE_POS.end()))),
        });
}

rclcpp::NodeOptions RobotFsmNode::make_rl_options()
{
    const auto policy_slot_names =
        get_string_array_param(*this, "policy_slot_names", std::vector<std::string>{});

    std::vector<rclcpp::Parameter> overrides{
        rclcpp::Parameter("policy_slot_names", policy_slot_names),
        rclcpp::Parameter("default_policy", get_string_param(*this, "default_policy", "")),
        rclcpp::Parameter("control_rate_hz", get_double_param(*this, "rl_control_rate_hz", 50.0)),
        rclcpp::Parameter("action_scale", get_double_param(*this, "action_scale", 0.25)),
        rclcpp::Parameter("clip_obs", get_double_param(*this, "clip_obs", 100.0)),
        rclcpp::Parameter("clip_actions", get_double_param(*this, "clip_actions", 100.0)),
        rclcpp::Parameter("obs_ang_vel_scale", get_double_param(*this, "obs_ang_vel_scale", 0.25)),
        rclcpp::Parameter("obs_dof_pos_scale", get_double_param(*this, "obs_dof_pos_scale", 1.0)),
        rclcpp::Parameter("obs_dof_vel_scale", get_double_param(*this, "obs_dof_vel_scale", 0.05)),
        rclcpp::Parameter("cmd_lin_vel_scale", get_double_param(*this, "cmd_lin_vel_scale", 2.0)),
        rclcpp::Parameter("cmd_ang_vel_scale", get_double_param(*this, "cmd_ang_vel_scale", 0.25)),
        rclcpp::Parameter("height_command_min_m", get_double_param(*this, "height_command_min_m", 0.10)),
        rclcpp::Parameter("height_command_max_m", get_double_param(*this, "height_command_max_m", 0.26)),
        rclcpp::Parameter("height_command_default_m", get_double_param(*this, "height_command_default_m", 0.18)),
        rclcpp::Parameter("height_command_step_m", get_double_param(*this, "height_command_step_m", 0.01)),
        rclcpp::Parameter(
            "height_command_zero_cmd_threshold",
            get_double_param(*this, "height_command_zero_cmd_threshold", 0.05)),
        rclcpp::Parameter("yaw_command_mode", get_string_param(*this, "yaw_command_mode", "rate")),
        rclcpp::Parameter(
            "heading_command_error_gain",
            get_double_param(*this, "heading_command_error_gain", 0.5)),
        rclcpp::Parameter(
            "heading_command_max_yaw_rate_rad_s",
            get_double_param(*this, "heading_command_max_yaw_rate_rad_s", 1.0)),
        rclcpp::Parameter("imu_ang_vel_filter_alpha", get_double_param(*this, "imu_ang_vel_filter_alpha", 1.0)),
        rclcpp::Parameter(
            "imu_projected_gravity_filter_alpha",
            get_double_param(*this, "imu_projected_gravity_filter_alpha", 1.0)),
        rclcpp::Parameter(
            "joint_pos_min",
            get_double_array_param(*this, "joint_pos_min", std::vector<double>(NUM_JOINTS, -3.14))),
        rclcpp::Parameter(
            "joint_pos_max",
            get_double_array_param(*this, "joint_pos_max", std::vector<double>(NUM_JOINTS, 3.14))),
        rclcpp::Parameter("max_imu_age_s", get_double_param(*this, "max_imu_age_s", 0.1)),
        rclcpp::Parameter(
            "policy_switch_sound_enabled",
            get_bool_param(*this, "policy_switch_sound_enabled", false)),
        rclcpp::Parameter(
            "policy_switch_sound_file",
            get_string_param(*this, "policy_switch_sound_file", "")),
        rclcpp::Parameter(
            "policy_switch_sound_interval_ms",
            static_cast<int>(get_int_param(*this, "policy_switch_sound_interval_ms", 150))),
        rclcpp::Parameter(
            "imu_to_body_rotation",
            get_double_array_param(*this, "imu_to_body_rotation", identity_rotation())),
    };

    for (const auto& slot_name : policy_slot_names) {
        const auto trimmed_slot = get_string_param(*this, slot_name + ".name", slot_name);
        overrides.emplace_back(slot_name + ".name", trimmed_slot);
        overrides.emplace_back(slot_name + ".path", get_string_param(*this, slot_name + ".path", ""));
        overrides.emplace_back(slot_name + ".history_steps", static_cast<int>(
            get_int_param(*this, slot_name + ".history_steps", 6)));
        overrides.emplace_back(slot_name + ".one_step_obs", static_cast<int>(
            get_int_param(*this, slot_name + ".one_step_obs", 45)));
        overrides.emplace_back(
            slot_name + ".default_joint_pos",
            get_double_array_param(*this, slot_name + ".default_joint_pos",
                std::vector<double>(STAND_POS.begin(), STAND_POS.end())));
        overrides.emplace_back(slot_name + ".action_scale",
                               get_double_param(*this, slot_name + ".action_scale", 0.25));
        overrides.emplace_back(slot_name + ".clip_obs",
                               get_double_param(*this, slot_name + ".clip_obs", 100.0));
        overrides.emplace_back(slot_name + ".clip_actions",
                               get_double_param(*this, slot_name + ".clip_actions", 100.0));
        overrides.emplace_back(slot_name + ".obs_ang_vel_scale",
                               get_double_param(*this, slot_name + ".obs_ang_vel_scale", 0.25));
        overrides.emplace_back(slot_name + ".obs_dof_pos_scale",
                               get_double_param(*this, slot_name + ".obs_dof_pos_scale", 1.0));
        overrides.emplace_back(slot_name + ".obs_dof_vel_scale",
                               get_double_param(*this, slot_name + ".obs_dof_vel_scale", 0.05));
        overrides.emplace_back(slot_name + ".cmd_lin_vel_scale",
                               get_double_param(*this, slot_name + ".cmd_lin_vel_scale", 2.0));
        overrides.emplace_back(slot_name + ".cmd_ang_vel_scale",
                               get_double_param(*this, slot_name + ".cmd_ang_vel_scale", 0.25));
        overrides.emplace_back(slot_name + ".height_command_enabled",
                               get_bool_param(*this, slot_name + ".height_command_enabled", false));
        overrides.emplace_back(slot_name + ".cmd_height_scale",
                               get_double_param(*this, slot_name + ".cmd_height_scale", 4.0));
    }

    return rclcpp::NodeOptions()
        .arguments({"--ros-args", "-r", "__node:=rl_controller_node"})
        .parameter_overrides(overrides);
}

void RobotFsmNode::on_motor_state(const kvoy_msgs::msg::MotorState::SharedPtr msg)
{
    for (int i = 0; i < NUM_JOINTS; ++i)
        current_joint_pos_[i] = msg->position[i];
    has_motor_state_ = true;
}

void RobotFsmNode::on_gamepad(const kvoy_msgs::msg::GamepadCmd::SharedPtr msg)
{
    const bool standup = msg->btn_standup;
    const bool liedown = msg->btn_liedown;
    const bool estop   = msg->btn_estop;

    // Rising-edge detection
    const bool press_standup = standup && !prev_btn_standup_;
    const bool press_liedown = liedown && !prev_btn_liedown_;
    const bool press_estop   = estop   && !prev_btn_estop_;

    prev_btn_standup_ = standup;
    prev_btn_liedown_ = liedown;
    prev_btn_estop_   = estop;

    // E-stop has highest priority from any state
    if (press_estop && fsm_state_ != FsmState::ESTOP) {
        transition_to(FsmState::ESTOP);
        return;
    }

    switch (fsm_state_) {
        case FsmState::WAITING:
            if (press_standup) {
                if (!has_motor_state_) {
                    RCLCPP_WARN(
                        get_logger(),
                        "Stand-up ignored: no valid /motor_state has been received yet");
                } else {
                    transition_to(FsmState::STANDUP);
                }
            }
            break;

        case FsmState::STANDING:
            if (press_standup) {
                transition_to(FsmState::RUNNING);
            } else if (press_liedown) {
                if (is_zero_velocity_command(*msg)) {
                    transition_to(FsmState::LIEDOWN);
                } else {
                    RCLCPP_WARN(get_logger(),
                                "Lie-down ignored: velocity command must be zero while standing");
                }
            }
            break;

        case FsmState::RUNNING:
            if (press_standup) transition_to(FsmState::STANDING);
            break;

        case FsmState::ESTOP:
            if (press_liedown) {
                transition_to(FsmState::WAITING);
            } else if (press_standup) {
                if (!has_motor_state_) {
                    RCLCPP_WARN(
                        get_logger(),
                        "Re-arm ignored: no valid /motor_state has been received yet");
                } else {
                    transition_to(FsmState::STANDUP);
                }
            }
            break;

        case FsmState::STANDUP:
        case FsmState::LIEDOWN:
            // Ignore buttons during motion - only e-stop is allowed (handled above)
            break;
    }
}

void RobotFsmNode::on_kinematics_done(KinematicsPhase phase)
{
    if (phase == KinematicsPhase::STANDUP) {
        transition_to(FsmState::STANDING);
    } else if (phase == KinematicsPhase::LIEDOWN) {
        transition_to(FsmState::WAITING);
    }
}

bool RobotFsmNode::is_zero_velocity_command(const kvoy_msgs::msg::GamepadCmd& msg) const
{
    return std::abs(msg.vx) <= kZeroCmdThreshold &&
           std::abs(msg.vy) <= kZeroCmdThreshold &&
           std::abs(msg.yaw_rate) <= kZeroCmdThreshold;
}

void RobotFsmNode::transition_to(FsmState next)
{
    auto state_name = [](FsmState s) -> const char* {
        switch (s) {
            case FsmState::WAITING:  return "WAITING";
            case FsmState::STANDING: return "STANDING";
            case FsmState::RUNNING:  return "RUNNING";
            case FsmState::STANDUP:  return "STANDUP";
            case FsmState::LIEDOWN:  return "LIEDOWN";
            case FsmState::ESTOP:    return "ESTOP";
        }
        return "UNKNOWN";
    };

    RCLCPP_INFO(get_logger(), "FSM: %s → %s",
                state_name(fsm_state_), state_name(next));

    // Exit current state
    switch (fsm_state_) {
        case FsmState::RUNNING:
            rl_controller_->set_active(false);
            break;
        default:
            break;
    }

    fsm_state_ = next;

    // Enter new state
    switch (fsm_state_) {
        case FsmState::STANDUP:
            kinematics_->start_standup(current_joint_pos_);
            break;

        case FsmState::RUNNING:
            rl_controller_->set_active(true);
            break;

        case FsmState::LIEDOWN:
            kinematics_->start_liedown(current_joint_pos_);
            break;

        case FsmState::ESTOP:
            rl_controller_->set_active(false);
            RCLCPP_WARN(get_logger(), "ESTOP active — all outputs halted");
            break;

        case FsmState::WAITING:
        case FsmState::STANDING:
            break;
    }

    publish_robot_state();
}

void RobotFsmNode::publish_robot_state()
{
    auto msg = kvoy_msgs::msg::RobotState();
    msg.header.stamp = now();
    msg.fsm_state    = static_cast<uint8_t>(fsm_state_);
    robot_state_pub_->publish(msg);
}

} // namespace kvoy

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    // Use a MultiThreadedExecutor so sub-nodes run their own callbacks
    rclcpp::executors::MultiThreadedExecutor executor;

    auto fsm = std::make_shared<kvoy::RobotFsmNode>();

    // Spin sub-nodes alongside FSM
    executor.add_node(fsm);
    executor.add_node(fsm->kinematics());
    executor.add_node(fsm->rl_controller());

    executor.spin();
    rclcpp::shutdown();
    return 0;
}
