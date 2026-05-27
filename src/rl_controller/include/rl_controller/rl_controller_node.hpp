#pragma once
#include <rclcpp/rclcpp.hpp>
#include <kvoy_msgs/msg/motor_cmd.hpp>
#include <kvoy_msgs/msg/motor_state.hpp>
#include <kvoy_msgs/msg/imu_data.hpp>
#include <kvoy_msgs/msg/gamepad_cmd.hpp>
#include "robot_kinematics/joint_config.hpp"

#include <array>
#include <atomic>
#include <cstdint>
#include <mutex>
#include <memory>
#include <string>
#include <unordered_map>

namespace kvoy {

// HimLoco policy inference node backed by TensorRT.
//
// Observation (per step, 45 dims):
//   commands(3) * commands_scale
//   ang_vel(3)  * obs_scales.ang_vel
//   gravity(3)
//   dof_pos(12) * obs_scales.dof_pos   (relative to default_pos)
//   dof_vel(12) * obs_scales.dof_vel
//   last_action(12)
//
// History: 6 steps → input tensor shape [1, 270]
// Output: 12 joint position offsets → target = default_pos + action * action_scale

class RLControllerNode : public rclcpp::Node
{
public:
    explicit RLControllerNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    ~RLControllerNode() override;

    // Called by FSM to enable/disable inference output
    void set_active(bool active);
    bool is_active() const { return active_; }

private:
    struct PolicyConfig;
    struct TensorRtContext;
    static constexpr int HIMLOCO_V1_ONE_STEP_OBS = 45;

    void on_motor_state(const kvoy_msgs::msg::MotorState::SharedPtr msg);
    void on_imu(const kvoy_msgs::msg::ImuData::SharedPtr msg);
    void on_gamepad(const kvoy_msgs::msg::GamepadCmd::SharedPtr msg);
    void control_loop();
    std::unique_ptr<TensorRtContext> load_engine(
        const std::string& policy_name,
        const std::string& engine_path,
        const PolicyConfig& config) const;
    bool select_policy_locked(const std::string& policy_name);
    bool select_policy_by_offset_locked(int offset);
    void reset_runtime_state_locked(TensorRtContext& policy);
    void request_policy_switch_sound_locked();
    void load_policies_from_params();
    void run_inference(
        TensorRtContext& trt,
        const std::vector<float>& obs_history,
        std::array<float, NUM_JOINTS>& action) const;
    std::array<float, NUM_JOINTS> clamp_joint_targets(
        const std::array<float, NUM_JOINTS>& target) const;
    PolicyConfig make_default_policy_config() const;
    void load_imu_params();

    // Build single-step observation vector (45 dims)
    std::array<float, HIMLOCO_V1_ONE_STEP_OBS> build_obs(
        const PolicyConfig& config,
        const std::array<float, NUM_JOINTS>& last_action) const;

    // Rotate world vector into body frame using quaternion (x,y,z,w)
    static std::array<float, 3> quat_rotate_inverse(
        const std::array<float, 4>& q, const std::array<float, 3>& v);
    static std::array<float, 3> mat3_mul_vec3(
        const std::array<float, 9>& m, const std::array<float, 3>& v);

    // ROS interfaces
    rclcpp::Subscription<kvoy_msgs::msg::MotorState>::SharedPtr state_sub_;
    rclcpp::Subscription<kvoy_msgs::msg::ImuData>::SharedPtr    imu_sub_;
    rclcpp::Subscription<kvoy_msgs::msg::GamepadCmd>::SharedPtr gamepad_sub_;
    rclcpp::Publisher<kvoy_msgs::msg::MotorCmd>::SharedPtr      cmd_pub_;
    rclcpp::TimerBase::SharedPtr                                timer_;

    std::unordered_map<std::string, std::unique_ptr<TensorRtContext>> policies_;
    std::vector<std::string> policy_order_;
    std::string active_policy_name_;
    bool policies_loaded_{false};

    // State (updated by subscribers)
    std::array<float, NUM_JOINTS> joint_pos_{};
    std::array<float, NUM_JOINTS> joint_vel_{};
    std::array<float, 3>          ang_vel_{};          // body frame [rad/s]
    std::array<float, 3>          projected_gravity_{0.0f, 0.0f, -1.0f};
    std::array<float, 3>          command_{};           // vx, vy, yaw_rate
    std::array<float, NUM_JOINTS> hold_position_target_{STAND_POS};
    bool has_motor_state_{false};
    bool has_imu_{false};
    rclcpp::Time last_imu_time_{0, 0, RCL_ROS_TIME};
    bool prev_btn_policy_prev_{false};
    bool prev_btn_policy_next_{false};

    bool active_{false};
    int hold_cycles_remaining_{0};
    mutable std::mutex mutex_;

    // Config (loaded from ROS params)
    float action_scale_;
    float clip_obs_;
    float clip_actions_;
    float obs_ang_vel_scale_;
    float obs_dof_pos_scale_;
    float obs_dof_vel_scale_;
    float cmd_lin_vel_scale_;
    float cmd_ang_vel_scale_;
    std::array<float, NUM_JOINTS> joint_pos_min_{};
    std::array<float, NUM_JOINTS> joint_pos_max_{};
    double max_imu_age_s_{0.1};
    bool policy_switch_sound_enabled_{false};
    std::string policy_switch_sound_file_;
    int policy_switch_sound_interval_ms_{150};
    std::shared_ptr<std::atomic<std::uint64_t>> sound_request_seq_{
        std::make_shared<std::atomic<std::uint64_t>>(0)};
    std::array<float, 9> imu_to_body_rotation_{
        1.0f, 0.0f, 0.0f,
        0.0f, 1.0f, 0.0f,
        0.0f, 0.0f, 1.0f};
};

} // namespace kvoy
