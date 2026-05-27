#include <rclcpp/rclcpp.hpp>
#include <kvoy_msgs/msg/motor_cmd.hpp>
#include <kvoy_msgs/msg/motor_state.hpp>
#include <kvoy_msgs/msg/robot_state.hpp>
#include "serial_comm/protocol.hpp"
#include "serial_comm/serial_port.hpp"
#include <thread>
#include <mutex>
#include <atomic>
#include <array>
#include <string>
#include <chrono>
#include <vector>

namespace kvoy {

class SerialCommNode : public rclcpp::Node
{
public:
    explicit SerialCommNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    ~SerialCommNode();

private:
    void open_port();
    bool try_open_port();
    void rx_thread_fn();
    void on_motor_cmd(const kvoy_msgs::msg::MotorCmd::SharedPtr msg);
    void on_robot_state(const kvoy_msgs::msg::RobotState::SharedPtr msg);
    void close_port();
    void close_port_locked();
    void process_rx_bytes(const uint8_t* data, std::size_t len);
    void drop_rx_candidate_and_resync(std::size_t candidate_len);
    bool build_state_msg(
        const protocol::StateFrame& frame,
        kvoy_msgs::msg::MotorState& msg);
    void store_latest_state_frame(const protocol::StateFrame& frame, uint64_t frame_seq);
    void publish_latest_state();
    void send_damping_command_from_latest_state(const char* reason);
    void capture_debug_rx_frame(const uint8_t* data);
    void debug_timer_callback();
    void handle_port_fault(const std::string& message);
    void reset_rx_parser_state();
    void check_rx_timeout();

    // ROS interfaces
    rclcpp::Subscription<kvoy_msgs::msg::MotorCmd>::SharedPtr   cmd_sub_;
    rclcpp::Subscription<kvoy_msgs::msg::RobotState>::SharedPtr robot_state_sub_;
    rclcpp::Publisher<kvoy_msgs::msg::MotorState>::SharedPtr    state_pub_;
    rclcpp::TimerBase::SharedPtr                                state_publish_timer_;
    rclcpp::TimerBase::SharedPtr                                debug_timer_;

    // Serial
    SerialPort        port_;
    std::string       port_name_;
    int               baud_rate_;
    std::atomic<uint8_t> cmd_ctrl_mode_{protocol::CTRL_MODE_DAMPING};
    std::chrono::steady_clock::time_point last_robot_state_time_{};
    bool     have_robot_state_{false};
    uint8_t  last_robot_fsm_state_{kvoy_msgs::msg::RobotState::FSM_WAITING};
    bool     have_last_robot_fsm_state_{false};
    std::array<float, protocol::NUM_JOINTS> motor_polarity_{};
    std::array<float, protocol::NUM_JOINTS> motor_zero_offset_{};
    std::mutex        port_mutex_;
    std::mutex        rx_mutex_;
    std::mutex        latest_state_mutex_;
    std::mutex        debug_log_mutex_;
    std::thread       rx_thread_;
    std::atomic<bool> running_{false};

    static constexpr std::size_t RX_READ_CHUNK = 8192;
    static constexpr std::size_t RX_MAX_READS_PER_CYCLE = 16;
    protocol::RawStateFrame rx_frame_buf_{};
    std::size_t rx_frame_pos_{0};
    int      reconnect_interval_ms_{200};
    int      rx_timeout_ms_{200};
    std::chrono::nanoseconds state_publish_period_{std::chrono::milliseconds(5)};
    std::chrono::steady_clock::time_point last_reconnect_attempt_{};
    std::chrono::steady_clock::time_point last_rx_frame_time_{};
    bool     have_rx_frame_time_{false};
    protocol::StateFrame latest_state_frame_{};
    uint64_t latest_state_seq_{0};
    uint64_t last_published_state_seq_{0};
    bool     have_latest_state_{false};
    protocol::RawStateFrame debug_rx_frame_{};
    bool     have_debug_rx_frame_{false};
    bool     debug_serial_io_{false};
    bool     debug_serial_hex_bytes_{false};
    std::atomic<uint64_t> tx_frame_count_{0};
    std::atomic<uint64_t> rx_byte_count_{0};
    std::atomic<uint64_t> rx_read_count_{0};
    std::atomic<uint64_t> rx_frame_count_{0};
    std::atomic<uint64_t> rx_publish_count_{0};
    std::atomic<uint64_t> rx_drop_count_{0};
    std::atomic<uint64_t> rx_resync_discarded_bytes_{0};
    std::atomic<uint64_t> rx_bad_version_count_{0};
    std::atomic<uint64_t> rx_bad_type_count_{0};
    std::atomic<uint64_t> rx_bad_length_count_{0};
    std::atomic<uint64_t> rx_bad_crc_count_{0};
    std::atomic<uint64_t> rx_bad_tail_count_{0};
    std::chrono::steady_clock::time_point last_debug_log_time_{};
    uint64_t last_debug_rx_byte_count_{0};
    uint64_t last_debug_rx_read_count_{0};
    uint64_t last_debug_rx_frame_count_{0};
    uint64_t last_debug_rx_publish_count_{0};
    uint64_t last_debug_rx_drop_count_{0};
    uint64_t last_debug_rx_resync_discarded_bytes_{0};

    void log_frame_hex(const char* prefix, const uint8_t* data, std::size_t len);
};

} // namespace kvoy
