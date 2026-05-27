#include "serial_comm/serial_comm_node.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <iomanip>
#include <cstring>
#include <sstream>
#include <stdexcept>
#include <cerrno>

namespace kvoy {
namespace {

constexpr auto kRobotStateTimeout = std::chrono::milliseconds(500);

std::array<float, kvoy::protocol::NUM_JOINTS> load_motor_polarity(kvoy::SerialCommNode& node)
{
    const auto values = node.get_parameter("motor_polarity").as_integer_array();
    if (values.size() != kvoy::protocol::NUM_JOINTS) {
        throw std::runtime_error("Parameter motor_polarity must contain exactly 12 entries");
    }

    std::array<float, kvoy::protocol::NUM_JOINTS> polarity{};
    for (std::size_t i = 0; i < values.size(); ++i) {
        if (values[i] != 1 && values[i] != -1) {
            std::ostringstream oss;
            oss << "Parameter motor_polarity[" << i << "] must be either 1 or -1";
            throw std::runtime_error(oss.str());
        }
        polarity[i] = static_cast<float>(values[i]);
    }

    return polarity;
}

std::array<float, kvoy::protocol::NUM_JOINTS> load_motor_zero_offset(kvoy::SerialCommNode& node)
{
    const auto values = node.get_parameter("motor_zero_offset").as_double_array();
    if (values.size() != kvoy::protocol::NUM_JOINTS) {
        throw std::runtime_error("Parameter motor_zero_offset must contain exactly 12 entries");
    }

    std::array<float, kvoy::protocol::NUM_JOINTS> offset{};
    for (std::size_t i = 0; i < values.size(); ++i) {
        if (!std::isfinite(values[i])) {
            std::ostringstream oss;
            oss << "Parameter motor_zero_offset[" << i << "] must be finite";
            throw std::runtime_error(oss.str());
        }
        offset[i] = static_cast<float>(values[i]);
    }

    return offset;
}

} // namespace

SerialCommNode::SerialCommNode(const rclcpp::NodeOptions& options)
: Node("serial_comm_node", options)
{
    declare_parameter("serial_port", "/dev/serial/by-id/usb-1a86_USB_Single_Serial_5792019699-if00");
    declare_parameter("baud_rate",   1000000);
    declare_parameter("reconnect_interval_ms", 200);
    declare_parameter("rx_timeout_ms", 200);
    declare_parameter("state_publish_rate_hz", 200.0);
    declare_parameter("motor_polarity", std::vector<int64_t>(protocol::NUM_JOINTS, 1));
    declare_parameter("motor_zero_offset", std::vector<double>(protocol::NUM_JOINTS, 0.0));
    declare_parameter("debug_serial_io", false);
    declare_parameter("debug_serial_hex_bytes", false);

    port_name_ = get_parameter("serial_port").as_string();
    baud_rate_ = get_parameter("baud_rate").as_int();
    reconnect_interval_ms_ = get_parameter("reconnect_interval_ms").as_int();
    rx_timeout_ms_ = get_parameter("rx_timeout_ms").as_int();
    const double state_publish_rate_hz =
        std::max(1.0, get_parameter("state_publish_rate_hz").as_double());
    state_publish_period_ = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / state_publish_rate_hz));
    motor_polarity_ = load_motor_polarity(*this);
    motor_zero_offset_ = load_motor_zero_offset(*this);
    debug_serial_io_ = get_parameter("debug_serial_io").as_bool();
    debug_serial_hex_bytes_ = get_parameter("debug_serial_hex_bytes").as_bool();
    last_debug_log_time_ = std::chrono::steady_clock::time_point::min();

    state_pub_ = create_publisher<kvoy_msgs::msg::MotorState>("/motor_state", 10);
    state_publish_timer_ = create_wall_timer(
        state_publish_period_, std::bind(&SerialCommNode::publish_latest_state, this));
    if (debug_serial_io_) {
        debug_timer_ = create_wall_timer(
            std::chrono::seconds(1), std::bind(&SerialCommNode::debug_timer_callback, this));
    }
    cmd_sub_   = create_subscription<kvoy_msgs::msg::MotorCmd>(
        "/motor_cmd", 10,
        std::bind(&SerialCommNode::on_motor_cmd, this, std::placeholders::_1));
    auto robot_state_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable();
    robot_state_sub_ = create_subscription<kvoy_msgs::msg::RobotState>(
        "/robot_state", robot_state_qos,
        std::bind(&SerialCommNode::on_robot_state, this, std::placeholders::_1));

    last_reconnect_attempt_ = std::chrono::steady_clock::time_point::min();
    if (!try_open_port()) {
        RCLCPP_WARN(
            get_logger(),
            "serial_comm_node started without an open serial port; will retry in background");
    }

    running_ = true;
    rx_thread_ = std::thread(&SerialCommNode::rx_thread_fn, this);

    RCLCPP_INFO(get_logger(), "serial_comm_node started on %s @ %d baud",
                port_name_.c_str(), baud_rate_);
    RCLCPP_INFO(get_logger(), "serial_comm_node state publish rate limited to %.1f Hz",
                state_publish_rate_hz);
    if (debug_serial_io_) {
        RCLCPP_INFO(
            get_logger(),
            "serial_comm_node debug enabled (hex_bytes=%s)",
            debug_serial_hex_bytes_ ? "true" : "false");
    }
}

SerialCommNode::~SerialCommNode()
{
    running_ = false;
    if (rx_thread_.joinable()) {
        rx_thread_.join();
    }
    close_port();
}

void SerialCommNode::close_port()
{
    std::lock_guard<std::mutex> lock(port_mutex_);
    close_port_locked();
}

void SerialCommNode::open_port()
{
    port_.open(port_name_, baud_rate_, SerialPort::Mode::ReadWrite);
}

bool SerialCommNode::try_open_port()
{
    {
        std::lock_guard<std::mutex> lock(port_mutex_);
        if (port_.is_open()) {
            return true;
        }
    }

    const auto now_tp = std::chrono::steady_clock::now();
    if (last_reconnect_attempt_ != std::chrono::steady_clock::time_point::min()) {
        const auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
            now_tp - last_reconnect_attempt_).count();
        if (elapsed_ms < reconnect_interval_ms_) {
            return false;
        }
    }
    last_reconnect_attempt_ = now_tp;

    std::lock_guard<std::mutex> lock(port_mutex_);
    if (port_.is_open()) {
        return true;
    }

    try {
        open_port();
        RCLCPP_INFO(get_logger(), "Serial port reopened: %s", port_name_.c_str());
        return true;
    } catch (const std::exception& e) {
        RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 2000,
            "Serial reopen failed for %s: %s", port_name_.c_str(), e.what());
        close_port_locked();
        return false;
    }
}

void SerialCommNode::close_port_locked()
{
    port_.close();
}

void SerialCommNode::reset_rx_parser_state()
{
    std::lock_guard<std::mutex> lock(rx_mutex_);
    rx_frame_pos_ = 0;
    have_rx_frame_time_ = false;
}

void SerialCommNode::handle_port_fault(const std::string& message)
{
    {
        std::lock_guard<std::mutex> lock(port_mutex_);
        close_port_locked();
    }
    reset_rx_parser_state();
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000, "%s", message.c_str());
}

void SerialCommNode::check_rx_timeout()
{
    bool timed_out = false;
    {
        std::lock_guard<std::mutex> lock(rx_mutex_);
        if (!have_rx_frame_time_) {
            return;
        }

        const auto now_tp = std::chrono::steady_clock::now();
        const auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
            now_tp - last_rx_frame_time_).count();
        timed_out = elapsed_ms > rx_timeout_ms_;
    }

    if (timed_out) {
        handle_port_fault(
            "Serial RX timeout: no valid motor-state frame received within timeout window");
    }
}

void SerialCommNode::log_frame_hex(const char* prefix, const uint8_t* data, std::size_t len)
{
    if (!debug_serial_io_ || !debug_serial_hex_bytes_) {
        return;
    }

    std::ostringstream oss;
    oss << prefix << " (" << len << " B):";
    oss << std::hex << std::setfill('0');
    for (std::size_t i = 0; i < len; ++i) {
        oss << ' ' << std::setw(2) << static_cast<unsigned>(data[i]);
    }
    RCLCPP_INFO(get_logger(), "%s", oss.str().c_str());
}

void SerialCommNode::capture_debug_rx_frame(const uint8_t* data)
{
    if (!debug_serial_io_ || !debug_serial_hex_bytes_) {
        return;
    }

    std::lock_guard<std::mutex> lock(debug_log_mutex_);
    std::memcpy(debug_rx_frame_.data(), data, debug_rx_frame_.size());
    have_debug_rx_frame_ = true;
}

void SerialCommNode::debug_timer_callback()
{
    if (!debug_serial_io_) {
        return;
    }

    const auto now_tp = std::chrono::steady_clock::now();
    const double elapsed_s =
        last_debug_log_time_ == std::chrono::steady_clock::time_point::min()
            ? 1.0
            : std::max(1e-6, std::chrono::duration<double>(now_tp - last_debug_log_time_).count());
    last_debug_log_time_ = now_tp;

    const uint64_t tx_frames = tx_frame_count_.load();
    const uint64_t rx_bytes = rx_byte_count_.load();
    const uint64_t rx_reads = rx_read_count_.load();
    const uint64_t rx_frames = rx_frame_count_.load();
    const uint64_t rx_published = rx_publish_count_.load();
    const uint64_t rx_drops = rx_drop_count_.load();
    const uint64_t resync_bytes = rx_resync_discarded_bytes_.load();

    RCLCPP_INFO(
        get_logger(),
        "serial stats: tx_frames=%llu rx_bytes=%llu(%.0f B/s) rx_reads=%llu(%.0f/s) rx_frames=%llu(%.1f Hz) rx_published=%llu(%.1f Hz) rx_drops=%llu(+%llu) resync_bytes=%llu(+%llu) bad_version=%llu bad_type=%llu bad_length=%llu bad_crc=%llu bad_tail=%llu ctrl_mode=%u port_open=%s",
        static_cast<unsigned long long>(tx_frames),
        static_cast<unsigned long long>(rx_bytes),
        static_cast<double>(rx_bytes - last_debug_rx_byte_count_) / elapsed_s,
        static_cast<unsigned long long>(rx_reads),
        static_cast<double>(rx_reads - last_debug_rx_read_count_) / elapsed_s,
        static_cast<unsigned long long>(rx_frames),
        static_cast<double>(rx_frames - last_debug_rx_frame_count_) / elapsed_s,
        static_cast<unsigned long long>(rx_published),
        static_cast<double>(rx_published - last_debug_rx_publish_count_) / elapsed_s,
        static_cast<unsigned long long>(rx_drops),
        static_cast<unsigned long long>(rx_drops - last_debug_rx_drop_count_),
        static_cast<unsigned long long>(resync_bytes),
        static_cast<unsigned long long>(resync_bytes - last_debug_rx_resync_discarded_bytes_),
        static_cast<unsigned long long>(rx_bad_version_count_.load()),
        static_cast<unsigned long long>(rx_bad_type_count_.load()),
        static_cast<unsigned long long>(rx_bad_length_count_.load()),
        static_cast<unsigned long long>(rx_bad_crc_count_.load()),
        static_cast<unsigned long long>(rx_bad_tail_count_.load()),
        static_cast<unsigned>(cmd_ctrl_mode_.load()),
        port_.is_open() ? "true" : "false");

    last_debug_rx_byte_count_ = rx_bytes;
    last_debug_rx_read_count_ = rx_reads;
    last_debug_rx_frame_count_ = rx_frames;
    last_debug_rx_publish_count_ = rx_published;
    last_debug_rx_drop_count_ = rx_drops;
    last_debug_rx_resync_discarded_bytes_ = resync_bytes;

    if (debug_serial_hex_bytes_) {
        protocol::RawStateFrame frame{};
        bool have_frame = false;
        {
            std::lock_guard<std::mutex> lock(debug_log_mutex_);
            if (have_debug_rx_frame_) {
                frame = debug_rx_frame_;
                have_frame = true;
                have_debug_rx_frame_ = false;
            }
        }
        if (have_frame) {
            log_frame_hex("Serial RX raw state frame sample", frame.data(), frame.size());
        }
    }
}

bool SerialCommNode::build_state_msg(
    const protocol::StateFrame& frame,
    kvoy_msgs::msg::MotorState& msg)
{
    for (int i = 0; i < 12; ++i) {
        const float sign = motor_polarity_[static_cast<std::size_t>(i)];
        const float offset = motor_zero_offset_[static_cast<std::size_t>(i)];
        const float position =
            frame.joints.position[static_cast<std::size_t>(i)] * sign + offset;
        const float velocity = frame.joints.velocity[static_cast<std::size_t>(i)] * sign;
        const float torque = frame.joints.torque[static_cast<std::size_t>(i)] * sign;
        if (!std::isfinite(position) || !std::isfinite(velocity) || !std::isfinite(torque)) {
            RCLCPP_ERROR_THROTTLE(
                get_logger(), *get_clock(), 2000,
                "Serial RX frame dropped: motor state contains non-finite values");
            ++rx_drop_count_;
            return false;
        }
        msg.position[i] = position;
        msg.velocity[i] = velocity;
        msg.torque[i]   = torque;
    }

    return true;
}

void SerialCommNode::store_latest_state_frame(
    const protocol::StateFrame& frame,
    uint64_t frame_seq)
{
    std::lock_guard<std::mutex> lock(latest_state_mutex_);
    latest_state_frame_ = frame;
    latest_state_seq_ = frame_seq;
    have_latest_state_ = true;
}

void SerialCommNode::publish_latest_state()
{
    kvoy_msgs::msg::MotorState msg;
    protocol::StateFrame frame;
    uint64_t frame_seq = 0;
    {
        std::lock_guard<std::mutex> lock(latest_state_mutex_);
        if (!have_latest_state_ || latest_state_seq_ == last_published_state_seq_) {
            return;
        }
        frame = latest_state_frame_;
        frame_seq = latest_state_seq_;
    }

    if (!build_state_msg(frame, msg)) {
        return;
    }
    msg.header.stamp = now();
    if (debug_serial_io_) {
        RCLCPP_INFO_THROTTLE(
            get_logger(), *get_clock(), 500,
            "Serial RX publish[%llu] latest_frame=%llu: pos=[%.3f %.3f %.3f] vel=[%.3f %.3f %.3f] tor=[%.3f %.3f %.3f]",
            static_cast<unsigned long long>(rx_publish_count_.load() + 1),
            static_cast<unsigned long long>(frame_seq),
            msg.position[0], msg.position[1], msg.position[2],
            msg.velocity[0], msg.velocity[1], msg.velocity[2],
            msg.torque[0], msg.torque[1], msg.torque[2]);
    }
    state_pub_->publish(msg);
    {
        std::lock_guard<std::mutex> lock(latest_state_mutex_);
        last_published_state_seq_ = frame_seq;
    }
    ++rx_publish_count_;
}

void SerialCommNode::send_damping_command_from_latest_state(const char* reason)
{
    protocol::StateFrame frame{};
    bool have_frame = false;
    {
        std::lock_guard<std::mutex> lock(latest_state_mutex_);
        if (have_latest_state_) {
            frame = latest_state_frame_;
            have_frame = true;
        }
    }

    if (!have_frame) {
        return;
    }

    std::array<float, protocol::NUM_JOINTS> tx_position{};
    std::array<float, protocol::NUM_JOINTS> tx_velocity{};
    std::array<float, protocol::NUM_JOINTS> tx_torque{};
    for (int i = 0; i < protocol::NUM_JOINTS; ++i) {
        tx_position[static_cast<std::size_t>(i)] = frame.joints.position[static_cast<std::size_t>(i)];
        tx_velocity[static_cast<std::size_t>(i)] = 0.0f;
        tx_torque[static_cast<std::size_t>(i)] = 0.0f;
    }

    protocol::RawCmdFrame cmd_frame{};
    protocol::encode_cmd(
        cmd_frame,
        protocol::CTRL_MODE_DAMPING,
        tx_position.data(),
        tx_velocity.data(),
        tx_torque.data());

    int written = 0;
    {
        std::lock_guard<std::mutex> lock(port_mutex_);
        if (!port_.is_open()) {
            RCLCPP_WARN(
                get_logger(),
                "Damping command skipped (%s): serial port is not open",
                reason);
            return;
        }
        written = port_.write_all(cmd_frame.data(), protocol::CMD_FRAME_LEN, 10);
    }

    if (written < 0) {
        handle_port_fault(std::string("Damping command failed: ") + std::strerror(errno));
        return;
    }
    if (written != protocol::CMD_FRAME_LEN) {
        handle_port_fault(
            "Damping command write incomplete: wrote " + std::to_string(written) +
            " bytes");
        return;
    }

    ++tx_frame_count_;
    RCLCPP_INFO(
        get_logger(),
        "Sent damping command (%s) with ctrl_mode=0",
        reason);
}

void SerialCommNode::drop_rx_candidate_and_resync(std::size_t candidate_len)
{
    using namespace kvoy::protocol;

    std::size_t next_pos = 0;
    for (std::size_t i = 1; i + 1 < candidate_len; ++i) {
        if (rx_frame_buf_[i] == HEADER0 && rx_frame_buf_[i + 1] == HEADER1) {
            next_pos = candidate_len - i;
            std::memmove(rx_frame_buf_.data(), rx_frame_buf_.data() + i, next_pos);
            break;
        }
    }

    if (next_pos == 0 && candidate_len > 0 && rx_frame_buf_[candidate_len - 1] == HEADER0) {
        rx_frame_buf_[0] = HEADER0;
        next_pos = 1;
    }

    if (candidate_len > next_pos) {
        rx_resync_discarded_bytes_.fetch_add(candidate_len - next_pos);
    }
    rx_frame_pos_ = next_pos;
}

void SerialCommNode::process_rx_bytes(const uint8_t* data, std::size_t len)
{
    using namespace kvoy::protocol;

    if (len == 0) {
        return;
    }

    std::lock_guard<std::mutex> lock(rx_mutex_);
    for (std::size_t idx = 0; idx < len; ++idx) {
        const uint8_t byte = data[idx];

        if (rx_frame_pos_ == 0) {
            if (byte != HEADER0) {
                ++rx_resync_discarded_bytes_;
                continue;
            }
            rx_frame_buf_[rx_frame_pos_++] = byte;
            continue;
        }

        if (rx_frame_pos_ == 1) {
            if (byte == HEADER1) {
                rx_frame_buf_[rx_frame_pos_++] = byte;
            } else if (byte == HEADER0) {
                rx_frame_buf_[0] = HEADER0;
                ++rx_resync_discarded_bytes_;
            } else {
                rx_frame_pos_ = 0;
                rx_resync_discarded_bytes_.fetch_add(2);
            }
            continue;
        }

        rx_frame_buf_[rx_frame_pos_++] = byte;

        if (rx_frame_pos_ == 3 && rx_frame_buf_[2] != VERSION) {
            ++rx_drop_count_;
            ++rx_bad_version_count_;
            drop_rx_candidate_and_resync(rx_frame_pos_);
            continue;
        }

        if (rx_frame_pos_ == 4 && rx_frame_buf_[3] != TYPE_STATE) {
            ++rx_drop_count_;
            ++rx_bad_type_count_;
            drop_rx_candidate_and_resync(rx_frame_pos_);
            continue;
        }

        if (rx_frame_pos_ == HEADER_LEN &&
            read_u16_le(rx_frame_buf_.data() + 4) != STATE_PAYLOAD_LEN) {
            ++rx_drop_count_;
            ++rx_bad_length_count_;
            drop_rx_candidate_and_resync(rx_frame_pos_);
            continue;
        }

        if (rx_frame_pos_ < STATE_FRAME_LEN) {
            continue;
        }

        const uint16_t expected_crc = crc16(rx_frame_buf_.data(), STATE_CRC_INPUT_LEN);
        const uint16_t actual_crc = read_u16_le(rx_frame_buf_.data() + STATE_CRC_OFFSET);
        if (actual_crc != expected_crc) {
            ++rx_drop_count_;
            ++rx_bad_crc_count_;
            drop_rx_candidate_and_resync(rx_frame_pos_);
            continue;
        }

        if (rx_frame_buf_[STATE_TAIL_OFFSET] != TAIL0 ||
            rx_frame_buf_[STATE_TAIL_OFFSET + 1] != TAIL1) {
            ++rx_drop_count_;
            ++rx_bad_tail_count_;
            drop_rx_candidate_and_resync(rx_frame_pos_);
            continue;
        }

        protocol::StateFrame frame;
        if (!decode_state(rx_frame_buf_.data(), frame)) {
            ++rx_drop_count_;
            drop_rx_candidate_and_resync(rx_frame_pos_);
            continue;
        }

        const auto frame_time = std::chrono::steady_clock::now();
        last_rx_frame_time_ = frame_time;
        have_rx_frame_time_ = true;
        const uint64_t frame_seq = ++rx_frame_count_;
        store_latest_state_frame(frame, frame_seq);
        capture_debug_rx_frame(rx_frame_buf_.data());
        rx_frame_pos_ = 0;
    }
}

void SerialCommNode::rx_thread_fn()
{
    std::array<uint8_t, RX_READ_CHUNK> chunk{};

    while (running_) {
        if (!try_open_port()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            continue;
        }

        bool read_any = false;
        bool read_fault = false;
        for (std::size_t i = 0; i < RX_MAX_READS_PER_CYCLE; ++i) {
            int ret = 0;
            {
                std::lock_guard<std::mutex> lock(port_mutex_);
                if (!port_.is_open()) {
                    break;
                }
                if (!read_any) {
                    ret = port_.wait_readable(2);
                    if (ret <= 0) {
                        read_fault = ret < 0;
                        break;
                    }
                }
                ret = port_.read_nonblocking(chunk.data(), chunk.size());
            }

            if (ret > 0) {
                read_any = true;
                rx_byte_count_.fetch_add(static_cast<std::size_t>(ret), std::memory_order_relaxed);
                rx_read_count_.fetch_add(1, std::memory_order_relaxed);
                process_rx_bytes(chunk.data(), static_cast<std::size_t>(ret));
                if (ret < static_cast<int>(chunk.size())) {
                    break;
                }
                continue;
            }
            if (ret == 0) {
                break;
            }

            read_fault = true;
            break;
        }

        check_rx_timeout();
        if (read_fault) {
            handle_port_fault(std::string("Serial read failed: ") + std::strerror(errno));
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
            continue;
        }
        if (!read_any) {
            std::this_thread::sleep_for(std::chrono::microseconds(500));
        }
    }
}

void SerialCommNode::on_motor_cmd(const kvoy_msgs::msg::MotorCmd::SharedPtr msg)
{
    using namespace kvoy::protocol;
    std::array<float, NUM_JOINTS> tx_position{};
    std::array<float, NUM_JOINTS> tx_velocity{};
    std::array<float, NUM_JOINTS> tx_torque{};

    for (int i = 0; i < 12; ++i) {
        if (!std::isfinite(msg->position[i]) ||
            !std::isfinite(msg->velocity[i]) ||
            !std::isfinite(msg->torque[i])) {
            RCLCPP_ERROR_THROTTLE(
                get_logger(), *get_clock(), 2000,
                "Serial write aborted: motor command contains non-finite values");
            return;
        }

        const float sign = motor_polarity_[static_cast<std::size_t>(i)];
        const float offset = motor_zero_offset_[static_cast<std::size_t>(i)];
        tx_position[static_cast<std::size_t>(i)] = (msg->position[i] - offset) * sign;
        tx_velocity[static_cast<std::size_t>(i)] = msg->velocity[i] * sign;
        tx_torque[static_cast<std::size_t>(i)] = msg->torque[i] * sign;
    }

    RawCmdFrame frame{};
    const auto now_steady = std::chrono::steady_clock::now();
    if (!have_robot_state_ || now_steady - last_robot_state_time_ > kRobotStateTimeout) {
        const uint8_t prev_mode = cmd_ctrl_mode_.exchange(protocol::CTRL_MODE_DAMPING);
        if (prev_mode != protocol::CTRL_MODE_DAMPING) {
            RCLCPP_WARN(
                get_logger(),
                "Serial ctrl_mode switched to 0 (damping): /robot_state heartbeat timed out");
        }
    }

    const uint8_t ctrl_mode = cmd_ctrl_mode_.load();
    encode_cmd(frame, ctrl_mode,
           tx_position.data(),
           tx_velocity.data(),
           tx_torque.data());

    int written = 0;
    {
        std::lock_guard<std::mutex> lock(port_mutex_);
        if (!port_.is_open()) {
            RCLCPP_ERROR_THROTTLE(
                get_logger(), *get_clock(), 2000, "Serial write skipped: port is not open");
            return;
        }
        written = port_.write_all(frame.data(), CMD_FRAME_LEN, 10);
    }

    if (written < 0) {
        handle_port_fault(std::string("Serial write failed: ") + std::strerror(errno));
        return;
    }
    if (written != CMD_FRAME_LEN) {
        std::ostringstream oss;
        oss << "Serial partial write: expected "
            << static_cast<unsigned>(CMD_FRAME_LEN)
            << " bytes, wrote " << written << " bytes";
        handle_port_fault(oss.str());
        return;
    }

    ++tx_frame_count_;
    if (debug_serial_io_) {
        RCLCPP_INFO_THROTTLE(
            get_logger(), *get_clock(), 500,
            "Serial TX cmd[%llu]: ctrl_mode=%u pos=[%.3f %.3f %.3f] vel=[%.3f %.3f %.3f] tor=[%.3f %.3f %.3f]",
            static_cast<unsigned long long>(tx_frame_count_.load()),
            static_cast<unsigned>(ctrl_mode),
            tx_position[0], tx_position[1], tx_position[2],
            tx_velocity[0], tx_velocity[1], tx_velocity[2],
            tx_torque[0], tx_torque[1], tx_torque[2]);
        log_frame_hex("Serial TX raw cmd frame", frame.data(), frame.size());
    }
}

void SerialCommNode::on_robot_state(const kvoy_msgs::msg::RobotState::SharedPtr msg)
{
    last_robot_state_time_ = std::chrono::steady_clock::now();
    have_robot_state_ = true;

    uint8_t next_mode = protocol::CTRL_MODE_MOTION;
    if (msg->fsm_state == kvoy_msgs::msg::RobotState::FSM_WAITING ||
        msg->fsm_state == kvoy_msgs::msg::RobotState::FSM_ESTOP) {
        next_mode = protocol::CTRL_MODE_DAMPING;
    }

    const uint8_t prev_mode = cmd_ctrl_mode_.exchange(next_mode);
    const bool had_last_state = have_last_robot_fsm_state_;
    const uint8_t prev_fsm_state = last_robot_fsm_state_;
    have_last_robot_fsm_state_ = true;
    last_robot_fsm_state_ = msg->fsm_state;

    if (prev_mode != next_mode) {
        RCLCPP_INFO(
            get_logger(),
            "Serial ctrl_mode switched to %u (%s) from fsm_state=%u",
            static_cast<unsigned>(next_mode),
            next_mode == protocol::CTRL_MODE_MOTION ? "motion" : "damping",
            static_cast<unsigned>(msg->fsm_state));
    }

    if (had_last_state &&
        prev_fsm_state == kvoy_msgs::msg::RobotState::FSM_LIEDOWN &&
        msg->fsm_state == kvoy_msgs::msg::RobotState::FSM_WAITING) {
        send_damping_command_from_latest_state("liedown complete");
    }
}

} // namespace kvoy

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<kvoy::SerialCommNode>());
    rclcpp::shutdown();
    return 0;
}
