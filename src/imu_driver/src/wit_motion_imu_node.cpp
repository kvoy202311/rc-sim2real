#include "imu_driver/wit_motion_imu_node.hpp"

#include <cmath>
#include <cerrno>
#include <cstring>
#include <stdexcept>

namespace kvoy {

namespace {

constexpr float G = 9.80665f;
constexpr float DEG2RAD = static_cast<float>(M_PI) / 180.0f;

int16_t le_i16(uint8_t lo, uint8_t hi)
{
    return static_cast<int16_t>((static_cast<uint16_t>(hi) << 8) | lo);
}

bool checksum_ok(const uint8_t* pkt)
{
    uint8_t sum = 0;
    for (int i = 0; i < 10; ++i) {
        sum += pkt[i];
    }
    return sum == pkt[10];
}

} // namespace

WitMotionImuNode::WitMotionImuNode(const rclcpp::NodeOptions& options)
: ImuDriverNode("imu_driver_node", options)
{
    start_driver();
}

WitMotionImuNode::~WitMotionImuNode()
{
    close();
}

void WitMotionImuNode::reset_state()
{
    ax_ = ay_ = az_ = 0.0f;
    gx_ = gy_ = gz_ = 0.0f;
    quat_x_ = quat_y_ = quat_z_ = 0.0f;
    quat_w_ = 1.0f;
    have_accel_ = false;
    have_gyro_ = false;
    have_quat_ = false;
    pending_accel_ = false;
    pending_gyro_ = false;
    pending_quat_ = false;
    rx_pos_ = 0;
}

void WitMotionImuNode::open()
{
    if (port_.is_open()) {
        return;
    }

    reset_state();
    port_.open(port_name_, baud_rate_, SerialPort::Mode::ReadOnly);
}

void WitMotionImuNode::close()
{
    port_.close();
    reset_state();
}

bool WitMotionImuNode::read_frame(kvoy_msgs::msg::ImuData& msg)
{
    if (!port_.is_open()) {
        return false;
    }

    bool received_bytes = false;
    for (;;) {
        uint8_t tmp[64];
        const int n = port_.read_nonblocking(tmp, sizeof(tmp));
        if (n < 0) {
            throw std::runtime_error(
                std::string("nonblocking read from HWT906 failed: ") + std::strerror(errno));
        }
        if (n == 0) {
            break;
        }

        received_bytes = true;
        for (int j = 0; j < n; ++j) {
            if (rx_pos_ >= static_cast<int>(rx_buf_.size())) {
                std::memmove(rx_buf_.data(), rx_buf_.data() + 1, rx_buf_.size() - 1);
                rx_pos_ = static_cast<int>(rx_buf_.size()) - 1;
            }
            rx_buf_[rx_pos_++] = tmp[j];
        }

        if (n < static_cast<int>(sizeof(tmp))) {
            break;
        }
    }

    if (!received_bytes) {
        return false;
    }

    int i = 0;
    while (i + PKT_LEN <= rx_pos_) {
        if (rx_buf_[i] != 0x55) {
            ++i;
            continue;
        }

        if (!checksum_ok(&rx_buf_[i])) {
            ++i;
            continue;
        }

        parse_packet(&rx_buf_[i]);
        i += PKT_LEN;
    }

    if (i > 0) {
        std::memmove(rx_buf_.data(), rx_buf_.data() + i, rx_pos_ - i);
        rx_pos_ -= i;
    }

    if (!have_accel_ || !have_gyro_ || !have_quat_) {
        return false;
    }

    if (!pending_accel_ || !pending_gyro_ || !pending_quat_) {
        return false;
    }

    if (!normalize_quaternion()) {
        return false;
    }

    msg.orientation.x = quat_x_;
    msg.orientation.y = quat_y_;
    msg.orientation.z = quat_z_;
    msg.orientation.w = quat_w_;
    msg.angular_velocity.x = gx_;
    msg.angular_velocity.y = gy_;
    msg.angular_velocity.z = gz_;
    msg.linear_acceleration.x = ax_;
    msg.linear_acceleration.y = ay_;
    msg.linear_acceleration.z = az_;

    pending_accel_ = false;
    pending_gyro_ = false;
    pending_quat_ = false;

    return true;
}

bool WitMotionImuNode::parse_packet(const uint8_t* pkt)
{
    switch (pkt[1]) {
        case 0x51:
            ax_ = le_i16(pkt[2], pkt[3]) / 32768.0f * 16.0f * G;
            ay_ = le_i16(pkt[4], pkt[5]) / 32768.0f * 16.0f * G;
            az_ = le_i16(pkt[6], pkt[7]) / 32768.0f * 16.0f * G;
            have_accel_ = true;
            pending_accel_ = true;
            return true;
        case 0x52:
            gx_ = le_i16(pkt[2], pkt[3]) / 32768.0f * 2000.0f * DEG2RAD;
            gy_ = le_i16(pkt[4], pkt[5]) / 32768.0f * 2000.0f * DEG2RAD;
            gz_ = le_i16(pkt[6], pkt[7]) / 32768.0f * 2000.0f * DEG2RAD;
            have_gyro_ = true;
            pending_gyro_ = true;
            return true;
        case 0x59:
            // WIT quaternion packet order is q0,q1,q2,q3 = w,x,y,z.
            quat_w_ = le_i16(pkt[2], pkt[3]) / 32768.0f;
            quat_x_ = le_i16(pkt[4], pkt[5]) / 32768.0f;
            quat_y_ = le_i16(pkt[6], pkt[7]) / 32768.0f;
            quat_z_ = le_i16(pkt[8], pkt[9]) / 32768.0f;
            have_quat_ = true;
            pending_quat_ = true;
            return true;
        default:
            return false;
    }
}

bool WitMotionImuNode::normalize_quaternion()
{
    const float norm = std::sqrt(
        quat_x_ * quat_x_ +
        quat_y_ * quat_y_ +
        quat_z_ * quat_z_ +
        quat_w_ * quat_w_);

    if (norm <= 1.0e-6f) {
        return false;
    }

    quat_x_ /= norm;
    quat_y_ /= norm;
    quat_z_ /= norm;
    quat_w_ /= norm;
    return true;
}

} // namespace kvoy

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<kvoy::WitMotionImuNode>());
    rclcpp::shutdown();
    return 0;
}
