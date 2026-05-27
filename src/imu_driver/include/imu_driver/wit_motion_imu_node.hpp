#pragma once
#include "imu_driver/imu_driver_node.hpp"
#include "imu_driver/serial_port.hpp"
#include <array>

namespace kvoy {

// Wit-Motion HWT906/WIT standard serial driver.
// Protocol: 0x55 + type(1B) + data(8B) + checksum(1B), 11 bytes per packet.
// Packet types used here:
//   0x51 = accel
//   0x52 = gyro
//   0x59 = quaternion
class WitMotionImuNode : public ImuDriverNode
{
public:
    explicit WitMotionImuNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    ~WitMotionImuNode() override;

protected:
    void open()  override;
    void close() override;
    bool read_frame(kvoy_msgs::msg::ImuData& msg) override;

private:
    bool parse_packet(const uint8_t* pkt);
    void reset_state();
    bool normalize_quaternion();

    SerialPort port_;

    // Latest parsed sensor values in the IMU sensor frame.
    float ax_{0}, ay_{0}, az_{0};   // m/s^2
    float gx_{0}, gy_{0}, gz_{0};   // rad/s
    float quat_x_{0}, quat_y_{0}, quat_z_{0}, quat_w_{1};
    bool have_accel_{false};
    bool have_gyro_{false};
    bool have_quat_{false};
    bool pending_accel_{false};
    bool pending_gyro_{false};
    bool pending_quat_{false};

    static constexpr int PKT_LEN = 11;
    std::array<uint8_t, 256> rx_buf_{};
    int rx_pos_{0};
};

} // namespace kvoy
