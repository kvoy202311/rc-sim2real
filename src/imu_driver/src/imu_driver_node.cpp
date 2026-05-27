#include "imu_driver/imu_driver_node.hpp"

#include <algorithm>

namespace kvoy {

ImuDriverNode::ImuDriverNode(const std::string& node_name,
                             const rclcpp::NodeOptions& options)
: Node(node_name, options)
{
    declare_parameter("serial_port",      "/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0");
    declare_parameter("baud_rate",        115200);  // HWT906 default
    declare_parameter("frame_id",         "imu_link");
    declare_parameter("publish_rate_hz",  200.0);
    declare_parameter("reconnect_interval_ms", 500);

    port_name_       = get_parameter("serial_port").as_string();
    baud_rate_       = get_parameter("baud_rate").as_int();
    frame_id_        = get_parameter("frame_id").as_string();
    publish_rate_hz_ = std::max(1.0, get_parameter("publish_rate_hz").as_double());
    reconnect_interval_ms_ = std::max(
        10,
        static_cast<int>(get_parameter("reconnect_interval_ms").as_int()));

    imu_pub_ = create_publisher<kvoy_msgs::msg::ImuData>("/imu/data", 10);

    auto period = std::chrono::duration<double>(1.0 / publish_rate_hz_);
    timer_ = create_wall_timer(period,
                               std::bind(&ImuDriverNode::timer_callback, this));
}

void ImuDriverNode::start_driver()
{
    try {
        open();
        driver_opened_ = true;
        RCLCPP_INFO(get_logger(), "%s started on %s @ %d baud, %.0f Hz",
                    get_name(), port_name_.c_str(),
                    baud_rate_, publish_rate_hz_);
    } catch (const std::exception& e) {
        RCLCPP_WARN(
            get_logger(),
            "%s started without an open IMU port (%s); will retry in background",
            get_name(), e.what());
        close();
    }
}

void ImuDriverNode::timer_callback()
{
    if (!driver_opened_) {
        try {
            if (last_reconnect_attempt_ == std::chrono::steady_clock::time_point{} ||
                std::chrono::duration_cast<std::chrono::milliseconds>(
                    std::chrono::steady_clock::now() - last_reconnect_attempt_).count() >=
                    reconnect_interval_ms_) {
                last_reconnect_attempt_ = std::chrono::steady_clock::now();
                open();
                driver_opened_ = true;
                RCLCPP_INFO_THROTTLE(
                    get_logger(), *get_clock(), 2000,
                    "%s reconnected on %s @ %d baud",
                    get_name(), port_name_.c_str(), baud_rate_);
            }
        } catch (...) {
            // Port is still unavailable; keep retrying.
        }
    }

    kvoy_msgs::msg::ImuData msg;
    msg.header.stamp = now();
    msg.header.frame_id = frame_id_;

    try {
        if (read_frame(msg)) {
            imu_pub_->publish(msg);
        }
    } catch (const std::exception& e) {
        RCLCPP_ERROR_THROTTLE(
            get_logger(), *get_clock(), 2000,
            "IMU read failed: %s", e.what());
        close();
        driver_opened_ = false;
    }
}

} // namespace kvoy
