#pragma once
#include <rclcpp/rclcpp.hpp>
#include <kvoy_msgs/msg/imu_data.hpp>
#include <chrono>

namespace kvoy {

// Abstract base — subclass this for each Wit-Motion model.
// Override open(), close(), read_frame(), and fill the ImuData message.
class ImuDriverNode : public rclcpp::Node
{
public:
    explicit ImuDriverNode(const std::string& node_name,
                           const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    virtual ~ImuDriverNode() = default;

protected:
    void start_driver();
    virtual void open()  = 0;
    virtual void close() = 0;
    // Called at publish_rate_hz_; fill msg and return true if data is valid.
    virtual bool read_frame(kvoy_msgs::msg::ImuData& msg) = 0;

    rclcpp::Publisher<kvoy_msgs::msg::ImuData>::SharedPtr imu_pub_;
    std::string port_name_;
    int         baud_rate_;
    std::string frame_id_;
    int         reconnect_interval_ms_;
    std::chrono::steady_clock::time_point last_reconnect_attempt_{};
    double      publish_rate_hz_;
    bool        driver_opened_{false};

private:
    void timer_callback();
    rclcpp::TimerBase::SharedPtr timer_;
};

} // namespace kvoy
