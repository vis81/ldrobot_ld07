#pragma once
#include <atomic>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

#include "ldrobot_ld07/serial_port.hpp"
#include "ldrobot_ld07/ld07_driver.hpp"

namespace ld07 {

class Ld07Node : public rclcpp::Node {
public:
    explicit Ld07Node(const rclcpp::NodeOptions& options = rclcpp::NodeOptions{});
    ~Ld07Node() override;

    // Raw fd for emergency stop in signal/terminate handlers.
    int serialFd() const { return port_.fd(); }

private:
    // Block until the serial port opens or rclcpp shuts down. Returns false on shutdown.
    bool waitForPort(const std::string& path, uint32_t baud);
    // Close port, wait for device to reappear, re-init sensor, restart streaming.
    void reconnect();

    void readerThread();
    sensor_msgs::msg::LaserScan makeScan(const Frame& frame);

    SerialPort port_;
    Driver     driver_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr pub_;

    std::thread       reader_thread_;
    std::atomic<bool> running_{false};

    // cached parameters
    std::string port_path_;
    uint32_t    baud_;
    std::string frame_id_;
    std::string topic_;
    float       range_min_, range_max_;
    float       angle_min_, angle_max_;
    uint8_t     confidence_min_;
    int         timeout_ms_;

    rclcpp::Time last_frame_time_;
    bool         first_frame_{true};
};

}  // namespace ld07
