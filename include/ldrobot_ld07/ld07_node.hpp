#pragma once
#include <atomic>
#include <chrono>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

#include "ldrobot_ld07/serial_port.hpp"
#include "rtrnet.hpp"

namespace ld07 {

class Ld07Node : public rclcpp::Node {
public:
    explicit Ld07Node(const rclcpp::NodeOptions& options = rclcpp::NodeOptions{});
    ~Ld07Node() override;

    int serialFd() const { return port_.fd(); }

private:
    bool waitForPort(const std::string& path, uint32_t baud);
    bool initSensor(int timeout_ms);
    void reconnect();
    void readerThread();

    SerialPort port_;
    RTRNet     lidar_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr pub_;

    std::thread       reader_thread_;
    std::atomic<bool> running_{false};

    std::string port_path_;
    uint32_t    baud_;
    int         timeout_ms_;
    int64_t     stamp_offset_ns_;
};

}  // namespace ld07
