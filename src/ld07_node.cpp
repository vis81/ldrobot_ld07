#include "ldrobot_ld07/ld07_node.hpp"

#include <cmath>
#include <limits>
#include <stdexcept>
#include <thread>

namespace ld07 {

Ld07Node::Ld07Node(const rclcpp::NodeOptions& options)
: rclcpp::Node("ldrobot_ld07_node", options)
{
    declare_parameter("comm.serial_port",   "/dev/ttyUSB0");
    declare_parameter("comm.baudrate",      921600);
    declare_parameter("comm.timeout_msec",  1000);
    declare_parameter("lidar.frame_id",     "laser_link");
    declare_parameter("lidar.topic",        "/scan");
    declare_parameter("lidar.range_min",    0.005);
    declare_parameter("lidar.range_max",    0.400);
    declare_parameter("lidar.angle_min",    -M_PI_4);
    declare_parameter("lidar.angle_max",     M_PI_4);
    declare_parameter("lidar.confidence_min", 0);

    const auto port_path = get_parameter("comm.serial_port").as_string();
    const auto baud      = static_cast<uint32_t>(get_parameter("comm.baudrate").as_int());
    timeout_ms_      = static_cast<int>(get_parameter("comm.timeout_msec").as_int());
    frame_id_        = get_parameter("lidar.frame_id").as_string();
    topic_           = get_parameter("lidar.topic").as_string();
    range_min_       = static_cast<float>(get_parameter("lidar.range_min").as_double());
    range_max_       = static_cast<float>(get_parameter("lidar.range_max").as_double());
    angle_min_       = static_cast<float>(get_parameter("lidar.angle_min").as_double());
    angle_max_       = static_cast<float>(get_parameter("lidar.angle_max").as_double());
    confidence_min_  = static_cast<uint8_t>(get_parameter("lidar.confidence_min").as_int());

    pub_ = create_publisher<sensor_msgs::msg::LaserScan>(topic_, rclcpp::SensorDataQoS());

    if (!port_.open(port_path, baud)) {
        RCLCPP_FATAL(get_logger(), "Cannot open serial port '%s': %s",
                     port_path.c_str(), strerror(errno));
        throw std::runtime_error("Cannot open serial port: " + port_path);
    }
    RCLCPP_INFO(get_logger(), "Opened %s at %u baud", port_path.c_str(), baud);

    driver_.sendCommand(port_, CMD_DIST_START);
    std::this_thread::sleep_for(std::chrono::milliseconds(200));  // let sensor start streaming

    running_ = true;
    reader_thread_ = std::thread(&Ld07Node::readerThread, this);
}

Ld07Node::~Ld07Node() {
    running_ = false;
    driver_.sendCommand(port_, CMD_DIST_STOP);
    port_.close();   // unblocks the blocking read() in the reader thread
    if (reader_thread_.joinable()) reader_thread_.join();
}

void Ld07Node::readerThread() {
    Frame frame;
    int consecutive_failures = 0;

    while (running_) {
        if (!driver_.readFrame(port_, frame, timeout_ms_)) {
            ++consecutive_failures;
            if (consecutive_failures == 5) {
                RCLCPP_WARN(get_logger(),
                    "5 consecutive read failures — check sensor connection");
            }
            continue;
        }
        consecutive_failures = 0;
        pub_->publish(makeScan(frame));
    }
}

sensor_msgs::msg::LaserScan Ld07Node::makeScan(const Frame& frame) {
    sensor_msgs::msg::LaserScan msg;
    msg.header.stamp    = now();
    msg.header.frame_id = frame_id_;

    msg.angle_min       = angle_min_;
    msg.angle_max       = angle_max_;
    msg.angle_increment = (angle_max_ - angle_min_) / static_cast<float>(NUM_POINTS - 1);
    msg.range_min       = range_min_;
    msg.range_max       = range_max_;

    const rclcpp::Time now_t = msg.header.stamp;
    if (first_frame_) {
        msg.scan_time  = 0.1f;  // placeholder for the first frame
        first_frame_   = false;
    } else {
        msg.scan_time = static_cast<float>((now_t - last_frame_time_).seconds());
    }
    last_frame_time_    = now_t;
    msg.time_increment  = msg.scan_time / static_cast<float>(NUM_POINTS - 1);

    msg.ranges.resize(NUM_POINTS);
    msg.intensities.resize(NUM_POINTS);

    for (size_t i = 0; i < NUM_POINTS; ++i) {
        const auto& pt = frame.points[i];
        const bool valid = !std::isnan(pt.range_m)
                        && pt.range_m >= range_min_
                        && pt.range_m <= range_max_
                        && pt.confidence >= confidence_min_;
        msg.ranges[i]      = valid ? pt.range_m
                                   : std::numeric_limits<float>::quiet_NaN();
        msg.intensities[i] = static_cast<float>(pt.confidence);
    }

    return msg;
}

}  // namespace ld07
