#include "ldrobot_ld07/ld07_node.hpp"

#include <cmath>
#include <limits>
#include <poll.h>
#include <stdexcept>
#include <sys/inotify.h>
#include <thread>
#include <unistd.h>

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

    port_path_ = get_parameter("comm.serial_port").as_string();
    baud_      = static_cast<uint32_t>(get_parameter("comm.baudrate").as_int());
    const auto& port_path = port_path_;
    const auto  baud      = baud_;
    timeout_ms_      = static_cast<int>(get_parameter("comm.timeout_msec").as_int());
    frame_id_        = get_parameter("lidar.frame_id").as_string();
    topic_           = get_parameter("lidar.topic").as_string();
    range_min_       = static_cast<float>(get_parameter("lidar.range_min").as_double());
    range_max_       = static_cast<float>(get_parameter("lidar.range_max").as_double());
    angle_min_       = static_cast<float>(get_parameter("lidar.angle_min").as_double());
    angle_max_       = static_cast<float>(get_parameter("lidar.angle_max").as_double());
    confidence_min_  = static_cast<uint8_t>(get_parameter("lidar.confidence_min").as_int());

    pub_ = create_publisher<sensor_msgs::msg::LaserScan>(topic_, rclcpp::QoS(10));

    if (!waitForPort(port_path, baud)) return;  // rclcpp shutdown while waiting
    RCLCPP_INFO(get_logger(), "Opened %s at %u baud", port_path.c_str(), baud);

    // Init sequence: CONFIG_ADDR → GET_CALIB → VIDEO_SIZE (required before streaming)
    if (driver_.initSensor(port_, timeout_ms_)) {
        const auto& c = driver_.calib();
        RCLCPP_INFO(get_logger(),
            "Calibration received: k0=%.4f k1=%.4f b0=%.4f b1=%.4f",
            c.k0, c.k1, c.b0, c.b1);
    } else {
        RCLCPP_WARN(get_logger(),
            "Calibration not received — falling back to linear angle mapping");
    }

    driver_.sendCommand(port_, CMD_DIST_START);

    running_ = true;
    reader_thread_ = std::thread(&Ld07Node::readerThread, this);
}

Ld07Node::~Ld07Node() {
    running_ = false;
    driver_.sendCommand(port_, CMD_DIST_STOP);
    port_.close();   // unblocks the blocking read() in the reader thread
    if (reader_thread_.joinable()) reader_thread_.join();
}

bool Ld07Node::waitForPort(const std::string& path, uint32_t baud) {
    if (port_.open(path, baud)) return true;

    RCLCPP_WARN(get_logger(), "Cannot open '%s': %s — waiting for device",
                path.c_str(), strerror(errno));

    // Watch the containing directory for device node creation.
    // Set up before the retry loop to close the race where the device appears
    // between the initial open() failure and the watch setup.
    const auto slash = path.rfind('/');
    const std::string dir = (slash != std::string::npos) ? path.substr(0, slash) : ".";
    const std::string dev = (slash != std::string::npos) ? path.substr(slash + 1) : path;

    int ifd = inotify_init1(IN_CLOEXEC | IN_NONBLOCK);
    if (ifd >= 0) inotify_add_watch(ifd, dir.c_str(), IN_CREATE);

    while (rclcpp::ok() && !port_.open(path, baud)) {
        if (ifd < 0) {
            std::this_thread::sleep_for(std::chrono::seconds(1));
            continue;
        }
        // inotify is an early wakeup; port_.open() is retried on every iteration
        // regardless (handles existing-but-not-yet-ready devices on startup).
        struct pollfd pfd{ifd, POLLIN, 0};
        poll(&pfd, 1, 1000);  // up to 1 s; woken early by device creation event

        if (!(pfd.revents & POLLIN)) continue;

        alignas(inotify_event) char buf[4096];
        ssize_t n = ::read(ifd, buf, sizeof(buf));
        if (n <= 0) continue;

        bool matched = false;
        for (const char* p = buf; p < buf + n; ) {
            const auto* ev = reinterpret_cast<const inotify_event*>(p);
            if ((ev->mask & IN_CREATE) && ev->len > 0 && dev == ev->name) {
                matched = true;
                break;
            }
            p += sizeof(inotify_event) + ev->len;
        }
        if (!matched) continue;  // unrelated device — keep waiting, don't skip the 1 s

        // Give udev a moment to finish configuring the new device node
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        // Outer while condition retries port_.open(); if udev isn't done yet,
        // the next 1 s timeout will catch it.
    }

    if (ifd >= 0) ::close(ifd);
    return port_.isOpen();
}

void Ld07Node::reconnect() {
    driver_.sendCommand(port_, CMD_DIST_STOP);  // best-effort — port may already be dead
    port_.close();

    if (!waitForPort(port_path_, baud_)) return;  // rclcpp shutdown
    RCLCPP_INFO(get_logger(), "Reconnected to %s", port_path_.c_str());

    if (driver_.initSensor(port_, timeout_ms_)) {
        const auto& c = driver_.calib();
        RCLCPP_INFO(get_logger(),
            "Calibration received: k0=%.4f k1=%.4f b0=%.4f b1=%.4f",
            c.k0, c.k1, c.b0, c.b1);
    } else {
        RCLCPP_WARN(get_logger(),
            "Calibration not received — falling back to linear angle mapping");
    }
    driver_.sendCommand(port_, CMD_DIST_START);
}

void Ld07Node::readerThread() {
    Frame frame;
    int consecutive_failures = 0;

    while (running_) {
        if (!driver_.readFrame(port_, frame, timeout_ms_)) {
            if (++consecutive_failures == 5) {
                RCLCPP_WARN(get_logger(),
                    "5 consecutive read failures — device disconnected, reconnecting");
                reconnect();
                consecutive_failures = 0;
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
        msg.scan_time = 0.1f;
        first_frame_  = false;
    } else {
        msg.scan_time = static_cast<float>((now_t - last_frame_time_).seconds());
    }
    last_frame_time_   = now_t;
    msg.time_increment = msg.scan_time / static_cast<float>(NUM_POINTS - 1);

    msg.ranges.assign(NUM_POINTS, std::numeric_limits<float>::quiet_NaN());
    msg.intensities.assign(NUM_POINTS, 0.f);

    if (driver_.calib().valid) {
        // Perspective-correct: transform each raw measurement to (x,y), scatter into angle bins
        for (size_t i = 0; i < NUM_POINTS; ++i) {
            const auto& pt = frame.points[i];
            if (pt.dist_mm == 0 || pt.confidence < confidence_min_) continue;

            float x_mm, y_mm;
            if (!driver_.transformPoint(pt.dist_mm, static_cast<int>(i), x_mm, y_mm)) continue;

            const float range = std::hypot(x_mm, y_mm) / 1000.0f;
            if (range < range_min_ || range > range_max_) continue;

            const float angle = std::atan2(y_mm, x_mm);
            if (angle < angle_min_ || angle > angle_max_) continue;

            const int bin = static_cast<int>(
                (angle - angle_min_) / msg.angle_increment + 0.5f);
            if (bin < 0 || bin >= static_cast<int>(NUM_POINTS)) continue;

            // Keep nearest point when multiple inputs land in the same bin
            if (std::isnan(msg.ranges[bin]) || range < msg.ranges[bin]) {
                msg.ranges[bin]      = range;
                msg.intensities[bin] = static_cast<float>(pt.confidence);
            }
        }
    } else {
        // Linear fallback: direct index-to-angle mapping (no calibration)
        for (size_t i = 0; i < NUM_POINTS; ++i) {
            const auto& pt = frame.points[i];
            const float range = static_cast<float>(pt.dist_mm) / 1000.0f;
            const bool valid  = pt.dist_mm > 0
                             && range >= range_min_
                             && range <= range_max_
                             && pt.confidence >= confidence_min_;
            msg.ranges[i]      = valid ? range : std::numeric_limits<float>::quiet_NaN();
            msg.intensities[i] = static_cast<float>(pt.confidence);
        }
    }

    return msg;
}

}  // namespace ld07
