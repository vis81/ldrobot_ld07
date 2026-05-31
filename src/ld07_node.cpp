#include "ldrobot_ld07/ld07_node.hpp"

#include <algorithm>
#include <poll.h>
#include <stdexcept>
#include <sys/inotify.h>
#include <thread>
#include <unistd.h>

namespace ld07 {

Ld07Node::Ld07Node(const rclcpp::NodeOptions& options)
: rclcpp::Node("ldrobot_ld07_node", options)
{
    declare_parameter("comm.serial_port",      "/dev/ttyUSB0");
    declare_parameter("comm.baudrate",         921600);
    declare_parameter("comm.timeout_msec",     1000);
    declare_parameter("lidar.frame_id",        "laser_link");
    declare_parameter("lidar.topic",           "/scan");
    declare_parameter("lidar.range_min",       0.005);
    declare_parameter("lidar.range_max",       0.400);
    declare_parameter("lidar.angle_min",      -0.7330);
    declare_parameter("lidar.angle_max",       0.7854);
    declare_parameter("lidar.confidence_min",   0);
    declare_parameter("lidar.stamp_offset_sec", 0.05);
    declare_parameter("lidar.neighbor_window",  2);
    declare_parameter("lidar.min_neighbors",    2);

    port_path_  = get_parameter("comm.serial_port").as_string();
    baud_       = static_cast<uint32_t>(get_parameter("comm.baudrate").as_int());
    timeout_ms_ = static_cast<int>(get_parameter("comm.timeout_msec").as_int());
    stamp_offset_ns_ = static_cast<int64_t>(
        get_parameter("lidar.stamp_offset_sec").as_double() * 1e9);

    lidar_.Configure(
        get_parameter("lidar.frame_id").as_string(),
        static_cast<float>(get_parameter("lidar.angle_min").as_double()),
        static_cast<float>(get_parameter("lidar.angle_max").as_double()),
        static_cast<float>(get_parameter("lidar.range_min").as_double()),
        static_cast<float>(get_parameter("lidar.range_max").as_double()),
        static_cast<uint8_t>(get_parameter("lidar.confidence_min").as_int())
    );
    lidar_.SetNeighborFilter(
        static_cast<int>(get_parameter("lidar.neighbor_window").as_int()),
        static_cast<int>(get_parameter("lidar.min_neighbors").as_int())
    );

    pub_ = create_publisher<sensor_msgs::msg::LaserScan>(
        get_parameter("lidar.topic").as_string(), rclcpp::QoS(10).reliable());

    if (!waitForPort(port_path_, baud_)) return;
    RCLCPP_INFO(get_logger(), "Opened %s at %u baud", port_path_.c_str(), baud_);

    if (initSensor(3000)) {
        RCLCPP_INFO(get_logger(), "Sensor initialised (parameters ready)");
    } else {
        RCLCPP_WARN(get_logger(), "Sensor init failed — proceeding anyway");
    }

    lidar_.SendCmd(port_, THIS_DEVICE_ADDRESS, PACK_GET_DISTANCE);

    running_ = true;
    reader_thread_ = std::thread(&Ld07Node::readerThread, this);
}

Ld07Node::~Ld07Node()
{
    running_ = false;
    lidar_.SendCmd(port_, THIS_DEVICE_ADDRESS, PACK_STOP);
    port_.close();
    if (reader_thread_.joinable()) reader_thread_.join();
}

bool Ld07Node::waitForPort(const std::string& path, uint32_t baud)
{
    if (port_.open(path, baud)) return true;

    RCLCPP_WARN(get_logger(), "Cannot open '%s' — waiting for device", path.c_str());

    const auto slash = path.rfind('/');
    const std::string dir = (slash != std::string::npos) ? path.substr(0, slash) : ".";
    const std::string dev = (slash != std::string::npos) ? path.substr(slash + 1) : path;

    int ifd = inotify_init1(IN_CLOEXEC | IN_NONBLOCK);
    if (ifd >= 0) inotify_add_watch(ifd, dir.c_str(), IN_CREATE);

    while (rclcpp::ok() && !port_.open(path, baud)) {
        if (ifd < 0) { std::this_thread::sleep_for(std::chrono::seconds(1)); continue; }
        struct pollfd pfd{ifd, POLLIN, 0};
        poll(&pfd, 1, 1000);
        if (!(pfd.revents & POLLIN)) continue;
        alignas(inotify_event) char buf[4096];
        ssize_t n = ::read(ifd, buf, sizeof(buf));
        if (n <= 0) continue;
        bool matched = false;
        for (const char* p = buf; p < buf + n; ) {
            const auto* ev = reinterpret_cast<const inotify_event*>(p);
            if ((ev->mask & IN_CREATE) && ev->len > 0 && dev == ev->name) { matched = true; break; }
            p += sizeof(inotify_event) + ev->len;
        }
        if (!matched) continue;
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    if (ifd >= 0) ::close(ifd);
    return port_.isOpen();
}

bool Ld07Node::initSensor(int timeout_ms)
{
    uint8_t buf[1024];

    // Feed incoming bytes to lidar_ until predicate is true or deadline passes.
    auto feedUntil = [&](int ms, auto pred) {
        auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(ms);
        while (std::chrono::steady_clock::now() < deadline && rclcpp::ok() && !pred()) {
            int remaining = static_cast<int>(
                std::chrono::duration_cast<std::chrono::milliseconds>(
                    deadline - std::chrono::steady_clock::now()).count());
            int n = port_.readSome(buf, sizeof(buf), std::max(1, std::min(remaining, 100)));
            if (n > 0) lidar_.UnpackData(buf, static_cast<uint32_t>(n));
        }
        return pred();
    };

    for (int attempt = 0; attempt < 3 && !lidar_.IsParametersReady(); ++attempt) {
        lidar_.SendCmd(port_, 0,                   PACK_CONFIG_ADDRESS);
        feedUntil(200, [] { return false; });  // brief drain
        lidar_.SendCmd(port_, THIS_DEVICE_ADDRESS, PACK_GET_COE);
        feedUntil(timeout_ms / 3, [&] { return false; });  // wait for COE
        lidar_.SendCmd(port_, THIS_DEVICE_ADDRESS, PACK_VIDEO_SIZE);
        feedUntil(timeout_ms / 3, [&] { return lidar_.IsParametersReady(); });
    }
    return lidar_.IsParametersReady();
}

void Ld07Node::reconnect()
{
    lidar_.SendCmd(port_, THIS_DEVICE_ADDRESS, PACK_STOP);
    port_.close();
    lidar_.Reset();

    if (!waitForPort(port_path_, baud_)) return;
    RCLCPP_INFO(get_logger(), "Reconnected to %s", port_path_.c_str());

    if (initSensor(3000)) {
        RCLCPP_INFO(get_logger(), "Sensor re-initialised");
    } else {
        RCLCPP_WARN(get_logger(), "Re-init failed — proceeding anyway");
    }
    lidar_.SendCmd(port_, THIS_DEVICE_ADDRESS, PACK_GET_DISTANCE);
}

void Ld07Node::readerThread()
{
    uint8_t buf[2048];
    int consecutive_failures = 0;

    while (running_) {
        int n = port_.readSome(buf, sizeof(buf), timeout_ms_);

        if (n < 0) {
            if (++consecutive_failures == 5) {
                RCLCPP_WARN(get_logger(), "5 read failures — reconnecting");
                reconnect();
                consecutive_failures = 0;
            }
            continue;
        }
        if (n == 0) continue;  // timeout, normal

        consecutive_failures = 0;
        lidar_.SetStamp(now() - rclcpp::Duration(0LL, stamp_offset_ns_));
        lidar_.UnpackData(buf, static_cast<uint32_t>(n));

        if (lidar_.IsFrameReady()) {
            pub_->publish(lidar_.GetLaserScan());
            lidar_.ResetFrameReady();
        }
    }
}

}  // namespace ld07
