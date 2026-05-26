#include <csignal>
#include <cstdlib>
#include <exception>
#include <unistd.h>

#include <rclcpp/rclcpp.hpp>
#include "ldrobot_ld07/ld07_node.hpp"

// Raw DIST_STOP frame — written directly from async-signal-safe / terminate context.
static const uint8_t kStopFrame[] = {
    0xAA, 0xAA, 0xAA, 0xAA, 0x01, 0x0F, 0x00, 0x00, 0x00, 0x00, 0x10
};

// fd of the open serial port; -1 when no port is open.
// Written once from main after node construction; read from signal/terminate handler.
static std::atomic<int> g_serial_fd{-1};

static void send_stop_raw() {
    int fd = g_serial_fd.load(std::memory_order_relaxed);
    if (fd >= 0) {
        // write() is async-signal-safe
        write(fd, kStopFrame, sizeof(kStopFrame));
    }
}

// SIGHUP and any future redirected signals: trigger rclcpp clean shutdown so
// the node destructor runs (which sends DIST_STOP, closes the port, joins the
// thread) before the process exits.
static void on_fatal_signal(int /*sig*/) {
    rclcpp::shutdown();
}

// std::terminate handler: last resort for unhandled exceptions / noexcept
// violations where stack unwinding may not run.  Use the raw fd path since
// the C++ runtime state may be inconsistent.
static void on_terminate() {
    send_stop_raw();
    std::abort();
}

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);

    // rclcpp installs handlers for SIGINT and SIGTERM; extend coverage.
    signal(SIGHUP,  on_fatal_signal);
    signal(SIGPIPE, SIG_IGN);   // broken pipe should not crash the node
    std::set_terminate(on_terminate);

    auto node = std::make_shared<ld07::Ld07Node>();
    g_serial_fd.store(node->serialFd(), std::memory_order_relaxed);

    try {
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        RCLCPP_FATAL(rclcpp::get_logger("main"), "%s", e.what());
    }

    // Normal exit path: clear the fd before rclcpp teardown so the terminate
    // handler won't fire a redundant write after the node destructor already
    // sent DIST_STOP and closed the port.
    g_serial_fd.store(-1, std::memory_order_relaxed);
    rclcpp::shutdown();
    return 0;
}
