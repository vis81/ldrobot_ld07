#include "ldrobot_ld07/serial_port.hpp"

#include <cerrno>
#include <chrono>
#include <cstring>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <unistd.h>

// Wrap linux/termios.h to get termios2/BOTHER without conflicting with <termios.h>
namespace asmtermios {
#include <linux/termios.h>
}
#include <termios.h>

namespace ld07 {

SerialPort::~SerialPort() { close(); }

bool SerialPort::open(const std::string& path, uint32_t baud) {
    // O_NONBLOCK during open prevents blocking on carrier-detect assertion
    fd_ = ::open(path.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd_ < 0) return false;

    struct asmtermios::termios2 opts;
    if (ioctl(fd_, _IOC(_IOC_READ, 'T', 0x2A, sizeof(opts)), &opts)) {
        ::close(fd_); fd_ = -1; return false;
    }

    opts.c_cflag |=  static_cast<tcflag_t>(CLOCAL | CREAD | CS8);
    opts.c_cflag &= ~static_cast<tcflag_t>(CSTOPB | PARENB | HUPCL);  // no DTR drop on close
    opts.c_lflag &= ~static_cast<tcflag_t>(ICANON | ECHO | ECHOE | ISIG | IEXTEN);
    opts.c_oflag &= ~static_cast<tcflag_t>(OPOST);
    opts.c_iflag &= ~static_cast<tcflag_t>(IXON | IXOFF | INLCR | IGNCR | ICRNL | IGNBRK);
    opts.c_cflag &= ~static_cast<tcflag_t>(CBAUD);
    opts.c_cflag |=  static_cast<tcflag_t>(BOTHER);
    opts.c_ispeed = baud;
    opts.c_ospeed = baud;
    // VMIN=1, VTIME=0: read() blocks until at least 1 byte arrives.
    // Matches pyserial default. Thread shutdown unblocks by closing the fd.
    opts.c_cc[VMIN]  = 1;
    opts.c_cc[VTIME] = 0;

    if (ioctl(fd_, _IOC(_IOC_WRITE, 'T', 0x2B, sizeof(opts)), &opts)) {
        ::close(fd_); fd_ = -1; return false;
    }

    // Switch to blocking mode now that the port is configured
    int flags = fcntl(fd_, F_GETFL, 0);
    fcntl(fd_, F_SETFL, flags & ~O_NONBLOCK);

    tcflush(fd_, TCIFLUSH);
    return true;
}

void SerialPort::close() {
    if (fd_ >= 0) { ::close(fd_); fd_ = -1; }
}

// Read exactly 'len' bytes within timeout_ms.
// With VMIN=1 the reads are purely blocking; we rely on close() to unblock
// the thread on shutdown rather than a timed poll.
int SerialPort::readBytes(uint8_t* buf, size_t len, int timeout_ms) {
    size_t total = 0;
    const auto deadline = std::chrono::steady_clock::now()
                        + std::chrono::milliseconds(timeout_ms);

    while (total < len) {
        if (std::chrono::steady_clock::now() >= deadline) break;

        ssize_t n = ::read(fd_, buf + total, len - total);
        if (n < 0) {
            if (errno == EINTR) continue;
            break;  // EIO — fd was closed or device disconnected
        }
        if (n == 0) break;
        total += static_cast<size_t>(n);
    }

    return static_cast<int>(total);
}

bool SerialPort::writeBytes(const uint8_t* buf, size_t len) {
    return ::write(fd_, buf, len) == static_cast<ssize_t>(len);
}

}  // namespace ld07
