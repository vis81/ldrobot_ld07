#pragma once
#include <cstddef>
#include <cstdint>
#include <string>

namespace ld07 {

class SerialPort {
public:
    SerialPort() = default;
    ~SerialPort();

    bool open(const std::string& path, uint32_t baud);
    void close();
    bool isOpen() const { return fd_ >= 0; }
    int  fd()     const { return fd_; }

    // Read exactly 'len' bytes within timeout_ms total. Returns bytes read.
    int  readBytes(uint8_t* buf, size_t len, int timeout_ms);
    // Read up to max_len bytes in a single call. Returns bytes read, 0 on timeout, -1 on error/EOF.
    int  readSome(uint8_t* buf, size_t max_len, int timeout_ms);
    bool writeBytes(const uint8_t* buf, size_t len);

private:
    int fd_{-1};
};

}  // namespace ld07
