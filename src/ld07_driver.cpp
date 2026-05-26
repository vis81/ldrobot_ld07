#include "ldrobot_ld07/ld07_driver.hpp"

#include <chrono>
#include <cmath>

namespace ld07 {

// Pre-built command frames (header + meta + checksum, no payload)
static const uint8_t kFrameStart[] = {0xAA,0xAA,0xAA,0xAA, 0x01,0x02,0x00,0x00,0x00,0x00, 0x03};
static const uint8_t kFrameStop[]  = {0xAA,0xAA,0xAA,0xAA, 0x01,0x0F,0x00,0x00,0x00,0x00, 0x10};

static constexpr uint8_t  kHeaderByte  = 0xAA;
static constexpr uint8_t  kCmdDist     = 0x02;
static constexpr uint16_t kDistDataLen = 324;  // 4 timestamp + 160×2 measurements

bool Driver::sendCommand(SerialPort& port, uint8_t cmd) {
    if (cmd == CMD_DIST_START) return port.writeBytes(kFrameStart, sizeof(kFrameStart));
    if (cmd == CMD_DIST_STOP)  return port.writeBytes(kFrameStop,  sizeof(kFrameStop));
    return false;
}

uint8_t Driver::checksum(const uint8_t* data, size_t len) {
    uint8_t sum = 0;
    for (size_t i = 0; i < len; ++i) sum += data[i];
    return sum;
}

bool Driver::syncHeader(SerialPort& port, int timeout_ms) {
    uint8_t consecutive = 0;
    uint8_t b;
    auto deadline = std::chrono::steady_clock::now()
                  + std::chrono::milliseconds(timeout_ms);

    while (consecutive < 4) {
        auto remaining_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
            deadline - std::chrono::steady_clock::now()).count();
        if (remaining_ms <= 0) return false;

        if (port.readBytes(&b, 1, static_cast<int>(remaining_ms)) != 1) return false;
        consecutive = (b == kHeaderByte) ? consecutive + 1 : 0;
    }
    return true;
}

bool Driver::readFrame(SerialPort& port, Frame& frame, int timeout_ms) {
    if (!syncHeader(port, timeout_ms)) return false;

    // 6-byte meta: [dev_addr, cmd, offset_lo, offset_hi, datalen_lo, datalen_hi]
    uint8_t meta[6];
    if (port.readBytes(meta, 6, timeout_ms) != 6) return false;

    const uint8_t  cmd     = meta[1];
    const uint16_t datalen = static_cast<uint16_t>(meta[4])
                           | (static_cast<uint16_t>(meta[5]) << 8);

    if (cmd != kCmdDist || datalen != kDistDataLen) return false;

    // payload (324 bytes) + checksum byte
    uint8_t payload[kDistDataLen + 1];
    if (port.readBytes(payload, kDistDataLen + 1, timeout_ms) != kDistDataLen + 1) return false;

    // checksum covers all bytes after the 4-byte header (meta + payload, excluding cs)
    const uint8_t expected_cs = checksum(meta, 6) + checksum(payload, kDistDataLen);
    if (expected_cs != payload[kDistDataLen]) return false;

    // timestamp: LE uint32 at payload[0..3]
    frame.timestamp_ms = static_cast<uint32_t>(payload[0])
                       | (static_cast<uint32_t>(payload[1]) << 8)
                       | (static_cast<uint32_t>(payload[2]) << 16)
                       | (static_cast<uint32_t>(payload[3]) << 24);

    // 160 measurements × 2 bytes LE starting at payload[4]
    // bits [8:0]  → distance in mm
    // bits [15:9] → confidence (0–127)
    const uint8_t* p = payload + 4;
    for (size_t i = 0; i < NUM_POINTS; ++i, p += 2) {
        const uint16_t raw    = static_cast<uint16_t>(p[0]) | (static_cast<uint16_t>(p[1]) << 8);
        const uint16_t dist_mm = raw & 0x01FFu;
        frame.points[i].confidence = static_cast<uint8_t>(raw >> 9);
        frame.points[i].range_m    = (dist_mm > 0)
            ? static_cast<float>(dist_mm) / 1000.0f
            : std::numeric_limits<float>::quiet_NaN();
    }

    return true;
}

}  // namespace ld07
