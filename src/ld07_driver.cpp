#include "ldrobot_ld07/ld07_driver.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>

namespace ld07 {

// Pre-built command frames (header + meta + checksum, no payload)
static const uint8_t kFrameStart[]      = {0xAA,0xAA,0xAA,0xAA, 0x01,0x02,0x00,0x00,0x00,0x00, 0x03};
static const uint8_t kFrameStop[]       = {0xAA,0xAA,0xAA,0xAA, 0x01,0x0F,0x00,0x00,0x00,0x00, 0x10};
static const uint8_t kFrameConfigAddr[] = {0xAA,0xAA,0xAA,0xAA, 0x00,0x16,0x00,0x00,0x00,0x00, 0x16};
static const uint8_t kFrameGetCalib[]   = {0xAA,0xAA,0xAA,0xAA, 0x01,0x12,0x00,0x00,0x00,0x00, 0x13};
static const uint8_t kFrameVideoSize[]  = {0xAA,0xAA,0xAA,0xAA, 0x01,0x15,0x00,0x00,0x00,0x00, 0x16};

static constexpr uint8_t  kHeaderByte   = 0xAA;
static constexpr uint8_t  kCmdDist      = 0x02;
static constexpr uint8_t  kCmdCalib     = 0x12;
static constexpr uint8_t  kCmdVideo     = 0x15;
static constexpr uint16_t kDistDataLen  = 324;   // 4 timestamp + 160×2 measurements
static constexpr uint16_t kCalibDataLen = 18;    // k0+k1+b0+b1 (4×uint32) + coe_u (uint16)
static constexpr uint16_t kVideoDataLen = 4;     // coe_u + coe_v (2×uint16)

bool Driver::sendCommand(SerialPort& port, uint8_t cmd) {
    switch (cmd) {
        case CMD_DIST_START:  return port.writeBytes(kFrameStart,      sizeof(kFrameStart));
        case CMD_DIST_STOP:   return port.writeBytes(kFrameStop,       sizeof(kFrameStop));
        case CMD_CONFIG_ADDR: return port.writeBytes(kFrameConfigAddr, sizeof(kFrameConfigAddr));
        case CMD_GET_CALIB:   return port.writeBytes(kFrameGetCalib,   sizeof(kFrameGetCalib));
        case CMD_VIDEO_SIZE:  return port.writeBytes(kFrameVideoSize,  sizeof(kFrameVideoSize));
        default: return false;
    }
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

// Read frames until one matching expected_cmd/expected_len arrives; copy payload into buf.
// Frames with wrong cmd/len or bad checksum are silently skipped within the deadline.
bool Driver::readResponse(SerialPort& port, uint8_t expected_cmd,
                          uint8_t* buf, uint16_t expected_len, int timeout_ms)
{
    auto deadline = std::chrono::steady_clock::now()
                  + std::chrono::milliseconds(timeout_ms);

    auto remaining = [&]() -> int {
        auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
            deadline - std::chrono::steady_clock::now()).count();
        return ms > 0 ? static_cast<int>(ms) : 0;
    };

    // Large enough for any valid frame (distance frames are the biggest at 325 bytes)
    uint8_t tmp[kDistDataLen + 1];

    while (remaining() > 0) {
        if (!syncHeader(port, remaining())) return false;

        uint8_t meta[6];
        if (port.readBytes(meta, 6, remaining()) != 6) return false;

        const uint8_t  cmd     = meta[1];
        const uint16_t datalen = static_cast<uint16_t>(meta[4])
                               | (static_cast<uint16_t>(meta[5]) << 8);
        const size_t   total   = static_cast<size_t>(datalen) + 1;

        if (total > sizeof(tmp)) return false;  // unexpected oversized frame

        if (port.readBytes(tmp, total, remaining()) != static_cast<int>(total)) return false;

        const uint8_t expected_cs = checksum(meta, 6) + checksum(tmp, datalen);
        if (expected_cs != tmp[datalen]) continue;  // bad checksum — skip frame

        if (cmd == expected_cmd && datalen == expected_len) {
            std::memcpy(buf, tmp, expected_len);
            return true;
        }
        // cmd/len mismatch — keep searching within deadline
    }
    return false;
}

bool Driver::initSensor(SerialPort& port, int timeout_ms) {
    calib_ = Calib{};  // always fetch fresh calibration on (re)init

    auto deadline = std::chrono::steady_clock::now()
                  + std::chrono::milliseconds(timeout_ms);

    auto remaining = [&]() -> int {
        auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
            deadline - std::chrono::steady_clock::now()).count();
        return ms > 0 ? static_cast<int>(ms) : 0;
    };

    // Step 1: broadcast address assignment (no response expected)
    sendCommand(port, CMD_CONFIG_ADDR);

    // Step 2: request calibration coefficients — retry up to 3× or until deadline
    for (int attempt = 0; attempt < 3 && !calib_.valid; ++attempt) {
        if (remaining() <= 0) break;
        sendCommand(port, CMD_GET_CALIB);

        uint8_t calib_buf[kCalibDataLen];
        if (!readResponse(port, kCmdCalib, calib_buf, kCalibDataLen,
                          std::min(remaining(), 1000))) continue;

        auto le32 = [](const uint8_t* p) -> uint32_t {
            return static_cast<uint32_t>(p[0])
                 | (static_cast<uint32_t>(p[1]) << 8)
                 | (static_cast<uint32_t>(p[2]) << 16)
                 | (static_cast<uint32_t>(p[3]) << 24);
        };
        calib_.k0    = static_cast<float>(le32(calib_buf +  0)) / 10000.0f;
        calib_.k1    = static_cast<float>(le32(calib_buf +  4)) / 10000.0f;
        calib_.b0    = static_cast<float>(le32(calib_buf +  8)) / 10000.0f;
        calib_.b1    = static_cast<float>(le32(calib_buf + 12)) / 10000.0f;
        calib_.valid = true;
    }

    // Step 3: request video size — signals init complete to the sensor
    for (int attempt = 0; attempt < 3; ++attempt) {
        if (remaining() <= 0) break;
        sendCommand(port, CMD_VIDEO_SIZE);
        uint8_t video_buf[kVideoDataLen];
        if (readResponse(port, kCmdVideo, video_buf, kVideoDataLen,
                         std::min(remaining(), 1000))) break;
    }

    return calib_.valid;
}

bool Driver::readFrame(SerialPort& port, Frame& frame, int timeout_ms) {
    if (!syncHeader(port, timeout_ms)) return false;

    uint8_t meta[6];
    if (port.readBytes(meta, 6, timeout_ms) != 6) return false;

    const uint8_t  cmd     = meta[1];
    const uint16_t datalen = static_cast<uint16_t>(meta[4])
                           | (static_cast<uint16_t>(meta[5]) << 8);

    if (cmd != kCmdDist || datalen != kDistDataLen) return false;

    uint8_t payload[kDistDataLen + 1];
    if (port.readBytes(payload, kDistDataLen + 1, timeout_ms) != kDistDataLen + 1) return false;

    const uint8_t expected_cs = checksum(meta, 6) + checksum(payload, kDistDataLen);
    if (expected_cs != payload[kDistDataLen]) return false;

    frame.timestamp_ms = static_cast<uint32_t>(payload[0])
                       | (static_cast<uint32_t>(payload[1]) << 8)
                       | (static_cast<uint32_t>(payload[2]) << 16)
                       | (static_cast<uint32_t>(payload[3]) << 24);

    const uint8_t* p = payload + 4;
    for (size_t i = 0; i < NUM_POINTS; ++i, p += 2) {
        const uint16_t raw = static_cast<uint16_t>(p[0]) | (static_cast<uint16_t>(p[1]) << 8);
        frame.points[i].dist_mm    = raw & 0x01FFu;
        frame.points[i].confidence = static_cast<uint8_t>((raw >> 9) << 1);  // 0–254
    }

    return true;
}

// Perspective-correct transform ported from vendor TransformSinglePoint (rtrnet.cpp).
// n: pixel index 0–159 (0 = left edge, 159 = right edge as stored in frame).
// n > 80 → right camera (k0/b0, +22.5° rotation).
// n ≤ 80 → left camera  (k1/b1, -22.5° rotation).
// Output x_mm/y_mm are in sensor-local Cartesian mm (x forward, y left).
bool Driver::transformPoint(uint16_t dist_mm, int n, float& x_mm, float& y_mm) const {
    if (!calib_.valid || dist_mm == 0) return false;

    static constexpr double kPi  = 3.14159265;
    static constexpr double kDeg = kPi / 180.0;

    const double dist   = static_cast<double>(dist_mm);
    const double k0 = calib_.k0, k1 = calib_.k1;
    const double b0 = calib_.b0, b1 = calib_.b1;

    double pixel_u, k, b, theta_deg, tmp_dist, tx, ty, rot;

    if (n > 80) {
        pixel_u   = static_cast<double>(80 - (n - 80));  // = 160 - n
        k = k0; b = b0;
        theta_deg = (b > 1.0) ? (k * pixel_u - b)
                               : (std::atan(k * pixel_u - b) * 180.0 / kPi);
        tmp_dist  = (dist - 1.22) / std::cos((22.5 - theta_deg) * kDeg);
        rot       = 22.5 * kDeg;
        const double tr = theta_deg * kDeg;
        tx = std::cos(rot) * tmp_dist * std::cos(tr) + std::sin(rot) * tmp_dist * std::sin(tr);
        ty = -std::sin(rot) * tmp_dist * std::cos(tr) + std::cos(rot) * tmp_dist * std::sin(tr);
        tx += 1.22;
        ty -= 5.315;
    } else {
        pixel_u   = static_cast<double>(80 - n);
        k = k1; b = b1;
        theta_deg = (b > 1.0) ? (k * pixel_u - b)
                               : (std::atan(k * pixel_u - b) * 180.0 / kPi);
        tmp_dist  = (dist - 1.22) / std::cos((22.5 + theta_deg) * kDeg);
        rot       = -22.5 * kDeg;
        const double tr = theta_deg * kDeg;
        tx = std::cos(rot) * tmp_dist * std::cos(tr) + std::sin(rot) * tmp_dist * std::sin(tr);
        ty = -std::sin(rot) * tmp_dist * std::cos(tr) + std::cos(rot) * tmp_dist * std::sin(tr);
        tx += 1.22;
        ty += 5.315;
    }

    x_mm = static_cast<float>(tx);
    y_mm = static_cast<float>(ty);
    return true;
}

}  // namespace ld07
