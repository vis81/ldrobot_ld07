#pragma once
#include <array>
#include <cstdint>
#include <limits>
#include "ldrobot_ld07/serial_port.hpp"

namespace ld07 {

static constexpr size_t NUM_POINTS = 160;

// Command bytes (host → sensor)
static constexpr uint8_t CMD_DIST_START  = 0x02;
static constexpr uint8_t CMD_DIST_STOP   = 0x0F;
static constexpr uint8_t CMD_CONFIG_ADDR = 0x16;  // broadcast address assignment
static constexpr uint8_t CMD_GET_CALIB   = 0x12;  // per-device perspective coefficients
static constexpr uint8_t CMD_VIDEO_SIZE  = 0x15;  // pixel dimensions (marks init complete)

struct Measurement {
    uint16_t dist_mm;    // raw distance in mm; 0 = no return
    uint8_t  confidence; // 0–254  (bits [15:9] × 2, vendor convention)
};

struct Frame {
    uint32_t timestamp_ms;
    std::array<Measurement, NUM_POINTS> points;
};

// Per-device calibration coefficients received via CMD_GET_CALIB.
// Used by transformPoint() to correct perspective projection.
struct Calib {
    float k0{0.f}, k1{0.f};  // slope for right/left camera
    float b0{0.f}, b1{0.f};  // intercept for right/left camera
    bool  valid{false};
};

class Driver {
public:
    // Initialization sequence: CONFIG_ADDR → GET_CALIB → VIDEO_SIZE.
    // Must be called before sendCommand(CMD_DIST_START).
    // Returns true when calibration received; false on timeout (streaming still works,
    // but makeScan falls back to linear angle mapping).
    bool initSensor(SerialPort& port, int timeout_ms);

    // Send pre-built command frame (CMD_DIST_START / CMD_DIST_STOP).
    bool sendCommand(SerialPort& port, uint8_t cmd);

    // Block until one complete, checksum-verified distance frame arrives or timeout.
    bool readFrame(SerialPort& port, Frame& frame, int timeout_ms);

    // Perspective-correct transform: (dist_mm, pixel_index 0–159) → (x_mm, y_mm).
    // Returns false if calibration is not valid or dist_mm == 0.
    bool transformPoint(uint16_t dist_mm, int n, float& x_mm, float& y_mm) const;

    const Calib& calib() const { return calib_; }

private:
    bool syncHeader(SerialPort& port, int timeout_ms);
    // Read frames until one matching expected_cmd/expected_len is found; copy payload into buf.
    bool readResponse(SerialPort& port, uint8_t expected_cmd,
                      uint8_t* buf, uint16_t expected_len, int timeout_ms);
    static uint8_t checksum(const uint8_t* data, size_t len);

    Calib calib_;
};

}  // namespace ld07
