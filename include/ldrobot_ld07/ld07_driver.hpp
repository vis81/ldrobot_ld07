#pragma once
#include <array>
#include <cstdint>
#include <limits>
#include "ldrobot_ld07/serial_port.hpp"

namespace ld07 {

static constexpr size_t NUM_POINTS = 160;

// Command bytes (host → sensor)
static constexpr uint8_t CMD_DIST_START = 0x02;
static constexpr uint8_t CMD_DIST_STOP  = 0x0F;

struct Measurement {
    float   range_m;      // NaN when distance == 0 (no return)
    uint8_t confidence;   // 0–127
};

struct Frame {
    uint32_t timestamp_ms;
    std::array<Measurement, NUM_POINTS> points;
};

class Driver {
public:
    // Send one of the pre-built command frames (CMD_DIST_START / CMD_DIST_STOP).
    bool sendCommand(SerialPort& port, uint8_t cmd);

    // Block until one complete, checksum-verified distance frame arrives or timeout.
    bool readFrame(SerialPort& port, Frame& frame, int timeout_ms);

private:
    // Consume bytes until four consecutive 0xAA bytes seen.
    bool syncHeader(SerialPort& port, int timeout_ms);

    static uint8_t checksum(const uint8_t* data, size_t len);
};

}  // namespace ld07
