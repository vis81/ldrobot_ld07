#pragma once

#include <stdint.h>
#include <vector>
#include <string>
#include <limits>

#include "trnet.h"
#include "ldrobot_ld07/serial_port.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "rclcpp/time.hpp"

#define THIS_DEVICE_ADDRESS 0x01

typedef enum {
    PACK_GET_DISTANCE   = 0x02,
    PACK_STOP           = 0x0F,
    PACK_GET_COE        = 0x12,
    PACK_VIDEO_SIZE     = 0x15,
    PACK_CONFIG_ADDRESS = 0x16,
} PackageIDTypeDef;

struct PointData {
    float    angle;
    uint16_t distance;
    uint8_t  confidence;
    double   x;
    double   y;
    PointData(float a, uint16_t d, uint8_t c, double x_ = 0, double y_ = 0)
        : angle(a), distance(d), confidence(c), x(x_), y(y_) {}
    PointData() {}
};

class RTRNet {
public:
    RTRNet();

    void SendCmd(ld07::SerialPort& port, uint8_t address, uint8_t id);
    bool UnpackData(const uint8_t* data, uint32_t len);

    void ResetFrameReady()  { frame_ready_ = false; }
    void Reset();

    sensor_msgs::msg::LaserScan GetLaserScan() const { return output_; }
    bool IsParametersReady() const { return parameters_ready_; }
    bool IsFrameReady()      const { return frame_ready_; }

    void Configure(const std::string& frame_id,
                   float angle_min, float angle_max,
                   float range_min, float range_max,
                   uint8_t confidence_min = 0);
    void SetStamp(const rclcpp::Time& t) { stamp_ = t; }

private:
    bool Transform(const TRData* tr_data);
    void TransformSinglePoint(uint16_t dist, int n, uint8_t confidence,
                              std::vector<PointData>& dst);
    void ToLaserscan(const std::vector<PointData>& src);

    std::vector<uint8_t> data_tmp_;
    uint32_t coe_[4]{};
    uint16_t coe_u_{0}, coe_v_{0};
    bool     parameters_ready_{false};
    bool     frame_ready_{false};

    sensor_msgs::msg::LaserScan output_;
    rclcpp::Time stamp_;

    std::string frame_id_{"ld07_link"};
    float   angle_min_{-0.7330f};
    float   angle_max_{ 0.7854f};
    float   range_min_{0.005f};
    float   range_max_{0.400f};
    uint8_t confidence_min_{0};
};
