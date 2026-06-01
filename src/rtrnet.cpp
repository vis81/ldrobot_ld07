#include "rtrnet.hpp"

#include <cstring>
#include <cmath>
#include <algorithm>
#include <limits>

RTRNet::RTRNet() {}

void RTRNet::Configure(const std::string& frame_id,
                       float angle_min, float angle_max,
                       float range_min, float range_max,
                       uint8_t confidence_min)
{
    frame_id_       = frame_id;
    angle_min_      = angle_min;
    angle_max_      = angle_max;
    range_min_      = range_min;
    range_max_      = range_max;
    confidence_min_ = confidence_min;
}

void RTRNet::Reset()
{
    data_tmp_.clear();
    parameters_ready_ = false;
    frame_ready_      = false;
}

void RTRNet::SendCmd(ld07::SerialPort& port, uint8_t address, uint8_t id)
{
    TRNet pkg;
    std::vector<uint8_t> out;
    TRData out_data;
    out_data.device_address = address;
    out_data.pack_id        = id;
    out_data.chunk_offset   = 0;
    pkg.Pack(out_data, out);
    port.writeBytes(out.data(), out.size());
}

bool RTRNet::UnpackData(const uint8_t* data, uint32_t len)
{
    for (uint32_t i = 0; i < len; ++i)
        data_tmp_.push_back(data[i]);

    // Prevent unbounded growth if framing is lost
    if (data_tmp_.size() > 8192)
        data_tmp_.erase(data_tmp_.begin(), data_tmp_.begin() + 4096);

    if (data_tmp_.size() < 4)
        return false;

    TRNet t;
    int pos = 0;

    for (int i = 0; i < static_cast<int>(data_tmp_.size()) - 4;) {
        if (!t.FindLeadingCode(data_tmp_.data() + i)) { ++i; continue; }

        const TRData* tr_data = t.Unpack(data_tmp_.data() + i,
                                         data_tmp_.size() - i);
        if (tr_data == nullptr) { ++i; continue; }

        switch (tr_data->pack_id) {
        case PACK_GET_DISTANCE:
            Transform(tr_data);
            break;

        case PACK_GET_COE:
            coe_[0] = *(const uint32_t*)(tr_data->data.data());
            coe_[1] = *(const uint32_t*)(tr_data->data.data() + 4);
            coe_[2] = *(const uint32_t*)(tr_data->data.data() + 8);
            coe_[3] = *(const uint32_t*)(tr_data->data.data() + 12);
            coe_u_  = *(const uint16_t*)(tr_data->data.data() + 16);
            break;

        case PACK_VIDEO_SIZE:
            coe_u_           = *(const uint16_t*)(tr_data->data.data());
            coe_v_           = *(const uint16_t*)(tr_data->data.data() + 2);
            parameters_ready_ = true;
            break;

        default:
            break;
        }

        i  += static_cast<int>(t.GetParseDataLen());
        pos = i;
    }

    if (pos > 0)
        data_tmp_.erase(data_tmp_.begin(), data_tmp_.begin() + pos);

    return true;
}

bool RTRNet::Transform(const TRData* tr_data)
{
    std::vector<PointData> tmp;
    int data_amount = static_cast<int>(tr_data->data.size()) - 4;
    int n = 0;

    for (int i = 0; i < data_amount; i += 2, ++n) {
        uint16_t value      = *(const uint16_t*)(tr_data->data.data() + i + 4);
        uint8_t  confidence = static_cast<uint8_t>((value >> 9) << 1);
        value              &= 0x1FFu;
        uint16_t center_dis = value;

        if (center_dis > 0 && confidence >= confidence_min_)
            TransformSinglePoint(center_dis, n, confidence, tmp);
    }

    ToLaserscan(tmp);
    frame_ready_ = true;
    return true;
}

void RTRNet::TransformSinglePoint(uint16_t dist, int n, uint8_t confidence,
                                  std::vector<PointData>& dst)
{
    const double k0 = static_cast<double>(coe_[0]) / 10000.0;
    const double k1 = static_cast<double>(coe_[1]) / 10000.0;
    const double b0 = static_cast<double>(coe_[2]) / 10000.0;
    const double b1 = static_cast<double>(coe_[3]) / 10000.0;
    const double pi = 3.14159265;

    double pixel_u = n, tmp_theta, tmp_dist, tmp_x, tmp_y;

    if (pixel_u > 80) {
        pixel_u  = pixel_u - 80;
        pixel_u  = 80 - pixel_u;
        tmp_theta = (b0 > 1.0) ? (k0 * pixel_u - b0)
                                : (std::atan(k0 * pixel_u - b0) * 180.0 / pi);
        tmp_dist  = (dist - 1.22) / std::cos((22.5 - tmp_theta) * pi / 180.0);
        tmp_theta *= pi / 180.0;
        tmp_x = std::cos(22.5*pi/180)*tmp_dist*std::cos(tmp_theta)
              + std::sin(22.5*pi/180)*tmp_dist*std::sin(tmp_theta);
        tmp_y = -std::sin(22.5*pi/180)*tmp_dist*std::cos(tmp_theta)
              + std::cos(22.5*pi/180)*tmp_dist*std::sin(tmp_theta);
        tmp_x += 1.22;
        tmp_y -= 5.315;
    } else {
        pixel_u  = 80 - pixel_u;
        tmp_theta = (b1 > 1.0) ? (k1 * pixel_u - b1)
                                : (std::atan(k1 * pixel_u - b1) * 180.0 / pi);
        tmp_dist  = (dist - 1.22) / std::cos((22.5 + tmp_theta) * pi / 180.0);
        tmp_theta *= pi / 180.0;
        tmp_x = std::cos(-22.5*pi/180)*tmp_dist*std::cos(tmp_theta)
              + std::sin(-22.5*pi/180)*tmp_dist*std::sin(tmp_theta);
        tmp_y = -std::sin(-22.5*pi/180)*tmp_dist*std::cos(tmp_theta)
              + std::cos(-22.5*pi/180)*tmp_dist*std::sin(tmp_theta);
        tmp_x += 1.22;
        tmp_y += 5.315;
    }

    PointData pd;
    pd.x          = tmp_x;
    pd.y          = tmp_y;
    pd.confidence = confidence;
    dst.push_back(pd);
}

void RTRNet::ToLaserscan(const std::vector<PointData>& src)
{
    constexpr int kBins = 160;

    output_.header.stamp    = stamp_;
    output_.header.frame_id = frame_id_;
    output_.angle_min       = angle_min_;
    output_.angle_max       = angle_max_;
    output_.range_min       = range_min_;
    output_.range_max       = range_max_;
    output_.angle_increment = (angle_max_ - angle_min_) / static_cast<float>(kBins - 1);
    output_.scan_time       = 1.0f / 28.0f;
    output_.time_increment  = output_.scan_time / static_cast<float>(kBins - 1);

    output_.ranges.assign(kBins, std::numeric_limits<float>::quiet_NaN());
    output_.intensities.assign(kBins, 0.0f);

    for (const auto& pt : src) {
        float range = std::hypot(static_cast<float>(pt.x),
                                 static_cast<float>(pt.y)) / 1000.0f;
        float angle = std::atan2(static_cast<float>(pt.y),
                                 static_cast<float>(pt.x));

        if (range < range_min_ || range > range_max_) continue;
        if (angle < angle_min_ || angle > angle_max_) continue;

        int bin = static_cast<int>(
            (angle - angle_min_) / output_.angle_increment + 0.5f);
        if (bin < 0 || bin >= kBins) continue;

        if (std::isnan(output_.ranges[bin]) || range < output_.ranges[bin]) {
            output_.ranges[bin]      = range;
            output_.intensities[bin] = static_cast<float>(pt.confidence);
        }
    }

    // Suppress isolated ghost returns: require >= min_neighbors_ valid bins
    // within ±neighbor_window_ of each valid bin.
    if (neighbor_window_ > 0 && min_neighbors_ > 0) {
        std::vector<bool> keep(kBins, true);
        for (int i = 0; i < kBins; ++i) {
            if (std::isnan(output_.ranges[i])) continue;
            int valid = 0;
            for (int j = i - neighbor_window_; j <= i + neighbor_window_; ++j) {
                if (j == i || j < 0 || j >= kBins) continue;
                if (!std::isnan(output_.ranges[j])) ++valid;
            }
            if (valid < min_neighbors_) keep[i] = false;
        }
        for (int i = 0; i < kBins; ++i) {
            if (!keep[i]) {
                output_.ranges[i]      = std::numeric_limits<float>::quiet_NaN();
                output_.intensities[i] = 0.0f;
            }
        }
    }

    std::reverse(output_.ranges.begin(),      output_.ranges.end());
    std::reverse(output_.intensities.begin(), output_.intensities.end());
}
