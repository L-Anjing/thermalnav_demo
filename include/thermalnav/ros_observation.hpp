#pragma once
#include "thermalnav/stereo_frontend.hpp"
#include <algorithm>
#include <cstring>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>

namespace thermalnav {
inline sensor_msgs::msg::PointCloud2
pointCloud(const std::vector<DepthPoint> &points,
           const std_msgs::msg::Header &header) {
  sensor_msgs::msg::PointCloud2 out;
  out.header = header;
  out.height = 1;
  out.width = points.size();
  out.point_step = 24;
  out.row_step = out.width * out.point_step;
  out.is_dense = true;
  const std::vector<std::string> names{
      "x", "y", "z", "confidence", "variance", "floor_reliable"};
  for (size_t i = 0; i < names.size(); ++i) {
    sensor_msgs::msg::PointField f;
    f.name = names[i];
    f.offset = i * 4;
    f.datatype = f.FLOAT32;
    f.count = 1;
    out.fields.push_back(f);
  }
  out.data.resize(out.row_step);
  for (size_t i = 0; i < points.size(); ++i) {
    const auto &p = points[i];
    float row[] = {float(p.p[0]), float(p.p[1]), float(p.p[2]),
                   p.confidence,  p.variance,    p.floor_reliable ? 1.f : 0.f};
    std::memcpy(out.data.data() + i * 24, row, 24);
  }
  return out;
}
inline std::vector<DepthPoint>
readPoints(const sensor_msgs::msg::PointCloud2 &cloud) {
  size_t count = size_t(cloud.width) * cloud.height;
  if (cloud.is_bigendian || cloud.point_step < 24 || count > 1000000 ||
      cloud.row_step < size_t(cloud.width) * cloud.point_step ||
      cloud.data.size() < size_t(cloud.row_step) * cloud.height)
    throw std::invalid_argument("invalid sparse cloud layout");
  const std::vector<std::string> names{
      "x", "y", "z", "confidence", "variance", "floor_reliable"};
  std::vector<uint32_t> offsets;
  for (const auto &name : names) {
    auto it = std::find_if(cloud.fields.begin(), cloud.fields.end(),
                           [&](const auto &f) { return f.name == name; });
    if (it == cloud.fields.end() ||
        it->datatype != sensor_msgs::msg::PointField::FLOAT32 ||
        it->count != 1 || it->offset + 4 > cloud.point_step)
      throw std::invalid_argument("missing sparse cloud field: " + name);
    offsets.push_back(it->offset);
  }
  std::vector<DepthPoint> out;
  out.reserve(count);
  for (uint32_t y = 0; y < cloud.height; ++y)
    for (uint32_t x = 0; x < cloud.width; ++x) {
      const auto *data =
          cloud.data.data() + size_t(y) * cloud.row_step + x * cloud.point_step;
      float row[6];
      for (int j = 0; j < 6; ++j)
        std::memcpy(&row[j], data + offsets[j], 4);
      out.push_back({{row[0], row[1], row[2]}, row[3], row[4], row[5] > .5f});
    }
  return out;
}
} // namespace thermalnav
