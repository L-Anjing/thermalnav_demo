#pragma once
#include "thermalnav/stereo_frontend.hpp"
#include <map>
#include <octomap/OcTree.h>

namespace thermalnav {
struct Observation {
  std::string source;
  double stamp{};
  cv::Matx33d R{cv::Matx33d::eye()};
  cv::Vec3d origin{};
  std::vector<DepthPoint> points;
};
struct FusionOptions {
  double resolution{.1}, floor{0.}, min_height{.12}, max_height{.65};
  double max_range{8.}, max_extent{100.};
  size_t max_nodes{2000000};
};
class OccupancyFusion {
public:
  explicit OccupancyFusion(FusionOptions options = {});
  Json integrate(const std::vector<Observation> &observations, BEV &bev);
  void project(BEV &bev) const;
  void clear();
  const octomap::OcTree &tree() const { return tree_; }

private:
  FusionOptions options_;
  octomap::OcTree tree_;
  std::map<std::pair<int, int>, float> ground_;
  std::map<std::string, double> last_stamps_;
};
Observation depthObservation(const cv::Mat1f &depth, const cv::Matx33d &K,
                             const cv::Matx33d &R, const cv::Vec3d &t,
                             double stamp, double max_range = 8.,
                             double variance = .0004);
cv::Mat1f degradeDepth(const cv::Mat1f &depth, double dropout,
                       double noise_sigma, double max_range, uint64_t seed);
Json compareOccupancy(const BEV &predicted, const BEV &reference);
} // namespace thermalnav
