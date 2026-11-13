#pragma once
#include "thermalnav/core.hpp"
#include <map>
namespace thermalnav {
struct ThermalOptions {
  std::string mode{"ideal"};
  double background{20}, person{34}, wall{20}, floor{19}, furniture{22},
      equipment{27}, noise_sigma{.25}, blur_sigma{.8}, threshold{30};
  double min_depth{.15},max_depth{8.},nonuniformity{.35},texture_amplitude{.2},
      person_variation{.6},challenging_contrast{.55},warm_offset{1.};
  int seed{42}, min_area{24}, downsample{2};
};
// Signal units are configurable proxies, not measured temperature/radiance.
cv::Mat1f syntheticThermal(const cv::Mat3b &rgb, const cv::Mat1f &depth,
                          const cv::Mat1i &visible_ids,
                          const std::map<int, std::string> &classes,
                          const ThermalOptions &options, uint64_t frame_seed);
std::vector<cv::Rect> thermalRegions(const cv::Mat1f &signal,
                                    const ThermalOptions &options);
std::optional<cv::Vec3d> localizeThermalRegion(
    const cv::Rect &box, const cv::Mat1f &signal, const cv::Mat1f &depth,
    const cv::Matx33d &K, const cv::Matx33d &R, const cv::Vec3d &t,
    double threshold, double min_depth=.15, double max_depth=8.);
std::vector<Candidate> nearestBaselineCandidates(BEV &bev, XY robot,
                                                const std::vector<XY> &excluded);
} // namespace thermalnav
