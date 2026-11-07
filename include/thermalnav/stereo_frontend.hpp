#pragma once
#include "thermalnav/core.hpp"
#include <opencv2/calib3d.hpp>

namespace thermalnav {
struct StereoCalibration {
  cv::Size size;
  cv::Matx33d left_K, right_K, right_from_left_R{cv::Matx33d::eye()};
  cv::Vec3d right_from_left_t{-.12, 0, 0};
  std::vector<double> left_D, right_D;
};
struct StereoOptions {
  int max_features{1200}, radius{4};
  double min_range{.5}, max_range{8.}, min_correlation{.85}, uniqueness{.06};
  double max_lr_error{.8}, max_sigma_z{.25};
};
struct DepthPoint {
  cv::Vec3d p;
  float confidence{1}, variance{.0004f};
  bool floor_reliable{false};
};
struct StereoResult {
  cv::Mat1b left_gray, right_gray;
  cv::Mat1f depth, confidence;
  cv::Mat3b matches;
  cv::Matx33d left_R, right_R;
  cv::Matx34d left_P, right_P;
  std::vector<DepthPoint> points;
  Json diagnostics;
};
class StereoFrontend {
public:
  explicit StereoFrontend(StereoOptions options = {});
  StereoResult estimate(const cv::Mat &left, const cv::Mat &right,
                        const StereoCalibration &calibration) const;

private:
  StereoOptions options_;
};
Json evaluateStereo(const std::vector<DepthPoint> &points,
                    const cv::Mat1f &reference, const cv::Matx33d &K);
} // namespace thermalnav
