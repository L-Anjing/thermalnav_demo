#pragma once
#include "thermalnav/core.hpp"
#include <opencv2/dnn.hpp>
#include <opencv2/objdetect.hpp>

namespace thermalnav {
class Detector {
public:
  Detector();
  std::vector<cv::Rect2d> detect(const cv::Mat3b &bgr);
  std::string name;

private:
  cv::HOGDescriptor hog_;
  cv::dnn::Net net_;
};
struct PolicyResult {
  Candidate candidate;
  std::string status;
  bool called{false};
};
class Policy {
public:
  explicit Policy(const std::string &mode);
  bool remote() const;
  PolicyResult choose(const std::vector<Candidate> &candidates,
                      const cv::Mat3b &bev, const cv::Mat3b &rgb,
                      const Json &context) const;
  std::string mode, status;
  uint64_t calls{};

private:
  std::string url_, model_, key_;
};
} // namespace thermalnav
