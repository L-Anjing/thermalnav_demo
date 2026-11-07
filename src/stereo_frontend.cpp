#include "thermalnav/stereo_frontend.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <opencv2/imgproc.hpp>

namespace thermalnav {
namespace {
cv::Mat1b gray(const cv::Mat &input) {
  if (input.type() == CV_8UC1)
    return input.clone();
  if (input.type() != CV_8UC3)
    throw std::invalid_argument("stereo expects mono8 or BGR8");
  cv::Mat1b result;
  cv::cvtColor(input, result, cv::COLOR_BGR2GRAY);
  return result;
}
double correlation(const cv::Mat1b &a, const cv::Mat1b &b, int ax, int bx,
                   int y, int radius) {
  double sa = 0, sb = 0, aa = 0, bb = 0, ab = 0;
  int n = 0;
  for (int v = y - radius; v <= y + radius; ++v)
    for (int off = -radius; off <= radius; ++off) {
      double x = a(v, ax + off), z = b(v, bx + off);
      sa += x;
      sb += z;
      aa += x * x;
      bb += z * z;
      ab += x * z;
      ++n;
    }
  aa -= sa * sa / n;
  bb -= sb * sb / n;
  if (aa < n * 9 || bb < n * 9)
    return -1.;
  return (ab - sa * sb / n) / std::sqrt(aa * bb);
}
struct Match {
  double disparity{}, score{}, margin{}, curvature{};
  bool valid{false};
};
Match search(const cv::Mat1b &source, const cv::Mat1b &target, int u, int v,
             int lo, int hi, int direction, const StereoOptions &options) {
  std::vector<double> scores(hi + 1, -2.);
  int best = -1;
  for (int d = lo; d <= hi; ++d) {
    int x = u + direction * d;
    if (x < options.radius || x >= target.cols - options.radius)
      continue;
    scores[d] = correlation(source, target, u, x, v, options.radius);
    if (best < 0 || scores[d] > scores[best])
      best = d;
  }
  if (best < lo || scores[best] < options.min_correlation)
    return {};
  double second = -1.;
  for (int d = lo; d <= hi; ++d)
    if (std::abs(d - best) > 2)
      second = std::max(second, scores[d]);
  double margin = scores[best] - second;
  if (margin < options.uniqueness)
    return {};
  double delta = 0, curvature = 0;
  if (best > lo && best < hi && scores[best - 1] > -1 &&
      scores[best + 1] > -1) {
    curvature = 2 * scores[best] - scores[best - 1] - scores[best + 1];
    if (curvature > 1e-6)
      delta = std::clamp(.5 * (scores[best + 1] - scores[best - 1]) / curvature,
                         -.5, .5);
  }
  return {best + delta, scores[best], margin, curvature, true};
}
} // namespace

StereoFrontend::StereoFrontend(StereoOptions options) : options_(options) {
  if (options.radius < 2 || options.max_features < 1 ||
      options.min_range <= 0 || options.max_range <= options.min_range ||
      options.min_correlation <= 0 || options.min_correlation > 1 ||
      options.uniqueness <= 0 || options.max_sigma_z <= 0)
    throw std::invalid_argument("invalid stereo options");
}
StereoResult StereoFrontend::estimate(const cv::Mat &left, const cv::Mat &right,
                                      const StereoCalibration &c) const {
  if (left.empty() || left.size() != c.size || right.size() != c.size ||
      c.size.width < 32 || c.size.height < 24 ||
      !cv::checkRange(cv::Mat(c.left_K)) ||
      !cv::checkRange(cv::Mat(c.right_K)) || c.left_K(0, 0) <= 0 ||
      c.left_K(1, 1) <= 0 || c.right_K(0, 0) <= 0 || c.right_K(1, 1) <= 0 ||
      !cv::checkRange(cv::Mat(c.right_from_left_R)) ||
      !cv::checkRange(cv::Mat(c.right_from_left_t)) ||
      std::abs(cv::determinant(cv::Mat(c.right_from_left_R)) - 1) > 1e-5 ||
      cv::norm(cv::Mat(c.right_from_left_R.t() * c.right_from_left_R -
                       cv::Matx33d::eye())) > 1e-5)
    throw std::invalid_argument("invalid stereo images/calibration");
  double baseline = cv::norm(c.right_from_left_t);
  if (baseline < .02 || baseline > .5)
    throw std::invalid_argument("invalid stereo baseline");
  StereoResult out;
  cv::Mat Q;
  cv::Rect roi1, roi2;
  cv::stereoRectify(c.left_K, c.left_D, c.right_K, c.right_D, c.size,
                    c.right_from_left_R, c.right_from_left_t, out.left_R,
                    out.right_R, out.left_P, out.right_P, Q,
                    cv::CALIB_ZERO_DISPARITY, 0, c.size, &roi1, &roi2);
  // This frontend uses a horizontal pair with positive left-minus-right
  // disparity.
  double fb = -out.right_P(0, 3);
  if (fb <= 0 || std::abs(out.right_P(1, 3)) > 1e-6)
    throw std::invalid_argument("expected horizontal left/right order");
  cv::Mat m1, m2, n1, n2;
  cv::initUndistortRectifyMap(c.left_K, c.left_D, out.left_R, out.left_P,
                              c.size, CV_32FC1, m1, m2);
  cv::initUndistortRectifyMap(c.right_K, c.right_D, out.right_R, out.right_P,
                              c.size, CV_32FC1, n1, n2);
  cv::remap(gray(left), out.left_gray, m1, m2, cv::INTER_LINEAR);
  cv::remap(gray(right), out.right_gray, n1, n2, cv::INTER_LINEAR);
  out.depth = cv::Mat1f(c.size, std::numeric_limits<float>::quiet_NaN());
  out.confidence = cv::Mat1f::zeros(c.size);
  cv::Mat1b mask = cv::Mat1b::zeros(c.size);
  mask(roi1 & roi2).setTo(255);
  std::vector<cv::Point2f> corners;
  cv::goodFeaturesToTrack(out.left_gray, corners, options_.max_features, .01, 7,
                          mask);
  std::map<std::pair<int, int>, int> tiles;
  int lo = std::max(1, int(std::floor(fb / options_.max_range)) - 1),
      hi = std::min(c.size.width - 2 * options_.radius - 1,
                    int(std::ceil(fb / options_.min_range)) + 1);
  cv::cvtColor(out.left_gray, out.matches, cv::COLOR_GRAY2BGR);
  int rejected_lr = 0, rejected_range = 0;
  for (const auto &corner : corners) {
    int u = cvRound(corner.x), v = cvRound(corner.y), r = options_.radius;
    if (u < r || u >= c.size.width - r || v < r || v >= c.size.height - r ||
        tiles[{u / 80, v / 60}] >= 24)
      continue;
    auto match =
        search(out.left_gray, out.right_gray, u, v, lo, hi, -1, options_);
    if (!match.valid)
      continue;
    int ur = cvRound(u - match.disparity);
    if (ur < r || ur >= c.size.width - r)
      continue;
    auto reverse =
        search(out.right_gray, out.left_gray, ur, v, lo, hi, 1, options_);
    if (!reverse.valid ||
        std::abs(ur + reverse.disparity - u) > options_.max_lr_error) {
      ++rejected_lr;
      continue;
    }
    double z = fb / match.disparity;
    double sigma_d = std::clamp(
        .25 + (1 - match.score) + .1 / std::max(.1, match.curvature), .25, 1.);
    double sigma_z = z * z / fb * sigma_d;
    if (z < options_.min_range || z > options_.max_range ||
        sigma_z > options_.max_sigma_z) {
      ++rejected_range;
      continue;
    }
    cv::Vec3d rect((u - out.left_P(0, 2)) * z / out.left_P(0, 0),
                   (v - out.left_P(1, 2)) * z / out.left_P(1, 1), z);
    cv::Vec3d p = out.left_R.t() * rect;
    if (p[2] <= 0)
      continue;
    auto pixel = c.left_K * p;
    int x = cvRound(pixel[0] / pixel[2]), y = cvRound(pixel[1] / pixel[2]);
    if (x < 0 || y < 0 || x >= c.size.width || y >= c.size.height)
      continue;
    float confidence =
        std::clamp(float(match.score * std::min(1., match.margin / .15) /
                         (1 + sigma_z / .1)),
                   .1f, 1.f);
    out.points.push_back({p, confidence, float(sigma_z * sigma_z), true});
    out.depth(y, x) = p[2];
    out.confidence(y, x) = confidence;
    ++tiles[{u / 80, v / 60}];
    cv::circle(out.matches, {u, v}, 2, {0, 255, 0}, -1);
    cv::line(out.matches, {u, v}, {ur, v}, {0, 180, 255}, 1);
  }
  out.diagnostics = {
      {"features", corners.size()},
      {"accepted", out.points.size()},
      {"rejected_lr", rejected_lr},
      {"rejected_range", rejected_range},
      {"baseline_m", baseline},
      {"valid_fraction", double(out.points.size()) / left.total()}};
  return out;
}
Json evaluateStereo(const std::vector<DepthPoint> &points,
                    const cv::Mat1f &reference, const cv::Matx33d &K) {
  std::vector<double> error, relative;
  double squared = 0;
  for (const auto &p : points) {
    auto uv = K * p.p;
    if (uv[2] <= 0)
      continue;
    int x = cvRound(uv[0] / uv[2]), y = cvRound(uv[1] / uv[2]);
    if (x < 0 || y < 0 || x >= reference.cols || y >= reference.rows)
      continue;
    double gt = reference(y, x);
    if (!std::isfinite(gt) || gt <= .15)
      continue;
    double e = std::abs(p.p[2] - gt);
    error.push_back(e);
    relative.push_back(e / gt);
    squared += e * e;
  }
  if (error.empty())
    return {{"compared_points", 0},
            {"mae_m", nullptr},
            {"median_relative_error", nullptr},
            {"p95_relative_error", nullptr}};
  std::sort(error.begin(), error.end());
  std::sort(relative.begin(), relative.end());
  double sum = 0;
  for (double e : error)
    sum += e;
  return {
      {"compared_points", error.size()},
      {"mae_m", sum / error.size()},
      {"rmse_m", std::sqrt(squared / error.size())},
      {"median_relative_error", relative[relative.size() / 2]},
      {"p95_relative_error", relative[size_t(.95 * (relative.size() - 1))]}};
}
} // namespace thermalnav
