#include "thermalnav/core.hpp"
#include <cmath>
#include <opencv2/imgproc.hpp>

namespace thermalnav {
cv::Point Renderer::pixel(XY p, XY robot) const {
  XY d = (p - robot) * (pixels_ / meters_);
  return {cvRound(pixels_ / 2. + d.x), cvRound(pixels_ / 2. - d.y)};
}
cv::Mat1b Renderer::crop(const BEV &bev, const cv::Mat1b &grid,
                         XY robot) const {
  // Sample world-space cell centres, avoiding the old flipped half-cell offset.
  cv::Mat1b out(pixels_, pixels_, uint8_t(255));
  double scale = meters_ / pixels_;
  for (int y = 0; y < pixels_; ++y)
    for (int x = 0; x < pixels_; ++x) {
      auto c = bev.cell(robot + XY((x + .5 - pixels_ / 2.) * scale,
                                   (pixels_ / 2. - y - .5) * scale));
      if (bev.inside(c))
        out(y, x) = grid(c);
    }
  return out;
}
cv::Mat3b Renderer::render(const BEV &bev, XY robot, double yaw,
                           const std::vector<XY> &trajectory,
                           const std::vector<Candidate> &candidates,
                           std::optional<XY> selected,
                           const std::vector<Track> &tracks, double hfov,
                           double range) const {
  auto grid = crop(bev, bev.occupancy(), robot);
  cv::Mat3b rgb(pixels_, pixels_, cv::Vec3b(128, 128, 128));
  rgb.setTo(cv::Scalar(255, 255, 255), grid == 0);
  rgb.setTo(cv::Scalar(0, 0, 0), grid == 100);
  for (size_t i = 1; i < trajectory.size(); ++i)
    cv::line(rgb, pixel(trajectory[i - 1], robot), pixel(trajectory[i], robot),
             {255, 0, 0}, 2, cv::LINE_AA);
  for (const auto &c : candidates)
    cv::circle(rgb, pixel(c.xy, robot), 5,
               c.kind == "frontier" ? cv::Scalar(0, 200, 0)
                                    : cv::Scalar(0, 180, 220),
               -1, cv::LINE_AA);
  if (selected)
    cv::circle(rgb, pixel(*selected, robot), 8, {0, 255, 255}, 2, cv::LINE_AA);
  for (const auto &t : tracks)
    cv::circle(rgb, pixel(t.xy, robot), 5, {0, 145, 255}, -1, cv::LINE_AA);
  auto centre = pixel(robot, robot);
  for (double angle : {yaw - hfov * CV_PI / 360, yaw + hfov * CV_PI / 360})
    cv::line(rgb, centre,
             pixel(robot + range * XY(std::cos(angle), std::sin(angle)), robot),
             {0, 210, 255}, 1, cv::LINE_AA);
  cv::arrowedLine(
      rgb, centre,
      pixel(robot + (17 * meters_ / pixels_) * XY(std::cos(yaw), std::sin(yaw)),
            robot),
      {0, 0, 255}, 3, cv::LINE_AA, 0, .45);
  return rgb;
}
cv::Mat3b Renderer::coverage(const BEV &bev, XY robot,
                             const std::vector<Track> &tracks) const {
  auto grid = crop(bev, bev.coverage(tracks), robot);
  cv::Mat3b rgb(pixels_, pixels_, cv::Vec3b(128, 128, 128));
  rgb.setTo(cv::Scalar(40, 40, 40), grid == 0);
  rgb.setTo(cv::Scalar(0, 210, 210), grid == 50);
  rgb.setTo(cv::Scalar(0, 145, 255), grid == 75);
  rgb.setTo(cv::Scalar(0, 0, 0), grid == 100);
  return rgb;
}
} // namespace thermalnav
