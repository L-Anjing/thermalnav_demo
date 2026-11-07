#pragma once

#include <nlohmann/json.hpp>
#include <opencv2/core.hpp>
#include <optional>
#include <string>
#include <vector>

namespace thermalnav {
using Json = nlohmann::json;
using XY = cv::Point2d;
struct Candidate {
  std::string id, kind;
  XY xy;
  std::vector<XY> path;
  double gain{}, cost{}, score{};
  int target_id{-1};
};
struct Track {
  int id{}, hits{1};
  XY xy;
  double first{}, last{};
  bool verified{false};
};
Json trackJson(const std::vector<Track> &tracks);
void addTrack(std::vector<Track> &tracks, XY xy, double stamp);
cv::Matx33d rotation(double x, double y, double z, double w);
std::vector<cv::Point> ray(cv::Point a, cv::Point b);
std::optional<cv::Vec3d> localizeBox(const cv::Rect2d &box,
                                     const cv::Mat1f &depth,
                                     const cv::Matx33d &K, const cv::Matx33d &R,
                                     const cv::Vec3d &t);

class BEV {
public:
  explicit BEV(double resolution = .1, double size = 30.,
               XY origin = {-15., -15.}, double footprint = .35);
  cv::Point cell(XY xy) const;
  XY world(cv::Point p) const;
  bool inside(cv::Point p) const;
  void expand(const std::vector<cv::Vec3d> &points);
  cv::Mat1b occupancy()
      const; // 255 unknown, 0 free, 100 occupied; ROS conversion at boundary.
  cv::Mat1b coverage(const std::vector<Track> &targets = {}) const;
  void integrate(const cv::Mat1f &depth, const cv::Matx33d &K,
                 const cv::Matx33d &R, const cv::Vec3d &t, XY robot,
                 double floor = 0., int stride = 2, double max_range = 8.);
  Json complete(const std::vector<cv::Vec3d> &points, XY robot);
  Json blindZone(const cv::Matx33d &K, const cv::Matx33d &R, const cv::Vec3d &t,
                 int width, int height, XY robot, double floor) const;
  bool acceptCertificate(const Json &proof, XY robot, double stamp);
  std::string footprintReason(XY xy) const;
  std::string pathReason(const std::vector<XY> &path) const;
  std::vector<Candidate> candidates(XY robot, const std::vector<XY> &failed,
                                    const std::vector<Track> &targets);
  uint64_t checksum(const cv::Mat1b &grid) const;

  double res, footprint;
  XY origin;
  cv::Mat1f log;
  cv::Mat1b seen, scanned, inferred, floor_evidence;
  cv::Mat_<uint16_t> visits;
  uint64_t frames{};
  std::optional<Json> certificate;
  std::string certificate_rejection, startup_check;
  Json diagnostics = Json::object(), nearfield_stats = Json::object(),
       blind_zone = Json::object();
  Json rejections = Json::object();
  int frontier_components{};
};

class Renderer {
public:
  explicit Renderer(int pixels = 448, double meters = 22.4)
      : pixels_(pixels), meters_(meters) {}
  cv::Point pixel(XY xy, XY robot) const;
  cv::Mat3b render(const BEV &bev, XY robot, double yaw,
                   const std::vector<XY> &trajectory,
                   const std::vector<Candidate> &candidates,
                   std::optional<XY> selected, const std::vector<Track> &tracks,
                   double hfov = 47., double range = 8.) const;
  cv::Mat3b coverage(const BEV &bev, XY robot,
                     const std::vector<Track> &tracks) const;

private:
  cv::Mat1b crop(const BEV &bev, const cv::Mat1b &grid, XY robot) const;
  int pixels_;
  double meters_;
};
} // namespace thermalnav
