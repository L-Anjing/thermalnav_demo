#include "thermalnav/core.hpp"

#include <algorithm>
#include <cmath>
#include <deque>
#include <limits>
#include <numeric>
#include <opencv2/imgproc.hpp>
#include <stdexcept>

namespace thermalnav {
namespace {
bool valid(float z, double maximum = 8.) {
  return std::isfinite(z) && z >= .15 && z <= maximum;
}
XY xy(const cv::Vec3d &p) { return {p[0], p[1]}; }
Json point(XY p) { return {p.x, p.y}; }
bool reliable(const cv::Mat1f &d, int x, int y, double maximum) {
  if (x < 1 || y < 1 || x >= d.cols - 1 || y >= d.rows - 1)
    return false;
  float lo = d(y, x), hi = lo;
  for (cv::Point off : {cv::Point(0, 0), {1, 0}, {-1, 0}, {0, 1}, {0, -1}}) {
    float z = d(y + off.y, x + off.x);
    if (!valid(z, maximum))
      return false;
    lo = std::min(lo, z);
    hi = std::max(hi, z);
  }
  return hi - lo < .08;
}
} // namespace

cv::Matx33d rotation(double x, double y, double z, double w) {
  double norm = std::sqrt(x * x + y * y + z * z + w * w);
  if (!std::isfinite(norm) || norm < 1e-8)
    throw std::invalid_argument("invalid quaternion");
  x /= norm;
  y /= norm;
  z /= norm;
  w /= norm;
  return {1 - 2 * (y * y + z * z), 2 * (x * y - z * w),
          2 * (x * z + y * w),     2 * (x * y + z * w),
          1 - 2 * (x * x + z * z), 2 * (y * z - x * w),
          2 * (x * z - y * w),     2 * (y * z + x * w),
          1 - 2 * (x * x + y * y)};
}

std::vector<cv::Point> ray(cv::Point a, cv::Point b) {
  std::vector<cv::Point> out;
  int dx = std::abs(b.x - a.x), dy = -std::abs(b.y - a.y);
  int sx = a.x < b.x ? 1 : -1, sy = a.y < b.y ? 1 : -1, err = dx + dy;
  for (;;) {
    out.push_back(a);
    if (a == b)
      break;
    int e = 2 * err;
    auto old = a;
    if (e >= dy) {
      err += dy;
      a.x += sx;
    }
    if (e <= dx) {
      err += dx;
      a.y += sy;
    }
    if (a.x != old.x && a.y != old.y) {
      out.emplace_back(old.x, a.y);
      out.emplace_back(a.x, old.y);
    }
  }
  return out;
}

BEV::BEV(double resolution, double size, XY map_origin, double radius)
    : res(resolution), footprint(radius), origin(map_origin) {
  if (!std::isfinite(res) || res < .02 || !std::isfinite(size) || size < res ||
      size / res > 2000 || !std::isfinite(radius) || radius <= 0 ||
      !std::isfinite(origin.x) || !std::isfinite(origin.y))
    throw std::invalid_argument("invalid map geometry");
  int n = static_cast<int>(std::ceil(size / res));
  log = cv::Mat1f::zeros(n, n);
  seen = cv::Mat1b::zeros(n, n);
  scanned = seen.clone();
  inferred = seen.clone();
  floor_evidence = seen.clone();
  traversed = seen.clone();
  visits = cv::Mat_<uint16_t>::zeros(n, n);
}
cv::Point BEV::cell(XY p) const {
  return {static_cast<int>(std::floor((p.x - origin.x) / res)),
          static_cast<int>(std::floor((p.y - origin.y) / res))};
}
XY BEV::world(cv::Point p) const {
  return origin + XY((p.x + .5) * res, (p.y + .5) * res);
}
bool BEV::inside(cv::Point p) const {
  return p.x >= 0 && p.y >= 0 && p.x < log.cols && p.y < log.rows;
}
void BEV::expand(const std::vector<cv::Vec3d> &points) {
  int left = 0, right = 0, top = 0, bottom = 0;
  for (const auto &p : points) {
    auto c = cell(xy(p));
    if (c.x < 0)
      left = std::max(left, -c.x + 10);
    if (c.y < 0)
      top = std::max(top, -c.y + 10);
    if (c.x >= log.cols)
      right = std::max(right, c.x + 11 - log.cols);
    if (c.y >= log.rows)
      bottom = std::max(bottom, c.y + 11 - log.rows);
  }
  if (!(left + right + top + bottom))
    return;
  if (log.cols + left + right > 2000 || log.rows + top + bottom > 2000)
    throw std::runtime_error("map size limit exceeded (2000 cells per axis)");
  for (cv::Mat *m : std::vector<cv::Mat *>{&log, &seen, &scanned, &inferred,
                                           &floor_evidence, &traversed, &visits}) {
    cv::Mat padded;
    cv::copyMakeBorder(*m, padded, top, bottom, left, right,
                       cv::BORDER_CONSTANT, 0);
    *m = padded;
  }
  origin -= XY(left * res, top * res);
}
cv::Mat1b BEV::occupancy() const {
  cv::Mat1b out(log.size(), uint8_t(255));
  out.setTo(0, traversed != 0);
  out.setTo(0, (seen != 0) & (log < 0));
  out.setTo(100, (seen != 0) & (log >= 0));
  return out;
}
cv::Mat1b BEV::coverage(const std::vector<Track> &targets) const {
  auto out = occupancy();
  out.setTo(50, (out == 0) & (scanned != 0));
  for (const auto &t : targets) {
    auto c = cell(t.xy);
    if (inside(c))
      out(c) = 75;
  }
  return out;
}

void BEV::integrate(const cv::Mat1f &depth, const cv::Matx33d &K,
                    const cv::Matx33d &R, const cv::Vec3d &t, XY robot,
                    double floor, int stride, double max_range) {
  if (depth.empty() || K(0, 0) <= 0 || K(1, 1) <= 0 || stride < 1 ||
      !cv::checkRange(cv::Mat(K)) || !cv::checkRange(cv::Mat(R)) ||
      !cv::checkRange(cv::Mat(t)))
    throw std::invalid_argument("invalid depth/camera geometry");
  std::vector<cv::Vec3d> floors, hits, stable, extent{t};
  int valid_count = 0;
  for (int v = 0; v < depth.rows; ++v)
    for (int u = 0; u < depth.cols; ++u) {
      double z = depth(v, u);
      if (!valid(z, max_range))
        continue;
      ++valid_count;
      cv::Vec3d p = R * cv::Vec3d((u - K(0, 2)) * z / K(0, 0),
                                  (v - K(1, 2)) * z / K(1, 1), z) +
                    t;
      double h = p[2] - floor;
      // Obstacles use EVERY pixel, including depth edges and one-pixel legs.
      bool obstacle = h > .12 && h < 1.8;
      if (obstacle) {
        hits.push_back(p);
        extent.push_back(p);
      }
      if (u % stride || v % stride || !reliable(depth, u, v, max_range))
        continue;
      bool ground = std::abs(h) <= .06;
      if (ground) {
        floors.push_back(p);
        extent.push_back(p);
      }
      if (ground || obstacle)
        stable.push_back(p);
    }
  expand(extent);
  cv::Mat1b hit = cv::Mat1b::zeros(log.size()), free = hit.clone();
  for (const auto &p : hits)
    hit(cell(xy(p))) = 255;
  for (const auto &p : floors)
    free(cell(xy(p))) = 255;
  // A 3D ray above the ground cannot certify its whole 2D shadow as free.
  // Only measured, reliable floor endpoints enter traversability evidence.
  free.setTo(0, (hit != 0) | ((seen != 0) & (log >= 0)));
  for (int y = 0; y < log.rows; ++y)
    for (int x = 0; x < log.cols; ++x) {
      if (free(y, x)) {
        log(y, x) = std::max(-4.f, log(y, x) - .55f);
        seen(y, x) = floor_evidence(y, x) = 255;
      }
      if (hit(y, x)) {
        log(y, x) = std::min(5.f, std::max(0.f, log(y, x)) + 1.4f);
        seen(y, x) = 255;
        floor_evidence(y, x) = inferred(y, x) = 0;
      }
    }
  nearfield_stats = complete(stable, robot);
  blind_zone = blindZone(K, R, t, depth.cols, depth.rows, robot, floor);
  observeCoverage(depth, K, R, t, floor, max_range);
  ++frames;
  auto occ = occupancy();
  diagnostics = {{"depth_pixels", depth.total()},
                 {"valid_depth", valid_count},
                 {"ground_points", floors.size()},
                 {"obstacle_points", hits.size()},
                 {"free_endpoints_in_map", floors.size()},
                 {"obstacle_endpoints_in_map", hits.size()},
                 {"ray_cleared_cells", 0},
                 {"observed_cells", cv::countNonZero(seen)},
                 {"free_cells", cv::countNonZero(occ == 0)},
                 {"occupied_cells", cv::countNonZero(occ == 100)},
                 {"scanned_cells", cv::countNonZero(scanned)},
                 {"occupancy_checksum", checksum(occ)},
                 {"sensor_origin", {t[0], t[1], t[2]}}};
}

void BEV::observeCoverage(const cv::Mat1f &depth, const cv::Matx33d &K,
                          const cv::Matx33d &R, const cv::Vec3d &t,
                          double floor, double max_range) {
  auto occ = occupancy();
  for (int y = 0; y < log.rows; ++y)
    for (int x = 0; x < log.cols; ++x) {
      if (occ(y, x) != 0)
        continue;
      XY w = world({x, y});
      cv::Vec3d c = R.t() * (cv::Vec3d(w.x, w.y, floor) - t);
      if (!valid(c[2], max_range))
        continue;
      int u = cvRound(K(0, 0) * c[0] / c[2] + K(0, 2)),
          v = cvRound(K(1, 1) * c[1] / c[2] + K(1, 2));
      if (u < 1 || v < 1 || u >= depth.cols - 1 || v >= depth.rows - 1)
        continue;
      float measured = std::numeric_limits<float>::infinity();
      bool all_valid = true;
      for (cv::Point d : {cv::Point(0, 0), {1, 0}, {-1, 0}, {0, 1}, {0, -1}}) {
        float z = depth(v + d.y, u + d.x);
        all_valid &= valid(z, max_range);
        measured = std::min(measured, z);
      }
      if (all_valid && c[2] <= measured + .04)
        scanned(y, x) = 255;
    }
}

Json BEV::complete(const std::vector<cv::Vec3d> &points, XY robot) {
  std::vector<std::vector<cv::Vec3d>> bins(120);
  for (const auto &p : points) {
    XY delta = xy(p) - robot;
    double range = cv::norm(delta);
    if (!std::isfinite(range) || range <= .05 || range > 2.)
      continue;
    int bin = static_cast<int>(
        std::floor((std::atan2(delta.y, delta.x) + CV_PI) / (CV_PI / 60)));
    bins[std::clamp(bin, 0, 119)].push_back(p);
  }
  auto occ = occupancy();
  int before = cv::countNonZero(inferred), count = 0;
  for (auto &bin : bins) {
    if (bin.size() < 3)
      continue;
    auto nearest =
        *std::min_element(bin.begin(), bin.end(), [&](auto a, auto b) {
          return cv::norm(xy(a) - robot) < cv::norm(xy(b) - robot);
        });
    if (std::count_if(bin.begin(), bin.end(),
                      [&](auto p) { return cv::norm(p - nearest) <= .12; }) < 3)
      continue;
    ++count;
    for (auto c : ray(cell(robot), cell(xy(nearest)))) {
      if (!inside(c) || occ(c) == 100)
        break;
      if (cv::norm(world(c) - robot) <= 2.)
        inferred(c) = 255;
    }
  }
  return {{"valid_bins", count},
          {"inferred_added", cv::countNonZero(inferred) - before},
          {"verified_free_promotions", 0}};
}
Json BEV::blindZone(const cv::Matx33d &K, const cv::Matx33d &R,
                    const cv::Vec3d &t, int width, int height, XY robot,
                    double floor) const {
  Json ranges = Json::array();
  for (double u : {0., K(0, 2), double(width - 1)}) {
    auto d = R * cv::Vec3d((u - K(0, 2)) / K(0, 0),
                           (height - 1 - K(1, 2)) / K(1, 1), 1.);
    if (d[2] < -1e-6 && t[2] > floor)
      ranges.push_back(cv::norm(xy(t + (floor - t[2]) / d[2] * d) - robot));
    else
      ranges.push_back(nullptr);
  }
  return {{"camera_height_m", t[2] - floor},
          {"hfov_deg", 180 / CV_PI *
                           (std::atan(K(0, 2) / K(0, 0)) +
                            std::atan((width - 1 - K(0, 2)) / K(0, 0)))},
          {"vfov_deg", 180 / CV_PI *
                           (std::atan(K(1, 2) / K(1, 1)) +
                            std::atan((height - 1 - K(1, 2)) / K(1, 1)))},
          {"floor_blind_left_center_right_m", ranges}};
}

// A point proof certifies only the actual measured pose, never future path
// centers. Store whole cells contained in that footprint; no interpolation.
Json BEV::recordVerifiedTraversal(const Json &proof, XY actual, double pose_stamp) {
  auto reject=[](const std::string &why){return Json{{"accepted",false},{"reason",why},{"added_cells",0}};};
  try {
    if(proof.at("mode")!="sim_only_safety_oracle" ||
       proof.at("provider")!="thermalnav_physx_startup_v1" || proof.at("frame")!="odom")
      return reject("wrong_provider_or_frame");
    if(proof.at("collision_free")!=true || proof.at("ground_supported")!=true ||
       proof.at("sim_pid").get<int>()<=0 || proof.at("query_count").get<int>()<9)
      return reject("failed_physics_validation");
    double stamp=proof.at("stamp"),radius=proof.at("radius");
    XY center(proof.at("xy").at(0),proof.at("xy").at(1));
    if(!std::isfinite(stamp)||!std::isfinite(pose_stamp)||std::abs(stamp-pose_stamp)>1e-5 ||
       !std::isfinite(center.x)||!std::isfinite(center.y)||!std::isfinite(actual.x)||!std::isfinite(actual.y)||
       cv::norm(center-actual)>.005 || !std::isfinite(radius)||radius<=0||radius>footprint+1e-9)
      return reject("unmatched_actual_pose_or_extent");
    int added=0,conflicts=0;
    auto lo=cell(center-XY(radius,radius)),hi=cell(center+XY(radius,radius));
    for(int y=lo.y;y<=hi.y;++y)for(int x=lo.x;x<=hi.x;++x){
      if(!inside({x,y}))continue;
      XY lower=origin+XY(x*res,y*res),upper=lower+XY(res,res);
      bool contained=true;
      for(XY corner:{lower,upper,XY(lower.x,upper.y),XY(upper.x,lower.y)})
        contained &= cv::norm(corner-center)<=radius+1e-9;
      if(!contained)continue;
      if(seen(y,x)&&log(y,x)>=0){++conflicts;continue;}
      if(!traversed(y,x)){traversed(y,x)=255;++added;}
    }
    return {{"accepted",true},{"mode","sim_only_safety_oracle"},{"stamp",stamp},
            {"added_cells",added},{"obstacle_conflicts",conflicts}};
  }catch(const Json::exception &){return reject("malformed_proof");}
}

bool BEV::acceptCertificate(const Json &proof, XY robot, double stamp) {
  auto reject = [&](const std::string &s) {
    certificate_rejection = s;
    return false;
  };
  if (certificate)
    return reject("already_certified");
  try {
    if (proof.at("provider") != "thermalnav_physx_startup_v1" ||
        proof.at("frame") != "odom")
      return reject("wrong_provider_or_frame");
    for (auto k : {"collision_free", "ground_supported", "initial_pose"})
      if (proof.at(k) != true)
        return reject("failed_physics_validation");
    if (!proof.at("xy").is_array() || proof.at("xy").size() != 2)
      return reject("invalid_radius_or_pose");
    XY p(proof.at("xy").at(0).get<double>(),
         proof.at("xy").at(1).get<double>());
    double r = proof.at("radius"),
           age = stamp - proof.at("stamp").get<double>();
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(r) ||
        r <= 0 || r > footprint)
      return reject("invalid_radius_or_pose");
    if (!std::isfinite(age) || age < -1e-6 || age > 3)
      return reject("stale_stamp");
    if (cv::norm(p - robot) > .04)
      return reject("mismatched_pose");
    if (proof.at("sim_pid").get<int>() <= 0 ||
        proof.at("query_count").get<int>() < 9)
      return reject("missing_validation_queries");
    certificate = proof;
    if (footprintReason(robot) == "occupied") {
      certificate.reset();
      return reject("observed_obstacle_conflict");
    }
    certificate_rejection.clear();
    return true;
  } catch (const Json::exception &) {
    return reject("malformed_proof");
  }
}
std::string BEV::footprintReason(XY p) const {
  if (!std::isfinite(p.x) || !std::isfinite(p.y))
    return "unknown_footprint";
  auto lo = cell(p - XY(footprint, footprint)),
       hi = cell(p + XY(footprint, footprint));
  XY cert;
  double radius = 0;
  if (certificate) {
    cert = {(*certificate)["xy"][0], (*certificate)["xy"][1]};
    radius = (*certificate)["radius"];
  }
  bool covered = certificate && cv::norm(p - cert) + footprint <= radius + 1e-9,
       unknown = false;
  for (int y = lo.y; y <= hi.y; ++y)
    for (int x = lo.x; x <= hi.x; ++x) {
      XY lower = origin + XY(x * res, y * res), upper = lower + XY(res, res);
      XY closest(std::clamp(p.x, lower.x, upper.x),
                 std::clamp(p.y, lower.y, upper.y));
      if (cv::norm(closest - p) >= footprint - 1e-9)
        continue;
      if (!inside({x, y})) {
        unknown = true;
        continue;
      }
      if (seen(y, x) && log(y, x) >= 0)
        return "occupied";
      if ((seen(y, x) && log(y, x) < 0) || traversed(y,x))
        continue;
      if (covered)
        continue;
      if (certificate) {
        bool square_covered = true;
        for (XY c : {lower, upper, XY(lower.x, upper.y), XY(upper.x, lower.y)})
          square_covered &= cv::norm(c - cert) <= radius;
        if (square_covered)
          continue;
      }
      unknown = true;
    }
  return unknown ? "unknown_footprint" : "";
}
std::string BEV::pathReason(const std::vector<XY> &path) const {
  if (path.empty())
    return "no_safe_path";
  for (size_t i = 1; i < path.size(); ++i) {
    int steps = std::max(
        1, int(std::ceil(cv::norm(path[i] - path[i - 1]) / (res / 2))));
    for (int j = 0; j <= steps; ++j) {
      auto reason = footprintReason(path[i - 1] + (path[i] - path[i - 1]) *
                                                      (double(j) / steps));
      if (!reason.empty())
        return reason;
    }
  }
  return footprintReason(path.back());
}

Json frontierClusterSnapshot(const cv::Mat &stats,const cv::Mat &centres,XY origin,double resolution) {
  Json out=Json::array();
  for(int i=1;i<stats.rows;++i){
    int cells=stats.at<int>(i,cv::CC_STAT_AREA);
    out.push_back({{"round_cluster_index",i},{"xy",{origin.x+(centres.at<double>(i,0)+.5)*resolution,origin.y+(centres.at<double>(i,1)+.5)*resolution}},
                   {"cells",cells},{"basic_filter",cells<4?"too_small":"passed"}});
  }
  return out;
}
std::vector<Candidate> BEV::candidates(XY robot, const std::vector<XY> &failed,
                                       const std::vector<Track> &targets,
                                       bool nearest_baseline) {
  auto occ = occupancy();
  cv::Mat1b boundary, adjacent;
  cv::dilate(occ == 255, adjacent, cv::Mat1b::ones(3, 3));
  boundary = (occ == 0) & (adjacent != 0);
  cv::Mat labels, stats, centres;
  int components =
      cv::connectedComponentsWithStats(boundary, labels, stats, centres, 8);
  frontier_components = components - 1;
  frontier_observations=frontierClusterSnapshot(stats,centres,origin,res);
  rejections = {{"unknown_footprint", 0}, {"occupied", 0},
                {"disconnected", 0},      {"no_safe_path", 0},
                {"too_small", 0},         {"repeat_visit", 0}};
  startup_check = footprintReason(robot);
  if (startup_check.empty())
    startup_check = "safe";
  auto start = cell(robot);
  if (startup_check != "safe" || !inside(start)) {
    rejections[startup_check == "safe" ? "no_safe_path" : startup_check] =
        frontier_components;
    return {};
  }
  cv::Mat1d dist(log.size(), -1.);
  cv::Mat_<cv::Vec2i> parent(log.size(), cv::Vec2i(-1, -1));
  cv::Mat1b blocked;
  int r = int(std::ceil(footprint / res));
  cv::dilate(
      occ != 0, blocked,
      cv::getStructuringElement(cv::MORPH_ELLIPSE, {2 * r + 1, 2 * r + 1}),
      {-1, -1}, 1, cv::BORDER_CONSTANT, 255);
  std::deque<cv::Point> queue{start};
  std::vector<cv::Point> reachable{start};
  dist(start) = 0;
  cv::Mat1b checked(log.size(), uint8_t(0)), safe(log.size(), uint8_t(0));
  auto is_safe = [&](cv::Point c) {
    if (!checked(c)) {
      safe(c) = footprintReason(world(c)).empty() ? 255 : 0;
      checked(c) = 255;
    }
    return safe(c) != 0;
  };
  while (!queue.empty()) {
    auto a = queue.front();
    queue.pop_front();
    XY aw = a == start ? robot : world(a);
    for (cv::Point d : {cv::Point(1, 0), {-1, 0}, {0, 1}, {0, -1}}) {
      auto b = a + d;
      if (!inside(b) || dist(b) >= 0 || occ(b) != 0)
        continue;
      if (blocked(b) && !is_safe(b))
        continue;
      if ((a == start || blocked(a) || blocked(b)) &&
          !pathReason({aw, world(b)}).empty())
        continue;
      dist(b) = dist(a) + res;
      parent(b) = {a.x, a.y};
      queue.push_back(b);
      reachable.push_back(b);
    }
  }
  std::vector<Candidate> out;
  auto add = [&](const std::string &kind, XY anchor, double gain,
                 int target_id = -1) {
    auto order = reachable;
    std::sort(order.begin(), order.end(), [&](auto a, auto b) {
      return cv::norm(XY(a) - anchor) < cv::norm(XY(b) - anchor);
    });
    const size_t limit=nearest_baseline && kind=="inspection_viewpoint" ? order.size() : std::min(size_t(120),order.size());
    for (size_t j = 0; j < limit; ++j) {
      auto c = order[j];
      XY w = world(c);
      if(nearest_baseline && kind=="inspection_viewpoint") {
        double observation_distance=cv::norm(XY(c)-anchor)*res;
        if(observation_distance<.9 || observation_distance>6.)continue;
        bool visible=true;
        for(auto v:ray(c,{cvRound(anchor.x),cvRound(anchor.y)}))
          if(!inside(v) || occ(v)!=0){visible=false;break;}
        if(!visible)continue;
      }
      double cost = dist(c);
      if (cost < .85)
        continue;
      if (std::any_of(failed.begin(), failed.end(),
                      [&](XY f) { return cv::norm(w - f) < 1.; })) {
        rejections["repeat_visit"] = rejections["repeat_visit"].get<int>() + 1;
        continue;
      }
      if (cv::norm(XY(c) - anchor) * res > (nearest_baseline && kind=="inspection_viewpoint"?6.:1.8)) {
        rejections["disconnected"] = rejections["disconnected"].get<int>() + 1;
        return;
      }
      std::vector<XY> path;
      auto p = c;
      for (;;) {
        path.push_back(world(p));
        auto prev = parent(p);
        if (prev[0] < 0)
          break;
        p = {prev[0], prev[1]};
      }
      std::reverse(path.begin(), path.end());
      path.front() = robot;
      auto reason = pathReason(path);
      if (!reason.empty()) {
        rejections[reason] = rejections[reason].get<int>() + 1;
        continue;
      }
      double score =
          gain / (1 + cost) - 2 * visits(c) + (kind == "verify_target" ? 8 : 0);
      out.push_back({kind + "_" + std::to_string(out.size()), kind, w, path,
                     gain, cost, score, target_id});
      return;
    }
    rejections["no_safe_path"] = rejections["no_safe_path"].get<int>() + 1;
  };
  for (int lab = 1; lab < components; ++lab) {
    if (stats.at<int>(lab, cv::CC_STAT_AREA) < 4) {
      rejections["too_small"] = rejections["too_small"].get<int>() + 1;
      continue;
    }
    XY anchor(centres.at<double>(lab, 0), centres.at<double>(lab, 1));
    int cx = int(anchor.x), cy = int(anchor.y), rad = 15;
    cv::Rect roi(std::max(0, cx - rad), std::max(0, cy - rad), 0, 0);
    roi.width = std::min(occ.cols, cx + rad + 1) - roi.x;
    roi.height = std::min(occ.rows, cy + rad + 1) - roi.y;
    add("frontier", anchor, nearest_baseline ? 0. : cv::countNonZero(occ(roi) == 255) * res * res);
  }
  const bool have_frontier = !out.empty();
  int n = cv::connectedComponentsWithStats((occ == 0) & (scanned == 0), labels,
                                           stats, centres, 8);
  for (int lab = 1; lab < n; ++lab)
    if ((!nearest_baseline || !have_frontier) && stats.at<int>(lab, cv::CC_STAT_AREA) >= 12)
      add("inspection_viewpoint",
          {centres.at<double>(lab, 0), centres.at<double>(lab, 1)},
          nearest_baseline ? 0. : stats.at<int>(lab, cv::CC_STAT_AREA) * res * res);
  for (const auto &t : targets)
    if (!nearest_baseline && !t.verified)
      add("verify_target", XY(cell(t.xy)), 4., t.id);
  std::stable_sort(out.begin(), out.end(), [nearest_baseline](const auto &a, const auto &b) {
    return nearest_baseline ? a.cost < b.cost : a.score > b.score;
  });
  if (out.size() > 24)
    out.resize(24);
  return out;
}
uint64_t BEV::checksum(const cv::Mat1b &grid) const {
  uint64_t hash = 14695981039346656037ULL;
  for (int y = 0; y < grid.rows; ++y)
    for (int x = 0; x < grid.cols; ++x) {
      hash ^= grid(y, x);
      hash *= 1099511628211ULL;
    }
  return hash;
}
void addTrack(std::vector<Track> &tracks, XY p, double stamp) {
  for (auto &t : tracks)
    if (cv::norm(t.xy - p) < .8) {
      if (stamp - t.last < .2)
        return;
      t.xy = .7 * t.xy + .3 * p;
      ++t.hits;
      t.last = stamp;
      t.verified = t.hits >= 3 && stamp - t.first >= 1.;
      return;
    }
  tracks.push_back({int(tracks.size()), 1, p, stamp, stamp, false});
}
Json trackJson(const std::vector<Track> &tracks) {
  Json out = Json::array();
  for (const auto &t : tracks)
    out.push_back({{"id", t.id},
                   {"label", "person"},
                   {"xy", point(t.xy)},
                   {"hits", t.hits},
                   {"first", t.first},
                   {"last", t.last},
                   {"verified", t.verified}});
  return out;
}
std::optional<cv::Vec3d> localizeBox(const cv::Rect2d &box,
                                     const cv::Mat1f &depth,
                                     const cv::Matx33d &K, const cv::Matx33d &R,
                                     const cv::Vec3d &t) {
  int xa = std::max(0, int(box.x + .3 * box.width)),
      xb = std::min(depth.cols, int(box.x + .7 * box.width));
  int ya = std::max(0, int(box.y + .3 * box.height)),
      yb = std::min(depth.rows, int(box.y + .7 * box.height));
  std::vector<float> vals;
  for (int y = ya; y < yb; ++y)
    for (int x = xa; x < xb; ++x)
      if (valid(depth(y, x)))
        vals.push_back(depth(y, x));
  if (vals.size() < 6)
    return std::nullopt;
  std::sort(vals.begin(), vals.end());
  size_t n = vals.size();
  double z = n % 2 ? vals[n / 2] : (vals[n / 2] + vals[n / 2 - 1]) * .5;
  return R * cv::Vec3d(((xa + xb) * .5 - K(0, 2)) * z / K(0, 0),
                       ((ya + yb) * .5 - K(1, 2)) * z / K(1, 1), z) +
         t;
}
} // namespace thermalnav
