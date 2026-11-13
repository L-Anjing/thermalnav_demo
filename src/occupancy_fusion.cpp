#include "thermalnav/occupancy_fusion.hpp"
#include <algorithm>
#include <cmath>
#include <random>

namespace thermalnav {
namespace {
using Key = octomap::OcTreeKey;
struct KeyLess {
  bool operator()(const Key &a, const Key &b) const {
    for (int i = 0; i < 3; ++i) {
      if (a[i] < b[i])
        return true;
      if (a[i] > b[i])
        return false;
    }
    return false;
  }
};
bool valid(double z, double range) {
  return std::isfinite(z) && z >= .15 && z <= range;
}
bool stable(const cv::Mat1f &d, int x, int y, double range) {
  if (x < 1 || y < 1 || x >= d.cols - 1 || y >= d.rows - 1)
    return false;
  float lo = d(y, x), hi = lo;
  for (cv::Point o : {cv::Point(0, 0), {1, 0}, {-1, 0}, {0, 1}, {0, -1}}) {
    float z = d(y + o.y, x + o.x);
    if (!valid(z, range))
      return false;
    lo = std::min(lo, z);
    hi = std::max(hi, z);
  }
  return hi - lo < .08;
}
} // namespace
OccupancyFusion::OccupancyFusion(FusionOptions options)
    : options_(options), tree_(options.resolution) {
  if (!std::isfinite(options.resolution) || options.resolution < .02 ||
      !std::isfinite(options.floor) ||
      options.max_height <= options.min_height || options.max_range <= .15 ||
      options.max_extent <= 0 || options.max_nodes < 100)
    throw std::invalid_argument("invalid occupancy fusion options");
  tree_.setClampingThresMin(.12);
  tree_.setClampingThresMax(.97);
}
void OccupancyFusion::clear() {
  tree_.clear();
  ground_.clear();
  last_stamps_.clear();
}
Json OccupancyFusion::integrate(const std::vector<Observation> &observations,
                                BEV &bev) {
  if (std::abs(bev.res - options_.resolution) > 1e-8)
    throw std::invalid_argument("BEV/voxel resolution mismatch");
  if (tree_.size() > options_.max_nodes)
    throw std::runtime_error("voxel node limit exceeded");
  struct Evidence {
    float hit{}, miss{};
  };
  std::map<Key, Evidence, KeyLess> batch;
  Json sources = Json::object();
  size_t rejected = 0, ground_points=0, obstacle_points=0, ray_misses=0, blocked_rays=0;

  std::vector<cv::Vec3d> extent;
  for (const auto &observation : observations) {
    if (observation.source.empty() || !std::isfinite(observation.stamp) ||
        !cv::checkRange(cv::Mat(observation.R)) ||
        !cv::checkRange(cv::Mat(observation.origin)))
      throw std::invalid_argument("invalid observation pose/stamp");
    if (last_stamps_.count(observation.source) &&
        observation.stamp <= last_stamps_.at(observation.source))
      continue;
    if (std::abs(observation.origin[0]) > options_.max_extent ||
        std::abs(observation.origin[1]) > options_.max_extent)
      throw std::runtime_error("sensor outside bounded mapping volume");
    std::map<Key, std::pair<cv::Vec3d, float>, KeyLess> endpoints;
    int accepted = 0;
    for (const auto &sample : observation.points) {
      if (!cv::checkRange(cv::Mat(sample.p)) ||
          !std::isfinite(sample.confidence) ||
          !std::isfinite(sample.variance) || sample.confidence <= 0 ||
          sample.variance < 0 || sample.p[2] <= .15 ||
          cv::norm(sample.p) > options_.max_range * 1.6) {
        ++rejected;
        continue;
      }
      auto p = observation.R * sample.p + observation.origin;
      if (std::abs(p[0]) > options_.max_extent ||
          std::abs(p[1]) > options_.max_extent || p[2] < options_.floor - .2 ||
          p[2] > options_.floor + 2.5) {
        ++rejected;
        continue;
      }
      Key key;
      if (!tree_.coordToKeyChecked(octomap::point3d(p[0], p[1], p[2]), key)) {
        ++rejected;
        continue;
      }
      float weight = std::clamp(
          sample.confidence / (1.f + sample.variance / .04f), .05f, 1.f);
      auto it = endpoints.find(key);
      if (it == endpoints.end() || weight > it->second.second)
        endpoints[key] = {p, weight};
      double z_direction =
          (observation.R * cv::Vec3d(sample.p[0] / sample.p[2],
                                     sample.p[1] / sample.p[2], 1.))[2];
      if (sample.floor_reliable && sample.confidence >= .6 &&
          std::abs(p[2] - options_.floor) <= .06 &&
          std::abs(z_direction) * std::sqrt(sample.variance) <= .03) {
        auto cell = std::make_pair(int(std::floor(p[0] / options_.resolution)),
                                   int(std::floor(p[1] / options_.resolution)));
        ground_[cell] = std::max(ground_[cell], weight);
      }
      if (std::abs(p[2]-options_.floor)<=.06 && sample.floor_reliable) ++ground_points;
      if(p[2]>options_.floor+options_.min_height && p[2]<=options_.floor+options_.max_height) ++obstacle_points;
      ++accepted;
    }
    std::map<Key, Evidence, KeyLess> local;
    octomap::point3d origin(observation.origin[0], observation.origin[1],
                            observation.origin[2]);
    octomap::KeyRay ray;
    for (const auto &[key, endpoint] : endpoints) {
      const auto &p = endpoint.first;
      float weight = endpoint.second;
      local[key].hit = std::max(local[key].hit, weight);
      if (tree_.computeRayKeys(origin, octomap::point3d(p[0], p[1], p[2]), ray))
        for (const auto &k : ray) {
          auto w = tree_.keyToCoord(k);
          if (w.z() < options_.floor - .2 || w.z() > options_.floor + 2.5)
            continue;
          local[k].miss = std::max(local[k].miss, weight);
          ++ray_misses;
          // Stop at a measured surface or a previously reliable occupied voxel.
          // The blocker itself may receive clearing evidence over later frames;
          // its occluded continuation receives none in this frame.
          auto known=tree_.search(k);
          if(endpoints.count(k) || (known && known->getOccupancy()>=.65)) {
            ++blocked_rays;
            break;
          }
        }
    }
    // One update per source/voxel/frame. Multiple sources are capped below,
    // since RGB stereo and camera depth may have correlated errors.
    for (const auto &[key, e] : local) {
      batch[key].hit = std::min(1.f, batch[key].hit + e.hit);
      batch[key].miss = std::min(1.f, batch[key].miss + e.miss);
    }
    sources[observation.source] = {{"points", accepted},
                                   {"endpoint_voxels", endpoints.size()}};
    last_stamps_[observation.source] = observation.stamp;
    for (const auto &[key, p] : endpoints)
      extent.push_back(p.first);
  }
  if (!extent.empty())
    bev.expand(extent);
  size_t conflicts = 0;
  for (const auto &[key, e] : batch) {
    if (e.hit > 0 && e.miss > 0)
      ++conflicts;
    // Measured surfaces win within a batch; clearing requires subsequent
    // consistent free-space observations, never other pixels of this frame.
    tree_.updateNode(key, e.hit > 0 ? .85f * e.hit : -.4f * e.miss, true);
  }
  tree_.updateInnerOccupancy();
  auto previous=bev.occupancy();
  auto projection=project(bev);
  auto occ = bev.occupancy();
  return {{"sources", sources},{"projection",projection},
          {"updated_voxels", batch.size()},
          {"ground_points",ground_points},{"obstacle_points",obstacle_points},
          {"ray_free_voxel_visits",ray_misses},{"occlusion_stopped_rays",blocked_rays},
          {"new_free_cells",cv::countNonZero((occ==0)&(previous!=0))},
          {"new_occupied_cells",cv::countNonZero((occ==100)&(previous!=100))},
          {"free_to_unknown_cells",cv::countNonZero((previous==0)&(occ==255))},
          {"voxel_nodes", tree_.size()},
          {"conflicting_voxels", conflicts},
          {"rejected_points", rejected},
          {"ground_supported_columns", ground_.size()},
          {"collision_height_m", {options_.min_height, options_.max_height}},
          {"floor_height_m", options_.floor},
          {"free_cells", cv::countNonZero(occ == 0)},
          {"occupied_cells", cv::countNonZero(occ == 100)},
          {"observed_cells", cv::countNonZero(bev.seen)}};
}
Json OccupancyFusion::project(BEV &bev) const {
  // Occupied hysteresis: a single miss must not erase a reliable obstacle.
  // Retain it while its 3-D evidence is uncertain; repeated misses below the
  // existing free threshold clear it. An empty tree (epoch reset) clears all.
  cv::Mat1b previous_occupied=(bev.seen!=0)&(bev.log>=0);
  bev.log.setTo(0);
  bev.seen.setTo(0);
  bev.floor_evidence.setTo(0);
  for (auto it = tree_.begin_leafs(); it != tree_.end_leafs(); ++it) {
    if (it.getZ() < options_.floor + options_.min_height ||
        it.getZ() > options_.floor + options_.max_height)
      continue;
    auto cell = bev.cell({it.getX(), it.getY()});
    if (!bev.inside(cell)) continue;
    if(it->getOccupancy()<.65 && !(previous_occupied(cell) && it->getOccupancy()>=.45))continue;
    bev.log(cell) = 3;
    bev.seen(cell) = 255;
    bev.inferred(cell) = 0;
  }
  int missing_clearance=0,uncertain_clearance=0,occupied_ground=0,free_ground=0;
  for (const auto &[key, weight] : ground_) {
    XY p((key.first + .5) * options_.resolution,
         (key.second + .5) * options_.resolution);
    auto cell = bev.cell(p);
    if (!bev.inside(cell)) continue;
    bev.floor_evidence(cell)=255; // Ground evidence exists even if upper clearance is unknown.
    if (bev.log(cell) > 0) {++occupied_ground;continue;}
    bool all_free = true,missing=false,uncertain=false;
    int first = int(std::ceil(options_.min_height / options_.resolution - .5));
    int last =
        int(std::floor(options_.max_height / options_.resolution - .5 + 1e-6));
    for (int i = first; i <= last; ++i) {
      auto node = tree_.search(p.x, p.y,
                               options_.floor + (i + .5) * options_.resolution);
      if(!node){missing=true;all_free=false;}
      else if(node->getOccupancy()>.45){uncertain=true;all_free=false;}
    }
    if(missing)++missing_clearance;
    else if(uncertain)++uncertain_clearance;
    if (all_free) {
      ++free_ground;
      bev.log(cell) = -3;
      bev.seen(cell) = bev.floor_evidence(cell) = 255;
    }
  }
  return {{"ground_with_unobserved_clearance",missing_clearance},
          {"ground_with_uncertain_clearance",uncertain_clearance},
          {"ground_with_occupied_clearance",occupied_ground},
          {"camera_verified_free_columns",free_ground}};
}
Observation depthObservation(const cv::Mat1f &depth, const cv::Matx33d &K,
                             const cv::Matx33d &R, const cv::Vec3d &t,
                             double stamp, double max_range, double variance) {
  if (!std::isfinite(variance) || variance < 0)
    throw std::invalid_argument("invalid depth variance");
  if (depth.empty() || !cv::checkRange(cv::Mat(K)) || K(0, 0) <= 0 ||
      K(1, 1) <= 0)
    throw std::invalid_argument("invalid depth camera");
  Observation out;
  out.source = "depth";
  out.stamp = stamp;
  out.R = R;
  out.origin = t;
  out.points.reserve(depth.total());
  for (int y = 0; y < depth.rows; ++y)
    for (int x = 0; x < depth.cols; ++x) {
      double z = depth(y, x);
      if (!valid(z, max_range))
        continue;
      out.points.push_back(
          {{(x - K(0, 2)) * z / K(0, 0), (y - K(1, 2)) * z / K(1, 1), z},
           1.f,
           static_cast<float>(variance),
           stable(depth, x, y, max_range)});
    }
  return out;
}
cv::Mat1f degradeDepth(const cv::Mat1f &depth, double dropout,
                       double noise_sigma, double max_range, uint64_t seed) {
  if (!std::isfinite(dropout) || dropout < 0 || dropout > 1 ||
      !std::isfinite(noise_sigma) || noise_sigma < 0 ||
      !std::isfinite(max_range) || max_range <= 0)
    throw std::invalid_argument("invalid depth degradation");
  cv::Mat1f result = depth.clone();
  std::mt19937_64 random(seed);
  std::uniform_real_distribution<double> uniform;
  std::normal_distribution<double> noise(0, std::max(1e-12, noise_sigma));
  for (int y = 0; y < depth.rows; ++y)
    for (int x = 0; x < depth.cols; ++x) {
      float z = depth(y, x);
      if (!valid(z, max_range) || uniform(random) < dropout)
        result(y, x) = std::numeric_limits<float>::quiet_NaN();
      else {
        z += noise_sigma > 0 ? noise(random) : 0;
        result(y, x) =
            valid(z, max_range) ? z : std::numeric_limits<float>::quiet_NaN();
      }
    }
  return result;
}
Json compareOccupancy(const BEV &predicted, const BEV &reference) {
  auto truth = reference.occupancy(), estimate = predicted.occupancy();
  int known = 0, predicted_known = 0, tp = 0, fp = 0, fn = 0, false_free = 0;
  for (int y = 0; y < truth.rows; ++y)
    for (int x = 0; x < truth.cols; ++x) {
      if (truth(y, x) == 255)
        continue;
      ++known;
      auto cell = predicted.cell(reference.world({x, y}));
      int p = predicted.inside(cell) ? estimate(cell) : 255;
      predicted_known += p != 255;
      tp += p == 100 && truth(y, x) == 100;
      fp += p == 100 && truth(y, x) == 0;
      fn += p != 100 && truth(y, x) == 100;
      false_free += p == 0 && truth(y, x) == 100;
    }
  return {{"reference_known_cells", known},
          {"predicted_known_reference_cells", predicted_known},
          {"known_completeness",
           known ? Json(double(predicted_known) / known) : Json(nullptr)},
          {"occupied_precision",
           tp + fp ? Json(double(tp) / (tp + fp)) : Json(nullptr)},
          {"occupied_recall",
           tp + fn ? Json(double(tp) / (tp + fn)) : Json(nullptr)},
          {"false_free_cells", false_free}};
}
} // namespace thermalnav
