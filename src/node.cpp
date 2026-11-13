#include "thermalnav/core.hpp"
#include "thermalnav/occupancy_fusion.hpp"
#include "thermalnav/perception.hpp"
#include "thermalnav/ros_observation.hpp"
#include <chrono>
#include <csignal>
#include <cv_bridge/cv_bridge.h>
#include <deque>
#include <future>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <iostream>
#include <limits>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <opencv2/imgproc.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <thread>
#include <visualization_msgs/msg/marker_array.hpp>

namespace thermalnav {
using Image = sensor_msgs::msg::Image;
using Odom = nav_msgs::msg::Odometry;
using Pose = geometry_msgs::msg::PoseStamped;
using Grid = nav_msgs::msg::OccupancyGrid;
using Trigger = std_srvs::srv::Trigger;
using Clock = std::chrono::steady_clock;
double wall() {
  return std::chrono::duration<double>(Clock::now().time_since_epoch()).count();
}
double stamp(const std_msgs::msg::Header &h) {
  return h.stamp.sec + h.stamp.nanosec * 1e-9;
}
Json pointJson(XY p) { return {p.x, p.y}; }

class Demo : public rclcpp::Node {
public:
  Demo()
      : Node("thermalnav_demo"), tf_(get_clock()),
        listener_(std::make_unique<tf2_ros::TransformListener>(tf_)),
        policy_(declare_parameter<std::string>("mode", "heuristic")) {
    enabled_ = !declare_parameter<bool>("observe", false);
    external_perception_ = declare_parameter<bool>("external_perception", false);
    external_manager_ = declare_parameter<bool>("external_manager", false);
    if(external_manager_){enabled_=false;state_="MAPPING_ONLY";}
    traversal_mode_=declare_parameter<std::string>("verified_traversal_mode","disabled");
    if(traversal_mode_!="disabled" && traversal_mode_!="sim_only_safety_oracle")
      throw std::invalid_argument("invalid verified_traversal_mode");
    floor_ = declare_parameter<double>("floor_height", 0.);
    stride_ = declare_parameter<int>("floor_stride", 2);
    max_range_ = declare_parameter<double>("max_range", 8.);
    reset_map_ = declare_parameter<bool>("reset_map_on_time_jump", true);
    mapping_source_ =
        declare_parameter<std::string>("mapping_source", "depth_only");
    dropout_ = declare_parameter<double>("mapping_depth_dropout", 0.);
    noise_ = declare_parameter<double>("mapping_depth_noise_sigma", 0.);
    mapping_range_ = declare_parameter<double>("mapping_depth_max_range", 8.);
    seed_ = declare_parameter<int>("mapping_seed", 42);
    hole_fraction_ =
        declare_parameter<double>("mapping_depth_hole_fraction", 0.);
    if (!std::isfinite(hole_fraction_) || hole_fraction_ < 0 ||
        hole_fraction_ > 1)
      throw std::invalid_argument("invalid depth hole fraction");
    if (mapping_source_ != "depth_only" && mapping_source_ != "stereo_only" &&
        mapping_source_ != "fused")
      throw std::invalid_argument("invalid mapping_source");
    degradeDepth(cv::Mat1f(1, 1, 1.f), dropout_, noise_, mapping_range_, seed_);
    FusionOptions options;
    options.floor = floor_;
    options.max_range = max_range_;
    options.max_height = declare_parameter<double>("collision_height", .65);
    fusion_ = std::make_unique<OccupancyFusion>(options);

    if (stride_ < 1 || !std::isfinite(floor_) || !std::isfinite(max_range_) ||
        max_range_ <= .15 || max_range_ > 30)
      throw std::invalid_argument("invalid mapper parameters");
    depth_topic_ = declare_parameter<std::string>("mapping_depth_topic",
                                                  "/robot0/front_cam/depth");
    auto depth_info_topic = declare_parameter<std::string>(
        "mapping_info_topic", "/robot0/front_cam/camera_info");
    auto stereo_info_topic = declare_parameter<std::string>(
        "stereo_info_topic", "/robot0/front_cam/camera_info");
    auto retained = rclcpp::QoS(1).transient_local().reliable();
    map_pub_ = create_publisher<Grid>("/thermalnav/bev", retained);
    cover_pub_ = create_publisher<Grid>("/thermalnav/coverage", retained);
    scan_mask_pub_ = create_publisher<Grid>("/thermalnav/scan_mask", retained);
    explored_pub_ = create_publisher<Grid>("/thermalnav/explored", retained);
    inferred_pub_ = create_publisher<Grid>("/thermalnav/inferred", retained);
    traversed_pub_ = create_publisher<Grid>("/thermalnav/verified_traversed", retained);
    camera_free_pub_ = create_publisher<Grid>("/thermalnav/camera_observed_free", retained);
    render_pub_ = create_publisher<Image>("/thermalnav/bev/rendered", retained);
    cover_render_pub_ =
        create_publisher<Image>("/thermalnav/bev/coverage_rendered", retained);
    marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
        "/thermalnav/markers", retained);
    path_pub_ = create_publisher<nav_msgs::msg::Path>("/thermalnav/trajectory",
                                                      retained);
    status_pub_ =
        create_publisher<std_msgs::msg::String>("/thermalnav/status", retained);
    goal_pub_ = create_publisher<Pose>("/goal_pose", 1);
    auto sensor = rclcpp::SensorDataQoS().keep_last(5);
    if(external_perception_) {
      thermal_info_sub_=create_subscription<sensor_msgs::msg::CameraInfo>("/thermalnav/thermal/camera_info",rclcpp::QoS(1).transient_local().reliable(),[this](sensor_msgs::msg::CameraInfo::ConstSharedPtr m){thermal_info_=m;});
      thermal_depth_sub_=create_subscription<Image>("/thermalnav/thermal/depth",rclcpp::QoS(5).reliable(),[this](Image::ConstSharedPtr m){thermal_depth_=m;});
      thermal_tracks_sub_=create_subscription<std_msgs::msg::String>("/thermalnav/thermal/detections",retained,[this](std_msgs::msg::String::ConstSharedPtr m){
        try{auto records=Json::parse(m->data);std::vector<Track> next;
          for(const auto &r:records){next.push_back({r.at("id"),r.at("hits"),{r.at("xy")[0],r.at("xy")[1]},r.at("first"),r.at("last"),r.at("verified")});}
          tracks_=std::move(next);
        }catch(const Json::exception &e){RCLCPP_WARN(get_logger(),"Thermal record rejected: %s",e.what());}
      });
    }
    voxel_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        "/thermalnav/occupied_voxels", retained);
    obstacle_voxel_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        "/thermalnav/obstacle_voxels", retained);
    sparse_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        "/thermalnav/stereo/points", sensor,
        [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud) {
          double time = stamp(cloud->header);
          if (time < last_stamp_ - .5)
            resetEpoch(time);
          sparse_frames_.push_back(cloud);
          if (sparse_frames_.size() > 12)
            sparse_frames_.pop_front();
        });

    info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
        depth_info_topic, sensor,
        [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr m) {
          cv::Matx33d K;
          std::copy(m->k.begin(), m->k.end(), K.val);
          if (m->header.frame_id.empty() || !cv::checkRange(cv::Mat(K)) ||
              K(0, 0) <= 0 || K(1, 1) <= 0 || !m->width || !m->height) {
            blocked_ = "invalid CameraInfo";
            return;
          }
          info_ = m;
          K_ = K;
        });
    stereo_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
        stereo_info_topic, sensor,
        [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr m) {
          stereo_info_ = m;
        });
    if(!external_perception_) {
    detection_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
        "/robot0/front_cam/camera_info", sensor,
        [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr m) {
          detection_info_ = m;
        });
    detection_depth_sub_ = create_subscription<Image>(
        "/robot0/front_cam/depth", sensor,
        [this](Image::ConstSharedPtr m) { detection_depth_ = m; });
    rgb_sub_ = create_subscription<Image>("/robot0/front_cam/rgb", sensor,
                                          [this](Image::ConstSharedPtr m) {
                                            rgbs_.push_back(m);
                                            if (rgbs_.size() > 15)
                                              rgbs_.pop_front();
                                          });
    }
    depth_sub_ = create_subscription<Image>(
        depth_topic_, rclcpp::QoS(5).reliable(), [this](Image::ConstSharedPtr m) {
          double s = stamp(m->header);
          if (last_received_stamp_ >= 0 && s < last_received_stamp_ - .5)
            resetEpoch(s);
          if (s <= last_stamp_ || (pending_ && s <= stamp(pending_->header)))
            return;
          last_received_stamp_ = s;
          depth_frames_.push_back(m);
          if (depth_frames_.size() > 20)
            depth_frames_.pop_front();
          if (!pending_)
            pending_since_ = wall();
          pending_ = m;
        });
    odom_sub_ = create_subscription<Odom>(
        "/robot0/odom", sensor, [this](Odom::ConstSharedPtr m) { onOdom(m); });
    cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
        "/robot0/cmd_vel", sensor,
        [this](geometry_msgs::msg::Twist::ConstSharedPtr m) {
          cmd_speed_ = std::abs(m->linear.x) + std::abs(m->linear.y) +
                       std::abs(m->angular.z);
          cmd_time_ = wall();
        });
    cert_sub_ = create_subscription<std_msgs::msg::String>(
        "/thermalnav/startup_certificate", retained,
        [this](std_msgs::msg::String::ConstSharedPtr m) {
          try {
            proof_ = Json::parse(m->data);
          } catch (const Json::exception &) {
            certificate_status_ = "rejected: malformed attestation";
          }
        });
    traversal_sub_=create_subscription<std_msgs::msg::String>(
        "/thermalnav/safety/reply",retained,[this](std_msgs::msg::String::ConstSharedPtr m){
          if(traversal_mode_!="sim_only_safety_oracle")return;
          try {
            auto reply=Json::parse(m->data);
            if(reply.value("mode",std::string())!="sim_only_safety_oracle")return;
            auto proof=reply.at("certificate");
            double s=proof.at("stamp");
            if(!odom_ || s<stamp(odom_->header)-1. || s>stamp(odom_->header)+.15)return;
            traversal_proofs_.push_back(proof);
            if(traversal_proofs_.size()>20)traversal_proofs_.pop_front();
            processTraversal();
          } catch(const Json::exception &) {}
        });
    start_srv_ = create_service<Trigger>(
        "/thermalnav/start", [this](const std::shared_ptr<Trigger::Request>,
                                    std::shared_ptr<Trigger::Response> res) {
          if(external_manager_){res->success=false;res->message="Navigation is owned by the separate baseline manager";return;}
          enabled_ = true;
          ++generation_;
          res->success = true;
          res->message = "Exploration enabled";
        });
    stop_srv_ = create_service<Trigger>(
        "/thermalnav/stop", [this](const std::shared_ptr<Trigger::Request>,
                                   std::shared_ptr<Trigger::Response> res) {
          enabled_ = false;
          cancel();
          res->success = true;
          res->message =
              "Current-pose PointGoal sent; wait for SRU zero command";
        });
    process_timer_ = create_wall_timer(std::chrono::milliseconds(100),
                                       [this] { process(); });
    tick_timer_ =
        create_wall_timer(std::chrono::seconds(1), [this] { tick(); });
    trajectory_.header.frame_id = "odom";
    RCLCPP_INFO(get_logger(), "ROS domain=%zu mapping=%s",
                get_node_base_interface()->get_context()->get_domain_id(),
                mapping_source_.c_str());
    RCLCPP_INFO(get_logger(), "C++ runtime; detector: %s; policy: %s",
        external_perception_?"external thermal proxy detector via ROS":detector_.name.c_str(), external_manager_?"external baseline manager":policy_.status.c_str());
  }
  ~Demo() override {
    if (detection_.valid())
      detection_.wait();
    if (decision_.valid())
      decision_.wait();
  }
  void close() {
    enabled_ = false;
    cancel();
  }

private:
  XY robot() const {
    return {odom_->pose.pose.position.x, odom_->pose.pose.position.y};
  }
  void resetEpoch(double s) {
    last_thermal_stamp_=-1;
    thermal_depth_.reset();
    last_received_stamp_ = s;
    sparse_frames_.clear();
    depth_frames_.clear();
    if (reset_map_)
      fusion_->clear();
    listener_.reset();
    tf_.clear();
    listener_ = std::make_unique<tf2_ros::TransformListener>(tf_);
    pending_.reset();
    detection_depth_.reset();
    rgbs_.clear();
    odoms_.clear();
    traversal_proofs_.clear();
    last_traversal_stamp_=-1;
    tracks_.clear();
    proof_ = Json();
    if (reset_map_) {
      bev_ = BEV();
      trajectory_.poses.clear();
      failed_.clear();
      reached_.clear();
      history_ = Json::array();
    }
    bev_.certificate.reset();
    certificate_status_ = "unavailable";
    active_.reset();
    sent_.reset();
    candidates_.clear();
    ++generation_;
    ++epoch_;
    reset_stamp_ = s;
    waiting_reset_odom_ = true;
    odom_.reset();
    last_stamp_ = -1.;
    last_map_wall_ = 0;
    blocked_ = "simulation time epoch reset; waiting for fresh odom";
    RCLCPP_WARN(get_logger(), "Simulation time rewound; reset_map=%s",
                reset_map_ ? "true" : "false");
  }
  void processTraversal() {
    if(!odom_)return;
    for(auto it=traversal_proofs_.begin();it!=traversal_proofs_.end();) {
      double s=it->at("stamp").get<double>();
      if(s<=last_traversal_stamp_ || s<stamp(odom_->header)-1.) {it=traversal_proofs_.erase(it);continue;}
      auto pose=std::find_if(odoms_.begin(),odoms_.end(),[&](const auto &o){return std::abs(stamp(o->header)-s)<1e-5;});
      if(pose==odoms_.end()){++it;continue;}
      auto p=(*pose)->pose.pose.position;
      traversal_status_=bev_.recordVerifiedTraversal(*it,{p.x,p.y},stamp((*pose)->header));
      last_traversal_stamp_=s;
      it=traversal_proofs_.erase(it);
    }
  }
  void onOdom(Odom::ConstSharedPtr m) {
    const auto &p = m->pose.pose.position;
    const auto &q = m->pose.pose.orientation;
    if (m->header.frame_id != "odom" || !std::isfinite(p.x) ||
        !std::isfinite(p.y) || !std::isfinite(p.z)) {
      blocked_ = "invalid odom/frame mismatch";
      return;
    }
    try {
      rotation(q.x, q.y, q.z, q.w);
    } catch (const std::exception &) {
      blocked_ = "invalid odom quaternion";
      return;
    }
    if (waiting_reset_odom_ && std::abs(stamp(m->header) - reset_stamp_) > .25)
      return;
    if (odom_ && stamp(m->header) < stamp(odom_->header))
      return;
    odom_ = m;
    last_odom_wall_ = wall();
    odoms_.push_back(m);
    if (odoms_.size() > 200)
      odoms_.pop_front();
    processTraversal();
    if (waiting_reset_odom_) {
      waiting_reset_odom_ = false;
      cancel();
      blocked_.clear();
    }
    Pose pose;
    pose.header = m->header;
    pose.pose = m->pose.pose;
    if (trajectory_.poses.empty() ||
        cv::norm(robot() - XY(trajectory_.poses.back().pose.position.x,
                              trajectory_.poses.back().pose.position.y)) >
            .08) {
      trajectory_.poses.push_back(pose);
      auto c = bev_.cell(robot());
      if (bev_.inside(c))
        bev_.visits(c) = std::min(65535, int(bev_.visits(c)) + 1);
    }
    // Publish the live endpoint at odometry rate, independently of mapping.
    // Keep the original history/visit sampling unchanged.
    auto live_path=trajectory_;
    live_path.header=m->header;
    if(!live_path.poses.empty()&&stamp(live_path.poses.back().header)!=stamp(m->header))
      live_path.poses.push_back(pose);
    path_pub_->publish(live_path);
  }
  void process() {
    if (detection_.valid() && detection_.wait_for(std::chrono::seconds(0)) ==
                                  std::future_status::ready) {
      try {
        auto result = detection_.get();
        if (detection_epoch_ == epoch_)
          for (const auto &p : result)
            addTrack(tracks_, p, detection_stamp_);
      } catch (const std::exception &e) {
        RCLCPP_WARN(get_logger(), "Detection unavailable: %s", e.what());
      }
    }

    launchDetection();
    auto camera = mapping_source_ == "stereo_only" ? stereo_info_ : info_;
    if (!camera || !odom_ || odoms_.empty()) {
      blocked_ = !camera ? "waiting for mapping CameraInfo"
                        : "waiting for mapping odometry";
      return;
    }
    Image::ConstSharedPtr m = pending_;
    sensor_msgs::msg::PointCloud2::ConstSharedPtr sparse;
    if (mapping_source_ == "stereo_only") {
      for (auto it = sparse_frames_.rbegin(); it != sparse_frames_.rend(); ++it)
        if (stamp((*it)->header) > last_stamp_) {
          sparse = *it;
          break;
        }
      if (!sparse)
        return;
      if (!m || stamp(m->header) != stamp(sparse->header) ||
          m->header.frame_id != camera->header.frame_id ||
          m->width != camera->width || m->height != camera->height) {
        auto anchor = std::make_shared<Image>();
        anchor->header = sparse->header;
        anchor->width = camera->width;
        anchor->height = camera->height;
        m = anchor;
      }
    }
    if (mapping_source_ == "fused") {
      for (auto cloud = sparse_frames_.rbegin();
           cloud != sparse_frames_.rend() && !sparse; ++cloud) {
        if (stamp((*cloud)->header) <= last_stamp_)
          continue;
        for (auto frame = depth_frames_.rbegin(); frame != depth_frames_.rend();
             ++frame) {
          if (rclcpp::Time((*cloud)->header.stamp) ==
              rclcpp::Time((*frame)->header.stamp)) {
            m = *frame;
            sparse = *cloud;
            break;
          }
        }
      }
      if (!sparse && (!m || wall() - pending_since_ < .3))
        return;
    }
    if (!m)
      return;
    if (stamp(m->header) <= last_stamp_) {
      pending_.reset();
      return;
    }
    if (m->header.frame_id != camera->header.frame_id ||
        m->width != camera->width || m->height != camera->height) {
      blocked_ = "depth/info mismatch";
      pending_.reset();
      return;
    }
    auto nearest =
        *std::min_element(odoms_.begin(), odoms_.end(), [&](auto a, auto b) {
          return std::abs(stamp(a->header) - stamp(m->header)) <
                 std::abs(stamp(b->header) - stamp(m->header));
        });
    double skew = std::abs(stamp(nearest->header) - stamp(m->header));
    if (skew > .25) {
      blocked_ = "depth/odom time mismatch";
      return;
    }
    cv::Matx33d K;
    std::copy(camera->k.begin(), camera->k.end(), K.val);
    cv::Matx33d R;
    cv::Vec3d t;
    try {
      auto tf = tf_.lookupTransform("odom", m->header.frame_id,
                                    rclcpp::Time(m->header.stamp));
      auto q = tf.transform.rotation;
      auto tr = tf.transform.translation;
      R = rotation(q.x, q.y, q.z, q.w);
      t = {tr.x, tr.y, tr.z};
    } catch (const tf2::TransformException &) {
      ++tf_failures_;
      blocked_ = "waiting for stamped camera TF";
      return;
    } catch (const std::exception &e) {
      blocked_ = e.what();
      return;
    }
    pending_.reset();
    cv::Mat1f depth;
    auto begin = Clock::now();
    try {

      bool virtual_frame =
          m->encoding.empty() && mapping_source_ == "stereo_only";
      if (m->encoding == "32FC1")
        depth = cv_bridge::toCvCopy(m, "32FC1")->image;
      else if (m->encoding == "16UC1")
        cv_bridge::toCvShare(m, "16UC1")->image.convertTo(depth, CV_32F, .001);
      else if (virtual_frame)
        depth = cv::Mat1f(m->height, m->width,
                          std::numeric_limits<float>::quiet_NaN());
      else {
        blocked_ = "unsupported depth encoding " + m->encoding;
        return;
      }
      std::vector<Observation> observations;
      cv::Mat1f measured =
          degradeDepth(depth, dropout_, noise_, mapping_range_,
                       seed_ + rclcpp::Time(m->header.stamp).nanoseconds());
      if (hole_fraction_ > 0) {
        int w = int(measured.cols * hole_fraction_),
            h = int(measured.rows * hole_fraction_);
        measured(
            cv::Rect((measured.cols - w) / 2, (measured.rows - h) / 2, w, h))
            .setTo(std::numeric_limits<float>::quiet_NaN());
      }
      size_t valid_depth = 0, stereo_count = 0;
      if (mapping_source_ != "stereo_only") {
        auto observation =
            depthObservation(measured, K, R, t, stamp(m->header), max_range_,
                             .0004 + noise_ * noise_);
        valid_depth = observation.points.size();
        observations.push_back(std::move(observation));
      }
      cv::Matx33d sparse_R;
      cv::Vec3d sparse_t;
      std::vector<DepthPoint> sparse_points;
      if (sparse) {
        auto pose = tf_.lookupTransform("odom", sparse->header.frame_id,
                                        rclcpp::Time(sparse->header.stamp));
        auto q = pose.transform.rotation;
        auto p = pose.transform.translation;
        sparse_R = rotation(q.x, q.y, q.z, q.w);
        sparse_t = {p.x, p.y, p.z};
        Observation observation;
        observation.source = "stereo";
        observation.stamp = stamp(sparse->header);
        observation.R = sparse_R;
        observation.origin = sparse_t;
        sparse_points = readPoints(*sparse);
        observation.points = sparse_points;
        stereo_count = observation.points.size();
        if (stereo_count)
          observations.push_back(std::move(observation));
      }
      bev_.diagnostics = fusion_->integrate(observations, bev_);
      std::vector<cv::Vec3d> completion;
      for(const auto &observation:observations) for(const auto &sample:observation.points) {
        if(!sample.floor_reliable || sample.confidence<.6)continue;
        auto p=observation.R*sample.p+observation.origin;
        auto h=p[2]-floor_;
        if(std::abs(h)<=.06 || (h>.12 && h<=fusion_->options().max_height)) completion.push_back(p);
      }
      bev_.nearfield_stats=bev_.complete(completion,robot());
      bev_.diagnostics["verified_traversed_cells"]=cv::countNonZero(bev_.traversed);
      bev_.diagnostics["traversal_validation"]=traversal_status_;
      bev_.diagnostics["ground_supported_but_unknown_cells"]=cv::countNonZero((bev_.floor_evidence!=0)&(bev_.occupancy()==255));
      bev_.diagnostics["depth_pixels"] = depth.total();
      bev_.diagnostics["valid_depth"] = valid_depth;
      bev_.diagnostics["stereo_points"] = stereo_count;
      bev_.diagnostics["stereo_status"] =
          mapping_source_ == "depth_only" ? "disabled"
          : stereo_count                  ? "paired"
          : sparse                        ? "zero_valid_matches: depth-only fallback"
                                          : "unavailable: depth-only fallback";
      bev_.diagnostics["fused_two_sources"] = valid_depth > 0 && stereo_count > 0;
      // Coverage belongs to the inspection (stereo/thermal-proxy) camera.
      // A wider, separately mounted depth camera cannot certify that coverage.
      if (!external_perception_ && mapping_source_ != "stereo_only" &&
          (stereo_info_ && stereo_info_->header.frame_id == m->header.frame_id))
        bev_.observeCoverage(measured, K, R, t, floor_, max_range_);
      else if (!external_perception_ && sparse && stereo_info_ &&
               stereo_info_->header.frame_id == sparse->header.frame_id) {
        cv::Matx33d sk;
        std::copy(stereo_info_->k.begin(), stereo_info_->k.end(), sk.val);
        cv::Mat1f raster(stereo_info_->height, stereo_info_->width,
                         std::numeric_limits<float>::quiet_NaN());
        for (const auto &p : sparse_points) {
          if (p.p[2] <= 0)
            continue;
          auto uv = sk * p.p;
          int x = cvRound(uv[0] / uv[2]), y = cvRound(uv[1] / uv[2]);
          if (x >= 0 && y >= 0 && x < raster.cols && y < raster.rows)
            raster(y, x) = p.p[2];
        }
        bev_.observeCoverage(raster, sk, sparse_R, sparse_t, floor_,
                             max_range_);
      }
      bev_.blind_zone =
          bev_.blindZone(K, R, t, m->width, m->height, robot(), floor_);
      bev_.blind_zone["camera_frame"] = m->header.frame_id;
      ++bev_.frames;
      usable_depth_ = valid_depth > 0 || stereo_count > 0;
    } catch (const std::exception &e) {
      blocked_ = "mapping/decode: " + std::string(e.what());
      return;
    }

    map_ms_ =
        std::chrono::duration<double, std::milli>(Clock::now() - begin).count();
    if (!proof_.is_null() && !bev_.certificate) {
      bool ok = bev_.acceptCertificate(proof_, robot(), stamp(m->header));
      certificate_status_ =
          ok ? "certified" : "rejected: " + bev_.certificate_rejection;
      if (!ok)
        proof_ = Json();
    }
    last_stamp_ = stamp(m->header);
    last_map_wall_ = wall();
    map_header_ = m->header;
    map_header_.frame_id = "odom";
    last_depth_ = {{"stamp", last_stamp_},
                   {"encoding", m->encoding},
                   {"shape", {m->height, m->width}},
                   {"odom_stamp_skew_s", skew},
                   {"camera_pose_odom_m", {t[0], t[1], t[2]}}};
    if (blocked_.rfind("waiting", 0) == 0 ||
        blocked_.rfind("depth/odom", 0) == 0 ||
        blocked_.rfind("stale sensors", 0) == 0 ||
        blocked_.rfind("depth/info", 0) == 0 ||
        blocked_.rfind("mapping/decode", 0) == 0 ||
        blocked_.rfind("unsupported depth", 0) == 0)
      blocked_.clear();
  }

  void launchDetection() {
    if(external_perception_)return;
    if (rgbs_.empty() || detection_.valid() || wall() - last_detect_wall_ < 2 ||
        !detection_info_ || !detection_depth_)
      return;
    auto depth = detection_depth_;
    auto info = detection_info_;
    if (depth->header.frame_id != info->header.frame_id ||
        depth->width != info->width || depth->height != info->height)
      return;
    auto rgb =
        *std::min_element(rgbs_.begin(), rgbs_.end(), [&](auto a, auto b) {
          return std::abs(stamp(a->header) - stamp(depth->header)) <
                 std::abs(stamp(b->header) - stamp(depth->header));
        });
    if (std::abs(stamp(rgb->header) - stamp(depth->header)) > .08 ||
        rgb->header.frame_id != depth->header.frame_id ||
        rgb->width != depth->width || rgb->height != depth->height)
      return;
    try {
      cv::Mat1f dep;
      if (depth->encoding == "32FC1")
        dep = cv_bridge::toCvCopy(depth, "32FC1")->image;
      else if (depth->encoding == "16UC1")
        cv_bridge::toCvShare(depth, "16UC1")
            ->image.convertTo(dep, CV_32F, .001);
      else
        return;
      auto pose = tf_.lookupTransform("odom", depth->header.frame_id,
                                      rclcpp::Time(depth->header.stamp));
      auto q = pose.transform.rotation;
      auto p = pose.transform.translation;
      auto R = rotation(q.x, q.y, q.z, q.w);
      cv::Vec3d t(p.x, p.y, p.z);
      cv::Matx33d K;
      std::copy(info->k.begin(), info->k.end(), K.val);
      rgb_ = cv_bridge::toCvCopy(rgb, "bgr8")->image;
      auto image = rgb_.clone();
      last_rgb_wall_ = last_detect_wall_ = wall();
      detection_stamp_ = stamp(rgb->header);
      detection_epoch_ = epoch_;
      detection_ = std::async(std::launch::async, [this, image, dep, K, R, t] {
        std::vector<XY> points;
        for (const auto &box : detector_.detect(image)) {
          auto p = localizeBox(box, dep, K, R, t);
          if (p)
            points.emplace_back((*p)[0], (*p)[1]);
        }
        return points;
      });
    } catch (const std::exception &e) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10000,
                           "Detection waiting: %s", e.what());
    }
  }
  void goal(XY xy) {
    if (!odom_)
      return;
    Pose p;
    p.header = odom_->header;
    p.pose.position = odom_->pose.pose.position;
    p.pose.position.x = xy.x;
    p.pose.position.y = xy.y;
    p.pose.orientation.w = 1.;
    goal_pub_->publish(p);
    sent_ = xy;
    last_goal_wall_ = last_progress_wall_ = wall();
    best_ = 1e9;
  }
  void cancel() {
    if (odom_)
      goal(robot());
    active_.reset();
    sent_.reset();
    ++generation_;
    cancel_until_ = wall() + 1.;
  }
  void chooseWaypoint(const Candidate &c) {
    size_t near = 0;
    for (size_t i = 1; i < c.path.size(); ++i)
      if (cv::norm(c.path[i] - robot()) < cv::norm(c.path[near] - robot()))
        near = i;
    std::optional<XY> chosen;
    for (size_t i = near + 1; i < c.path.size(); ++i) {
      double d = cv::norm(c.path[i] - robot());
      if (d > 1.6)
        break;
      if (!bev_.pathReason({robot(), c.path[i]}).empty())
        break;
      if (d > .85)
        chosen = c.path[i];
    }
    if (!chosen) {
      failed_.push_back(c.xy);
      ++nav_failures_;
      blocked_ = "no_safe_path: no straight SRU segment beyond arrival radius";
      cancel();
      return;
    }
    goal(*chosen);
  }
  void activate(Candidate c) {
    blocked_.clear();
    active_ = std::move(c);
    ++selected_;
    started_wall_ = wall();
    state_ = active_->kind == "frontier"        ? "EXPLORE"
             : active_->kind == "verify_target" ? "VERIFY_TARGET"
                                                : "INSPECT";
    auto selected = *active_;
    chooseWaypoint(selected);
    RCLCPP_INFO(get_logger(), "%s %s xy=(%.2f, %.2f)", state_.c_str(),
                selected.id.c_str(), selected.xy.x, selected.xy.y);
  }
  std::vector<XY> excluded() const {
    auto result = failed_;
    result.insert(result.end(), reached_.begin(), reached_.end());
    return result;
  }
  std::vector<XY> trajectoryXY() const {
    std::vector<XY> out;
    for (const auto &p : trajectory_.poses)
      out.emplace_back(p.pose.position.x, p.pose.position.y);
    return out;
  }
  double yaw() const {
    const auto &q = odom_->pose.pose.orientation;
    return std::atan2(2 * (q.w * q.z + q.x * q.y),
                      1 - 2 * (q.y * q.y + q.z * q.z));
  }
  void tick() {
    double now = wall();
    if(external_perception_ && thermal_depth_ && thermal_info_ && bev_.frames &&
       rclcpp::Time(thermal_depth_->header.stamp).nanoseconds()>last_thermal_stamp_) {
      auto m=thermal_depth_;auto info=thermal_info_;
      if(m->header.frame_id==info->header.frame_id && m->width==info->width && m->height==info->height)try {
        auto tr=tf_.lookupTransform("odom",m->header.frame_id,rclcpp::Time(m->header.stamp));
        auto q=tr.transform.rotation;auto t=tr.transform.translation;cv::Matx33d k;std::copy(info->k.begin(),info->k.end(),k.val);
        cv::Mat1f depth=cv_bridge::toCvCopy(m,"32FC1")->image;
        bev_.observeCoverage(depth,k,rotation(q.x,q.y,q.z,q.w),{t.x,t.y,t.z},floor_,6.);
        last_thermal_stamp_=rclcpp::Time(m->header.stamp).nanoseconds();
      }catch(const std::exception &e){RCLCPP_WARN_THROTTLE(get_logger(),*get_clock(),10000,"Thermal coverage waiting: %s",e.what());}
    }
    std::optional<PolicyResult> reply;
    if (decision_.valid() && decision_.wait_for(std::chrono::seconds(0)) ==
                                 std::future_status::ready) {
      try {
        reply = decision_.get();
        policy_.calls += reply->called;
        policy_.status = reply->status;
      } catch (const std::exception &) {
        policy_.status = "VLM unavailable: worker failed; heuristic fallback";
      }
    }
    if (!odom_) {
      publishStatus();
      return;
    }
    bool fresh = now - last_odom_wall_ <= 3 && now - last_map_wall_ <= 5 &&
                 now - cmd_time_ <= 2 && last_stamp_ >= 0 && usable_depth_;
    if (enabled_ && !fresh) {
      if (active_)
        cancel();
      blocked_ = "stale sensors/command: navigation held";
      publish();
      return;
    }
    if (enabled_ && now >= cancel_until_) {
      if (active_ && sent_) {
        double d = cv::norm(*sent_ - robot());
        if (d < best_ - .08) {
          best_ = d;
          last_progress_wall_ = now;
        }
        if (!bev_.pathReason({robot(), *sent_}).empty()) {
          failed_.push_back(active_->xy);
          ++nav_failures_;
          blocked_ = "active segment invalidated by new map";
          cancel();
        } else if (d <= .75 && cmd_speed_ < .03 && now - last_goal_wall_ > 1) {
          if (cv::norm(active_->xy - robot()) <= .8) {
            ++completed_;
            completed_kinds_[active_->kind] =
                completed_kinds_[active_->kind].get<int>() + 1;
            reached_.push_back(active_->xy);
            history_.push_back({{"id", active_->id},
                                {"kind", active_->kind},
                                {"result", "reached"},
                                {"xy", pointJson(active_->xy)}});
            auto c = bev_.cell(active_->xy);
            if (bev_.inside(c))
              bev_.visits(c) = std::min(65535, int(bev_.visits(c)) + 5);
            active_.reset();
            sent_.reset();
            cancel_until_ = now + 1.;
          } else {
            auto c = *active_;
            chooseWaypoint(c);
          }
        } else if (now - last_progress_wall_ > 15 || now - started_wall_ > 90) {
          failed_.push_back(active_->xy);
          ++nav_failures_;
          blocked_ = "navigation timeout";
          cancel();
        }
      } else if (reply && decision_generation_ == generation_) {
        auto cs = bev_.candidates(robot(), excluded(), tracks_);
        auto match = std::find_if(cs.begin(), cs.end(), [&](const auto &c) {
          return c.kind == reply->candidate.kind &&
                 cv::norm(c.xy - reply->candidate.xy) < .3;
        });
        if (match != cs.end())
          activate(*match);
      } else if (!decision_.valid() && now - last_decision_ > 3 &&
                 cmd_speed_ < .03) {
        last_decision_ = now;
        candidates_ = bev_.candidates(robot(), excluded(), tracks_);
        if (!candidates_.empty()) {
          if (policy_.remote()) {
            Json context = {
                {"candidates", Json::array()},
                {"history", history_},
                {"detections", trackJson(tracks_)},
                {"coverage",
                 {{"observed_cells", cv::countNonZero(bev_.scanned)},
                  {"mapped_cells", cv::countNonZero(bev_.seen)}}}};
            for (const auto &c : candidates_)
              context["candidates"].push_back({{"candidate_id", c.id},
                                               {"kind", c.kind},
                                               {"xy", pointJson(c.xy)},
                                               {"gain", c.gain},
                                               {"cost", c.cost}});
            auto bev_rgb =
                renderer_.render(bev_, robot(), yaw(), trajectoryXY(),
                                 candidates_, sent_, tracks_);
            cv::Mat3b bgr;
            cv::cvtColor(bev_rgb, bgr, cv::COLOR_RGB2BGR);
            for (const auto &c : candidates_)
              cv::putText(bgr, c.id, renderer_.pixel(c.xy, robot()),
                          cv::FONT_HERSHEY_SIMPLEX, .3, {0, 0, 255}, 1);
            auto image = now - last_rgb_wall_ < 3 ? rgb_.clone() : cv::Mat3b();
            auto cs = candidates_;
            auto policy = policy_;
            decision_generation_ = generation_;
            decision_ = std::async(
                std::launch::async, [policy, cs, bgr, image, context] {
                  return policy.choose(cs, bgr, image, context);
                });
          } else
            activate(candidates_.front());
        } else {
          auto occ = bev_.occupancy();
          int remaining = cv::countNonZero((occ == 0) & (bev_.scanned == 0));
          bool unverified =
              std::any_of(tracks_.begin(), tracks_.end(),
                          [](const auto &t) { return !t.verified; });
          if (remaining == 0 && bev_.frontier_components == 0 && !unverified &&
              cv::countNonZero(occ == 0) > 0) {
            state_ = "FINISH";
            enabled_ = false;
          } else
            blocked_ =
                "no safe reachable candidate; startup=" + bev_.startup_check +
                "; unscanned=" + std::to_string(remaining);
        }
      }
    }
    publish();
  }
  Grid grid(const cv::Mat1b &data) const {
    Grid m;
    m.header = map_header_;
    m.info.resolution = bev_.res;
    m.info.width = data.cols;
    m.info.height = data.rows;
    m.info.origin.position.x = bev_.origin.x;
    m.info.origin.position.y = bev_.origin.y;
    m.info.origin.orientation.w = 1.;
    m.data.reserve(data.total());
    for (int y = 0; y < data.rows; ++y)
      for (int x = 0; x < data.cols; ++x)
        m.data.push_back(data(y, x) == 255 ? -1
                                           : static_cast<int8_t>(data(y, x)));
    return m;
  }
  void publish() {
    if (last_stamp_ < 0) {
      publishStatus();
      return;
    }

    std::vector<DepthPoint> occupied, obstacles;
    for (auto it = fusion_->tree().begin_leafs();
         it != fusion_->tree().end_leafs(); ++it)
      if (it->getOccupancy() >= .65) {
        occupied.push_back({{it.getX(), it.getY(), it.getZ()},
                            float(it->getOccupancy()),
                            0,
                            false});
        const auto &options = fusion_->options();
        if (it.getZ() >= options.floor + options.min_height &&
            it.getZ() <= options.floor + options.max_height)
          obstacles.push_back(occupied.back());
      }
    voxel_pub_->publish(pointCloud(occupied, map_header_));
    obstacle_voxel_pub_->publish(pointCloud(obstacles, map_header_));
    auto occ = bev_.occupancy();
    map_pub_->publish(grid(occ));
    cv::Mat1b traversed(occ.size(),uint8_t(255)),camera_free(occ.size(),uint8_t(255));
    traversed.setTo(0,(bev_.traversed!=0)&(occ!=100));
    camera_free.setTo(0,(bev_.seen!=0)&(bev_.log<0));
    traversed_pub_->publish(grid(traversed));
    camera_free_pub_->publish(grid(camera_free));
    cover_pub_->publish(grid(bev_.coverage(tracks_)));
    cv::Mat1b mask(occ.size(),uint8_t(255));mask.setTo(0,occ==0);mask.setTo(100,(occ==0)&(bev_.scanned!=0));scan_mask_pub_->publish(grid(mask));
    cv::Mat1b explored(occ.size(), uint8_t(255)),
        inferred(occ.size(), uint8_t(255));
    explored.setTo(50, bev_.inferred);
    explored.setTo(100, bev_.seen);
    inferred.setTo(50, bev_.inferred);
    explored_pub_->publish(grid(explored));
    inferred_pub_->publish(grid(inferred));
    double hfov = bev_.blind_zone.value("hfov_deg", 47.);
    auto image =
        renderer_.render(bev_, robot(), yaw(), trajectoryXY(), candidates_,
                         sent_, tracks_, hfov, max_range_);
    render_pub_->publish(
        *cv_bridge::CvImage(map_header_, "rgb8", image).toImageMsg());
    cover_render_pub_->publish(
        *cv_bridge::CvImage(map_header_, "rgb8",
                            renderer_.coverage(bev_, robot(), tracks_))
             .toImageMsg());
    visualization_msgs::msg::MarkerArray markers;
    visualization_msgs::msg::Marker clear;
    clear.action = clear.DELETEALL;
    markers.markers.push_back(clear);
    auto marker = [&](XY xy, const std::string &ns, int id, cv::Vec3f color,
                      const std::string &text = "") {
      visualization_msgs::msg::Marker m;
      m.header = odom_->header;
      m.ns = ns;
      m.id = id;
      m.type = text.empty() ? m.SPHERE : m.TEXT_VIEW_FACING;
      m.action = m.ADD;
      m.pose.position.x = xy.x;
      m.pose.position.y = xy.y;
      m.pose.position.z = text.empty() ? .15 : .7;
      m.pose.orientation.w = 1.;
      m.scale.x = m.scale.y = m.scale.z = .25;
      m.color.r = color[0];
      m.color.g = color[1];
      m.color.b = color[2];
      m.color.a = 1.;
      m.text = text;
      markers.markers.push_back(m);
    };
    for (size_t i = 0; i < candidates_.size(); ++i)
      marker(candidates_[i].xy, "candidates", int(i), {.2, .8, 1.});
    if (sent_)
      marker(*sent_, "selected", 0, {1., .7, 0.});
    for (const auto &t : tracks_)
      marker(t.xy, "targets", t.id, {1., 0., 0.},
             t.verified ? "person verified" : "person suspect");
    marker(robot(), "policy", 0, {1., 1., 1.}, state_ + " " + policy_.status);
    marker_pub_->publish(markers);
    publishStatus();
  }
  void publishStatus() {
    auto occ = bev_.occupancy();
    double area = bev_.res * bev_.res;
    int total = cv::countNonZero(occ == 0),
        scanned = cv::countNonZero((occ == 0) & (bev_.scanned != 0));
    Json status = {
        {"runtime", "cpp"},
        {"external_perception", external_perception_},
        {"external_manager", external_manager_},
        {"thermal_coverage_stamp_ns", last_thermal_stamp_},
        {"mapping_source", mapping_source_},
        {"verified_traversal_mode",traversal_mode_},
        {"mapping_depth_dropout", dropout_},
        {"mapping_depth_noise_sigma", noise_},
        {"state", state_},
        {"enabled", enabled_},
        {"policy", policy_.status},
        {"vlm_calls", policy_.calls},
        {"pose_source", "sim_gt_pose_via_odom"},
        {"map_frame", "odom"},
        {"robot_xy", odom_ ? pointJson(robot()) : Json(nullptr)},
        {"trajectory_poses", trajectory_.poses.size()},
        {"tf_failures", tf_failures_},
        {"last_depth", last_depth_},
        {"bev", bev_.diagnostics},
        {"occupancy_grid_checksum", bev_.checksum(occ)},
        {"coverage_grid_checksum", bev_.checksum(bev_.coverage(tracks_))},
        {"map_frames", bev_.frames},
        {"mapping_ms", map_ms_},
        {"epoch", epoch_},
        {"selected", selected_},
        {"completed", completed_},
        {"completed_by_kind", completed_kinds_},
        {"certificate", certificate_status_},
        {"startup_footprint", odom_ ? (bev_.footprintReason(robot()).empty()
                                           ? "safe"
                                           : bev_.footprintReason(robot()))
                                    : "unavailable"},
        {"frontier_components", bev_.frontier_components},
        {"rejections", bev_.rejections},
        {"nearfield", bev_.nearfield_stats},
        {"inferred_m2", cv::countNonZero(bev_.inferred) * area},
        {"blind_zone", bev_.blind_zone},
        {"failed", nav_failures_},
        {"free_m2", total * area},
        {"occupied_m2", cv::countNonZero(occ == 100) * area},
        {"observed_m2", cv::countNonZero(bev_.seen) * area},
        {"scanned_m2", scanned * area},
        {"coverage_fraction", double(scanned) / std::max(1, total)},
        {"targets", trackJson(tracks_)},
        {"candidates", candidates_.size()},
        {"blocked", blocked_}};
    std_msgs::msg::String m;
    if(external_perception_)for(auto &target:status["targets"]){target["label"]="suspected_person";target["source"]="thermal_proxy_threshold";}
    m.data = status.dump();
    status_pub_->publish(m);
  }
  BEV bev_;
  std::unique_ptr<OccupancyFusion> fusion_;
  std::string mapping_source_;
  double dropout_{}, noise_{}, mapping_range_{8.}, pending_since_{};
  double hole_fraction_{};
  int seed_{42};
  std::deque<sensor_msgs::msg::PointCloud2::ConstSharedPtr> sparse_frames_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sparse_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr voxel_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr obstacle_voxel_pub_;

  Renderer renderer_;
  tf2_ros::Buffer tf_;
  std::unique_ptr<tf2_ros::TransformListener> listener_;
  Policy policy_;
  Detector detector_;
  sensor_msgs::msg::CameraInfo::ConstSharedPtr info_, stereo_info_,
      detection_info_;
  Image::ConstSharedPtr detection_depth_;
  std::string depth_topic_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr
      stereo_info_sub_,
      detection_info_sub_;
  rclcpp::Subscription<Image>::SharedPtr detection_depth_sub_;
  cv::Matx33d K_;
  Odom::ConstSharedPtr odom_;
  std::deque<Odom::ConstSharedPtr> odoms_;
  Image::ConstSharedPtr pending_;
  std::deque<Image::ConstSharedPtr> rgbs_;
  std::deque<Image::ConstSharedPtr> depth_frames_;
  cv::Mat3b rgb_;
  std::vector<Track> tracks_;
  std::vector<Candidate> candidates_;
  std::optional<Candidate> active_;
  std::optional<XY> sent_;
  std::vector<XY> failed_, reached_;
  nav_msgs::msg::Path trajectory_;
  std_msgs::msg::Header map_header_;
  Json proof_, last_depth_ = Json::object(), history_ = Json::array();
  Json completed_kinds_ = {
      {"frontier", 0}, {"inspection_viewpoint", 0}, {"verify_target", 0}};
  std::future<std::vector<XY>> detection_;
  std::future<PolicyResult> decision_;
  uint64_t generation_{}, decision_generation_{}, epoch_{}, detection_epoch_{},
      tf_failures_{};
  int selected_{}, completed_{}, nav_failures_{}, stride_{};
  double floor_{}, max_range_{}, last_stamp_{-1}, last_received_stamp_{-1},
      reset_stamp_{}, last_map_wall_{}, last_odom_wall_{};
  double last_rgb_wall_{}, last_detect_wall_{}, detection_stamp_{}, map_ms_{},
      cmd_speed_{}, cmd_time_{}, cancel_until_{};
  double last_decision_{}, last_goal_wall_{}, last_progress_wall_{},
      started_wall_{}, best_{1e9};
  bool external_perception_{false},external_manager_{false};
  int64_t last_thermal_stamp_{-1};
  Image::ConstSharedPtr thermal_depth_;
  sensor_msgs::msg::CameraInfo::ConstSharedPtr thermal_info_;
  rclcpp::Subscription<Image>::SharedPtr thermal_depth_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr thermal_info_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr thermal_tracks_sub_;
  bool enabled_{true}, reset_map_{true}, waiting_reset_odom_{false},
      usable_depth_{false};
  std::string state_{"EXPLORE"}, blocked_{"waiting for sensors"},
      certificate_status_{"unavailable"};
  rclcpp::Publisher<Grid>::SharedPtr map_pub_, cover_pub_, scan_mask_pub_, explored_pub_,
      inferred_pub_;
  rclcpp::Publisher<Image>::SharedPtr render_pub_, cover_render_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Publisher<Pose>::SharedPtr goal_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
      marker_pub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr info_sub_;
  rclcpp::Subscription<Image>::SharedPtr rgb_sub_, depth_sub_;
  rclcpp::Subscription<Odom>::SharedPtr odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::Publisher<Grid>::SharedPtr traversed_pub_,camera_free_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr cert_sub_,traversal_sub_;
  std::deque<Json> traversal_proofs_;
  std::string traversal_mode_;
  double last_traversal_stamp_{-1};
  Json traversal_status_=Json::object();
  rclcpp::Service<Trigger>::SharedPtr start_srv_, stop_srv_;
  rclcpp::TimerBase::SharedPtr process_timer_, tick_timer_;
};
} // namespace thermalnav

namespace {
volatile std::sig_atomic_t interrupted = 0;
void stopSignal(int) { interrupted = 1; }
} // namespace
int main(int argc, char **argv) {
  try {
    rclcpp::init(argc, argv, rclcpp::InitOptions(),
                 rclcpp::SignalHandlerOptions::None);
    std::signal(SIGINT, stopSignal);
    std::signal(SIGTERM, stopSignal);
    auto node = std::make_shared<thermalnav::Demo>();
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    while (rclcpp::ok() && !interrupted) {
      executor.spin_some();
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    if (rclcpp::ok()) {
      node->close();
      auto deadline =
          std::chrono::steady_clock::now() + std::chrono::seconds(2);
      while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline) {
        executor.spin_some();
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
      }
    }
    executor.remove_node(node);
    node.reset();
    if (rclcpp::ok())
      rclcpp::shutdown();
    return 0;
  } catch (const std::exception &e) {
    std::cerr << "ThermalNav: " << e.what() << '\n';
    if (rclcpp::ok())
      rclcpp::shutdown();
    return 1;
  }
}
