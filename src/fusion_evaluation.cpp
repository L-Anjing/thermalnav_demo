#include "thermalnav/occupancy_fusion.hpp"
#include "thermalnav/ros_observation.hpp"
#include <cv_bridge/cv_bridge.h>
#include <deque>
#include <iostream>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace thermalnav {
class Evaluation : public rclcpp::Node {
public:
  Evaluation()
      : Node("thermalnav_evaluation"), tf_(get_clock()),
        listener_(std::make_unique<tf2_ros::TransformListener>(tf_)) {
    FusionOptions options;
    options.floor = declare_parameter<double>("floor_height", 0.);
    options.max_height = declare_parameter<double>("collision_height", .65);
    options.max_range = declare_parameter<double>("max_range", 8.);
    reference_ = std::make_unique<OccupancyFusion>(options);
    range_ = options.max_range;
    auto depth_topic = declare_parameter<std::string>(
        "depth_topic", "/robot0/front_cam/depth");
    auto info_topic = declare_parameter<std::string>(
        "info_topic", "/robot0/front_cam/camera_info");
    auto stereo_reference_topic = declare_parameter<std::string>(
        "stereo_reference_topic", "/robot0/front_cam/depth");
    auto stereo_info_topic = declare_parameter<std::string>(
        "stereo_info_topic", "/robot0/front_cam/camera_info");
    auto sensor = rclcpp::SensorDataQoS().keep_last(20);
    auto retained = rclcpp::QoS(1).transient_local().reliable();
    depth_ = create_subscription<sensor_msgs::msg::Image>(
        depth_topic, sensor, [this](sensor_msgs::msg::Image::ConstSharedPtr m) {
          if (!depths_.empty() &&
              time(m->header) < time(depths_.back()->header) - 500000000LL) {
            depths_.clear();
            clouds_.clear();
            reference_->clear();
            stereo_references_.clear();
            listener_.reset();
            tf_.clear();
            listener_ = std::make_unique<tf2_ros::TransformListener>(tf_);
            reference_bev_ = BEV();
            last_map_ = -1;
          }
          depths_.push_back(m);
          if (depths_.size() > 100)
            depths_.pop_front();
        });
    info_ = create_subscription<sensor_msgs::msg::CameraInfo>(
        info_topic, sensor,
        [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr m) {
          camera_ = m;
        });
    stereo_reference_ = create_subscription<sensor_msgs::msg::Image>(
        stereo_reference_topic, sensor,
        [this](sensor_msgs::msg::Image::ConstSharedPtr m) {
          stereo_references_.push_back(m);
          if (stereo_references_.size() > 100)
            stereo_references_.pop_front();
        });
    stereo_info_ = create_subscription<sensor_msgs::msg::CameraInfo>(
        stereo_info_topic, sensor,
        [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr m) {
          stereo_camera_ = m;
        });
    sparse_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        "/thermalnav/stereo/points", sensor,
        [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr m) {
          clouds_.push_back(m);
          if (clouds_.size() > 100)
            clouds_.pop_front();
        });
    map_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/thermalnav/bev", retained,
        [this](nav_msgs::msg::OccupancyGrid::ConstSharedPtr m) {
          pending_ = m;
        });
    status_ = create_subscription<std_msgs::msg::String>(
        "/thermalnav/status", retained,
        [this](std_msgs::msg::String::ConstSharedPtr m) {
          try {
            runtime_ = Json::parse(m->data);
          } catch (const Json::exception &) {
          };
        });
    output_ = create_publisher<std_msgs::msg::String>("/thermalnav/evaluation",
                                                      retained);
    timer_ = create_wall_timer(std::chrono::milliseconds(100),
                               [this] { process(); });
  }

private:
  static int64_t time(const std_msgs::msg::Header &h) {
    return rclcpp::Time(h.stamp).nanoseconds();
  }
  void process() {
    if (!pending_ || !camera_ || time(pending_->header) <= last_map_)
      return;
    sensor_msgs::msg::Image::ConstSharedPtr depth;
    for (auto it = depths_.rbegin(); it != depths_.rend(); ++it)
      if (time((*it)->header) == time(pending_->header)) {
        depth = *it;
        break;
      }
    if (!depth || depth->header.frame_id != camera_->header.frame_id ||
        depth->width != camera_->width || depth->height != camera_->height)
      return;
    try {
      cv::Mat1f image;
      if (depth->encoding == "32FC1")
        image = cv_bridge::toCvCopy(depth, "32FC1")->image;
      else if (depth->encoding == "16UC1")
        cv_bridge::toCvShare(depth, "16UC1")
            ->image.convertTo(image, CV_32F, .001);
      else
        return;
      cv::Matx33d K;
      std::copy(camera_->k.begin(), camera_->k.end(), K.val);
      auto tf = tf_.lookupTransform("odom", depth->header.frame_id,
                                    rclcpp::Time(depth->header.stamp));
      auto q = tf.transform.rotation;
      auto t = tf.transform.translation;
      auto R = rotation(q.x, q.y, q.z, q.w);
      double stamp = time(depth->header) * 1e-9;
      reference_->integrate(
          {depthObservation(image, K, R, {t.x, t.y, t.z}, stamp, range_)},
          reference_bev_);
      const auto &grid = *pending_;
      if (grid.header.frame_id != "odom" || grid.info.width > 2000 ||
          grid.info.height > 2000 ||
          grid.data.size() != size_t(grid.info.width) * grid.info.height ||
          std::abs(grid.info.resolution - .1) > .0001)
        throw std::invalid_argument("invalid evaluation map");
      BEV estimate(grid.info.resolution, grid.info.resolution,
                   {grid.info.origin.position.x, grid.info.origin.position.y});
      estimate.log = cv::Mat1f(grid.info.height, grid.info.width, 0.f);
      estimate.seen = cv::Mat1b(grid.info.height, grid.info.width, uint8_t(0));
      for (uint32_t y = 0; y < grid.info.height; ++y)
        for (uint32_t x = 0; x < grid.info.width; ++x) {
          int v = grid.data[y * grid.info.width + x];
          if (v == 0 || v == 100) {
            estimate.seen(y, x) = 255;
            estimate.log(y, x) = v == 0 ? -3 : 3;
          }
        }
      Json metrics = {
          {"stamp", stamp},
          {"map_reference",
           "unmasked depth observations at published map stamps"},
          {"occupancy", compareOccupancy(estimate, reference_bev_)}};
      metrics["stereo_status"] = "reference unavailable";
      for (auto it = clouds_.rbegin(); it != clouds_.rend(); ++it) {
        if (time((*it)->header) != time(depth->header) || !stereo_camera_ ||
            (*it)->header.frame_id != stereo_camera_->header.frame_id)
          continue;
        for (auto ref = stereo_references_.rbegin();
             ref != stereo_references_.rend(); ++ref) {
          if (time((*ref)->header) != time((*it)->header) ||
              (*ref)->header.frame_id != (*it)->header.frame_id ||
              (*ref)->width != stereo_camera_->width ||
              (*ref)->height != stereo_camera_->height)
            continue;
          cv::Mat1f truth;
          if ((*ref)->encoding == "32FC1")
            truth = cv_bridge::toCvCopy(*ref, "32FC1")->image;
          else if ((*ref)->encoding == "16UC1")
            cv_bridge::toCvShare(*ref, "16UC1")
                ->image.convertTo(truth, CV_32F, .001);
          else
            continue;
          cv::Matx33d sk;
          std::copy(stereo_camera_->k.begin(), stereo_camera_->k.end(), sk.val);
          metrics["stereo"] = evaluateStereo(readPoints(**it), truth, sk);
          metrics["stereo_status"] = "same-camera reference";
          break;
        }
        break;
      }
      if (!runtime_.empty())
        for (const char *key :
             {"mapping_source", "mapping_ms", "coverage_fraction", "scanned_m2",
              "completed", "targets"})
          if (runtime_.contains(key))
            metrics[key] = runtime_[key];
      std_msgs::msg::String out;
      out.data = metrics.dump();
      output_->publish(out);
      std::cout << out.data << std::endl;
      last_map_ = time(grid.header);
      pending_.reset();
    } catch (const tf2::TransformException &) {
    } catch (const std::exception &e) {
      RCLCPP_WARN(get_logger(), "Evaluation skipped: %s", e.what());
      pending_.reset();
    }
  }
  tf2_ros::Buffer tf_;
  std::unique_ptr<tf2_ros::TransformListener> listener_;
  std::unique_ptr<OccupancyFusion> reference_;
  double range_{8.};
  BEV reference_bev_;
  int64_t last_map_{-1};
  Json runtime_ = Json::object();
  sensor_msgs::msg::CameraInfo::ConstSharedPtr camera_;
  sensor_msgs::msg::CameraInfo::ConstSharedPtr stereo_camera_;
  std::deque<sensor_msgs::msg::Image::ConstSharedPtr> stereo_references_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr stereo_reference_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr stereo_info_;
  nav_msgs::msg::OccupancyGrid::ConstSharedPtr pending_;
  std::deque<sensor_msgs::msg::Image::ConstSharedPtr> depths_;
  std::deque<sensor_msgs::msg::PointCloud2::ConstSharedPtr> clouds_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr info_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sparse_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr status_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr output_;
  rclcpp::TimerBase::SharedPtr timer_;
};
} // namespace thermalnav
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<thermalnav::Evaluation>());
  } catch (const std::exception &e) {
    std::cerr << e.what() << '\n';
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
