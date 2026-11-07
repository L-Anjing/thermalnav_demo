#include "thermalnav/ros_observation.hpp"
#include <chrono>
#include <cv_bridge/cv_bridge.h>
#include <deque>
#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace thermalnav {
using Image = sensor_msgs::msg::Image;
using Info = sensor_msgs::msg::CameraInfo;
class StereoNode : public rclcpp::Node {
public:
  StereoNode() : Node("thermalnav_stereo"), tf_(get_clock()), listener_(tf_) {
    auto left_topic =
        declare_parameter<std::string>("left_topic", "/robot0/front_cam/rgb");
    auto right_topic = declare_parameter<std::string>(
        "right_topic", "/robot0/front_cam_right/rgb");
    auto left_info_topic = declare_parameter<std::string>(
        "left_info_topic", "/robot0/front_cam/camera_info");
    auto right_info_topic = declare_parameter<std::string>(
        "right_info_topic", "/robot0/front_cam_right/camera_info");
    auto qos = rclcpp::SensorDataQoS().keep_last(8);
    auto retained = rclcpp::QoS(1).transient_local().reliable();
    left_sub_ = create_subscription<Image>(
        left_topic, qos, [this](Image::ConstSharedPtr m) { push(left_, m); });
    right_sub_ = create_subscription<Image>(
        right_topic, qos, [this](Image::ConstSharedPtr m) { push(right_, m); });
    left_info_sub_ = create_subscription<Info>(
        left_info_topic, qos,
        [this](Info::ConstSharedPtr m) { left_info_ = m; });
    right_info_sub_ = create_subscription<Info>(
        right_info_topic, qos,
        [this](Info::ConstSharedPtr m) { right_info_ = m; });
    points_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        "/thermalnav/stereo/points", qos);
    left_gray_ =
        create_publisher<Image>("/thermalnav/stereo/left_gray", retained);
    right_gray_ =
        create_publisher<Image>("/thermalnav/stereo/right_gray", retained);
    sparse_ = create_publisher<Image>("/thermalnav/stereo/depth_sparse", qos);
    confidence_ = create_publisher<Image>("/thermalnav/stereo/confidence", qos);
    matches_ = create_publisher<Image>("/thermalnav/stereo/matches", retained);
    status_ = create_publisher<std_msgs::msg::String>(
        "/thermalnav/stereo/status", retained);
    left_rectified_info_ =
        create_publisher<Info>("/thermalnav/stereo/left_camera_info", retained);
    right_rectified_info_ = create_publisher<Info>(
        "/thermalnav/stereo/right_camera_info", retained);
    timer_ =
        create_wall_timer(std::chrono::milliseconds(20), [this] { process(); });
    status_timer_ =
        create_wall_timer(std::chrono::seconds(1), [this] { publishStatus(); });
  }

private:
  static int64_t stamp(const Image &m) {
    return rclcpp::Time(m.header.stamp).nanoseconds();
  }
  void push(std::deque<Image::ConstSharedPtr> &queue, Image::ConstSharedPtr m) {
    auto &previous =
        &queue == &left_ ? last_left_received_ : last_right_received_;
    if (previous >= 0 && stamp(*m) < previous - 500000000LL) {
      left_.clear();
      right_.clear();
      last_ = -1;
      last_left_received_ = last_right_received_ = -1;
      ++epoch_;
    }
    if (previous >= 0 && stamp(*m) <= previous)
      return;
    previous = stamp(*m);
    queue.push_back(m);
    if (queue.size() > 8) {
      queue.pop_front();
      ++dropped_;
    }
  }
  void process() {
    if (!left_info_ || !right_info_ || left_.empty() || right_.empty())
      return;
    Image::ConstSharedPtr l, r;
    for (auto it = left_.rbegin(); it != left_.rend() && !l; ++it)
      for (auto jt = right_.rbegin(); jt != right_.rend(); ++jt)
        if (stamp(**it) == stamp(**jt) && stamp(**it) > last_) {
          l = *it;
          r = *jt;
          break;
        }
    if (!l) {
      reason_ = "waiting for same-stamp stereo pair";
      return;
    }
    if (l->width != r->width || l->height != r->height ||
        l->width != left_info_->width || l->height != left_info_->height ||
        r->width != right_info_->width || r->height != right_info_->height ||
        l->header.frame_id != left_info_->header.frame_id ||
        r->header.frame_id != right_info_->header.frame_id) {
      reason_ = "stereo image/CameraInfo mismatch";
      last_ = stamp(*l);
      return;
    }
    try {
      auto transform =
          tf_.lookupTransform(r->header.frame_id, l->header.frame_id,
                              rclcpp::Time(l->header.stamp));
      auto q = transform.transform.rotation;
      auto t = transform.transform.translation;
      StereoCalibration c;
      c.size = {int(l->width), int(l->height)};
      std::copy(left_info_->k.begin(), left_info_->k.end(), c.left_K.val);
      std::copy(right_info_->k.begin(), right_info_->k.end(), c.right_K.val);
      c.left_D = left_info_->d;
      c.right_D = right_info_->d;
      c.right_from_left_R = rotation(q.x, q.y, q.z, q.w);
      c.right_from_left_t = {t.x, t.y, t.z};
      auto begin = std::chrono::steady_clock::now();
      auto result = frontend_.estimate(
          cv_bridge::toCvCopy(l, l->encoding == "mono8" ? "mono8" : "bgr8")
              ->image,
          cv_bridge::toCvCopy(r, r->encoding == "mono8" ? "mono8" : "bgr8")
              ->image,
          c);
      diagnostics_ = result.diagnostics;
      diagnostics_["processing_ms"] =
          std::chrono::duration<double, std::milli>(
              std::chrono::steady_clock::now() - begin)
              .count();
      diagnostics_["stamp"] = double(stamp(*l)) * 1e-9;
      points_->publish(pointCloud(result.points, l->header));
      left_gray_->publish(
          *cv_bridge::CvImage(l->header, "mono8", result.left_gray)
               .toImageMsg());
      right_gray_->publish(
          *cv_bridge::CvImage(r->header, "mono8", result.right_gray)
               .toImageMsg());
      sparse_->publish(
          *cv_bridge::CvImage(l->header, "32FC1", result.depth).toImageMsg());
      confidence_->publish(
          *cv_bridge::CvImage(l->header, "32FC1", result.confidence)
               .toImageMsg());
      matches_->publish(
          *cv_bridge::CvImage(l->header, "bgr8", result.matches).toImageMsg());
      auto li = *left_info_, ri = *right_info_;
      li.header = l->header;
      ri.header = r->header;
      std::copy(result.left_R.val, result.left_R.val + 9, li.r.begin());
      std::copy(result.right_R.val, result.right_R.val + 9, ri.r.begin());
      std::copy(result.left_P.val, result.left_P.val + 12, li.p.begin());
      std::copy(result.right_P.val, result.right_P.val + 12, ri.p.begin());
      li.d.assign(li.d.size(), 0.);
      ri.d.assign(ri.d.size(), 0.);
      left_rectified_info_->publish(li);
      right_rectified_info_->publish(ri);
      last_ = stamp(*l);
      ++pairs_;
      reason_.clear();
      while (!left_.empty() && stamp(*left_.front()) <= last_)
        left_.pop_front();
      while (!right_.empty() && stamp(*right_.front()) <= last_)
        right_.pop_front();
    } catch (const tf2::TransformException &) {
      reason_ = "waiting for stamped stereo TF";
    } catch (const std::exception &e) {
      reason_ = e.what();
      last_ = stamp(*l);
    }
  }
  void publishStatus() {
    auto data = diagnostics_;
    data["pairs"] = pairs_;
    data["dropped_frames"] = dropped_;
    data["epoch"] = epoch_;
    data["blocked"] = reason_;
    std_msgs::msg::String m;
    m.data = data.dump();
    status_->publish(m);
  }
  tf2_ros::Buffer tf_;
  tf2_ros::TransformListener listener_;
  StereoFrontend frontend_;
  Info::ConstSharedPtr left_info_, right_info_;
  std::deque<Image::ConstSharedPtr> left_, right_;
  int64_t last_{-1}, last_left_received_{-1}, last_right_received_{-1};
  uint64_t pairs_{}, dropped_{}, epoch_{};
  std::string reason_{"waiting for stereo cameras"};
  Json diagnostics_ = Json::object();
  rclcpp::Subscription<Image>::SharedPtr left_sub_, right_sub_;
  rclcpp::Subscription<Info>::SharedPtr left_info_sub_, right_info_sub_;
  rclcpp::Publisher<Image>::SharedPtr left_gray_, right_gray_, sparse_,
      confidence_, matches_;
  rclcpp::Publisher<Info>::SharedPtr left_rectified_info_,
      right_rectified_info_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr points_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_;
  rclcpp::TimerBase::SharedPtr timer_, status_timer_;
};
} // namespace thermalnav
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<thermalnav::StereoNode>());
  } catch (const std::exception &e) {
    std::cerr << e.what() << '\n';
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
