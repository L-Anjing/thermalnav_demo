#include "thermalnav/core.hpp"
#include <chrono>
#include <csignal>
#include <cv_bridge/cv_bridge.h>
#include <future>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <iostream>
#include <limits>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <thread>

namespace {
volatile std::sig_atomic_t interrupted = 0;
void stopSignal(int) { interrupted = 1; }
double wall() {
  return std::chrono::duration<double>(
             std::chrono::steady_clock::now().time_since_epoch())
      .count();
}
double stamp(const std_msgs::msg::Header &h) {
  return h.stamp.sec + h.stamp.nanosec * 1e-9;
}
using thermalnav::Json;
using thermalnav::XY;
using Odom = nav_msgs::msg::Odometry;
using Image = sensor_msgs::msg::Image;
using Pose = geometry_msgs::msg::PoseStamped;
using Trigger = std_srvs::srv::Trigger;

class Tools : public rclcpp::Node {
public:
  Tools() : Node("thermalnav_tools"), tf_(get_clock()), listener_(tf_) {
    task_ = declare_parameter<std::string>("task", "doctor");
    duration_ = declare_parameter<double>("duration", 10.);
    distance_ = declare_parameter<double>("distance", 1.5);
    timeout_ = declare_parameter<double>("timeout", 150.);
    if (!std::isfinite(duration_) || duration_ <= 0 ||
        !std::isfinite(distance_) || distance_ <= .85 ||
        !std::isfinite(timeout_) || timeout_ <= 0)
      throw std::invalid_argument("invalid duration, distance or timeout");
    auto sensor = rclcpp::SensorDataQoS();
    auto retained = rclcpp::QoS(1).transient_local().reliable();
    rgb_sub_ = create_subscription<Image>("/robot0/front_cam/rgb", sensor,
                                          [this](Image::ConstSharedPtr m) {
                                            rgb_ = m;
                                            ++rgb_count_;
                                          });
    depth_sub_ = create_subscription<Image>("/robot0/front_cam/depth", sensor,
                                            [this](Image::ConstSharedPtr m) {
                                              depth_ = m;
                                              ++depth_count_;
                                            });
    info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
        "/robot0/front_cam/camera_info", sensor,
        [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr m) {
          info_ = m;
          ++info_count_;
        });
    odom_sub_ = create_subscription<Odom>(
        "/robot0/odom", sensor, [this](Odom::ConstSharedPtr m) {
          const auto &p = m->pose.pose.position;
          const auto &q = m->pose.pose.orientation;
          if (m->header.frame_id != "odom" || !std::isfinite(p.x) ||
              !std::isfinite(p.y) || !std::isfinite(p.z))
            return;
          try {
            thermalnav::rotation(q.x, q.y, q.z, q.w);
          } catch (const std::exception &) {
            return;
          }
          odom_ = m;
          odom_wall_ = wall();
          ++odom_count_;
        });
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
        "/robot0/imu", sensor,
        [this](sensor_msgs::msg::Imu::ConstSharedPtr) { ++imu_count_; });
    cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
        "/robot0/cmd_vel", sensor,
        [this](geometry_msgs::msg::Twist::ConstSharedPtr m) {
          cmd_speed_ = std::abs(m->linear.x) + std::abs(m->linear.y) +
                       std::abs(m->angular.z);
          cmd_wall_ = wall();
        });
    status_sub_ = create_subscription<std_msgs::msg::String>(
        "/thermalnav/status", retained,
        [this](std_msgs::msg::String::ConstSharedPtr m) {
          try {
            auto data = Json::parse(m->data);
            if (data.is_object()) {
              status_ = data;
              status_wall_ = wall();
            }
          } catch (const Json::exception &) {
          }
        });
    if (task_ == "manual-check")
      goal_pub_ = create_publisher<Pose>("/goal_pose", 1);
    if (task_ == "validate") {
      start_ = create_client<Trigger>("/thermalnav/start");
      stop_ = create_client<Trigger>("/thermalnav/stop");
    }
  }
  int run() {
    if (task_ == "doctor")
      return doctor();
    if (task_ == "startup-audit") {
      if (!wait([this] { return freshStatus(); }, duration_))
        throw std::runtime_error("No fresh ThermalNav status");
      std::cout << "certificate=" << status_.value("certificate", "unavailable")
                << " startup_footprint="
                << status_.value("startup_footprint", "unavailable") << '\n';
      return status_.value("certificate", "unavailable") == "certified" ? 0 : 1;
    }
    if (task_ == "manual-check")
      return manualCheck();
    if (task_ == "validate")
      return validate();
    throw std::invalid_argument(
        "task must be doctor, startup-audit, manual-check or validate");
  }
  void cancelMotion() {
    if (!motion_started_)
      return;
    if (goal_pub_ && odom_) {
      Pose goal;
      goal.header = odom_->header;
      goal.pose = odom_->pose.pose;
      goal_pub_->publish(goal);
      spinFor(1.5, true);
    } else if (stop_) {
      if (!call(stop_, true))
        std::cerr << "FAIL: ThermalNav stop service did not acknowledge "
                     "cancellation\n";
    }
    motion_started_ = false;
  }

private:
  template <typename Condition>
  bool wait(Condition condition, double seconds, bool cancelling = false) {
    double end = wall() + seconds;
    while (rclcpp::ok() && (cancelling || !interrupted) && wall() < end) {
      rclcpp::spin_some(shared_from_this());
      if (condition())
        return true;
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    return condition();
  }
  void spinFor(double seconds, bool cancelling = false) {
    wait([] { return false; }, seconds, cancelling);
  }
  bool freshStatus() const {
    return status_.is_object() && !status_.empty() && wall() - status_wall_ < 3;
  }
  XY robot() const {
    return {odom_->pose.pose.position.x, odom_->pose.pose.position.y};
  }
  bool call(rclcpp::Client<Trigger>::SharedPtr client,
            bool cancelling = false) {
    if (!wait([&] { return client->service_is_ready(); }, 5, cancelling))
      return false;
    auto future =
        client->async_send_request(std::make_shared<Trigger::Request>());
    if (!wait(
            [&] {
              return future.wait_for(std::chrono::seconds(0)) ==
                     std::future_status::ready;
            },
            5, cancelling))
      return false;
    return future.get()->success;
  }
  int doctor() {
    spinFor(duration_);
    bool ok = true;
    for (auto item : {std::pair<const char *, int>{"rgb", rgb_count_},
                      {"depth", depth_count_},
                      {"camera_info", info_count_},
                      {"odom", odom_count_},
                      {"imu", imu_count_}}) {
      bool pass = item.second > 1;
      ok &= pass;
      std::cout << (pass ? "PASS " : "FAIL ") << item.first
                << " samples=" << item.second << '\n';
    }
    if (depth_) {
      try {
        cv::Mat1f depth;
        if (depth_->encoding == "32FC1")
          depth = cv_bridge::toCvCopy(depth_, "32FC1")->image;
        else if (depth_->encoding == "16UC1")
          cv_bridge::toCvShare(depth_, "16UC1")
              ->image.convertTo(depth, CV_32F, .001);
        else
          throw std::runtime_error("unsupported depth encoding");
        int valid = 0;
        double lo = std::numeric_limits<double>::infinity(), hi = 0;
        for (int y = 0; y < depth.rows; ++y)
          for (int x = 0; x < depth.cols; ++x) {
            double d = depth(y, x);
            if (std::isfinite(d) && d >= .15 && d <= 8.) {
              ++valid;
              lo = std::min(lo, d);
              hi = std::max(hi, d);
            }
          }
        ok &= valid > 0;
        std::cout << (valid ? "PASS " : "FAIL ") << "depth " << depth_->encoding
                  << ' ' << depth_->width << 'x' << depth_->height
                  << " valid=" << valid << " range_m=" << lo << ',' << hi
                  << '\n';
        tf_.lookupTransform("odom", depth_->header.frame_id,
                            rclcpp::Time(depth_->header.stamp));
        std::cout << "PASS stamped camera TF\n";
      } catch (const std::exception &e) {
        ok = false;
        std::cout << "FAIL depth/TF: " << e.what() << '\n';
      }
    }
    bool matching =
        depth_ && info_ && depth_->header.frame_id == info_->header.frame_id &&
        depth_->width == info_->width && depth_->height == info_->height &&
        std::isfinite(info_->k[0]) && std::isfinite(info_->k[4]) &&
        info_->k[0] > 0 && info_->k[4] > 0;
    ok &= matching;
    std::cout << (matching ? "PASS " : "FAIL ")
              << "depth/CameraInfo geometry\n";
    if (depth_ && odom_) {
      double skew = std::abs(stamp(depth_->header) - stamp(odom_->header));
      ok &= skew <= .25;
      std::cout << (skew <= .25 ? "PASS " : "FAIL ")
                << "depth/odom skew_s=" << skew << '\n';
    }
    bool controller = false;
    for (const auto &sub : get_subscriptions_info_by_topic("/goal_pose"))
      controller |= sub.node_name().find("navigation") != std::string::npos ||
                    sub.node_name().find("rl_nav") != std::string::npos;
    ok &= controller;
    std::cout << (controller ? "PASS " : "FAIL ")
              << "SRU PointGoal subscriber\n";
    auto publishers = get_publishers_info_by_topic("/robot0/cmd_vel");
    ok &= publishers.size() == 1;
    std::cout << (publishers.size() == 1 ? "PASS " : "FAIL ")
              << "cmd_vel publishers=" << publishers.size() << '\n';
    std::cout
        << "Go2: keep use_sim_time=false; stamped TF uses sensor timestamps.\n";
    return ok && !interrupted ? 0 : 1;
  }
  int manualCheck() {
    if (!wait(
            [this] {
              return odom_ && wall() - odom_wall_ < 2 &&
                     goal_pub_->get_subscription_count() > 0;
            },
            15))
      throw std::runtime_error("No fresh odom or PointGoal subscriber");
    if (!wait([this] { return freshStatus(); }, 3) ||
        status_.value("enabled", true))
      throw std::runtime_error(
          "Run the demo in observe mode before manual-check");
    auto q = odom_->pose.pose.orientation;
    double yaw = std::atan2(2 * (q.w * q.z + q.x * q.y),
                            1 - 2 * (q.y * q.y + q.z * q.z));
    XY initial = robot(),
       target = initial + distance_ * XY(std::cos(yaw), std::sin(yaw));
    Pose goal;
    goal.header = odom_->header;
    goal.pose.position = odom_->pose.pose.position;
    goal.pose.position.x = target.x;
    goal.pose.position.y = target.y;
    goal.pose.orientation.w = 1.;
    motion_started_ = true;
    goal_pub_->publish(goal);
    std::cout << "PointGoal " << target << std::endl;
    double end = wall() + timeout_, report = 0;
    bool moved = false, nonzero = false, passed = false;
    while (rclcpp::ok() && !interrupted && wall() < end) {
      spinFor(.05);
      if (!odom_ || wall() - odom_wall_ > 3 || wall() - cmd_wall_ > 2 ||
          !std::isfinite(cmd_speed_))
        break;
      double displacement = cv::norm(robot() - initial),
             error = cv::norm(robot() - target);
      moved |= displacement > .15;
      nonzero |= cmd_speed_ > .05;
      if (wall() > report) {
        std::cout << "displacement=" << displacement << " error=" << error
                  << " command=" << cmd_speed_ << std::endl;
        report = wall() + 10;
      }
      if (moved && nonzero && error <= .75 && cmd_speed_ < .02) {
        passed = true;
        break;
      }
    }
    cancelMotion();
    std::cout << (passed ? "PASS" : "FAIL")
              << " motion + arrival + zero command\n";
    return passed ? 0 : 1;
  }
  int validate() {
    if (!wait(
            [this] {
              return freshStatus() && status_.value("map_frames", 0) >= 8;
            },
            180))
      throw std::runtime_error("No initialized sensor map");
    if (status_.value("enabled", true))
      throw std::runtime_error(
          "Start the demo in observe mode before validate");
    std::cout << "STATIONARY " << status_.dump() << std::endl;
    // Arm cleanup before the RPC: an interrupted reply must still stop a start
    // request that the server may already have executed.
    motion_started_ = true;
    if (!call(start_))
      throw std::runtime_error("Exploration start failed");
    double begun = wall(), end = begun + 240, report = 0;
    bool passed = false;
    while (rclcpp::ok() && !interrupted && wall() < end) {
      spinFor(.1);
      if (!freshStatus())
        break;
      if (wall() > report) {
        std::cout << status_.dump() << std::endl;
        report = wall() + 20;
      }
      if (status_.value("completed_by_kind", Json::object())
              .value("frontier", 0) >= 3) {
        passed = true;
        break;
      }
      if (wall() - begun > 30 &&
          status_.value("blocked", std::string()).rfind("no safe", 0) == 0)
        break;
    }
    cancelMotion();
    std::cout << "FINAL " << status_.dump() << std::endl;
    return passed ? 0 : 1;
  }
  std::string task_;
  double duration_, distance_, timeout_;
  tf2_ros::Buffer tf_;
  tf2_ros::TransformListener listener_;
  Image::ConstSharedPtr rgb_, depth_;
  sensor_msgs::msg::CameraInfo::ConstSharedPtr info_;
  Odom::ConstSharedPtr odom_;
  Json status_ = Json::object();
  int rgb_count_{}, depth_count_{}, info_count_{}, odom_count_{}, imu_count_{};
  double odom_wall_{}, cmd_wall_{}, status_wall_{}, cmd_speed_{};
  bool motion_started_{false};
  rclcpp::Subscription<Image>::SharedPtr rgb_sub_, depth_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr info_sub_;
  rclcpp::Subscription<Odom>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr status_sub_;
  rclcpp::Publisher<Pose>::SharedPtr goal_pub_;
  rclcpp::Client<Trigger>::SharedPtr start_, stop_;
};
} // namespace

int main(int argc, char **argv) {
  rclcpp::init(argc, argv, rclcpp::InitOptions(),
               rclcpp::SignalHandlerOptions::None);
  std::signal(SIGINT, stopSignal);
  std::signal(SIGTERM, stopSignal);
  std::shared_ptr<Tools> tools;
  int result = 1;
  try {
    tools = std::make_shared<Tools>();
    result = tools->run();
  } catch (const std::exception &e) {
    std::cerr << "ThermalNav tools: " << e.what() << '\n';
  }
  if (tools)
    tools->cancelMotion();
  tools.reset();
  rclcpp::shutdown();
  return result;
}
