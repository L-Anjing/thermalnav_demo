#include "thermalnav/core.hpp"
#include "thermalnav/ros_observation.hpp"
#include <chrono>
#include <csignal>
#include <cstring>
#include <fstream>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <gtest/gtest.h>
#include <iostream>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <opencv2/imgproc.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <spawn.h>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <sys/wait.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <thread>
#include <unistd.h>

extern char **environ;
namespace {
using thermalnav::Json;
using Image = sensor_msgs::msg::Image;
using Pose = geometry_msgs::msg::PoseStamped;
using Grid = nav_msgs::msg::OccupancyGrid;
using Trigger = std_srvs::srv::Trigger;
std::string binary = THERMALNAV_BINARY;
std::string tools_binary = THERMALNAV_TOOLS_BINARY;
std::string stereo_binary = THERMALNAV_STEREO_BINARY;
std::string evaluation_binary = THERMALNAV_EVALUATION_BINARY;
double wall() {
  return std::chrono::duration<double>(
             std::chrono::steady_clock::now().time_since_epoch())
      .count();
}
double stamp(const std_msgs::msg::Header &h) {
  return h.stamp.sec + h.stamp.nanosec * 1e-9;
}

class Process {
public:
  explicit Process(const std::string &executable,
                   std::vector<std::string> arguments = {"--ros-args", "-p",
                                                         "observe:=true"}) {
    char name[] = "/tmp/thermalnav_runtime_XXXXXX";
    int fd = mkstemp(name);
    if (fd < 0)
      throw std::runtime_error("Cannot create test log");
    log_ = name;
    posix_spawn_file_actions_t actions;
    posix_spawn_file_actions_init(&actions);
    posix_spawn_file_actions_adddup2(&actions, fd, STDOUT_FILENO);
    posix_spawn_file_actions_adddup2(&actions, fd, STDERR_FILENO);
    posix_spawn_file_actions_addclose(&actions, fd);
    std::vector<char *> args{const_cast<char *>(executable.c_str())};
    for (auto &argument : arguments)
      args.push_back(argument.data());
    args.push_back(nullptr);
    int error = posix_spawn(&pid_, executable.c_str(), &actions, nullptr,
                            args.data(), environ);
    posix_spawn_file_actions_destroy(&actions);
    close(fd);
    if (error) {
      unlink(log_.c_str());
      throw std::runtime_error("Cannot launch C++ runtime: " +
                               std::string(std::strerror(error)));
    }
  }
  ~Process() {
    stop();
    unlink(log_.c_str());
  }
  bool alive() {
    if (pid_ <= 0)
      return false;
    int status;
    pid_t result = waitpid(pid_, &status, WNOHANG);
    if (result == pid_) {
      status_ = status;
      pid_ = -1;
      return false;
    }
    return result == 0;
  }
  void stop() {
    if (pid_ <= 0)
      return;
    kill(pid_, SIGINT);
    double end = wall() + 8;
    while (alive() && wall() < end)
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    if (pid_ > 0) {
      kill(pid_, SIGKILL);
      waitpid(pid_, &status_, 0);
      pid_ = -1;
    }
  }
  int exitCode() const {
    return WIFEXITED(status_) ? WEXITSTATUS(status_) : -1;
  }
  bool success() const {
    return WIFEXITED(status_) && WEXITSTATUS(status_) == 0;
  }
  std::string log() const {
    std::ifstream file(log_);
    return {std::istreambuf_iterator<char>(file), {}};
  }

private:
  pid_t pid_{-1};
  int status_{-1};
  std::string log_;
};

class RuntimeTest : public ::testing::Test {
protected:
  void SetUp() override {
    stereo_scene_ =
        std::string(
            ::testing::UnitTest::GetInstance()->current_test_info()->name())
            .find("StereoFusion") != std::string::npos;
    if (stereo_scene_ &&
        std::string(
            ::testing::UnitTest::GetInstance()->current_test_info()->name())
                .find("Only") != std::string::npos)
      send_mapping_depth_ = false;
    if (stereo_scene_)
      process_ = std::make_unique<Process>(
          binary,
          std::vector<std::string>{
              "--ros-args", "-p", "observe:=true", "-p",
              send_mapping_depth_ ? "mapping_source:=fused"
                                  : "mapping_source:=stereo_only",
              "-p", "mapping_depth_hole_fraction:=0.5", "-p",
              "mapping_depth_topic:=/robot0/d455f_depth/depth", "-p",
              "mapping_info_topic:=/robot0/d455f_depth/camera_info", "-p",
              "stereo_info_topic:=/robot0/twin612_left/camera_info"});
    else
      process_ = std::make_unique<Process>(binary);

    node_ = std::make_shared<rclcpp::Node>("thermalnav_synthetic_navigation");
    std::cout
        << "TEST ROS domain="
        << node_->get_node_base_interface()->get_context()->get_domain_id()
        << " env=" << std::getenv("ROS_DOMAIN_ID") << std::endl;
    dynamic_ = std::make_unique<tf2_ros::TransformBroadcaster>(node_);
    static_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(node_);
    auto sensor = rclcpp::SensorDataQoS();
    imu_pub_ =
        node_->create_publisher<sensor_msgs::msg::Imu>("/robot0/imu", sensor);
    auto retained = rclcpp::QoS(1).transient_local().reliable();
    stereo_status_sub_ = node_->create_subscription<std_msgs::msg::String>(
        "/thermalnav/stereo/status", retained,
        [this](std_msgs::msg::String::ConstSharedPtr m) {
          stereo_status_ = Json::parse(m->data);
        });
    depth_pub_ =
        node_->create_publisher<Image>("/robot0/front_cam/depth", rclcpp::QoS(5).reliable());
    rgb_pub_ = node_->create_publisher<Image>("/robot0/front_cam/rgb", rclcpp::QoS(5).reliable());
    info_pub_ = node_->create_publisher<sensor_msgs::msg::CameraInfo>(
        "/robot0/front_cam/camera_info", sensor);
    odom_pub_ = node_->create_publisher<nav_msgs::msg::Odometry>("/robot0/odom",
                                                                 sensor);
    cmd_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>(
        "/robot0/cmd_vel", sensor);
    status_sub_ = node_->create_subscription<std_msgs::msg::String>(
        "/thermalnav/status", retained,
        [this](std_msgs::msg::String::ConstSharedPtr m) {
          status_ = Json::parse(m->data);
        });
    grid_sub_ = node_->create_subscription<Grid>(
        "/thermalnav/bev", retained,
        [this](Grid::ConstSharedPtr m) { grid_ = m; });
    right_pub_ = node_->create_publisher<Image>(
        stereo_scene_ ? "/robot0/twin612_right/rgb"
                      : "/robot0/front_cam_right/rgb",
        rclcpp::QoS(8).reliable());
    right_info_pub_ = node_->create_publisher<sensor_msgs::msg::CameraInfo>(
        stereo_scene_ ? "/robot0/twin612_right/camera_info"
                      : "/robot0/front_cam_right/camera_info",
        sensor);
    mapping_depth_pub_ =
        node_->create_publisher<Image>("/robot0/d455f_depth/depth", rclcpp::QoS(5).reliable());
    mapping_info_pub_ = node_->create_publisher<sensor_msgs::msg::CameraInfo>(
        "/robot0/d455f_depth/camera_info", sensor);
    thermal_left_pub_ =
        node_->create_publisher<Image>("/robot0/twin612_left/rgb", rclcpp::QoS(8).reliable());
    thermal_info_pub_ = node_->create_publisher<sensor_msgs::msg::CameraInfo>(
        "/robot0/twin612_left/camera_info", sensor);
    thermal_reference_pub_ = node_->create_publisher<Image>(
        "/robot0/twin612_left/depth_reference", sensor);
    sparse_sub_ = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/thermalnav/stereo/points", sensor,
        [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr m) {
          sparse_cloud_ = m;
        });
    evaluation_sub_ = node_->create_subscription<std_msgs::msg::String>(
        "/thermalnav/evaluation", retained,
        [this](std_msgs::msg::String::ConstSharedPtr m) {
          evaluation_ = Json::parse(m->data);
        });
    goal_sub_ = node_->create_subscription<Pose>(
        "/goal_pose", 10,
        [this](Pose::ConstSharedPtr m) { goals_.push_back(*m); });
    geometry_msgs::msg::TransformStamped mount;
    mount.header.frame_id = "robot0/base_link";
    mount.child_frame_id = "robot0/front_cam";
    mount.transform.translation.x = .32487;
    mount.transform.translation.y = -.00095;
    mount.transform.translation.z = .05362;
    auto &q = mount.transform.rotation;
    q.x = -.5;
    q.y = .5;
    q.z = -.5;
    q.w = .5;
    static_->sendTransform(mount);
    auto right_mount = mount;
    right_mount.child_frame_id = "robot0/front_cam_right";
    right_mount.transform.translation.x += .12;
    static_->sendTransform(right_mount);
    if (stereo_scene_) {
      auto depth_mount = mount;
      depth_mount.child_frame_id = "robot0/d455f_depth";
      depth_mount.transform.rotation.x = 1.;
      depth_mount.transform.rotation.y = depth_mount.transform.rotation.z =
          depth_mount.transform.rotation.w = 0.;
      static_->sendTransform(depth_mount);
      auto thermal_mount = depth_mount;
      thermal_mount.child_frame_id = "robot0/twin612_left";
      thermal_mount.transform.translation.z += .1;
      static_->sendTransform(thermal_mount);
      thermal_mount.child_frame_id = "robot0/twin612_right";
      thermal_mount.transform.translation.x += .12;
      static_->sendTransform(thermal_mount);
      cv::Mat1b image(512, 640);
      cv::RNG rng(42);
      rng.fill(image, cv::RNG::UNIFORM, 0, 255);
      cv::GaussianBlur(image, image, {3, 3}, .5);
      cv::cvtColor(image, texture_left_, cv::COLOR_GRAY2RGB);
      cv::warpAffine(texture_left_, texture_right_,
                     cv::Matx23d(1, 0, -(25. / .012) * .12 / 2.5, 0, 1, 0),
                     texture_left_.size(), cv::INTER_LINEAR);
      stereo_process_ = std::make_unique<Process>(
          stereo_binary,
          std::vector<std::string>{
              "--ros-args", "-p", "left_topic:=/robot0/twin612_left/rgb", "-p",
              "right_topic:=/robot0/twin612_right/rgb", "-p",
              "left_info_topic:=/robot0/twin612_left/camera_info", "-p",
              "right_info_topic:=/robot0/twin612_right/camera_info"});
      evaluation_process_ = std::make_unique<Process>(
          evaluation_binary,
          std::vector<std::string>{
              "--ros-args", "-p", "depth_topic:=/robot0/d455f_depth/depth",
              "-p", "info_topic:=/robot0/d455f_depth/camera_info", "-p",
              "stereo_reference_topic:=/robot0/twin612_left/depth_reference",
              "-p", "stereo_info_topic:=/robot0/twin612_left/camera_info"});
    }

    timer_ = node_->create_wall_timer(std::chrono::milliseconds(100),
                                      [this] { publish(); });
  }
  void TearDown() override {
    for (auto *child : {stereo_process_.get(), evaluation_process_.get()})
      if (child) {
        child->stop();
        if (HasFailure() || !child->success())
          std::cerr << child->log();
        EXPECT_TRUE(child->success());
      }

    if (process_) {
      process_->stop();
      if (HasFailure() || !process_->success())
        std::cerr << process_->log();
      EXPECT_TRUE(process_->success());
    }
    timer_.reset();
    dynamic_.reset();
    static_.reset();
    node_.reset();
    process_.reset();
  }
  template <typename Condition>
  bool until(Condition condition, double seconds = 12) {
    double end = wall() + seconds;
    while (rclcpp::ok() && wall() < end) {
      if (!process_->alive())
        return false;
      rclcpp::spin_some(node_);
      if (condition())
        return true;
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    if (!condition()) {
      std::cerr << "ROS graph at timeout:";
      for (const auto &name : node_->get_node_names())
        std::cerr << " " << name;
      std::cerr << "; stamp=" << stamp_ << std::endl;
    }
    return condition();
  }
  void spinFor(double seconds) {
    double end = wall() + seconds;
    while (rclcpp::ok() && wall() < end) {
      rclcpp::spin_some(node_);
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  }
  bool call(const std::string &service) {
    auto client = node_->create_client<Trigger>(service);
    if (!until([&] { return client->service_is_ready(); }, 5))
      return false;
    auto future =
        client->async_send_request(std::make_shared<Trigger::Request>());
    if (!until([&] {
          return future.wait_for(std::chrono::seconds(0)) ==
                 std::future_status::ready;
        }))
      return false;
    return future.get()->success;
  }
  void publish() {
    if (!frozen_)
      stamp_ += .1;
    auto time = rclcpp::Time(static_cast<int64_t>(std::llround(stamp_ * 1e9)))
                    .
                    operator builtin_interfaces::msg::Time();
    nav_msgs::msg::Odometry odom;
    odom.header.frame_id = "odom";
    odom.header.stamp = time;
    odom.child_frame_id = "robot0/base_link";
    odom.pose.pose.position.x = x_;
    odom.pose.pose.position.y = y_;
    odom.pose.pose.position.z = stereo_scene_ ? 2.34638 : .35;
    odom.pose.pose.orientation.z = std::sin(yaw_ / 2);
    odom.pose.pose.orientation.w = std::cos(yaw_ / 2);
    odom_pub_->publish(odom);
    geometry_msgs::msg::TransformStamped tf;
    tf.header = odom.header;
    tf.child_frame_id = odom.child_frame_id;
    tf.transform.translation.x = x_;
    tf.transform.translation.y = y_;
    tf.transform.translation.z = odom.pose.pose.position.z;
    tf.transform.rotation = odom.pose.pose.orientation;
    dynamic_->sendTransform(tf);
    cmd_pub_->publish(geometry_msgs::msg::Twist());
    sensor_msgs::msg::Imu imu;
    imu.header = odom.header;
    imu.header.frame_id = "robot0/base_link";
    imu_pub_->publish(imu);
    sensor_msgs::msg::CameraInfo info;
    info.header.frame_id = "robot0/front_cam";
    info.header.stamp = time;
    info.width = bad_info_ ? 320 : 640;
    info.height = 360;
    info.k = {733., 0., 320., 0., 733., 180., 0., 0., 1.};
    info_pub_->publish(info);
    Image depth;
    depth.header = info.header;
    depth.width = 640;
    depth.height = 360;
    depth.encoding = millimeters_ ? "16UC1" : "32FC1";
    depth.step = 640 * (millimeters_ ? 2 : 4);
    depth.data.resize(depth.step * depth.height);
    for (int v = 0; v < 360; ++v)
      for (int u = 0; u < 640; ++u) {
        float value =
            invalid_depth_ ? 0.f
            : stereo_scene_
                ? 2.4f
                : float(v > 180 ? std::min(6., .40362 * 733 / (v - 180)) : 6.);
        size_t offset = size_t(v) * depth.step + u * (millimeters_ ? 2 : 4);
        if (millimeters_) {
          uint16_t mm = static_cast<uint16_t>(value * 1000);
          std::memcpy(depth.data.data() + offset, &mm, 2);
        } else
          std::memcpy(depth.data.data() + offset, &value, 4);
      }
    if (send_depth_)
      depth_pub_->publish(depth);
    Image rgb;
    rgb.header = info.header;
    rgb.width = 640;
    rgb.height = 360;
    rgb.encoding = "rgb8";
    rgb.step = 640 * 3;
    rgb.data.resize(rgb.step * rgb.height);
    rgb_pub_->publish(rgb);
    if (stereo_scene_) {
      auto mapping = depth;
      mapping.header.frame_id = "robot0/d455f_depth";
      auto mapping_info = info;
      mapping_info.header = mapping.header;
      mapping_info.k = {640. / (2 * std::tan(87 * CV_PI / 360)),
                        0,
                        320,
                        0,
                        360. / (2 * std::tan(58 * CV_PI / 360)),
                        180,
                        0,
                        0,
                        1};
      mapping_info_pub_->publish(mapping_info);
      if (send_mapping_depth_)
        mapping_depth_pub_->publish(mapping);
      auto thermal = rgb;
      thermal.header.frame_id = "robot0/twin612_left";
      thermal.height = 512;
      thermal.step = 640 * 3;
      thermal.data.resize(thermal.height * thermal.step);
      std::memcpy(thermal.data.data(), texture_left_.data, thermal.data.size());
      thermal_left_pub_->publish(thermal);
      auto thermal_info = info;
      thermal_info.header = thermal.header;
      thermal_info.height = 512;
      thermal_info.k = {25. / .012, 0, 320, 0, 25. / .012, 256, 0, 0, 1};
      thermal_info_pub_->publish(thermal_info);
      Image reference;
      reference.header = thermal.header;
      reference.width = 640;
      reference.height = 512;
      reference.encoding = "32FC1";
      reference.step = 640 * 4;
      reference.data.resize(reference.step * reference.height);
      for (size_t i = 0; i < 640 * 512; ++i) {
        float value = 2.5f;
        std::memcpy(reference.data.data() + i * 4, &value, 4);
      }
      thermal_reference_pub_->publish(reference);
      if (send_right_) {
        auto right = thermal;
        right.header.frame_id = "robot0/twin612_right";
        if (mismatch_stamp_)
          right.header.stamp =
              rclcpp::Time(rclcpp::Time(time).nanoseconds() + 1000000)
                  .
                  operator builtin_interfaces::msg::Time();
        std::memcpy(right.data.data(), texture_right_.data, right.data.size());
        right_pub_->publish(right);
        auto ri = thermal_info;
        ri.header = right.header;
        right_info_pub_->publish(ri);
      }
    }
  }

  int at(const Grid &grid, double x, double y) const {
    int col = int(
        std::floor((x - grid.info.origin.position.x) / grid.info.resolution));
    int row = int(
        std::floor((y - grid.info.origin.position.y) / grid.info.resolution));
    if (col < 0 || row < 0 || col >= int(grid.info.width) ||
        row >= int(grid.info.height))
      return -1;
    return grid.data.at(size_t(row) * grid.info.width + col);
  }
  std::unique_ptr<Process> process_;
  std::unique_ptr<Process> stereo_process_, evaluation_process_;
  rclcpp::Publisher<Image>::SharedPtr right_pub_;
  rclcpp::Publisher<Image>::SharedPtr mapping_depth_pub_, thermal_left_pub_,
      thermal_reference_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr mapping_info_pub_,
      thermal_info_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr right_info_pub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sparse_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr evaluation_sub_;
  sensor_msgs::msg::PointCloud2::ConstSharedPtr sparse_cloud_;
  Json evaluation_ = Json::object();
  Json stereo_status_ = Json::object();
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr stereo_status_sub_;
  cv::Mat3b texture_left_, texture_right_;
  bool stereo_scene_{false}, send_right_{true}, mismatch_stamp_{false},
      send_mapping_depth_{true};

  rclcpp::Node::SharedPtr node_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> dynamic_;
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> static_;
  rclcpp::Publisher<Image>::SharedPtr rgb_pub_, depth_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr info_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr status_sub_;
  rclcpp::Subscription<Grid>::SharedPtr grid_sub_;
  rclcpp::Subscription<Pose>::SharedPtr goal_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  Json status_ = Json::object();
  Grid::ConstSharedPtr grid_;
  std::vector<Pose> goals_;
  double stamp_{10.}, x_{}, y_{}, yaw_{};
  bool frozen_{false}, bad_info_{false}, millimeters_{false},
      invalid_depth_{false}, send_depth_{true};
};

TEST_F(RuntimeTest, WorldMappingTimingEncodingsAndStop) {
  ASSERT_TRUE(until([&] {
    return status_.value("map_frames", 0) >= 3 && grid_;
  })) << status_.dump();
  EXPECT_EQ(status_["runtime"], "cpp");
  EXPECT_EQ(status_["map_frame"], "odom");
  EXPECT_EQ(status_["selected"], 0);
  EXPECT_TRUE(goals_.empty());
  auto old = grid_;
  bool free = false, occupied = false;
  for (auto value : old->data) {
    ASSERT_TRUE(value == -1 || value == 0 || value == 100);
    free |= value == 0;
    occupied |= value == 100;
  }
  EXPECT_TRUE(free);
  EXPECT_TRUE(occupied);
  EXPECT_EQ(at(*old, .5, 0), -1);
  auto checksum = status_["occupancy_grid_checksum"];
  x_ = 1.;
  yaw_ = CV_PI / 2;
  ASSERT_TRUE(until([&] {
    return status_["occupancy_grid_checksum"] != checksum &&
           status_.at("last_depth").at("camera_pose_odom_m")[1].get<double>() >
               .3;
  })) << status_.dump();
  ASSERT_TRUE(
      until([&] { return stamp(grid_->header) > stamp(old->header) + .5; }));
  auto next = grid_;
  int count = 0;
  for (size_t i = 0; i < old->data.size(); ++i)
    if (old->data[i] == 100 && count++ % 50 == 0) {
      double x = old->info.origin.position.x +
                 (i % old->info.width + .5) * old->info.resolution;
      double y = old->info.origin.position.y +
                 (i / old->info.width + .5) * old->info.resolution;
      EXPECT_EQ(at(*next, x, y), 100);
    }
  millimeters_ = true;
  ASSERT_TRUE(
      until([&] { return status_["last_depth"]["encoding"] == "16UC1"; }));
  frozen_ = true;
  ASSERT_TRUE(until([&] {
    return std::abs(status_["last_depth"]["stamp"].get<double>() - stamp_) <
           1e-6;
  }));
  auto frames = status_["map_frames"];
  spinFor(2.5);
  EXPECT_EQ(status_["map_frames"], frames);
  frozen_ = false;
  ASSERT_TRUE(call("/thermalnav/start"));
  ASSERT_TRUE(until([&] { return status_.value("enabled", false); }));
  EXPECT_EQ(status_["selected"], 0);
  ASSERT_TRUE(call("/thermalnav/stop"));
  ASSERT_TRUE(until(
      [&] { return !status_.value("enabled", true) && !goals_.empty(); }));
  EXPECT_DOUBLE_EQ(goals_.back().pose.position.x, x_);
  EXPECT_DOUBLE_EQ(goals_.back().pose.position.y, y_);
  ASSERT_TRUE(call("/thermalnav/start"));
  invalid_depth_ = true;
  ASSERT_TRUE(until([&] {
    return status_["bev"].value("valid_depth", -1) == 0 &&
           status_.value("blocked", std::string()).find("stale sensors") !=
               std::string::npos;
  })) << status_.dump();
  invalid_depth_ = false;
  ASSERT_TRUE(
      until([&] { return status_["bev"].value("valid_depth", 0) > 0; }));
  send_depth_ = false;
  ASSERT_TRUE(until(
      [&] {
        return status_.value("blocked", std::string()).find("stale sensors") !=
               std::string::npos;
      },
      10));
  ASSERT_TRUE(call("/thermalnav/stop"));
  send_depth_ = true;
  bad_info_ = true;
  ASSERT_TRUE(until([&] {
    return status_.value("blocked", std::string())
               .find("depth/info mismatch") != std::string::npos;
  }));
  frames = status_["map_frames"];
  bad_info_ = false;
  ASSERT_TRUE(until([&] { return status_["map_frames"] > frames; }));
  EXPECT_EQ(status_.value("blocked", std::string()).find("depth/info mismatch"),
            std::string::npos);
  x_ = y_ = yaw_ = 0.;
  stamp_ = .5;
  ASSERT_TRUE(until([&] {
    return status_.value("epoch", 0) == 1 &&
           status_.value("map_frames", 0) >= 2;
  })) << status_.dump();
  EXPECT_LT(status_["last_depth"]["stamp"].get<double>(), 10.);
  EXPECT_EQ(status_["completed"], 0);
  // Exercise the native tools on the same isolated synthetic ROS graph.
  Process doctor(tools_binary,
                 {"--ros-args", "-p", "task:=doctor", "-p", "duration:=3.0"});
  ASSERT_TRUE(until([&] { return !doctor.alive(); }, 8));
  EXPECT_TRUE(doctor.success()) << doctor.log();
  size_t goals_before = goals_.size();
  Process manual(tools_binary, {"--ros-args", "-p", "task:=manual-check", "-p",
                                "timeout:=1.0"});
  ASSERT_TRUE(until([&] { return !manual.alive(); }, 10));
  EXPECT_EQ(manual.exitCode(), 1)
      << manual.log(); // stationary mock cannot arrive
  ASSERT_TRUE(until([&] { return goals_.size() >= goals_before + 2; }));
  EXPECT_DOUBLE_EQ(goals_.back().pose.position.x, x_);
  EXPECT_DOUBLE_EQ(goals_.back().pose.position.y, y_);
  Process validate(tools_binary, {"--ros-args", "-p", "task:=validate"});
  ASSERT_TRUE(until([&] { return status_.value("enabled", false); }, 10))
      << validate.log();
  validate.stop();
  ASSERT_TRUE(until([&] { return !status_.value("enabled", true); }));
  EXPECT_EQ(validate.exitCode(), 1) << validate.log();

  Grid::ConstSharedPtr retained;
  auto sub = node_->create_subscription<Grid>(
      "/thermalnav/bev", rclcpp::QoS(1).transient_local().reliable(),
      [&](Grid::ConstSharedPtr m) { retained = m; });
  ASSERT_TRUE(until([&] { return bool(retained); }));
  EXPECT_EQ(retained->header.frame_id, "odom");
}

TEST_F(RuntimeTest, StereoFusionAndIndependentEvaluation) {
  ASSERT_TRUE(until(
      [&] {
        return sparse_cloud_ && sparse_cloud_->width > 100 &&
               status_.value("map_frames", 0) >= 3;
      },
      25))
      << status_.dump();
  EXPECT_EQ(status_["mapping_source"], "fused");
  EXPECT_EQ(sparse_cloud_->header.frame_id, "robot0/twin612_left");
  EXPECT_EQ(status_["coverage_fraction"], 0.);
  ASSERT_TRUE(until(
      [&] {
        return status_["bev"].contains("sources") &&
               status_["bev"]["sources"].contains("stereo");
      },
      20))
      << status_.dump();
  EXPECT_GT(status_["bev"]["sources"]["stereo"]["points"].get<int>(), 100);
  EXPECT_TRUE(status_["bev"]["fused_two_sources"].get<bool>());
  EXPECT_LT(status_["bev"]["valid_depth"].get<int>(), 640 * 360);
  EXPECT_EQ(status_["selected"], 0);
  EXPECT_TRUE(goals_.empty());
  ASSERT_TRUE(until(
      [&] {
        return evaluation_.contains("stereo") &&
               evaluation_["stereo"].value("compared_points", 0) > 100;
      },
      20))
      << evaluation_.dump();
  EXPECT_LT(evaluation_["stereo"]["median_relative_error"].get<double>(), .03);
  EXPECT_LT(evaluation_["stereo"]["p95_relative_error"].get<double>(), .1);
  EXPECT_EQ(evaluation_["occupancy"]["false_free_cells"], 0);
  std::cout << "SYNTHETIC_FUSION_METRICS " << evaluation_.dump() << std::endl;
  auto points_before = stamp(sparse_cloud_->header);
  mismatch_stamp_ = true;
  spinFor(2.5);
  double last_unpaired = stamp(sparse_cloud_->header);
  spinFor(1.);
  EXPECT_DOUBLE_EQ(stamp(sparse_cloud_->header), last_unpaired);
  EXPECT_GE(last_unpaired, points_before);
  mismatch_stamp_ = false;
  ASSERT_TRUE(
      until([&] { return stamp(sparse_cloud_->header) > last_unpaired; }, 15));
  send_right_ = false;
  ASSERT_TRUE(until([&] {
    return stereo_status_.value("blocked", std::string()) == "right_stream_interrupted";
  }, 10)) << stereo_status_.dump();
  ASSERT_TRUE(until(
      [&] {
        return status_["bev"]
                   .value("stereo_status", std::string())
                   .find("fallback") != std::string::npos;
      },
      15))
      << status_.dump();
  send_right_ = true;
  ASSERT_TRUE(until([&] {
    return stereo_status_.value("blocked", std::string()).empty();
  }, 15)) << stereo_status_.dump();
  ASSERT_TRUE(until(
      [&] {
        return status_["bev"].value("stereo_status", std::string()) == "paired";
      },
      20));
  stamp_ = .5;
  ASSERT_TRUE(until(
      [&] {
        return status_.value("epoch", 0) >= 1 &&
               status_.value("map_frames", 0) >= 2 &&
               status_["bev"].value("stereo_status", std::string()) == "paired";
      },
      20))
      << status_.dump();
  EXPECT_LT(status_["last_depth"]["stamp"].get<double>(), 10.);
  ASSERT_TRUE(until(
      [&] {
        return evaluation_.contains("stamp") &&
               evaluation_["stamp"].get<double>() < 10.;
      },
      15));
  for (const auto &goal : goals_) {
    EXPECT_DOUBLE_EQ(goal.pose.position.x, x_);
    EXPECT_DOUBLE_EQ(goal.pose.position.y, y_);
  }
}

TEST_F(RuntimeTest, StereoFusionOnlyWithoutMappingDepth) {
  ASSERT_TRUE(until(
      [&] {
        return status_.value("map_frames", 0) >= 3 && sparse_cloud_ &&
               sparse_cloud_->width > 100;
      },
      25))
      << status_.dump();
  EXPECT_EQ(status_["mapping_source"], "stereo_only");
  EXPECT_EQ(status_["bev"]["valid_depth"], 0);
  ASSERT_TRUE(status_["bev"]["sources"].contains("stereo"));
  EXPECT_FALSE(status_["bev"]["sources"].contains("depth"));
  EXPECT_GT(status_["observed_m2"].get<double>(), 0.);
  EXPECT_EQ(status_["selected"], 0);
  EXPECT_TRUE(goals_.empty());
}
} // namespace

int main(int argc, char **argv) {
  std::vector<char *> remaining{argv[0]};
  for (int i = 1; i < argc; ++i) {
    if (std::string(argv[i]) == "--binary" && i + 1 < argc)
      binary = argv[++i];
    else if (std::string(argv[i]) == "--tools-binary" && i + 1 < argc)
      tools_binary = argv[++i];
    else if (std::string(argv[i]) == "--stereo-binary" && i + 1 < argc)
      stereo_binary = argv[++i];
    else if (std::string(argv[i]) == "--evaluation-binary" && i + 1 < argc)
      evaluation_binary = argv[++i];
    else
      remaining.push_back(argv[i]);
  }
  int count = int(remaining.size());
  remaining.push_back(nullptr);
  ::testing::InitGoogleTest(&count, remaining.data());
  auto domain = std::to_string(130 + getpid() % 60);
  setenv("ROS_DOMAIN_ID", domain.c_str(), 1);
  setenv("ROS_LOCALHOST_ONLY", "1", 1);
  // This isolated test uses localhost discovery. Inheriting a workstation URI
  // that also sets deprecated NetworkInterfaceAddress conflicts with the
  // localhost-only interface configuration generated by rmw_cyclonedds.
  unsetenv("CYCLONEDDS_URI");
  setenv("ROS_LOG_DIR", "/tmp/thermalnav_test_ros", 1);
  int ros_argc = 1;
  rclcpp::init(ros_argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
