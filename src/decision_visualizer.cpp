// Read-only visualization process. No goal, safety request or velocity publishers.
#include "thermalnav/decision_view.hpp"
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <cv_bridge/cv_bridge.h>
#include <chrono>
#include <sstream>
#include <iomanip>
#include <iostream>
#include <algorithm>
#include <cmath>

namespace thermalnav {
using Grid=nav_msgs::msg::OccupancyGrid;using Odom=nav_msgs::msg::Odometry;
using Path=nav_msgs::msg::Path;using Marker=visualization_msgs::msg::Marker;using String=std_msgs::msg::String;
class DecisionVisualizer:public rclcpp::Node {
public:
  DecisionVisualizer():Node("thermalnav_decision_visualizer"){
    labels_=declare_parameter<int>("label_limit",12);enabled_=declare_parameter<bool>("enabled",true);
    double hz=declare_parameter<double>("update_hz",2.);if(hz<=0||hz>5||labels_<0||labels_>50)throw std::invalid_argument("invalid visualization limits");
    auto retained=rclcpp::QoS(1).transient_local().reliable();auto live=rclcpp::QoS(2).best_effort();
    trace_sub_=create_subscription<String>("/thermalnav/exploration/decision_trace",retained,[this](String::ConstSharedPtr m){try{auto j=Json::parse(m->data);if(j.at("frame_id")!="odom")return;trace_=std::move(j);last_sequence_=trace_.value("snapshot_sequence",uint64_t(0));}catch(const Json::exception &) {}});
    map_sub_=create_subscription<Grid>("/thermalnav/bev",retained,[this](Grid::ConstSharedPtr m){if(m->header.frame_id=="odom"&&m->data.size()==size_t(m->info.width)*m->info.height)map_=m;});
    odom_sub_=create_subscription<Odom>("/robot0/odom",live,[this](Odom::ConstSharedPtr m){if(m->header.frame_id=="odom")odom_=m;});
    path_sub_=create_subscription<Path>("/thermalnav/trajectory",retained,[this](Path::ConstSharedPtr m){if(m->header.frame_id=="odom")trajectory_=m;});
    status_sub_=create_subscription<String>("/thermalnav/baseline/status",retained,[this](String::ConstSharedPtr m){try{status_=Json::parse(m->data);}catch(const Json::exception &) {}});
    targets_sub_=create_subscription<String>("/thermalnav/thermal/detections",retained,[this](String::ConstSharedPtr m){try{targets_=Json::parse(m->data);}catch(const Json::exception &) {}});
    markers_pub_=create_publisher<visualization_msgs::msg::MarkerArray>("/thermalnav/exploration/markers",retained);
    image_pub_=create_publisher<sensor_msgs::msg::Image>("/thermalnav/bev/decision_image",retained);
    route_pub_=create_publisher<Path>("/thermalnav/exploration/selected_path",retained);
    diagnostics_pub_=create_publisher<String>("/thermalnav/exploration/visualization_status",retained);
    timer_=create_wall_timer(std::chrono::milliseconds(int(1000./hz)),[this]{tick();});
  }
private:
  static XY xy(const Json &p){return {p.at(0).get<double>(),p.at(1).get<double>()};}
  static int id(const std::string &key){uint32_t h=2166136261u;for(auto c:key){h^=uint8_t(c);h*=16777619u;}return int(h&0x7fffffffu);}
  Marker base(const std::string &ns,const std::string &key,const std_msgs::msg::Header &h){
    Marker m;m.header=h;m.ns=ns;m.id=id(key);m.action=Marker::ADD;m.pose.orientation.w=1.;m.color.a=1.;m.lifetime=rclcpp::Duration::from_seconds(2.5);return m;
  }
  void tick(){
    if(!enabled_||!map_||!odom_||trace_.empty())return;
    auto start=std::chrono::steady_clock::now();
    try{
      auto header=map_->header;XY robot(odom_->pose.pose.position.x,odom_->pose.pose.position.y);auto q=odom_->pose.pose.orientation;
      double yaw=std::atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z));
      visualization_msgs::msg::MarkerArray array;std::set<std::pair<std::string,std::string>> keys;
      auto add=[&](Marker m,const std::string &key){keys.insert({m.ns,key});array.markers.push_back(std::move(m));};
      for(const auto &g:decisionGlyphs(trace_,robot,labels_)){
        auto m=base(g.ns,g.key,header);m.type=g.text.empty()?Marker::SPHERE:Marker::TEXT_VIEW_FACING;
        m.pose.position.x=g.xy.x;m.pose.position.y=g.xy.y;m.pose.position.z=g.z;
        m.scale.x=m.scale.y=m.scale.z=g.size;m.color.r=g.rgb[0];m.color.g=g.rgb[1];m.color.b=g.rgb[2];m.text=g.text;add(m,g.key);
      }
      auto robot_marker=base("robot","live",odom_->header);robot_marker.type=Marker::ARROW;robot_marker.pose=odom_->pose.pose;
      robot_marker.scale.x=.65;robot_marker.scale.y=.12;robot_marker.scale.z=.12;robot_marker.color.r=1;add(robot_marker,"live");
      Path route;route.header=header;
      if(!trace_["selected"].is_null()){
        auto selected=trace_["selected"];XY goal=xy(selected["xy"]);auto link=base("decision_association_not_a_path","link",header);
        link.type=Marker::LINE_LIST;link.scale.x=.035;link.color.r=link.color.g=.75;link.color.b=.25;
        int n=std::max(1,int(std::ceil(cv::norm(goal-robot)/.25)));
        for(int i=0;i<n;i+=2){geometry_msgs::msg::Point a,b;XY p=robot+(goal-robot)*(double(i)/n),z=robot+(goal-robot)*(double(std::min(i+1,n))/n);a.x=p.x;a.y=p.y;a.z=.12;b.x=z.x;b.y=z.y;b.z=.12;link.points.push_back(a);link.points.push_back(b);}add(link,"link");
        for(const auto &p:selected["path"]){geometry_msgs::msg::PoseStamped pose;pose.header=header;pose.pose.orientation.w=1;pose.pose.position.x=p[0];pose.pose.position.y=p[1];pose.pose.position.z=.08;route.poses.push_back(pose);}
      }
      // Publishing an empty Path explicitly clears the previous candidate route.
      route_pub_->publish(route);
      if(!trace_["active_navigation_goal"].is_null()){
        XY p=xy(trace_["active_navigation_goal"]["xy"]);auto m=base("actual_sru_pointgoal","current",header);m.type=Marker::CYLINDER;m.pose.position.x=p.x;m.pose.position.y=p.y;m.pose.position.z=.16;m.scale.x=m.scale.y=.28;m.scale.z=.18;m.color.r=1;m.color.b=1;add(m,"current");
      }
      for(const auto &target:targets_){auto m=base("person_candidates","P"+std::to_string(target.at("id").get<int>()),header);m.type=Marker::SPHERE;XY p=xy(target["xy"]);m.pose.position.x=p.x;m.pose.position.y=p.y;m.pose.position.z=.25;m.scale.x=m.scale.y=m.scale.z=.3;m.color.r=1;m.color.g=.5;m.color.a=target.value("verified",false)?1.:.4;add(m,"P"+std::to_string(target.at("id").get<int>()));}
      auto text=base("decision_summary","current",header);text.type=Marker::TEXT_VIEW_FACING;text.pose.position.x=robot.x-3.5;text.pose.position.y=robot.y+3.;text.pose.position.z=1.5;text.scale.z=.21;text.color.r=text.color.g=text.color.b=1;
      std::ostringstream info;info<<"STATE: "<<status_.value("state",trace_.value("exploration_state",std::string()))<<"\nPOLICY: baseline_nearest | DECISION "<<trace_.value("decision_id",0)
        <<"\nSELECTED: "<<(trace_["selected"].is_null()?"none":trace_["selected"]["display_id"].get<std::string>())<<"\nREASON: "<<trace_.value("selection_reason",std::string())
        <<"\nREACHABLE: "<<trace_.value("reachable_candidate_count",0)<<" | REJECTED: "<<trace_.value("rejected_candidate_count",0)
        <<"\nREACHED: "<<status_.value("completed",0)<<" | FAILED: "<<status_.value("failed",0)
        <<std::fixed<<std::setprecision(1)<<"\nMAPPED: "<<status_.value("mapped_m2",0.)<<"m2 | SCANNED: "<<status_.value("scanned_m2",0.)<<"m2"
        <<"\nPERSON TRACKS: "<<trace_.value("confirmed_person_tracks",0)<<" (not unique GT people)";
      text.text=info.str();add(text,"current");
      for(const auto &[ns,key]:removedDecisionGlyphs(previous_keys_,keys)){auto m=base(ns,key,header);m.action=Marker::DELETE;array.markers.push_back(m);}previous_keys_=keys;markers_pub_->publish(array);
      BEV b(map_->info.resolution,std::max(map_->info.width,map_->info.height)*map_->info.resolution,{map_->info.origin.position.x,map_->info.origin.position.y});
      for(unsigned y=0;y<map_->info.height;++y)for(unsigned x=0;x<map_->info.width;++x){auto v=map_->data[size_t(y)*map_->info.width+x];if(v==0||v==100){b.seen(y,x)=255;b.log(y,x)=v==0?-3.f:3.f;}}
      std::vector<XY> history;if(trajectory_)for(auto &p:trajectory_->poses)history.push_back({p.pose.position.x,p.pose.position.y});
      std::vector<Track> tracks;for(const auto &r:targets_)tracks.push_back({r["id"],r.value("hits",1),xy(r["xy"]),r.value("first",0.),r.value("last",0.),r.value("verified",false)});
      double hfov=0,range=0;if(trace_.contains("camera")){hfov=trace_["camera"]["hfov_deg"];range=trace_["camera"]["range_m"];}
      auto image=decisionImage(b,trace_,robot,yaw,history,tracks,hfov,range);
      image_pub_->publish(*cv_bridge::CvImage(header,"bgr8",image).toImageMsg());
      double elapsed=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-start).count();
      Json diagnostics={{"snapshot_sequence",last_sequence_},{"marker_count",keys.size()},{"update_ms",elapsed},{"map_frame",header.frame_id},{"map_stamp_ns",rclcpp::Time(header.stamp).nanoseconds()},{"robot_stamp_ns",rclcpp::Time(odom_->header.stamp).nanoseconds()},{"candidate_map",trace_.value("candidate_map",Json::object())},{"render_semantics","current map with last actual decision; FOV is not scan coverage"},{"goal_or_velocity_publishers",0}};
      String message;message.data=diagnostics.dump();diagnostics_pub_->publish(message);
    }catch(const std::exception &e){RCLCPP_WARN_THROTTLE(get_logger(),*get_clock(),5000,"Visualization only: %s",e.what());}
  }
  bool enabled_{};int labels_{};uint64_t last_sequence_{};Json trace_=Json::object(),status_=Json::object(),targets_=Json::array();
  Grid::ConstSharedPtr map_;Odom::ConstSharedPtr odom_;Path::ConstSharedPtr trajectory_;
  std::set<std::pair<std::string,std::string>> previous_keys_;
  rclcpp::Subscription<String>::SharedPtr trace_sub_,status_sub_,targets_sub_;rclcpp::Subscription<Grid>::SharedPtr map_sub_;rclcpp::Subscription<Odom>::SharedPtr odom_sub_;rclcpp::Subscription<Path>::SharedPtr path_sub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_pub_;rclcpp::Publisher<Path>::SharedPtr route_pub_;rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;rclcpp::Publisher<String>::SharedPtr diagnostics_pub_;rclcpp::TimerBase::SharedPtr timer_;
};
}
int main(int argc,char **argv){rclcpp::init(argc,argv);try{rclcpp::spin(std::make_shared<thermalnav::DecisionVisualizer>());}catch(const std::exception &e){std::cerr<<e.what()<<'\n';rclcpp::shutdown();return 1;}rclcpp::shutdown();return 0;}
