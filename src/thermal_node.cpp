#include "thermalnav/synthetic_thermal.hpp"
#include <cv_bridge/cv_bridge.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/msg/marker_array.hpp>
#include <opencv2/imgproc.hpp>
#include <filesystem>
#include <fstream>
#include <map>
#include <algorithm>
#include <iostream>
#include <limits>

namespace thermalnav {
using Image=sensor_msgs::msg::Image;
using Info=sensor_msgs::msg::CameraInfo;
using String=std_msgs::msg::String;
using Header=std_msgs::msg::Header;
int64_t thermalStamp(const Header &h) {return rclcpp::Time(h.stamp).nanoseconds();}
struct ThermalFrame {Image::ConstSharedPtr rgb,depth,semantic,raw;};
class ThermalNode: public rclcpp::Node {
public:
  explicit ThermalNode(bool detector):Node(detector?"thermalnav_thermal_detector":"thermalnav_thermal"),detector_(detector),tf_(get_clock()),listener_(tf_) {
    auto qos=rclcpp::QoS(10).reliable();auto retained=rclcpp::QoS(1).transient_local().reliable();
    auto prefix=declare_parameter<std::string>("camera_prefix","/robot0/front_cam");
    options_.mode=declare_parameter<std::string>("mode","ideal");
    options_.seed=declare_parameter<int>("seed",42);
    options_.person=declare_parameter<double>("person_response",34.);
    options_.background=declare_parameter<double>("background_response",20.);
    options_.wall=declare_parameter<double>("wall_response",20.);
    options_.floor=declare_parameter<double>("floor_response",19.);
    options_.furniture=declare_parameter<double>("furniture_response",22.);
    options_.equipment=declare_parameter<double>("equipment_response",27.);
    options_.noise_sigma=declare_parameter<double>("noise_sigma",.25);
    options_.blur_sigma=declare_parameter<double>("blur_sigma",.8);
    options_.downsample=declare_parameter<int>("downsample",2);
    options_.min_depth=declare_parameter<double>("min_depth",.15);
    options_.max_depth=declare_parameter<double>("max_depth",8.);
    options_.nonuniformity=declare_parameter<double>("background_nonuniformity",.35);
    options_.texture_amplitude=declare_parameter<double>("rgb_texture_amplitude",.2);
    options_.person_variation=declare_parameter<double>("person_spatial_amplitude",.6);
    options_.challenging_contrast=declare_parameter<double>("challenging_contrast",.55);
    options_.warm_offset=declare_parameter<double>("challenging_warm_offset",1.);
    display_min_=declare_parameter<double>("display_min",15.);
    display_max_=declare_parameter<double>("display_max",40.);
    if(display_max_<=display_min_ || options_.max_depth<=options_.min_depth || options_.min_depth<=0)throw std::invalid_argument("invalid thermal range");
    options_.threshold=declare_parameter<double>("threshold",30.);
    options_.min_area=declare_parameter<int>("min_area",24);
    records_=declare_parameter<std::string>("records_file","/tmp/thermalnav_baseline/detections.json");
    auto input=[&](std::string topic, int kind) {
      image_subs_.push_back(create_subscription<Image>(topic,qos,[this,kind](Image::ConstSharedPtr m) {
        auto t=thermalStamp(m->header);
        auto &prev=input_stamps_[kind];
        if(prev>0 && t<prev-500000000LL) {frames_.clear();label_frames_.clear();tracks_.clear();heights_.clear();last_=-1;++epoch_;}
        prev=t; auto &f=frames_[t];
        if(kind==0) f.rgb=m; else if(kind==1) f.depth=m;else if(kind==2) f.semantic=m;else f.raw=m;
        while(frames_.size()>20)frames_.erase(frames_.begin());
      }));
    };
    const std::string output="/thermalnav/thermal";
    if(!detector_) {
      input(prefix+"/rgb",0);input(prefix+"/depth",1);input(prefix+"/semantic_segmentation",2);
      labels_sub_=create_subscription<String>(prefix+"/semantic_segmentation_labels",qos,[this](String::ConstSharedPtr m) {
        try {auto j=Json::parse(m->data);std::map<int,std::string> next;
          for(auto it=j.begin();it!=j.end();++it) {
            if(!it.value().is_object() || !it.value().contains("class"))continue;
            try {std::string c=it.value()["class"];std::transform(c.begin(),c.end(),c.begin(),[](unsigned char v){return std::tolower(v);});
              if(c.rfind("person",0)==0)c="person";
              next[std::stoi(it.key())]=c;
            }catch(const std::exception &){}
          }
          if(!j.contains("time_stamp"))throw std::runtime_error("semantic labels require renderer timestamp");
          int64_t time=j["time_stamp"].at("sec").get<int64_t>()*1000000000LL+j["time_stamp"].at("nanosec").get<int64_t>();
          classes_=next;label_frames_[time]=std::move(next);
          while(label_frames_.size()>20)label_frames_.erase(label_frames_.begin());
        }catch(const std::exception &e){blocked_=e.what();}
      });
      raw_pub_=create_publisher<Image>(output+"/raw",qos);
      color_pub_=create_publisher<Image>(output+"/colorized",retained);
      depth_pub_=create_publisher<Image>(output+"/depth",qos);
      rgb_pub_=create_publisher<Image>(output+"/rgb",qos);
      info_pub_=create_publisher<Info>(output+"/camera_info",retained);
    } else {
      input(output+"/raw",3);input(output+"/depth",1);
      detections_pub_=create_publisher<String>(output+"/detections",retained);
      boxes_pub_=create_publisher<String>(output+"/boxes",qos);
      mask_pub_=create_publisher<Image>(output+"/hot_mask",qos);
      markers_pub_=create_publisher<visualization_msgs::msg::MarkerArray>(output+"/markers",retained);
    }
    info_sub_=create_subscription<Info>(detector_?output+"/camera_info":prefix+"/camera_info",qos,[this](Info::ConstSharedPtr m){info_=m;});
    status_pub_=create_publisher<String>(output+(detector_?"/detector_status":"/status"),retained);
    timer_=create_wall_timer(std::chrono::milliseconds(100),[this]{process();});
    status_timer_=create_wall_timer(std::chrono::seconds(1),[this]{status();});
  }
private:
  void process() {
    if(!info_){blocked_="missing CameraInfo";return;}
    if(!detector_ && classes_.empty()){blocked_="missing semantic labels";return;}
    auto selected=frames_.end();
    for(auto it=frames_.begin();it!=frames_.end();++it)
      if(it->first>last_ && it->second.depth && (detector_?bool(it->second.raw):bool(it->second.rgb && it->second.semantic && label_frames_.count(it->first))))selected=it;
    if(selected==frames_.end()){blocked_="waiting for exact-stamp aligned inputs";return;}
    auto f=selected->second;auto anchor=detector_?f.raw:f.rgb;
    if(anchor->header.frame_id!=f.depth->header.frame_id || anchor->header.frame_id!=info_->header.frame_id ||
       anchor->width!=f.depth->width || anchor->height!=f.depth->height || anchor->width!=info_->width || anchor->height!=info_->height ||
       (!detector_ && (f.semantic->header.frame_id!=anchor->header.frame_id || f.semantic->width!=anchor->width || f.semantic->height!=anchor->height))) {
      blocked_="image/CameraInfo geometry mismatch";last_=selected->first;return;
    }
    try {
      cv::Mat1f depth;
      if(f.depth->encoding=="32FC1")depth=cv_bridge::toCvCopy(f.depth,"32FC1")->image;
      else if(f.depth->encoding=="16UC1")cv_bridge::toCvShare(f.depth,"16UC1")->image.convertTo(depth,CV_32F,.001);
      else throw std::runtime_error("unsupported depth encoding");
      if(!detector_) {
        auto ids=cv_bridge::toCvCopy(f.semantic,"32SC1")->image;
        auto rgb=cv_bridge::toCvCopy(f.rgb,"bgr8")->image;
        const auto &frame_classes=label_frames_.at(selected->first);
        auto signal=syntheticThermal(rgb,depth,ids,frame_classes,options_,uint64_t(selected->first));
        visible_pixels_=0;
        for(auto &[id,c]:frame_classes)if(c=="person")visible_pixels_+=cv::countNonZero(ids==id);
        raw_pub_->publish(*cv_bridge::CvImage(anchor->header,"32FC1",signal).toImageMsg());
        cv::Mat1b display;cv::Mat clean=signal.clone();cv::patchNaNs(clean,display_min_);clean.convertTo(display,CV_8U,255./(display_max_-display_min_),-display_min_*255./(display_max_-display_min_));
        cv::Mat3b color;cv::applyColorMap(display,color,cv::COLORMAP_INFERNO);
        color_pub_->publish(*cv_bridge::CvImage(anchor->header,"bgr8",color).toImageMsg());
        // Only frames for which a valid, aligned thermal observation exists
        // certify scan coverage. Invalid signal pixels remain invalid depth.
        depth.setTo(std::numeric_limits<float>::quiet_NaN(),signal!=signal);
        depth_pub_->publish(*cv_bridge::CvImage(anchor->header,"32FC1",depth).toImageMsg());
        rgb_pub_->publish(*f.rgb);auto info=*info_;info.header=anchor->header;info_pub_->publish(info);
      } else {
        // This node has no semantic/instance/GT subscription or asset access.
        auto signal=cv_bridge::toCvCopy(f.raw,"32FC1")->image;
        auto transform=tf_.lookupTransform("odom",anchor->header.frame_id,rclcpp::Time(anchor->header.stamp));
        auto q=transform.transform.rotation;auto p=transform.transform.translation;
        auto R=rotation(q.x,q.y,q.z,q.w);cv::Vec3d t(p.x,p.y,p.z);cv::Matx33d K;std::copy(info_->k.begin(),info_->k.end(),K.val);
        auto boxes=thermalRegions(signal,options_);last_boxes_=boxes.size();
        cv::Mat1b hot=signal>options_.threshold;
        mask_pub_->publish(*cv_bridge::CvImage(anchor->header,"mono8",hot).toImageMsg());
        Json box_records=Json::array();
        for(const auto &box:boxes)box_records.push_back({{"bbox_xywh",{box.x,box.y,box.width,box.height}},{"response_contrast",cv::mean(signal(box),hot(box))[0]-options_.threshold}});
        String box_message;box_message.data=Json({{"source","thermal_proxy_threshold"},{"stamp_ns",selected->first},{"frame_id",anchor->header.frame_id},{"boxes",box_records}}).dump();boxes_pub_->publish(box_message);
        for(const auto &box:boxes) {
          auto world=localizeThermalRegion(box,signal,depth,K,R,t,options_.threshold,options_.min_depth,options_.max_depth);if(!world)continue;
          addTrack(tracks_,{(*world)[0],(*world)[1]},selected->first*1e-9);
          for(const auto &track:tracks_)if(cv::norm(track.xy-XY((*world)[0],(*world)[1]))<.8)heights_[track.id]=(*world)[2];
        }
        Json records=trackJson(tracks_);
        for(auto &record:records){record["label"]="suspected_person";record["source"]="thermal_proxy_threshold";record["frame_id"]="odom";record["sensor_frame"]=anchor->header.frame_id;record["time_source"]="sensor_sim_time";record["confidence"]=std::min(1.,record["hits"].get<int>()/5.);record["confidence_type"]="temporal_consistency_proxy";record["z"]=heights_[record["id"].get<int>()];record["xyz"]={record["xy"][0],record["xy"][1],record["z"]};}
        String msg;msg.data=records.dump();detections_pub_->publish(msg);
        std::filesystem::path file(records_);if(file.has_parent_path())std::filesystem::create_directories(file.parent_path());
        auto tmp=file;tmp+=".tmp";std::ofstream(tmp)<<records.dump(2);std::filesystem::rename(tmp,file);
        visualization_msgs::msg::MarkerArray markers;
        for(const auto &track:tracks_){visualization_msgs::msg::Marker m;m.header=anchor->header;m.header.frame_id="odom";m.ns="thermal_proxy_candidates";m.id=track.id;m.type=m.SPHERE;m.action=m.ADD;m.pose.orientation.w=1.;m.pose.position.x=track.xy.x;m.pose.position.y=track.xy.y;m.pose.position.z=heights_[track.id];m.scale.x=m.scale.y=m.scale.z=.25;m.color.r=1.;m.color.g=.45;m.color.a=1.;markers.markers.push_back(m);}markers_pub_->publish(markers);
      }
      last_=selected->first;++processed_;last_frame_wall_=std::chrono::steady_clock::now();blocked_.clear();
      frames_.erase(frames_.begin(),frames_.upper_bound(last_));
      label_frames_.erase(label_frames_.begin(),label_frames_.upper_bound(last_));
    }catch(const std::exception &e){blocked_=e.what();}
  }
  void status(){String m;m.data=Json({{"source",detector_?"thermal_proxy_threshold":"semantic_conditioned_thermal_proxy"},{"processed_frames",processed_},{"epoch",epoch_},{"stamp_ns",last_},{"frame_age_s",processed_?std::chrono::duration<double>(std::chrono::steady_clock::now()-last_frame_wall_).count():1e9},{"blocked",blocked_},{"mode",options_.mode},{"visible_person_pixels",visible_pixels_},{"boxes",last_boxes_},{"tracks",tracks_.size()},{"units","proxy_signal_not_radiometric_temperature"}}).dump();status_pub_->publish(m);}
  bool detector_;ThermalOptions options_;tf2_ros::Buffer tf_;tf2_ros::TransformListener listener_;
  double display_min_{15.},display_max_{40.};
  std::map<int64_t,ThermalFrame> frames_;std::map<int,int64_t> input_stamps_;std::map<int,std::string> classes_;
  std::map<int64_t,std::map<int,std::string>> label_frames_;
  std::vector<Track> tracks_;int64_t last_{-1};uint64_t processed_{},epoch_{};int visible_pixels_{},last_boxes_{};
  std::map<int,double> heights_;
  std::string records_,blocked_{"waiting for sensors"};Info::ConstSharedPtr info_;
  std::vector<rclcpp::Subscription<Image>::SharedPtr> image_subs_;
  rclcpp::Subscription<Info>::SharedPtr info_sub_;rclcpp::Subscription<String>::SharedPtr labels_sub_;
  rclcpp::Publisher<Image>::SharedPtr raw_pub_,color_pub_,depth_pub_,rgb_pub_;rclcpp::Publisher<Info>::SharedPtr info_pub_;
  rclcpp::Publisher<String>::SharedPtr status_pub_,detections_pub_;
  rclcpp::Publisher<String>::SharedPtr boxes_pub_;rclcpp::Publisher<Image>::SharedPtr mask_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_pub_;
  rclcpp::TimerBase::SharedPtr timer_,status_timer_;
  std::chrono::steady_clock::time_point last_frame_wall_;
};
} // namespace thermalnav
int main(int argc,char **argv){rclcpp::init(argc,argv);try{
#ifdef THERMALNAV_DETECTOR
  rclcpp::spin(std::make_shared<thermalnav::ThermalNode>(true));
#else
  rclcpp::spin(std::make_shared<thermalnav::ThermalNode>(false));
#endif
}catch(const std::exception &e){std::cerr<<e.what()<<std::endl;rclcpp::shutdown();return 1;}rclcpp::shutdown();return 0;}
