// Evaluation-only executable: GT never enters any navigation/perception node.
#include "thermalnav/core.hpp"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <cv_bridge/cv_bridge.h>
#include <fstream>
#include <iostream>
#include <algorithm>
#include <set>
#include <map>
#include <thread>
#include <cmath>
using namespace thermalnav;
int main(int argc,char **argv) {
  if(argc<3){std::cerr<<"Usage: thermalnav_baseline_evaluate GT.json detections.json [metrics.csv] [visibility_seconds]\n";return 2;}
  try {
    Json gt,records;std::ifstream a(argv[1]),b(argv[2]);a>>gt;b>>records;
    if(gt.value("purpose",std::string())!="offline_evaluation_only")throw std::runtime_error("GT file must be explicitly marked for offline evaluation");
    auto people=gt.at("people");Json result=Json::object();
    // Derive visibility independently from actual rendered semantic pixels and
    // depth. This optional evaluator is the ONLY consumer of person GT poses.
    double seconds=argc>4?std::stod(argv[4]):0.;
    if(seconds>0) {
      int ros_argc=1;rclcpp::init(ros_argc,argv);auto node=std::make_shared<rclcpp::Node>("thermalnav_offline_visibility_evaluation");
      tf2_ros::Buffer tf(node->get_clock());tf2_ros::TransformListener listener(tf);
      sensor_msgs::msg::CameraInfo::ConstSharedPtr info;std::map<int64_t,std::pair<sensor_msgs::msg::Image::ConstSharedPtr,sensor_msgs::msg::Image::ConstSharedPtr>> frames;
      std::map<int64_t,std::set<int>> label_ids;std::vector<int> visible(people.size(),0);int compared=0;auto qos=rclcpp::QoS(10).reliable();
      auto di=node->create_subscription<sensor_msgs::msg::Image>("/robot0/front_cam/depth",qos,[&](sensor_msgs::msg::Image::ConstSharedPtr m){frames[rclcpp::Time(m->header.stamp).nanoseconds()].first=m;while(frames.size()>20)frames.erase(frames.begin());});
      auto se=node->create_subscription<sensor_msgs::msg::Image>("/robot0/front_cam/semantic_segmentation",qos,[&](sensor_msgs::msg::Image::ConstSharedPtr m){frames[rclcpp::Time(m->header.stamp).nanoseconds()].second=m;while(frames.size()>20)frames.erase(frames.begin());});
      auto ci=node->create_subscription<sensor_msgs::msg::CameraInfo>("/robot0/front_cam/camera_info",qos,[&](sensor_msgs::msg::CameraInfo::ConstSharedPtr m){info=m;});
      auto la=node->create_subscription<std_msgs::msg::String>("/robot0/front_cam/semantic_segmentation_labels",qos,[&](std_msgs::msg::String::ConstSharedPtr m){try{auto j=Json::parse(m->data);int64_t time=j.at("time_stamp").at("sec").get<int64_t>()*1000000000LL+j.at("time_stamp").at("nanosec").get<int64_t>();std::set<int> ids;for(auto it=j.begin();it!=j.end();++it)if(it.value().is_object() && it.value().value("class",std::string()).rfind("person",0)==0)ids.insert(std::stoi(it.key()));label_ids[time]=std::move(ids);while(label_ids.size()>20)label_ids.erase(label_ids.begin());}catch(const std::exception &) {}});
      bool saw_active=false,finished=false;
      auto state=node->create_subscription<std_msgs::msg::String>("/thermalnav/baseline/status",rclcpp::QoS(1).transient_local().reliable(),[&](std_msgs::msg::String::ConstSharedPtr m){try{auto j=Json::parse(m->data);if(!j.value("terminal",true)&&j.value("enabled",false))saw_active=true;if(saw_active&&j.value("terminal",false))finished=true;}catch(const Json::exception &) {}});
      auto end=std::chrono::steady_clock::now()+std::chrono::milliseconds(int(seconds*1000));
      while(rclcpp::ok() && !finished && std::chrono::steady_clock::now()<end){rclcpp::spin_some(node);std::this_thread::sleep_for(std::chrono::milliseconds(5));
        if(!info)continue;
        auto it=std::find_if(frames.begin(),frames.end(),[&](auto &x){return x.second.first && x.second.second && label_ids.count(x.first);});if(it==frames.end())continue;
        const auto &ids=label_ids.at(it->first);
        auto depth=it->second.first,seg=it->second.second;
        if(depth->width!=seg->width || depth->height!=seg->height || depth->header.frame_id!=seg->header.frame_id || info->header.frame_id!=depth->header.frame_id){frames.erase(it);continue;}
        try {
          auto tr=tf.lookupTransform("odom",depth->header.frame_id,rclcpp::Time(depth->header.stamp));auto q=tr.transform.rotation;auto t=tr.transform.translation;auto R=rotation(q.x,q.y,q.z,q.w);
          cv::Mat1f d=cv_bridge::toCvCopy(depth,"32FC1")->image;cv::Mat1i mask=cv_bridge::toCvCopy(seg,"32SC1")->image;std::vector<int> frame_visible(people.size(),0);
          for(int y=0;y<d.rows;y+=4)for(int x=0;x<d.cols;x+=4)if(ids.count(mask(y,x)) && std::isfinite(d(y,x)) && d(y,x)>.15 && d(y,x)<8){double z=d(y,x);auto p=R*cv::Vec3d((x-info->k[2])*z/info->k[0],(y-info->k[5])*z/info->k[4],z)+cv::Vec3d(t.x,t.y,t.z);double best=.8;int index=-1;
            for(size_t k=0;k<people.size();++k){double dist=cv::norm(XY(p[0],p[1])-XY(people[k]["xy"][0],people[k]["xy"][1]));if(dist<best){best=dist;index=int(k);}}
            if(index>=0)++frame_visible[size_t(index)];}
          for(size_t k=0;k<visible.size();++k){if(frame_visible[k]>=3)++visible[k];}
          ++compared;frames.erase(it);
        }catch(const std::exception &){frames.erase(it);}
      }
      Json visibility=Json::array();for(size_t k=0;k<people.size();++k)visibility.push_back({{"instance_id",people[k]["instance_id"]},{"visible_frames",visible[k]},{"visible_in_evaluation_window",visible[k]>0}});
      result["visibility"]=visibility;result["visibility_compared_frames"]=compared;result["visibility_source"]="rendered_visible_semantic_pixels_and_aligned_depth";
      std::ofstream(std::string(argv[1])+".visibility.json")<<Json({{"purpose","offline_evaluation_only"},{"visibility",visibility},{"compared_frames",compared}}).dump(2);
      node.reset();rclcpp::shutdown();
    }
    {std::ifstream latest(argv[2]);if(latest)latest>>records;}
    std::vector<Json> confirmed;for(const auto &r:records)if(r.value("verified",false))confirmed.push_back(r);
    struct Pair{double distance;size_t p,d;};std::vector<Pair> pairs;
    for(size_t p=0;p<people.size();++p)for(size_t d=0;d<confirmed.size();++d){double dx=people[p]["xy"][0].get<double>()-confirmed[d]["xy"][0].get<double>(),dy=people[p]["xy"][1].get<double>()-confirmed[d]["xy"][1].get<double>();pairs.push_back({std::hypot(dx,dy),p,d});}
    std::sort(pairs.begin(),pairs.end(),[](auto x,auto y){return x.distance<y.distance;});std::set<size_t> matched_people,matched_tracks;double error=0;
    for(auto p:pairs)if(p.distance<=1. && !matched_people.count(p.p) && !matched_tracks.count(p.d)){matched_people.insert(p.p);matched_tracks.insert(p.d);error+=p.distance;}
    Json counts={{"kind","synthetic_thermal_proxy_evaluation_not_real_thermal_detection"},{"persons_in_scene",people.size()},{"confirmed_candidates",confirmed.size()},{"matched_people",matched_people.size()},{"unmatched_candidates",confirmed.size()-matched_tracks.size()},{"recall_all_placed",people.empty()?0.:double(matched_people.size())/people.size()},{"association_radius_m",1.0},{"mean_xy_error_m",matched_people.empty()?Json(nullptr):Json(error/matched_people.size())}};
    result.update(counts);
    result["per_person"]=Json::array();
    for(size_t i=0;i<people.size();++i){bool visible=false;for(auto &v:result.value("visibility",Json::array()))if(v["instance_id"]==people[i]["instance_id"])visible=v.value("visible_in_evaluation_window",false);
      int duplicates=0;for(size_t d=0;d<confirmed.size();++d)if(!matched_tracks.count(d)&&cv::norm(XY(confirmed[d]["xy"][0],confirmed[d]["xy"][1])-XY(people[i]["xy"][0],people[i]["xy"][1]))<=1.)++duplicates;
      result["per_person"].push_back({{"instance_id",people[i]["instance_id"]},{"detected",matched_people.count(i)>0},{"visible_in_recorded_window",visible},{"classification",matched_people.count(i)?"correctly_detected":visible?"visible_but_missed":"not_in_recorded_valid_view"},{"duplicate_tracks",duplicates}});
    }
    if(argc>3){std::ifstream input(argv[3]);std::string header,row;std::getline(input,header);std::getline(input,row);std::string file=std::string(argv[3])+".evaluated.csv";std::ofstream out(file);out<<header<<",persons_in_scene,person_recall_all_placed,mean_xy_error_m\n"<<row<<','<<people.size()<<','<<result["recall_all_placed"]<<','<<(matched_people.empty()?std::string():std::to_string(error/matched_people.size()))<<'\n';result["evaluated_csv"]=file;}
    std::cout<<result.dump(2)<<std::endl;return 0;
  }catch(const std::exception &e){std::cerr<<e.what()<<std::endl;return 1;}
}
