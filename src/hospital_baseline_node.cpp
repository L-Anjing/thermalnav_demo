#include "thermalnav/synthetic_thermal.hpp"
#include "thermalnav/baseline_search.hpp"
#include "thermalnav/decision_view.hpp"
#include <sensor_msgs/msg/camera_info.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <cv_bridge/cv_bridge.h>
#include <filesystem>
#include <fstream>
#include <set>
#include <algorithm>
#include <iostream>
#include <csignal>
#include <thread>

namespace thermalnav {
using Grid=nav_msgs::msg::OccupancyGrid;using Odom=nav_msgs::msg::Odometry;
using Pose=geometry_msgs::msg::PoseStamped;using String=std_msgs::msg::String;
double baselineWall(){return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();}
class HospitalBaseline:public rclcpp::Node {
public:
  HospitalBaseline():Node("thermalnav_baseline"),tf_(get_clock()),tf_listener_(tf_) {
    bounds_=declare_parameter<std::vector<double>>("search_bounds",{-4.,15.,-3.,9.});
    if(bounds_.size()!=4 || bounds_[0]>=bounds_[1] || bounds_[2]>=bounds_[3])throw std::invalid_argument("invalid search bounds");
    footprint_=declare_parameter<double>("footprint_radius",.35);
    timeout_=declare_parameter<double>("goal_timeout",90.);progress_timeout_=declare_parameter<double>("progress_timeout",15.);
    initial_observation_=declare_parameter<double>("initial_observation_seconds",10.);
    max_sim_=declare_parameter<double>("max_search_sim_seconds",300.);
    initial_stage_=declare_parameter<std::string>("initial_stage","frontier");
    if(initial_stage_!="frontier"&&initial_stage_!="remaining_scan")throw std::invalid_argument("invalid initial_stage");
    scan_stage_=initial_stage_=="remaining_scan";
    csv_=declare_parameter<std::string>("metrics_file","/tmp/thermalnav_baseline/metrics.csv");
    std::filesystem::path parent=std::filesystem::path(csv_).parent_path();if(!parent.empty())std::filesystem::create_directories(parent);
    events_.open(parent/"events.jsonl",std::ios::trunc);trajectory_file_.open(parent/"trajectory.csv",std::ios::trunc);trajectory_file_<<"sim_time,wall_time,x,y,z,yaw,phase\n";
    publish_decisions_=declare_parameter<bool>("publish_decisions",true);
    enabled_=declare_parameter<bool>("auto_start",true);
    safety_mode_=declare_parameter<std::string>("safety_mode","depth_only");
    if(safety_mode_!="depth_only"&&safety_mode_!="sim_only_safety_oracle")throw std::invalid_argument("unknown safety_mode");
    auto retained=rclcpp::QoS(1).transient_local().reliable();auto qos=rclcpp::QoS(10).reliable();
    map_sub_=create_subscription<Grid>("/thermalnav/bev",retained,[this](Grid::ConstSharedPtr m){map_=m;map_wall_=baselineWall();});
    cover_sub_=create_subscription<Grid>("/thermalnav/scan_mask",retained,[this](Grid::ConstSharedPtr m){cover_=m;});
    odom_sub_=create_subscription<Odom>("/robot0/odom",qos,[this](Odom::ConstSharedPtr m){
      if(odom_ && rclcpp::Time(m->header.stamp).nanoseconds()<rclcpp::Time(odom_->header.stamp).nanoseconds()-500000000LL){
        odom_=m;enabled_=false;cancel();state_="STOPPED";reason_="simulation epoch reset; restart baseline explicitly";report();return;
      }
      XY next(m->pose.pose.position.x,m->pose.pose.position.y);
      if(enabled_ && !terminal_ && active_){
        if(!path_point_)path_point_=next;
        double d=cv::norm(next-*path_point_);
        if(d>.5)path_point_=next; // Reset/teleport is not navigation distance.
        else if(d>=.02){path_length_+=d;path_point_=next;auto cell=std::make_pair(int(std::floor(next.x/.1)),int(std::floor(next.y/.1)));if(cell!=last_cell_){if(visited_.count(cell)){redundant_+=d;revisited_cells_.insert(cell);}visited_.insert(cell);last_cell_=cell;}}
      }
      odom_=m;odom_wall_=baselineWall();
      if(trajectory_file_.is_open()&&(sensorTime()-last_trajectory_stamp_>=1.||trajectory_.empty()||cv::norm(next-trajectory_.back())>.05)){
        auto q=m->pose.pose.orientation;double yaw=std::atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z));
        trajectory_file_<<sensorTime()<<','<<baselineWall()<<','<<next.x<<','<<next.y<<','<<m->pose.pose.position.z<<','<<yaw<<','<<phase()<<'\n';trajectory_file_.flush();last_trajectory_stamp_=sensorTime();}
      if(trajectory_.empty() || cv::norm(next-trajectory_.back())>.05)trajectory_.push_back(next);
    });
    cmd_sub_=create_subscription<geometry_msgs::msg::Twist>("/robot0/cmd_vel",qos,[this](geometry_msgs::msg::Twist::ConstSharedPtr m){speed_=std::abs(m->linear.x)+std::abs(m->linear.y)+std::abs(m->angular.z);cmd_wall_=baselineWall();});
    detector_sub_=create_subscription<String>("/thermalnav/thermal/detections",retained,[this](String::ConstSharedPtr m){try{records_=Json::parse(m->data);}catch(const Json::exception &) {}});
    thermal_sub_=create_subscription<String>("/thermalnav/thermal/status",retained,[this](String::ConstSharedPtr m){try{thermal_=Json::parse(m->data);}catch(const Json::exception &) {}});
    detector_status_sub_=create_subscription<String>("/thermalnav/thermal/detector_status",retained,[this](String::ConstSharedPtr m){try{detector_status_=Json::parse(m->data);}catch(const Json::exception &) {}});
    info_sub_=create_subscription<sensor_msgs::msg::CameraInfo>("/thermalnav/thermal/camera_info",qos,[this](sensor_msgs::msg::CameraInfo::ConstSharedPtr m){info_=m;});
    goal_pub_=create_publisher<Pose>("/goal_pose",1);selected_pub_=create_publisher<Pose>("/thermalnav/baseline/selected_goal",retained);
    safety_pub_=create_publisher<String>("/thermalnav/safety/request",10);
    safety_sub_=create_subscription<String>("/thermalnav/safety/reply",retained,[this](String::ConstSharedPtr m){safetyReply(m->data);});
    decision_pub_=create_publisher<String>("/thermalnav/exploration/decision_trace",retained);
    status_pub_=create_publisher<String>("/thermalnav/baseline/status",retained);
    render_pub_=create_publisher<sensor_msgs::msg::Image>("/thermalnav/baseline/bev",retained);
    gaps_pub_=create_publisher<Grid>("/thermalnav/baseline/coverage_gaps",retained);
    markers_pub_=create_publisher<visualization_msgs::msg::MarkerArray>("/thermalnav/baseline/candidates",retained);
    stop_=create_service<std_srvs::srv::Trigger>("/thermalnav/baseline/stop",[this](std::shared_ptr<std_srvs::srv::Trigger::Request>,std::shared_ptr<std_srvs::srv::Trigger::Response> res){enabled_=false;cancel();if(!terminal_){end_=baselineWall();state_="STOPPED";reason_="operator stop";}report();res->success=true;res->message="Baseline disabled; current-pose PointGoal sent";});
    scan_service_=create_service<std_srvs::srv::Trigger>("/thermalnav/baseline/scan_remaining",[this](std::shared_ptr<std_srvs::srv::Trigger::Request>,std::shared_ptr<std_srvs::srv::Trigger::Response> res){if(active_||speed_>=.03){res->success=false;res->message="wait for idle state";return;}enabled_=true;terminal_=false;end_=0;start_sim_=sensorTime();scan_stage_=true;stage_started_=false;state_="REMAINING_AREA_SCAN";event("test_stage_override",{{"phase","REMAINING_AREA_SCAN"}});res->success=true;res->message="explicit existing-map inspection test";});
    timer_=create_wall_timer(std::chrono::milliseconds(250),[this]{tick();});
  }
  void close(){enabled_=false;cancel();if(!end_)end_=baselineWall();report();}
private:
  XY robot()const{return {odom_->pose.pose.position.x,odom_->pose.pose.position.y};}
  bool inBounds(XY p,double margin=0)const{return p.x>=bounds_[0]+margin && p.x<=bounds_[1]-margin && p.y>=bounds_[2]+margin && p.y<=bounds_[3]-margin;}
  double sensorTime()const{return odom_?rclcpp::Time(odom_->header.stamp).seconds():0.;}
  std::string pathSafety(const std::vector<XY> &path)const{
    auto reason=bev_->pathReason(path);
    if(reason.empty()||safety_mode_=="depth_only")return reason;
    return oraclePathReason(*bev_,path,safety_proof_,sensorTime());
  }
  void safetyRequest(std::vector<Candidate> proposals,bool refresh=false){
    if(safety_mode_!="sim_only_safety_oracle"||!odom_||proposals.empty())return;
    if(pending_id_&&baselineWall()-request_wall_<3)return;
    proposed_=std::move(proposals);refresh_=refresh;pending_id_=++request_sequence_;request_wall_=baselineWall();
    Json j={{"request_id",pending_id_},{"frame","odom"},{"radius",footprint_},{"start",{robot().x,robot().y}},{"paths",Json::array()}};
    for(const auto &c:proposed_){Json points=Json::array();for(auto p:c.path)points.push_back({p.x,p.y});j["paths"].push_back({{"id",c.id},{"points",points}});}
    String m;m.data=j.dump();safety_pub_->publish(m);
  }
  void event(const std::string &type,Json data=Json::object()){
    data["event"]=type;data["sim_time"]=sensorTime();data["wall_time"]=baselineWall();data["phase"]=phase();
    if(odom_)data["robot_xy"]={robot().x,robot().y};
    if(events_.is_open()){events_<<data.dump()<<'\n';events_.flush();}
    if(type!="sample"&&type!="safety_request_timeout")publishDecision(type,data);
  }
  void auditCandidate(const Candidate &c,const std::string &status,const std::string &reason,bool verified,bool assigned=true){
    if(!publish_decisions_)return;
    Json *entry=nullptr;
    for(auto &item:round_audit_)if(item.value("policy_id",std::string())==c.id&&std::hypot(item["xy"][0].get<double>()-c.xy.x,item["xy"][1].get<double>()-c.xy.y)<1e-6){entry=&item;break;}
    if(!entry){
      std::string id=assigned?decisionDisplayId(c.id):display_ids_.identify("U",c.xy);
      round_audit_.push_back({{"round_index",round_audit_.size()},{"display_id",id},{"policy_id",assigned?c.id:std::string()},{"xy",{c.xy.x,c.xy.y}},{"kind",c.kind}});entry=&round_audit_.back();
    }
    (*entry)["status"]=status;(*entry)["reason"]=reason;
    if(verified){(*entry)["path_distance_m"]=c.cost;(*entry)["distance_source"]="validated_high_level_path";}
    else if(!entry->contains("path_distance_m")){(*entry)["path_distance_m"]=nullptr;(*entry)["distance_source"]="not_validated";}
    (*entry)["proposed_path_length_m"]=c.cost;(*entry)["checked_sim_time"]=sensorTime();
  }
  void publishDecision(const std::string &event_type,const Json &event_data=Json::object()){
    if(!publish_decisions_||!odom_||!map_)return;
    try{
      if(event_type=="candidate_selected")++decision_sequence_;
      int reachable=0,rejected=0;Json reasons=Json::object();
      for(const auto &c:round_audit_){std::string status=c["status"];if(status=="reachable")++reachable;if(status=="rejected"||status=="failed"){++rejected;auto r=c.value("reason",std::string());reasons[r]=reasons.value(r,0)+1;}}
      auto q=odom_->pose.pose.orientation;auto pos=odom_->pose.pose.position;
      bool ended=event_type=="goal_result"&&event_data.value("state",std::string())!="ACTIVE";
      bool selected=active_.has_value()&&!ended&&!terminal_;
      Json j={{"schema_version",1},{"decision_id",decision_sequence_},{"snapshot_sequence",++snapshot_sequence_},{"candidate_round",candidate_round_},
        {"timestamp_ns",rclcpp::Time(odom_->header.stamp).nanoseconds()},{"frame_id","odom"},{"candidate_map",round_map_},{"event",event_type},
        {"exploration_state",state_},{"phase",phase()},{"policy_mode","baseline_nearest"},{"safety_mode",safety_mode_},
        {"robot_pose",{{"xy",{pos.x,pos.y}},{"z",pos.z},{"quaternion_xyzw",{q.x,q.y,q.z,q.w}}}},
        {"raw_frontiers",round_raw_},{"candidates",round_audit_},{"candidate_count",round_audit_.size()},
        {"reachable_candidate_count",reachable},{"rejected_candidate_count",rejected},{"rejection_reasons",reasons},
        {"selected",nullptr},{"active_navigation_goal",nullptr},{"selection_reason","no_safe_candidate"},
        {"navigation_status",terminal_?"stopped":observing_?"observing":paused_?"waiting_safety":next_segment_?"validating_next_segment":sent_?"pointgoal_active":"idle"},
        {"completed_goals",completed_},{"failed_goals",failed_count_},{"confirmed_person_tracks",0},{"native_rejections",bev_->rejections},{"startup_footprint",bev_->footprintReason(robot())},{"geometric_m2",mapped_*bev_->res*bev_->res},
        {"thermal_scanned_m2",scanned_*bev_->res*bev_->res},{"thermal_free_denominator_m2",free_*bev_->res*bev_->res},{"search_bounds",bounds_}};
      for(auto &r:records_)if(r.value("verified",false))j["confirmed_person_tracks"]=j["confirmed_person_tracks"].get<int>()+1;
      if(selected){auto &c=*active_;Json path=Json::array();for(auto p:c.path)path.push_back({p.x,p.y});
        j["selected"]={{"display_id",decisionDisplayId(c.id)},{"policy_id",c.id},{"kind",c.kind},{"xy",{c.xy.x,c.xy.y}},{"path_distance_m",c.cost},{"path",path},{"path_semantics","validated high-level candidate path; not predicted SRU trajectory"}};
        if(c.view_yaw)j["selected"]["yaw"]=*c.view_yaw;
        j["selection_reason"]=c.kind=="frontier"?"nearest_reachable_frontier":"nearest_reachable_inspection_viewpoint";
        if(sent_)j["active_navigation_goal"]={{"xy",{sent_->x,sent_->y}},{"interface","/goal_pose"}};
      }else if(event_type=="candidates_generated"&&candidate_round_)j["selection_reason"]="awaiting_safety_validation_or_selection";
      if(event_type=="termination")j["termination_reason"]=reason_;
      auto cam=camera();if(cam.valid())j["camera"]={{"hfov_deg",180./CV_PI*(std::atan(cam.K(0,2)/cam.K(0,0))+std::atan((cam.width-1-cam.K(0,2))/cam.K(0,0)))},{"range_m",cam.range},{"fov_is_scan_coverage",false}};
      String msg;msg.data=j.dump();decision_pub_->publish(msg);
    }catch(const std::exception &e){RCLCPP_WARN_ONCE(get_logger(),"Decision visualization failed without changing navigation: %s",e.what());}
  }
  std::string phase()const{return scan_stage_?"REMAINING_AREA_SCAN":"FRONTIER_EXPLORE";}
  int known()const{return mapped_;}
  void result(Candidate c,const std::string &state,const std::string &reason){
    auditCandidate(c,state=="REJECTED"?"rejected":state=="FAILED"?"failed":state=="REACHED"?"reached":"deferred",reason,false);
    auto &r=history_.associate(c);bool issued=r.state=="ACTIVE"&&r.attempts>0;
    r.xy=c.xy;r.region=c.look_at;r.robot=robot();r.known=known();r.last_sim=sensorTime();r.reason=reason;r.state=state;
    if(state=="REJECTED"){++candidate_rejections_;rejection_reasons_[reason]=rejection_reasons_.value(reason,0)+1;}
    if(state=="FAILED"){++r.failures;if(issued){++failed_count_;failure_reasons_[reason]=failure_reasons_.value(reason,0)+1;if(reason.find("timeout")!=std::string::npos)++timeouts_;}}
    event("goal_result",{{"candidate_id",r.id},{"kind",c.kind},{"goal_xy",{c.xy.x,c.xy.y}},{"path_m",c.cost},{"state",state},{"reason",reason},{"sent_to_sru",issued},{"attempts",r.attempts}});
  }
  void safetyReply(const std::string &data){
    if(safety_mode_!="sim_only_safety_oracle"||!pending_id_||!bev_)return;
    try{
      auto j=Json::parse(data);if(j.at("mode")!="sim_only_safety_oracle"||j.at("request_id").get<uint64_t>()!=pending_id_)return;
      pending_id_=0;double age=sensorTime()-j.at("stamp").get<double>();
      if(!std::isfinite(age)||age<-.15||age>1.){reason_="stale_safety_reply";return;}
      if(!startup_certificate_&&bev_->acceptCertificate(j.at("certificate"),robot(),sensorTime()))startup_certificate_=bev_->certificate;
      std::vector<Candidate> valid;
      for(auto proof:j.at("results")){
        auto it=std::find_if(proposed_.begin(),proposed_.end(),[&](const auto &c){return c.id==proof.at("id");});if(it==proposed_.end())continue;
        std::string why=proof.at("safe").get<bool>()?"":proof.value("reason",std::string("oracle_rejected"));
        proof["stamp"]=j.at("stamp");proof["mode"]=j.at("mode");auto path=it->path;path.front()=robot();
        if(why.empty())why=oraclePathReason(*bev_,path,proof,sensorTime());
        if(!why.empty()){
          safety_rejections_[why]=safety_rejections_.value(why,0)+1;
          if(!refresh_)result(*it,"REJECTED","safety:"+why);
          else if(active_){auto c=*active_;publishCancel();result(c,attempt_active_?"FAILED":"REJECTED","safety:"+why);clearActive();}
          continue;
        }
        proofs_[it->id]=proof;valid.push_back(*it);if(!refresh_)auditCandidate(*it,"reachable","safety_validated",true);
      }
      if(refresh_){
        if(active_&&!valid.empty()){
          safety_proof_=proofs_.at(active_->id);
          if(paused_&&sent_){paused_=false;goal(*sent_);}
          else if(next_segment_){XY p=*next_segment_;next_segment_.reset();goal(p);}
        }
      }else{oracle_ready_=true;candidates_=std::move(valid);}
    }catch(const std::exception &e){reason_="malformed_safety_reply";event("safety_protocol_error",{{"reason",e.what()}});}
  }
  void publishCancel(){if(!odom_)return;Pose m;m.header=odom_->header;m.pose=odom_->pose.pose;goal_pub_->publish(m);}
  void clearActive(){active_.reset();sent_.reset();next_segment_.reset();pending_id_=0;attempt_active_=false;paused_=false;observing_=false;path_point_.reset();}
  void goal(XY p){
    if(!odom_||cv::norm(p-robot())<=.75)return;
    if(goal_pub_->get_subscription_count()==0){reason_="missing_sru_goal_subscriber";return;}
    if(!attempt_active_&&active_){auto &r=history_.associate(*active_);if(r.failures)++repeated_failed_;++r.attempts;r.state="ACTIVE";r.xy=active_->xy;++attempted_;attempt_active_=true;active_wall_=sensorTime();}
    Pose m;m.header=odom_->header;m.pose=odom_->pose.pose;m.pose.position.x=p.x;m.pose.position.y=p.y;goal_pub_->publish(m);selected_pub_->publish(m);
    sent_=p;goal_wall_=sensorTime();best_=cv::norm(robot()-p);progress_wall_=sensorTime();last_goal_cmd_wall_=baselineWall();
    event("sru_goal_sent",{{"candidate_id",active_?active_->id:""},{"segment_xy",{p.x,p.y}},{"goal_distance",best_},{"safety_reason",pathSafety({robot(),p})}});
  }
  void cancel(){if(active_||sent_)publishCancel();if(active_)result(*active_,observing_?"EXHAUSTED":"DEFERRED",observing_?"observation_interrupted":"cancelled_without_navigation_verdict");clearActive();}
  bool snapshot(){
    if(!map_ || !cover_ || !odom_)return false;
    if(map_->header.frame_id!="odom" || cover_->header.frame_id!="odom" || odom_->header.frame_id!="odom") {reason_="frame mismatch";return false;}
    if(map_->header.stamp!=cover_->header.stamp || map_->info.width!=cover_->info.width || map_->info.height!=cover_->info.height || map_->info.resolution!=cover_->info.resolution || map_->info.origin.position.x!=cover_->info.origin.position.x || map_->info.origin.position.y!=cover_->info.origin.position.y)return false;
    auto &g=*map_;if(g.info.resolution<=0 || g.data.size()!=size_t(g.info.width)*g.info.height || cover_->data.size()!=g.data.size())return false;
    bev_=std::make_unique<BEV>(g.info.resolution,std::max(g.info.width,g.info.height)*g.info.resolution,XY(g.info.origin.position.x,g.info.origin.position.y),footprint_);
    mapped_=scanned_=free_=0;
    for(unsigned y=0;y<g.info.height;++y)for(unsigned x=0;x<g.info.width;++x){size_t i=size_t(y)*g.info.width+x;auto p=bev_->world({int(x),int(y)});if(!inBounds(p))continue;
      auto v=g.data[i];if(v==0 || v==100){bev_->seen(y,x)=255;bev_->log(y,x)=v==0?-3.f:3.f;++mapped_;if(v==0){++free_;bev_->floor_evidence(y,x)=255;if(cover_->data[i]==100){bev_->scanned(y,x)=255;++scanned_;}}}
    }if(startup_certificate_)bev_->certificate=startup_certificate_;return true;
  }
  void waypoint(){
    if(!active_)return;
    std::optional<XY> next;
    size_t end=active_->path.size()-1;
    XY destination=active_->xy;
    if(active_->approach&&!approach_done_){
      if(cv::norm(*active_->approach-robot())<=.75&&speed_<.03){approach_done_=true;event("inspection_approach_reached",{{"candidate_id",active_->id}});}
      else{destination=*active_->approach;double best=1e9;for(size_t i=0;i<active_->path.size();++i){double d=cv::norm(active_->path[i]-destination);if(d<best){best=d;end=i;}}}
    }
    for(size_t i=0;i<=end;++i){auto p=active_->path[i];double d=cv::norm(p-robot());if(d>.75+1e-3&&d<=1.6&&!observedObstacleOnPath(*bev_,{robot(),p}))next=p;}
    double distance=cv::norm(destination-robot());if(distance>.75&&distance<=1.6&&!observedObstacleOnPath(*bev_,{robot(),destination}))next=destination;
    if(!next){auto c=*active_;publishCancel();result(c,attempt_active_?"FAILED":"REJECTED","no_safe_segment");clearActive();return;}
    if(pathSafety({robot(),*next}).empty()){goal(*next);return;}
    if(safety_mode_=="sim_only_safety_oracle"){
      next_segment_=next;auto c=*active_;c.path={robot(),*next};safetyRequest({c},true);reason_="validating_next_segment";return;
    }
    auto c=*active_;publishCancel();result(c,attempt_active_?"FAILED":"REJECTED","unsafe_segment");clearActive();
  }
  ScanCamera camera(){
    ScanCamera c;if(!info_||!odom_)return c;
    try{auto tr=tf_.lookupTransform("odom",info_->header.frame_id,rclcpp::Time(info_->header.stamp));auto q=tr.transform.rotation,t=odom_->pose.pose.orientation;
      c.R=rotation(q.x,q.y,q.z,q.w);auto p=tr.transform.translation;c.t={p.x,p.y,p.z};c.base=robot();c.yaw=std::atan2(2*(t.w*t.z+t.x*t.y),1-2*(t.y*t.y+t.z*t.z));c.width=info_->width;c.height=info_->height;for(int i=0;i<9;++i)c.K.val[i]=info_->k[i];
    }catch(const std::exception &){}return c;
  }
  void switchStage(){scan_stage_start_known_=known();scan_stage_=true;state_=phase();stage_started_=true;oracle_ready_=false;candidates_.clear();event("stage_transition",{{"reason","no executable nondeferred frontier"}});}
  void terminate(const std::string &why){
    cancel();terminal_=true;end_=baselineWall();reason_=why;
    int pending=0;for(auto &r:records_)if(!r.value("verified",false))++pending;
    double area=bev_->res*bev_->res,unknown=(bounds_[1]-bounds_[0])*(bounds_[3]-bounds_[2])-mapped_*area;
    state_=(free_==scanned_&&unknown<area&&pending==0)?"COMPLETED":((mapped_>0)?"PARTIAL":"BLOCKED");
    event("termination",{{"state",state_},{"reason",reason_},{"pending_person_records",pending}});report();
  }
  void selectGoal(){
    if(pending_id_)return;
    bool approved=oracle_ready_;oracle_ready_=false;
    if(!approved){
      ++candidate_round_;round_audit_=Json::array();round_raw_=Json::array();
      round_map_={{"stamp_ns",rclcpp::Time(map_->header.stamp).nanoseconds()},{"frame_id",map_->header.frame_id},{"resolution",map_->info.resolution},{"origin",{map_->info.origin.position.x,map_->info.origin.position.y}},{"width",map_->info.width},{"height",map_->info.height}};
      candidates_.clear();
      if(scan_stage_){std::vector<XY> excluded;for(const auto &[id,r]:history_.records)if(!history_.available(r,robot(),known(),sensorTime()))excluded.push_back(r.xy);candidates_=inspectionViewpoints(*bev_,robot(),camera(),gaps_,excluded,safety_mode_=="sim_only_safety_oracle",bounds_);}
      else{
        std::vector<XY> excluded;
        for(const auto &[id,r]:history_.records)if(!history_.available(r,robot(),known(),sensorTime()))excluded.push_back(r.xy);
        candidates_=nearestBaselineCandidates(*bev_,robot(),excluded);round_raw_=bev_->frontier_observations;
        candidates_.erase(std::remove_if(candidates_.begin(),candidates_.end(),[](const auto &c){return c.kind!="frontier";}),candidates_.end());
        if(candidates_.empty()&&safety_mode_=="sim_only_safety_oracle"){
          candidates_=finiteSafetyProposals(*bev_,robot(),excluded,safety_rejections_,&round_raw_);
          candidates_.erase(std::remove_if(candidates_.begin(),candidates_.end(),[](const auto &c){return c.kind!="frontier";}),candidates_.end());
        }
      }
      for(auto &raw:round_raw_)raw["display_id"]=display_ids_.identify("R",{raw["xy"][0],raw["xy"][1]});
      std::vector<Candidate> usable;
      for(auto c:candidates_){
        if(!inBounds(c.xy,footprint_)||std::any_of(c.path.begin(),c.path.end(),[&](XY p){return !inBounds(p,footprint_);})){auditCandidate(c,"rejected","outside_search_bounds",false,false);continue;}
        auto &r=history_.associate(c);if(!history_.available(r,robot(),known(),sensorTime())){c.id=r.id;auditCandidate(c,"rejected",r.state=="FAILED"?"recently_failed":r.state=="REACHED"?"recently_reached":"deferred_or_exhausted",false);continue;}c.id=r.id;
        if(observedObstacleOnPath(*bev_,c.path)){result(c,"REJECTED","observed_occupied_sweep");continue;}
        auditCandidate(c,safety_mode_=="sim_only_safety_oracle"?"pending":"reachable",safety_mode_=="sim_only_safety_oracle"?"awaiting_safety_validation":"depth_path_validated",safety_mode_!="sim_only_safety_oracle");usable.push_back(c);
      }
      candidates_=std::move(usable);publishDecision("candidates_generated");
      if(!candidates_.empty()&&safety_mode_=="sim_only_safety_oracle"){safetyRequest(candidates_);reason_="waiting_for_finite_safety_validation";return;}
      if(safety_mode_=="depth_only")candidates_.erase(std::remove_if(candidates_.begin(),candidates_.end(),[&](const auto &c){return !bev_->pathReason(c.path).empty();}),candidates_.end());
    }
    if(candidates_.empty()){
      if(!scan_stage_){switchStage();return;}
      if(approved){reason_="inspection batch rejected; evaluate other viewpoints";return;}
      for(auto &g:gaps_)if(g.reason=="pending_safety_validation")g.reason="visible_viewpoints_deferred_or_unreachable";
      if(!camera().valid()){state_="INITIALIZE";reason_="missing stamped thermal camera geometry";return;}
      terminate("no safe executable frontier or inspection viewpoint; unknown and scan gaps retained");return;
    }
    std::sort(candidates_.begin(),candidates_.end(),[](const auto &a,const auto &z){return a.cost<z.cost;});
    active_=candidates_.front();approach_done_=false;if(proofs_.count(active_->id))safety_proof_=proofs_.at(active_->id);
    active_wall_=sensorTime();attempt_active_=false;state_=phase();reason_.clear();inspection_cells_.clear();
    if(scan_stage_&&active_->look_at){
      cv::Mat labels,stats,centers;auto occ=bev_->occupancy();cv::connectedComponentsWithStats((occ==0)&(bev_->scanned==0),labels,stats,centers,8);
      auto target_cell=bev_->cell(*active_->look_at);
      int lab=bev_->inside(target_cell)?labels.at<int>(target_cell):-1;
      if(lab>0)for(int y=0;y<labels.rows;++y)for(int x=0;x<labels.cols;++x)if(labels.at<int>(y,x)==lab)inspection_cells_.push_back(bev_->world({x,y}));
    }
    event("candidate_selected",{{"candidate_id",active_->id},{"kind",active_->kind},{"goal_xy",{active_->xy.x,active_->xy.y}},{"path_m",active_->cost},{"expected_scan_m2",active_->expected_scan_m2},{"desired_yaw",active_->view_yaw?Json(*active_->view_yaw):Json(nullptr)},{"terminal_yaw_supported",false},{"region",active_->look_at?Json({active_->look_at->x,active_->look_at->y}):Json({active_->xy.x,active_->xy.y})}});
    waypoint();
  }
  void tick(){
    double now=baselineWall();
    if(terminal_){draw();report();return;}
    if(!snapshot()){if(active_&&now-last_snapshot_wall_>2){event("sensor_pause",{{"reason","map/scan snapshot mismatch"}});cancel();}state_="INITIALIZE";reason_="waiting for synchronized map/scan mask and odom";report();return;}
    last_snapshot_wall_=now;
    if(start_==0){start_=now;start_sim_=sensorTime();event("experiment_start",{{"initial_xy",{robot().x,robot().y}},{"safety_mode",safety_mode_},{"initial_stage",initial_stage_},{"bounds",bounds_}});}
    if(!enabled_){state_="STOPPED";report();return;}if(terminal_){draw();report();return;}
    if(sensorTime()-start_sim_>max_sim_){terminate("configured simulation-time experiment budget reached");return;}
    if(now-start_<initial_observation_){state_="INITIALIZE";reason_="collecting initial aligned observations";draw();report();return;}
    if(now-map_wall_>5||now-odom_wall_>3||now-cmd_wall_>3||thermal_.value("processed_frames",0)==0||detector_status_.value("processed_frames",0)==0||thermal_.value("frame_age_s",1e9)>5||detector_status_.value("frame_age_s",1e9)>5){cancel();state_="INITIALIZE";reason_="waiting for fresh mapping, thermal, detector and SRU";report();return;}
    if(!stage_started_){stage_started_=true;state_=phase();event("stage_transition",{{"reason","initial stage"}});}
    if(pending_id_&&now-request_wall_>3){pending_id_=0;event("safety_request_timeout");}
    if(active_){
      state_=observing_?"OBSERVING":phase();
      if(observing_){
        if(sensorTime()-observe_start_>=2.){
          int gained=0;for(auto p:inspection_cells_){auto c=bev_->cell(p);if(bev_->inside(c)&&bev_->scanned(c))++gained;}
          last_scan_gain_=gained*bev_->res*bev_->res;
          if(gained){total_scan_gain_+=last_scan_gain_;++inspections_succeeded_;result(*active_,"REACHED","thermal_coverage_gain");}
          else result(*active_,"EXHAUSTED","no_coverage_gain");
          auto actual=camera();double desired=active_->view_yaw.value_or(actual.yaw);
          event("inspection_result",{{"candidate_id",active_->id},{"new_scan_m2",last_scan_gain_},
            {"desired_yaw",desired},{"actual_yaw",actual.yaw},{"actual_ground_target_in_fov",active_->look_at&&actual.visible(*active_->look_at,robot(),actual.yaw)},
            {"terminal_yaw_supported",false}});clearActive();
        }
      }else if(sent_){
        double sim=sensorTime(),d=cv::norm(*sent_-robot());
        if(d<best_-.08){best_=d;progress_wall_=sim;}
        // Arrival is processed before validating an obsolete continuation.
        if(sruArrived(robot(),*sent_,speed_,sim-goal_wall_)){
          event("segment_reached",{{"candidate_id",active_->id},{"error_m",d}});
          if(cv::norm(active_->xy-robot())<=.75){
            ++completed_;if(active_->kind=="inspection_viewpoint"){++inspections_reached_;observing_=true;observe_start_=sim;sent_.reset();pending_id_=0;}
            else{++frontiers_reached_;result(*active_,"REACHED","sru_arrival_and_zero");clearActive();}
          }else{sent_.reset();pending_id_=0;waypoint();}
        }else{
          std::string unsafe=pathSafety({robot(),*sent_});
          if(sim-progress_wall_>progress_timeout_||sim-active_wall_>timeout_){auto c=*active_;publishCancel();result(c,"FAILED",sim-active_wall_>timeout_?"goal_timeout":"progress_timeout");clearActive();}
          else if(unsafe=="occupied"){auto c=*active_;publishCancel();result(c,"FAILED","new_observed_obstacle");clearActive();}
          else if(!unsafe.empty()&&safety_mode_=="sim_only_safety_oracle"){
            if(!paused_){publishCancel();paused_=true;event("safety_pause",{{"candidate_id",active_->id},{"reason",unsafe}});}
            if(!pending_id_){auto c=*active_;c.path={robot(),*sent_};safetyRequest({c},true);}
          }else if(safety_mode_=="sim_only_safety_oracle"&&!pending_id_&&sim-safety_proof_.value("stamp",sim)>.25){auto c=*active_;c.path={robot(),*sent_};safetyRequest({c},true);}
        }
      }else if(!observing_&&!pending_id_)waypoint();
    }
    if(!active_&&speed_<.03){
      if(scan_stage_&&initial_stage_=="frontier"&&scan_stage_start_known_>0&&known()-scan_stage_start_known_>std::max(50,int(.1*scan_stage_start_known_))){scan_stage_=false;stage_started_=true;event("stage_transition",{{"reason","geometry expanded during inspection; reassess deferred frontiers"}});}
      state_=phase();selectGoal();}
    draw();report();
  }
  void draw(){
    auto gaps=*map_;
    for(unsigned y=0;y<gaps.info.height;++y)for(unsigned x=0;x<gaps.info.width;++x){size_t index=size_t(y)*gaps.info.width+x;
      if(!inBounds(bev_->world({int(x),int(y)})) || gaps.data[index]<0)gaps.data[index]=-1;
      else if(gaps.data[index]==100)gaps.data[index]=100;
      else gaps.data[index]=bev_->scanned(y,x)?0:(terminal_?75:50);
    }
    gaps_pub_->publish(gaps);
    auto q=odom_->pose.pose.orientation;double yaw=std::atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z));
    render_pub_->publish(*cv_bridge::CvImage(map_->header,"rgb8",renderer_.render(*bev_,robot(),yaw,trajectory_,candidates_,sent_,{})).toImageMsg());
    visualization_msgs::msg::MarkerArray array;visualization_msgs::msg::Marker clear;clear.action=clear.DELETEALL;array.markers.push_back(clear);
    int i=0;for(const auto &c:candidates_){visualization_msgs::msg::Marker m;m.header=map_->header;m.ns=c.kind;m.id=i++;m.type=m.SPHERE;m.action=m.ADD;m.pose.orientation.w=1;m.pose.position.x=c.xy.x;m.pose.position.y=c.xy.y;m.pose.position.z=.1;m.scale.x=m.scale.y=m.scale.z=.2;m.color.g=1;m.color.b=c.kind=="frontier"?0:1;m.color.a=1;array.markers.push_back(m);}markers_pub_->publish(array);
  }
  void report(){
    double area=map_?map_->info.resolution*map_->info.resolution:0,region=(bounds_[1]-bounds_[0])*(bounds_[3]-bounds_[2]);int confirmed=0;for(auto &r:records_)if(r.value("verified",false))++confirmed;
    double search_time=start_?(end_?end_:baselineWall())-start_:0;
    Json j={{"policy","paper_nearest_frontier_then_nearest_unscanned"},{"state",state_},{"blocked",reason_},{"terminal",terminal_},{"enabled",enabled_},{"completed",completed_},{"failed",failed_count_},{"candidate_count",candidates_.size()},{"mapped_m2",mapped_*area},{"geometric_coverage_fraction",mapped_*area/region},{"unmapped_roi_m2",std::max(0.,region-mapped_*area)},{"scanned_m2",scanned_*area},{"thermal_coverage_fraction",double(scanned_)/std::max(1,free_)},{"remaining_unscanned_m2",(free_-scanned_)*area},{"blocked_scan_gap_m2",terminal_?(free_-scanned_)*area:0.},{"search_time_s",search_time},{"path_length_m",path_length_},{"redundant_cell_reentry_path_m",redundant_},{"persons_detected",confirmed},{"detections_source","thermal_proxy_threshold"},{"search_complete",state_=="COMPLETED"}};
    if(bev_){j["rejections"]=bev_->rejections;j["start_evidence"]=footprintEvidence(*bev_,robot());j["start_state_validation"]=bev_->footprintReason(robot());j["startup_certificate"]=startup_certificate_.has_value();}
    j["safety_mode"]=safety_mode_;j["safety_rejections"]=safety_rejections_;j["pending_safety_request"]=pending_id_;
    if(sent_&&bev_)j["swept_path_validation"]=pathSafety({robot(),*sent_});
    j["goals_attempted"]=attempted_;j["navigation_failures"]=failed_count_;j["candidate_rejections"]=candidate_rejections_;j["navigation_timeouts"]=timeouts_;j["repeated_failed_candidates"]=repeated_failed_;j["failure_reasons"]=failure_reasons_;j["candidate_rejection_reasons"]=rejection_reasons_;j["frontiers_reached"]=frontiers_reached_;j["inspections_reached"]=inspections_reached_;j["inspections_with_gain"]=inspections_succeeded_;j["last_inspection_gain_m2"]=last_scan_gain_;j["total_inspection_gain_m2"]=total_scan_gain_;j["phase"]=phase();j["thermal_coverage_denominator_m2"]=free_*area;j["geometric_coverage_denominator_m2"]=region;j["gaps"]=Json::array();
    double unreachable=0,unobservable=0;
    for(auto &g:gaps_){
      if(terminal_)for(const auto &[id,r]:history_.records)if(r.region&&cv::norm(*r.region-g.center)<1.){if(r.reason=="no_coverage_gain")g.reason="no_coverage_gain_from_tested_views";else if(r.reason.rfind("safety:",0)==0)g.reason="unreachable_with_current_safety_evidence";}
      if(g.reason=="unreachable_with_current_safety_evidence")unreachable+=g.cells*area;
      else if(g.reason=="no_visible_viewpoint"||g.reason=="no_coverage_gain_from_tested_views")unobservable+=g.cells*area;
      j["gaps"].push_back({{"xy",{g.center.x,g.center.y}},{"area_m2",g.cells*area},{"classification",g.reason}});
    }
    j["unreachable_m2"]=unreachable;j["unobservable_from_tested_views_m2"]=unobservable;j["revisited_area_m2"]=revisited_cells_.size()*.01;
    String m;m.data=j.dump();status_pub_->publish(m);
    if(!terminal_&&sensorTime()-last_sample_>=1.){event("sample",{{"state",state_},{"mapped_m2",mapped_*area},{"scanned_m2",scanned_*area},{"persons",confirmed},{"reached",completed_},{"failed",failed_count_}});last_sample_=sensorTime();}
    std::filesystem::path file(csv_);if(file.has_parent_path())std::filesystem::create_directories(file.parent_path());auto tmp=file;tmp+=".tmp";std::ofstream out(tmp);
    out<<"state,mapped_m2,geometric_coverage_fraction,scanned_m2,thermal_coverage_fraction,remaining_unscanned_m2,search_time_s,path_length_m,redundant_cell_reentry_path_m,persons_detected,failed_goals,completed_goals,search_complete,blocked_reason,unmapped_roi_m2,blocked_scan_gap_m2,goals_attempted,candidate_rejections,navigation_failures,navigation_timeouts,repeated_failed_candidates,frontiers_reached,inspections_reached,inspections_with_gain,geometric_denominator_m2,thermal_denominator_m2,unreachable_m2,unobservable_m2,revisited_area_m2,total_inspection_gain_m2\n";
    out<<state_<<','<<mapped_*area<<','<<mapped_*area/region<<','<<scanned_*area<<','<<double(scanned_)/std::max(1,free_)<<','<<(free_-scanned_)*area<<','<<search_time<<','<<path_length_<<','<<redundant_<<','<<confirmed<<','<<failed_count_<<','<<completed_<<','<<j["search_complete"]<<",\""<<reason_<<"\","<<std::max(0.,region-mapped_*area)<<','<<(terminal_?(free_-scanned_)*area:0.)<<','<<attempted_<<','<<candidate_rejections_<<','<<failed_count_<<','<<timeouts_<<','<<repeated_failed_<<','<<frontiers_reached_<<','<<inspections_reached_<<','<<inspections_succeeded_<<','<<region<<','<<free_*area<<','<<unreachable<<','<<unobservable<<','<<revisited_cells_.size()*.01<<','<<total_scan_gain_<<'\n';out.close();std::filesystem::rename(tmp,file);
  }
  std::vector<double> bounds_;double footprint_,timeout_,progress_timeout_,map_wall_{},odom_wall_{},cmd_wall_{},speed_{},start_{},active_wall_{},goal_wall_{},best_{},progress_wall_{},path_length_{},redundant_{};
  std::optional<XY> path_point_;
  double initial_observation_{10.},end_{};int blocked_iterations_{};bool terminal_{false};
  int mapped_{},free_{},scanned_{},failed_count_{},completed_{};bool enabled_{};
  std::string csv_,state_{"INITIALIZE"},reason_;std::unique_ptr<BEV> bev_;Renderer renderer_;
  Grid::ConstSharedPtr map_,cover_;Odom::ConstSharedPtr odom_;std::vector<XY> trajectory_,reached_;std::vector<std::pair<XY,double>> failed_;std::vector<Candidate> candidates_;std::optional<Candidate> active_;std::optional<XY> sent_;
  std::set<std::pair<int,int>> revisited_cells_,visited_;std::pair<int,int> last_cell_{INT32_MIN,INT32_MIN};Json records_=Json::array(),thermal_=Json::object(),detector_status_=Json::object();
  rclcpp::Subscription<Grid>::SharedPtr map_sub_,cover_sub_;rclcpp::Subscription<Odom>::SharedPtr odom_sub_;rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::Subscription<String>::SharedPtr detector_sub_,thermal_sub_,detector_status_sub_;rclcpp::Publisher<Pose>::SharedPtr goal_pub_,selected_pub_;rclcpp::Publisher<String>::SharedPtr status_pub_;rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr render_pub_;rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_;rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<Grid>::SharedPtr gaps_pub_;
  tf2_ros::Buffer tf_;tf2_ros::TransformListener tf_listener_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr info_sub_;sensor_msgs::msg::CameraInfo::ConstSharedPtr info_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr scan_service_;
  GoalHistory history_;std::ofstream events_,trajectory_file_;std::vector<ScanGap> gaps_;std::vector<XY> inspection_cells_;
  Json failure_reasons_=Json::object(),rejection_reasons_=Json::object();
  int scan_stage_start_known_{};
  int attempted_{},candidate_rejections_{},timeouts_{},repeated_failed_{},frontiers_reached_{},inspections_reached_{},inspections_succeeded_{};
  bool scan_stage_{false},stage_started_{false},attempt_active_{false},paused_{false},observing_{false},approach_done_{false};std::optional<XY> next_segment_;
  double total_scan_gain_{};
  double start_sim_{},max_sim_{300.},observe_start_{},last_scan_gain_{},last_sample_{},last_trajectory_stamp_{},last_goal_cmd_wall_{};std::string initial_stage_;
  bool publish_decisions_{true};SpatialDisplayIds display_ids_;Json round_audit_=Json::array(),round_raw_=Json::array(),round_map_=Json::object();
  uint64_t decision_sequence_{},snapshot_sequence_{},candidate_round_{};rclcpp::Publisher<String>::SharedPtr decision_pub_;
  std::string safety_mode_;std::optional<Json> startup_certificate_;
  Json safety_proof_=Json::object(),safety_rejections_=Json::object();
  std::map<std::string,Json> proofs_;std::vector<Candidate> proposed_;
  uint64_t request_sequence_{},pending_id_{};double request_wall_{},last_snapshot_wall_{};bool refresh_{false},oracle_ready_{false};
  rclcpp::Publisher<String>::SharedPtr safety_pub_;rclcpp::Subscription<String>::SharedPtr safety_sub_;
};
} // namespace thermalnav
namespace {volatile std::sig_atomic_t baseline_interrupted=0;void baselineSignal(int){baseline_interrupted=1;}}
int main(int argc,char **argv){rclcpp::init(argc,argv,rclcpp::InitOptions(),rclcpp::SignalHandlerOptions::None);std::signal(SIGINT,baselineSignal);std::signal(SIGTERM,baselineSignal);
  try{auto node=std::make_shared<thermalnav::HospitalBaseline>();
    while(rclcpp::ok() && !baseline_interrupted){rclcpp::spin_some(node);std::this_thread::sleep_for(std::chrono::milliseconds(10));}
    if(rclcpp::ok()){node->close();for(int i=0;i<20;++i){rclcpp::spin_some(node);std::this_thread::sleep_for(std::chrono::milliseconds(10));}}
  }catch(const std::exception &e){std::cerr<<e.what()<<std::endl;rclcpp::shutdown();return 1;}rclcpp::shutdown();return 0;}
