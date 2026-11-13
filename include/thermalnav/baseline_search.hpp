#pragma once
#include "thermalnav/startup_safety.hpp"
#include <map>
#include <queue>

namespace thermalnav {
struct GoalRecord {
  std::string id,kind,state{"DEFERRED"},reason;
  XY xy,robot;
  int attempts{},failures{},known{};
  double last_sim{-1e9};
  std::optional<XY> region;
};
class GoalHistory {
public:
  GoalRecord &associate(const Candidate &c) {
    GoalRecord *best=nullptr;double distance=.75;
    for(auto &[id,r]:records)if(r.kind==c.kind&&cv::norm(r.xy-c.xy)<distance){best=&r;distance=cv::norm(r.xy-c.xy);}
    if(best)return *best;
    std::string id=c.kind+"_"+std::to_string(records.size()+1);
    GoalRecord r;r.id=id;r.kind=c.kind;r.xy=c.xy;r.region=c.look_at;
    return records.emplace(id,r).first->second;
  }
  bool available(const GoalRecord &r,XY robot,int known,double sim)const {
    if(r.state=="ACTIVE")return false;
    bool changed=std::abs(known-r.known)>=std::max(50,int(.1*r.known))||cv::norm(robot-r.robot)>1.5;
    if(r.state=="REACHED"||r.state=="EXHAUSTED")return changed&&sim-r.last_sim>10.;
    if(r.state=="FAILED"||r.state=="REJECTED"||r.state=="DEFERRED")
      return changed||(sim-r.last_sim>60.*std::min(4,std::max(1,r.failures))&&r.failures<2);
    return true;
  }
  std::map<std::string,GoalRecord> records;
};
inline bool observedObstacleOnPath(const BEV &b,const std::vector<XY> &path) {
  for(size_t i=0;i<path.size();++i){XY a=i?path[i-1]:path[i],z=path[i];int n=std::max(1,int(std::ceil(cv::norm(z-a)/.05)));
    for(int k=0;k<=n;++k)if(b.footprintReason(a+(z-a)*(double(k)/n))=="occupied")return true;}
  return false;
}
struct ScanCamera {
  cv::Matx33d K=cv::Matx33d::eye(),R=cv::Matx33d::eye();
  cv::Vec3d t{};
  XY base;
  double yaw{},range{6.};int width{},height{};
  bool valid()const{return width>1&&height>1&&K(0,0)>0&&K(1,1)>0;}
  bool visible(XY point,XY viewpoint,double heading)const {
    double delta=heading-yaw;cv::Matx33d rz(std::cos(delta),-std::sin(delta),0,std::sin(delta),std::cos(delta),0,0,0,1);
    auto optical=(rz*R).t()*(cv::Vec3d(point.x,point.y,0.)-(cv::Vec3d(viewpoint.x,viewpoint.y,0.)+rz*(t-cv::Vec3d(base.x,base.y,0.))));
    if(optical[2]<.15||optical[2]>range)return false;
    double u=K(0,0)*optical[0]/optical[2]+K(0,2),v=K(1,1)*optical[1]/optical[2]+K(1,2);
    return u>=2&&u<width-2&&v>=2&&v<height-2;
  }
};
struct ScanGap {XY center;int cells{};std::string reason{"unprocessed"};};
// Routes are proposals until the manager validates their entire swept footprint.
// Unknown centers are permitted only with the explicit simulation safety oracle.
class InspectionRoutes {
public:
  InspectionRoutes(const BEV &b,XY start,bool oracle,const std::vector<double>&bounds):b_(b),start_(start) {
    auto occ=b.occupancy();int r=int(std::ceil(b.footprint/b.res+.5));
    cv::Mat1b kernel(2*r+1,2*r+1,uint8_t(0));
    for(int y=-r;y<=r;++y)for(int x=-r;x<=r;++x){double dx=std::max(0.,std::abs(x)*b.res-b.res/2),dy=std::max(0.,std::abs(y)*b.res-b.res/2);if(std::hypot(dx,dy)<b.footprint-1e-9)kernel(y+r,x+r)=255;}
    cv::Mat1b blocked;cv::dilate(occ==100,blocked,kernel);
    allowed_=(blocked==0)&(oracle?(occ!=100):(occ==0));
    for(int y=0;y<occ.rows;++y)for(int x=0;x<occ.cols;++x){auto p=b.world({x,y});if(cv::norm(p-start)>5.9||(!bounds.empty()&&(p.x<bounds[0]+b.footprint||p.x>bounds[1]-b.footprint||p.y<bounds[2]+b.footprint||p.y>bounds[3]-b.footprint)))allowed_(y,x)=0;}
    distance_=cv::Mat1d(occ.size(),1e9);parent_=cv::Mat_<cv::Vec2i>(occ.size(),cv::Vec2i(-1,-1));auto root=b.cell(start);if(!b.inside(root)||!allowed_(root))return;
    using Entry=std::pair<double,int>;std::priority_queue<Entry,std::vector<Entry>,std::greater<Entry>> queue;distance_(root)=0;queue.push({0,root.y*occ.cols+root.x});
    while(!queue.empty()){auto [d,index]=queue.top();queue.pop();cv::Point p(index%occ.cols,index/occ.cols);if(d!=distance_(p)||d>5.9)continue;
      for(cv::Point delta:{cv::Point(1,0),{-1,0},{0,1},{0,-1},{1,1},{1,-1},{-1,1},{-1,-1}}){
        auto z=p+delta;double step=b.res*std::hypot(delta.x,delta.y);
        if(!b.inside(z)||!allowed_(z)||d+step>=distance_(z))continue;
        if(delta.x&&delta.y&&(!allowed_(p+cv::Point(delta.x,0))||!allowed_(p+cv::Point(0,delta.y))))continue;
        distance_(z)=d+step;parent_(z)={p.x,p.y};queue.push({distance_(z),z.y*occ.cols+z.x});}}

  }
  std::vector<XY> path(XY end)const {
    auto cell=b_.cell(end),root=b_.cell(start_);if(!b_.inside(cell)||distance_(cell)>5.9)return {};
    std::vector<XY> reverse{end};for(int i=0;cell!=root&&i<128;++i){auto p=parent_(cell);if(p[0]<0)return {};cell={p[0],p[1]};reverse.push_back(b_.world(cell));}
    if(cell!=root)return {};
    reverse.back()=start_;std::reverse(reverse.begin(),reverse.end());
    std::vector<XY> out{start_};for(size_t i=0;i+1<reverse.size();){size_t next=i+1;for(size_t j=reverse.size()-1;j>i;--j)if(!observedObstacleOnPath(b_,{reverse[i],reverse[j]})){next=j;break;}out.push_back(reverse[next]);i=next;}
    if(observedObstacleOnPath(b_,out))return {};
    return out;
  }
private:const BEV &b_;XY start_;cv::Mat1b allowed_;cv::Mat1d distance_;cv::Mat_<cv::Vec2i> parent_;
};
inline double inspectionPathLength(const std::vector<XY>&path){double d=0;for(size_t i=1;i<path.size();++i)d+=cv::norm(path[i]-path[i-1]);return d;}
inline std::vector<Candidate> inspectionViewpoints(const BEV &b,XY robot,const ScanCamera &camera,
    std::vector<ScanGap> &gaps,const std::vector<XY> &excluded={},bool oracle=false,const std::vector<double>&bounds={}) {
  gaps.clear();std::vector<Candidate> out;if(!camera.valid())return out;auto occ=b.occupancy();cv::Mat labels,stats,centers;
  int count=cv::connectedComponentsWithStats((occ==0)&(b.scanned==0),labels,stats,centers,8);
  InspectionRoutes routes(b,robot,oracle,bounds);
  for(int id=1;id<count;++id){int area=stats.at<int>(id,cv::CC_STAT_AREA);if(area<12)continue;
    XY center=b.origin+XY((centers.at<double>(id,0)+.5)*b.res,(centers.at<double>(id,1)+.5)*b.res);gaps.push_back({center,area,"no_visible_viewpoint"});
    std::vector<XY> all,samples;for(int y=stats.at<int>(id,cv::CC_STAT_TOP);y<stats.at<int>(id,cv::CC_STAT_TOP)+stats.at<int>(id,cv::CC_STAT_HEIGHT);++y)for(int x=stats.at<int>(id,cv::CC_STAT_LEFT);x<stats.at<int>(id,cv::CC_STAT_LEFT)+stats.at<int>(id,cv::CC_STAT_WIDTH);++x)if(labels.at<int>(y,x)==id)all.push_back(b.world({x,y}));
    // Spread samples over the complete component, including its visible edges.
    for(size_t i=0;i<all.size();i+=std::max(size_t(1),(all.size()+127)/128))samples.push_back(all[i]);
    std::vector<Candidate> views;
    for(int y=0;y<occ.rows;y+=2)for(int x=0;x<occ.cols;x+=2){if(occ(y,x)!=0)continue;XY p=b.world({x,y});double d=cv::norm(p-robot);
      if(d<1.05||d>5.||b.footprintReason(p)=="occupied"||std::any_of(excluded.begin(),excluded.end(),[&](XY q){return cv::norm(p-q)<.75;}))continue;
      for(auto target:samples){
        double heading=std::atan2(target.y-p.y,target.x-p.x);XY direction(std::cos(heading),std::sin(heading));
        if(!camera.visible(target,p,heading)||!camera.visible(target,p-direction*.375,heading)||!camera.visible(target,p-direction*.75,heading))continue;
        bool clear=true;for(auto c:ray(b.cell(p),b.cell(target)))if(!b.inside(c)||occ(c)!=0){clear=false;break;}if(!clear)continue;
        auto path=routes.path(p);std::optional<XY> approach;
        if(path.empty())continue;
        // Position-only SRU cannot guarantee yaw. Arrange the final approach
        // toward the target, then check the real camera/coverage after arrival.
        XY last_direction=p-path[path.size()-2];double arrival=std::atan2(last_direction.y,last_direction.x);
        if(!camera.visible(target,p,arrival)||!camera.visible(target,p-direction*.75,arrival)){
          XY q=p-direction*1.6;auto cell=b.cell(q);
          if(!b.inside(cell)||occ(cell)!=0||b.footprintReason(q)=="occupied"||observedObstacleOnPath(b,{q,p}))continue;
          path=routes.path(q);if(path.empty())continue;path.push_back(p);approach=q;
        }else heading=arrival;
        double cost=inspectionPathLength(path);if(cost>5.9)continue;
        std::vector<XY> dense{path.front()};for(size_t i=1;i<path.size();++i){auto a=path[i-1],z=path[i];int steps=std::max(1,int(std::ceil(cv::norm(z-a)/.1)));for(int k=1;k<=steps;++k)dense.push_back(a+(z-a)*(double(k)/steps));}
        Candidate c{"inspection_"+std::to_string(id)+"_"+std::to_string(views.size()),"inspection_viewpoint",p,dense,0,cost,-cost,-1};c.look_at=target;c.approach=approach;c.view_yaw=heading;c.expected_scan_m2=b.res*b.res;views.push_back(c);break;
      }
    }
    std::sort(views.begin(),views.end(),[](const auto&a,const auto&z){return a.cost<z.cost;});
    for(auto &v:views){if(std::any_of(out.begin(),out.end(),[&](const auto &c){return cv::norm(c.xy-v.xy)<.9;}))continue;out.push_back(v);gaps.back().reason="pending_safety_validation";if(out.size()>=8)break;}
    if(views.empty())gaps.back().reason="no_safe_visible_approach";
    if(out.size()>=8)break;
  }
  std::sort(out.begin(),out.end(),[](const auto&a,const auto&z){return a.cost<z.cost;});return out;
}
// Match the existing SRU's 0.75 m terminal radius exactly. A residual target
// cannot be sent inside that radius and counted as a fresh movement command.
inline bool sruArrived(XY robot,XY goal,double command_speed,double since_send) {
  return cv::norm(robot-goal)<=.75&&command_speed<.03&&since_send>=1.;
}
} // namespace thermalnav
