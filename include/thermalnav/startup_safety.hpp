#pragma once
#include "thermalnav/core.hpp"
#include <opencv2/imgproc.hpp>
#include <algorithm>
#include <cmath>

namespace thermalnav {
// Diagnostic only. Inferred evidence and certificates never enter occupancy().
inline Json footprintEvidence(const BEV &b, XY p) {
  Json count={{"observed_free",0},{"occupied",0},{"unknown",0},{"inferred",0}};
  auto lo=b.cell(p-XY(b.footprint,b.footprint)),hi=b.cell(p+XY(b.footprint,b.footprint));
  for(int y=lo.y;y<=hi.y;++y)for(int x=lo.x;x<=hi.x;++x){
    XY lower=b.origin+XY(x*b.res,y*b.res),upper=lower+XY(b.res,b.res);
    XY closest(std::clamp(p.x,lower.x,upper.x),std::clamp(p.y,lower.y,upper.y));
    if(cv::norm(closest-p)>=b.footprint)continue;
    std::string key=!b.inside({x,y})||!b.seen(y,x)?"unknown":b.log(y,x)<0?"observed_free":"occupied";
    count[key]=count[key].get<int>()+1;
    if(b.inside({x,y})&&b.inferred(y,x))count["inferred"]=count["inferred"].get<int>()+1;
  }
  return count;
}
// Complete footprint swept along a finite path must lie inside boxes actually
// checked by PhysX. A point certificate cannot certify any forward motion.
inline std::string oraclePathReason(const BEV &b,const std::vector<XY> &path,
                                    const Json &proof,double stamp) {
  auto raw=b.pathReason(path);
  if(raw.empty()||raw=="occupied"||path.empty())return raw;
  try {
    double age=stamp-proof.at("stamp").get<double>();
    if(proof.at("mode")!="sim_only_safety_oracle"||!std::isfinite(age)||age<-.15||age>1.)return "stale_safety_proof";
    auto half=proof.at("half_extent").get<double>();
    if(!std::isfinite(half)||half<b.footprint||half>b.footprint+.101)return "invalid_safety_extent";
    auto &centers=proof.at("centers");
    for(size_t i=0;i<path.size();++i){
      XY a=i?path[i-1]:path[0],z=path[i];
      int steps=std::max(1,int(std::ceil(cv::norm(z-a)/.025)));
      for(int j=0;j<=steps;++j){
        XY p=a+(z-a)*(double(j)/steps);
        // Occupied always wins, including observations acquired after proof.
        if(b.footprintReason(p)=="occupied")return "occupied";
        bool covered=false;
        for(const auto &c:centers){double x=c.at(0),y=c.at(1);if(std::isfinite(x)&&std::isfinite(y)&&std::abs(p.x-x)+b.footprint<=half+1e-8&&std::abs(p.y-y)+b.footprint<=half+1e-8){covered=true;break;}}
        if(!covered)return "outside_validated_sweep";
      }
    }
    return "";
  }catch(const Json::exception &){return "malformed_safety_proof";}
}
// Proposals, not executable goals. Only actual observed-free cells can be
// endpoints. Unknown swept space still requires a separate live PhysX proof.
inline std::vector<Candidate> finiteSafetyProposals(const BEV &b,XY robot,
    const std::vector<XY> &excluded,Json &rejections,Json *audit=nullptr) {
  auto occ=b.occupancy();cv::Mat1b adjacent;
  cv::dilate(occ==255,adjacent,cv::Mat1b::ones(3,3));
  cv::Mat labels,stats,centers;
  int n=cv::connectedComponentsWithStats((occ==0)&(adjacent!=0),labels,stats,centers,8);
  if(audit)*audit=frontierClusterSnapshot(stats,centers,b.origin,b.res);
  std::vector<Candidate> out;
  for(int phase=0;phase<2;++phase){
    if(phase){if(!out.empty())break;n=cv::connectedComponentsWithStats((occ==0)&(b.scanned==0),labels,stats,centers,8);}
    for(int id=1;id<n;++id){
      if(stats.at<int>(id,cv::CC_STAT_AREA)<(phase?12:4))continue;
      std::vector<std::pair<double,XY>> choices;
      for(int y=stats.at<int>(id,cv::CC_STAT_TOP);y<stats.at<int>(id,cv::CC_STAT_TOP)+stats.at<int>(id,cv::CC_STAT_HEIGHT);++y)
        for(int x=stats.at<int>(id,cv::CC_STAT_LEFT);x<stats.at<int>(id,cv::CC_STAT_LEFT)+stats.at<int>(id,cv::CC_STAT_WIDTH);++x){
          if(labels.at<int>(y,x)!=id||occ(y,x)!=0)continue;
          XY p=b.world({x,y});double d=cv::norm(p-robot);
          if(d<1.05||d>5.)continue;
          if(std::any_of(excluded.begin(),excluded.end(),[&](XY q){return cv::norm(p-q)<1.;}))continue;
          if(b.pathReason({robot,p})=="occupied"){rejections["occupied_sweep"]=rejections.value("occupied_sweep",0)+1;continue;}
          choices.push_back({d,p});
        }
      std::sort(choices.begin(),choices.end(),[](auto &a,auto &z){return a.first<z.first;});
      for(auto &[cost,p]:choices){
        if(std::any_of(out.begin(),out.end(),[&](const Candidate &c){return cv::norm(p-c.xy)<.9;}))continue;
        if(phase){
          bool visible=false;
          for(int y=0;y<b.log.rows&&!visible;++y)for(int x=0;x<b.log.cols&&!visible;++x){
            if(labels.at<int>(y,x)!=id)continue;
            double d=cv::norm(b.world({x,y})-p);if(d<.9||d>6.)continue;
            visible=true;for(auto v:ray(b.cell(p),{x,y}))if(occ(v)!=0){visible=false;break;}
          }
          if(!visible)continue;
        }
        std::string kind=phase?"inspection_viewpoint":"frontier";
        std::vector<XY> path;int steps=std::max(1,int(std::ceil(cost/.1)));
        for(int i=0;i<=steps;++i)path.push_back(robot+(p-robot)*(double(i)/steps));
        auto endpoint=b.footprintReason(p),sweep=b.pathReason(path);
        if(!endpoint.empty())rejections["endpoint_"+endpoint]=rejections.value("endpoint_"+endpoint,0)+1;
        if(!sweep.empty())rejections["swept_"+sweep]=rejections.value("swept_"+sweep,0)+1;
        out.push_back({kind+"_oracle_"+std::to_string(out.size()),kind,p,path,0,cost,-cost,-1});
        if(out.size()>=8)break;
      }
      if(out.size()>=8)break;
    }
  }
  std::sort(out.begin(),out.end(),[](const auto &a,const auto &z){return a.cost<z.cost;});
  return out;
}
} // namespace thermalnav
