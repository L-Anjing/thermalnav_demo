#include "thermalnav/synthetic_thermal.hpp"
#include <opencv2/imgproc.hpp>
#include <cmath>
#include <limits>
namespace thermalnav {
cv::Mat1f syntheticThermal(const cv::Mat3b &rgb, const cv::Mat1f &depth,
                          const cv::Mat1i &ids,
                          const std::map<int, std::string> &classes,
                          const ThermalOptions &o, uint64_t seed) {
  if (rgb.size()!=depth.size() || ids.size()!=depth.size() || depth.empty() ||
      (o.mode!="ideal" && o.mode!="noisy" && o.mode!="challenging") ||
      o.noise_sigma<0 || o.blur_sigma<0 || o.downsample<1 || o.min_depth<=0 || o.max_depth<=o.min_depth)
    throw std::invalid_argument("unaligned thermal inputs or invalid options");
  cv::Mat1f out(depth.size());
  cv::Mat1b valid(depth.size(), uint8_t(0));
  cv::RNG rng(uint64_t(o.seed)^seed);
  for(int y=0;y<depth.rows;++y) for(int x=0;x<depth.cols;++x) {
    bool ok=std::isfinite(depth(y,x)) && depth(y,x)>o.min_depth && depth(y,x)<=o.max_depth;
    valid(y,x)=ok?255:0;
    auto it=classes.find(ids(y,x)); std::string c=it==classes.end()?"background":it->second;
    double mean=o.background;
    if(c=="person") mean=o.person;
    else if(c.find("floor")!=std::string::npos) mean=o.floor;
    else if(c.find("wall")!=std::string::npos) mean=o.wall;
    else if(c.find("equipment")!=std::string::npos || c.find("computer")!=std::string::npos) mean=o.equipment;
    else if(c.find("chair")!=std::string::npos || c.find("table")!=std::string::npos || c.find("furniture")!=std::string::npos) mean=o.furniture;
    if(o.mode=="challenging") {
      if(c=="person") mean=o.background+o.challenging_contrast*(o.person-o.background);
      if(c.find("equipment")!=std::string::npos || c.find("computer")!=std::string::npos) mean=o.person-o.warm_offset;
    }
    double noise=0;
    if(o.mode!="ideal") {
      noise=rng.gaussian(o.noise_sigma)+o.nonuniformity*std::sin(x*.021)*std::cos(y*.017);
      // Visible RGB contributes only small texture, never the class response.
      auto p=rgb(y,x);noise+=o.texture_amplitude*((p[0]+p[1]+p[2])/765.-.5);
      if(c=="person") noise+=o.person_variation*std::sin(y*.05);
    }
    out(y,x)=ok?float(mean+noise):0.f;
  }
  if(o.mode!="ideal") {
    if(o.blur_sigma>0) cv::GaussianBlur(out,out,{},o.blur_sigma);
    if(o.downsample>1) {
      cv::Mat small;cv::resize(out,small,{std::max(1,out.cols/o.downsample),std::max(1,out.rows/o.downsample)},0,0,cv::INTER_AREA);
      cv::resize(small,out,out.size(),0,0,cv::INTER_LINEAR);
    }
  }
  out.setTo(std::numeric_limits<float>::quiet_NaN(),valid==0);
  return out;
}
std::vector<cv::Rect> thermalRegions(const cv::Mat1f &s,const ThermalOptions &o) {
  cv::Mat labels,stats,centres;int n=cv::connectedComponentsWithStats(s>o.threshold,labels,stats,centres,8);
  std::vector<cv::Rect> boxes;
  for(int i=1;i<n;++i) if(stats.at<int>(i,cv::CC_STAT_AREA)>=o.min_area)
    boxes.emplace_back(stats.at<int>(i,cv::CC_STAT_LEFT),stats.at<int>(i,cv::CC_STAT_TOP),stats.at<int>(i,cv::CC_STAT_WIDTH),stats.at<int>(i,cv::CC_STAT_HEIGHT));
  return boxes;
}
std::vector<Candidate> nearestBaselineCandidates(BEV &bev,XY robot,const std::vector<XY> &excluded) {
  return bev.candidates(robot,excluded,{},true);
}
std::optional<cv::Vec3d> localizeThermalRegion(
    const cv::Rect &box,const cv::Mat1f &signal,const cv::Mat1f &depth,
    const cv::Matx33d &K,const cv::Matx33d &R,const cv::Vec3d &t,double threshold,double min_depth,double max_depth) {
  if(signal.size()!=depth.size() || K(0,0)<=0 || K(1,1)<=0)return {};
  std::vector<cv::Vec3d> points;std::vector<double> zs;
  auto roi=box & cv::Rect(0,0,depth.cols,depth.rows);int hot=0;
  for(int y=roi.y;y<roi.y+roi.height;++y)for(int x=roi.x;x<roi.x+roi.width;++x)if(signal(y,x)>threshold){
    ++hot;double z=depth(y,x);if(!std::isfinite(z) || z<min_depth || z>max_depth)continue;
    points.push_back({(x-K(0,2))*z/K(0,0),(y-K(1,2))*z/K(1,1),z});zs.push_back(z);
  }
  if(points.size()<8 || points.size()<size_t(hot/2))return {};
  auto median=[](std::vector<double> a){auto mid=a.begin()+a.size()/2;std::nth_element(a.begin(),mid,a.end());return *mid;};
  double z=median(zs);std::vector<double> x,y,depths;
  for(auto p:points)if(std::abs(p[2]-z)<=std::max(.15,.1*z)){x.push_back(p[0]);y.push_back(p[1]);depths.push_back(p[2]);}
  if(depths.size()<8)return {};
  return R*cv::Vec3d(median(x),median(y),median(depths))+t;
}
} // namespace thermalnav
