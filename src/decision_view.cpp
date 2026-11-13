#include "thermalnav/decision_view.hpp"
#include <opencv2/imgproc.hpp>
#include <algorithm>
#include <iomanip>
#include <sstream>
namespace thermalnav {
std::string decisionDisplayId(const std::string &id){
  if(id.rfind("frontier_",0)==0)return "F"+id.substr(id.find_last_of('_')+1);
  if(id.rfind("inspection",0)==0)return "I"+id.substr(id.find_last_of('_')+1);
  return id;
}
std::string SpatialDisplayIds::identify(const std::string &prefix,XY p){
  Entry *best=nullptr;double distance=1.;
  for(auto &e:entries_)if(e.prefix==prefix&&cv::norm(e.point-p)<distance){best=&e;distance=cv::norm(e.point-p);}
  if(best){best->point=p;return best->id;}
  std::string id=prefix+std::to_string(++counters_[prefix]);entries_.push_back({prefix,id,p});return id;
}
std::set<std::pair<std::string,std::string>> removedDecisionGlyphs(const std::set<std::pair<std::string,std::string>> &old_keys,const std::set<std::pair<std::string,std::string>> &new_keys){
  std::set<std::pair<std::string,std::string>> removed;
  std::set_difference(old_keys.begin(),old_keys.end(),new_keys.begin(),new_keys.end(),std::inserter(removed,removed.end()));return removed;
}
static XY point(const Json &j){return {j.at(0).get<double>(),j.at(1).get<double>()};}
std::vector<DecisionGlyph> decisionGlyphs(const Json &s,XY robot,int limit){
  std::vector<DecisionGlyph> out;
  for(const auto &r:s.value("raw_frontiers",Json::array()))out.push_back({"raw_frontier",r.at("display_id"),"",point(r.at("xy")),.07,.12,{.55,.9,.55}});
  auto candidates=s.value("candidates",Json::array());std::vector<size_t> order;
  for(size_t i=0;i<candidates.size();++i)order.push_back(i);
  std::stable_sort(order.begin(),order.end(),[&](size_t a,size_t b){return cv::norm(point(candidates[a]["xy"])-robot)<cv::norm(point(candidates[b]["xy"])-robot);});
  std::vector<XY> labels;int shown=0;
  for(auto i:order){const auto &c=candidates[i];std::string id=c.at("display_id"),state=c.at("status"),kind=c.at("kind");XY p=point(c.at("xy"));
    bool rejected=state=="rejected"||state=="failed";
    cv::Vec3d color=rejected?cv::Vec3d(.85,.25,.25):state=="reachable"?(kind=="frontier"?cv::Vec3d(0,1,0):cv::Vec3d(0,1,1)):cv::Vec3d(.6,.6,.6);
    out.push_back({rejected?state:kind=="frontier"?"reachable_frontier":"inspection",id,"",p,.14,.2,color});
    if(shown>=limit||std::any_of(labels.begin(),labels.end(),[&](XY q){return cv::norm(p-q)<.55;}))continue;
    std::string text=id+" | ";
    if(rejected)text+=(state=="failed"?"FAILED: ":"")+c.value("reason",std::string("rejected"));
    else if(state=="reachable"&&!c.at("path_distance_m").is_null()){std::ostringstream stream;stream<<std::fixed<<std::setprecision(1)<<c.at("path_distance_m").get<double>();text+=stream.str()+"m";}
    else text+=state=="pending"?"pending safety":state;
    out.push_back({"candidate_labels",id,text,p+XY(.12,.15),.42,.17,color});labels.push_back(p);++shown;
  }
  if(s.contains("selected")&&!s["selected"].is_null()){
    auto c=s["selected"];std::string id=c["display_id"];XY p=point(c["xy"]);
    out.push_back({"selected",id,"",p,.22,.4,{1,.85,0}});
    out.push_back({"selected_label",id,id+" | SELECTED"+(c.contains("path_distance_m")?" | "+std::to_string(c["path_distance_m"].get<double>()).substr(0,std::to_string(c["path_distance_m"].get<double>()).find('.')+2)+"m":std::string())+"\n"+s.value("selection_reason",std::string()),p+XY(.15,.25),.7,.2,{1,.85,0}});
  }
  return out;
}
cv::Mat3b decisionImage(const BEV &map,const Json &s,XY robot,double yaw,const std::vector<XY> &trajectory,const std::vector<Track> &targets,double hfov,double range){
  Renderer renderer;auto bgr=renderer.render(map,robot,yaw,trajectory,{},std::nullopt,targets,hfov,range);
  // A geometric FOV indicator, explicitly not an inspection certificate.
  auto overlay=bgr.clone();std::vector<cv::Point> fan{renderer.pixel(robot,robot)};
  for(int i=0;i<=32;++i){double a=yaw-hfov*CV_PI/360.+i*hfov*CV_PI/(180.*32);fan.push_back(renderer.pixel(robot+range*XY(std::cos(a),std::sin(a)),robot));}
  cv::Mat1b mask(bgr.rows,bgr.cols,uint8_t(0));cv::fillConvexPoly(mask,fan,255);
  for(int y=0;y<bgr.rows;++y)for(int x=0;x<bgr.cols;++x)if(mask(y,x)&&bgr(y,x)==cv::Vec3b(255,255,255))overlay(y,x)={0,255,255};
  cv::addWeighted(overlay,.16,bgr,.84,0,bgr);
  if(s.contains("selected")&&!s["selected"].is_null()&&s["selected"].contains("path")){
    const auto &path=s["selected"]["path"];
    for(size_t i=1;i<path.size();++i)cv::line(bgr,renderer.pixel(point(path[i-1]),robot),renderer.pixel(point(path[i]),robot),{255,65,180},1,cv::LINE_AA);
  }
  for(const auto &g:decisionGlyphs(s,robot,0))if(g.text.empty()){
    cv::Scalar color(g.rgb[2]*255,g.rgb[1]*255,g.rgb[0]*255);
    cv::circle(bgr,renderer.pixel(g.xy,robot),g.ns=="selected"?8:g.ns=="raw_frontier"?3:5,color,g.ns=="selected"?2:-1,cv::LINE_AA);
  }
  if(s.contains("selected")&&!s["selected"].is_null())cv::putText(bgr,s["selected"]["display_id"].get<std::string>(),renderer.pixel(point(s["selected"]["xy"]),robot)+cv::Point(8,-8),cv::FONT_HERSHEY_SIMPLEX,.4,{0,255,255},1,cv::LINE_AA);
  return bgr;
}
} // namespace thermalnav
