#pragma once
#include "thermalnav/core.hpp"
#include <map>
#include <set>
namespace thermalnav {
std::string decisionDisplayId(const std::string &policy_id);
class SpatialDisplayIds {
public:
  std::string identify(const std::string &prefix,XY point);
private:
  struct Entry{std::string prefix,id;XY point;};std::vector<Entry> entries_;
  std::map<std::string,int> counters_;
};
struct DecisionGlyph {
  std::string ns,key,text;
  XY xy;
  double z{.12},size{.22};
  cv::Vec3d rgb{0,1,0};
};
std::vector<DecisionGlyph> decisionGlyphs(const Json &snapshot,XY robot,int label_limit=12);
std::set<std::pair<std::string,std::string>> removedDecisionGlyphs(
    const std::set<std::pair<std::string,std::string>> &old_keys,
    const std::set<std::pair<std::string,std::string>> &new_keys);
cv::Mat3b decisionImage(const BEV &map,const Json &snapshot,XY robot,double yaw,
                        const std::vector<XY> &trajectory,const std::vector<Track> &targets,
                        double hfov=47.,double range=6.);
} // namespace thermalnav
