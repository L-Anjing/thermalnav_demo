#include "thermalnav/decision_view.hpp"
#include "thermalnav/synthetic_thermal.hpp"
#include <gtest/gtest.h>
using namespace thermalnav;
static Json scene(){return {{"raw_frontiers",Json::array({{{"display_id","R1"},{"xy",{2.,0.}}}})},
  {"candidates",Json::array({{{"display_id","F3"},{"policy_id","frontier_3"},{"xy",{2.,0.}},{"kind","frontier"},{"status","reachable"},{"path_distance_m",4.5}},
                            {{"display_id","F5"},{"xy",{3.,1.}},{"kind","frontier"},{"status","rejected"},{"reason","occupied"},{"path_distance_m",nullptr},{"proposed_path_length_m",3.}}})},
  {"selected",{{"display_id","F3"},{"policy_id","frontier_3"},{"xy",{2.,0.}}}},{"selection_reason","nearest_reachable_frontier"}};}
TEST(DecisionView, UsesActualDistanceAndReasonNotStraightLineOrInventedReasoning){
  auto s=scene();auto glyphs=decisionGlyphs(s,{0,0});bool distance=false,rejection=false,selected=false;
  for(auto &g:glyphs){distance|=g.text=="F3 | 4.5m";rejection|=g.text=="F5 | occupied";selected|=g.text.find("F3 | SELECTED")!=std::string::npos;}
  EXPECT_TRUE(distance);EXPECT_TRUE(rejection);EXPECT_TRUE(selected);EXPECT_EQ(decisionDisplayId("frontier_3"),"F3");
}
TEST(DecisionView, DisplayIdentitiesAndDeletesAreStable){
  SpatialDisplayIds ids;auto first=ids.identify("R",{1,2});EXPECT_EQ(ids.identify("R",{1.1,2.1}),first);EXPECT_NE(ids.identify("R",{5,5}),first);
  std::set<std::pair<std::string,std::string>> old{{"reachable","F1"},{"reachable","F2"}},now{{"reachable","F2"}};
  auto deleted=removedDecisionGlyphs(old,now);ASSERT_EQ(deleted.size(),1u);EXPECT_EQ(deleted.begin()->second,"F1");
}
TEST(DecisionView, RenderingCannotChangePolicyChoiceOrMapEvidence){
  BEV b(.1,12.,{-6,-6});b.seen.setTo(255);b.log.setTo(-3);b.floor_evidence.setTo(255);
  b.seen(cv::Rect(90,50,10,10)).setTo(0);b.log(cv::Rect(90,50,10,10)).setTo(0);
  auto before=nearestBaselineCandidates(b,{0,0},{});ASSERT_FALSE(before.empty());auto hash=b.checksum(b.occupancy());
  auto s=scene();auto original=s.dump();
  for(bool enabled:{false,true})if(enabled){auto image=decisionImage(b,s,{0,0},0,{},{});EXPECT_EQ(image.size(),cv::Size(448,448));decisionGlyphs(s,{0,0});}
  auto after=nearestBaselineCandidates(b,{0,0},{});ASSERT_EQ(before.size(),after.size());
  for(size_t i=0;i<before.size();++i){EXPECT_EQ(before[i].id,after[i].id);EXPECT_EQ(before[i].xy,after[i].xy);EXPECT_EQ(before[i].path,after[i].path);EXPECT_EQ(before[i].cost,after[i].cost);}
  EXPECT_EQ(hash,b.checksum(b.occupancy()));EXPECT_EQ(s.dump(),original);
}
TEST(DecisionView, BgrColorsAndWorldCropRemainConsistent){
  BEV b(.1,12.,{-6,-6});b.seen.setTo(255);b.log.setTo(-3);b.log(b.cell({.85,.85}))=3;
  Json s={{"raw_frontiers",Json::array()},{"candidates",Json::array()},{"selected",nullptr}};
  auto image=decisionImage(b,s,{0,0},0,{{-2,-1},{-1,-1}},{},47,6);Renderer renderer;
  EXPECT_EQ(image(renderer.pixel({.85,.85},{0,0})),cv::Vec3b(0,0,0));
  auto blue=image(renderer.pixel({-1.5,-1},{0,0}));EXPECT_GT(blue[0],200);EXPECT_LT(blue[2],50);
  EXPECT_EQ(image(renderer.pixel({10,10},{0,0})),cv::Vec3b(128,128,128));
}
