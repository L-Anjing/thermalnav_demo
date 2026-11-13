#include "thermalnav/synthetic_thermal.hpp"
#include "thermalnav/startup_safety.hpp"
#include "thermalnav/baseline_search.hpp"
#include <gtest/gtest.h>
#include <limits>
using namespace thermalnav;
namespace {
BEV freeMap(){BEV b(.1,12.,{-6.,-6.});b.log.setTo(-3);b.seen.setTo(255);b.floor_evidence.setTo(255);return b;}
}
TEST(ThermalProxy, SemanticResponseDoesNotFollowVisibleColor) {
  cv::Mat3b rgb(50,60,cv::Vec3b(255,255,255));cv::Mat1f depth(50,60,2.f);cv::Mat1i ids(50,60,0);ids(cv::Rect(20,10,12,25)).setTo(7);
  auto signal=syntheticThermal(rgb,depth,ids,{{0,"wall"},{7,"person"}},ThermalOptions(),1);
  EXPECT_FLOAT_EQ(signal(15,25),34.f);EXPECT_FLOAT_EQ(signal(15,5),20.f);
  rgb.setTo(cv::Scalar(0,0,0));auto dark=syntheticThermal(rgb,depth,ids,{{0,"wall"},{7,"person"}},ThermalOptions(),1);
  EXPECT_EQ(cv::norm(signal,dark),0.);EXPECT_EQ(thermalRegions(signal,ThermalOptions()).size(),1u);
}
TEST(ThermalProxy, InvisibleAndInvalidPixelsCannotCreateHotPeople) {
  cv::Mat3b rgb(50,60,cv::Vec3b(100,100,100));cv::Mat1f depth(50,60,2.f);cv::Mat1i ids(50,60,0);
  auto hidden=syntheticThermal(rgb,depth,ids,{{0,"wall"},{7,"person"}},ThermalOptions(),1);
  EXPECT_TRUE(thermalRegions(hidden,ThermalOptions()).empty());
  ids(cv::Rect(20,10,12,25)).setTo(7);depth(cv::Rect(20,10,12,25)).setTo(std::numeric_limits<float>::quiet_NaN());
  auto invalid=syntheticThermal(rgb,depth,ids,{{0,"wall"},{7,"person"}},ThermalOptions(),1);
  EXPECT_TRUE(std::isnan(invalid(15,25)));EXPECT_TRUE(thermalRegions(invalid,ThermalOptions()).empty());
  EXPECT_THROW(syntheticThermal(rgb,cv::Mat1f(4,4,2.f),ids,{},ThermalOptions(),1),std::invalid_argument);
}
TEST(ThermalProxy, NoiseIsRepeatableButChangesWithSensorFrame) {
  ThermalOptions o;o.mode="noisy";cv::Mat3b rgb(50,60,cv::Vec3b(100,100,100));cv::Mat1f d(50,60,2.f);cv::Mat1i ids(50,60,0);
  auto a=syntheticThermal(rgb,d,ids,{},o,1),b=syntheticThermal(rgb,d,ids,{},o,1),c=syntheticThermal(rgb,d,ids,{},o,2);
  EXPECT_EQ(cv::norm(a,b),0.);EXPECT_GT(cv::norm(a,c),0.);
}
TEST(ThermalProxy, WarmNonpersonCanTriggerThresholdDetector) {
  cv::Mat3b rgb(50,60,cv::Vec3b(100,100,100));cv::Mat1f d(50,60,2.f);cv::Mat1i ids(50,60,0);ids(cv::Rect(20,10,12,25)).setTo(4);
  ThermalOptions o;o.mode="challenging";auto signal=syntheticThermal(rgb,d,ids,{{0,"wall"},{4,"equipment"}},o,1);
  EXPECT_FALSE(thermalRegions(signal,o).empty());
}
TEST(ThermalProxy, LocalizationUsesHotMaskAndRejectsBackgroundInsideBox) {
  cv::Mat1f signal(60,60,20.f),depth(60,60,8.f);
  signal(cv::Rect(15,15,30,30)).setTo(34.f);depth(cv::Rect(15,15,30,30)).setTo(2.f);
  signal(cv::Rect(19,19,22,22)).setTo(20.f);depth(cv::Rect(19,19,22,22)).setTo(8.f);
  auto p=localizeThermalRegion({15,15,30,30},signal,depth,{100,0,30,0,100,30,0,0,1},cv::Matx33d::eye(),{1,2,3},30.);
  ASSERT_TRUE(p);EXPECT_NEAR((*p)[2],5.,1e-6);EXPECT_NEAR((*p)[0],1.,.03);EXPECT_NEAR((*p)[1],2.,.03);
}
TEST(HospitalBaseline, UsesReachablePathCostAndNeverVerificationTargets) {
  auto b=freeMap();for(auto rect:{cv::Rect(90,50,10,10),cv::Rect(25,10,10,10)}){b.seen(rect).setTo(0);b.log(rect).setTo(0);}
  auto c=nearestBaselineCandidates(b,{0,0},{});ASSERT_GE(c.size(),2u);
  EXPECT_EQ(c.front().kind,"frontier");
  for(size_t i=0;i<c.size();++i){EXPECT_EQ(c[i].kind,"frontier");EXPECT_TRUE(b.pathReason(c[i].path).empty());if(i){EXPECT_LE(c[i-1].cost,c[i].cost);}}
}
TEST(HospitalBaseline, GeneratesScanViewpointOnlyAfterFrontiersAreAbsent) {
  auto b=freeMap();b.scanned.setTo(255);b.scanned(cv::Rect(80,60,12,12)).setTo(0);
  auto c=nearestBaselineCandidates(b,{0,0},{});ASSERT_FALSE(c.empty());EXPECT_EQ(b.frontier_components,0);
  for(const auto &v:c){EXPECT_EQ(v.kind,"inspection_viewpoint");EXPECT_TRUE(b.pathReason(v.path).empty());}
}
TEST(HospitalBaseline, OccupiedWallBlocksInspectionAndUnknownFootprintStopsStartup) {
  auto b=freeMap();b.scanned.setTo(255);b.scanned(cv::Rect(85,60,12,12)).setTo(0);b.log.col(65).setTo(3);
  EXPECT_TRUE(nearestBaselineCandidates(b,{0,0},{}).empty());
  b=freeMap();b.seen(cv::Rect(57,57,7,7)).setTo(0);b.log(cv::Rect(57,57,7,7)).setTo(0);
  EXPECT_TRUE(nearestBaselineCandidates(b,{0,0},{}).empty());EXPECT_EQ(b.startup_check,"unknown_footprint");
}
TEST(StartupSafety, CertifiedSweepsDoNotModifyObservedMapOrCoverOutsideSpace) {
  BEV b(.1,12.,{-6.,-6.});auto old=b.checksum(b.occupancy());
  Json proof={{"mode","sim_only_safety_oracle"},{"stamp",10.},{"half_extent",.45},{"centers",Json::array()}};
  for(int i=0;i<=20;++i)proof["centers"].push_back({i*.05,0.});
  EXPECT_TRUE(oraclePathReason(b,{{0,0},{1,0}},proof,10.2).empty());
  EXPECT_EQ(oraclePathReason(b,{{0,0},{1,.3}},proof,10.2),"outside_validated_sweep");
  EXPECT_EQ(oraclePathReason(b,{{0,0},{1,0}},proof,12.),"stale_safety_proof");
  EXPECT_EQ(b.checksum(b.occupancy()),old);EXPECT_EQ(cv::countNonZero(b.seen),0);
  auto c=b.cell({.7,0});b.seen(c)=255;b.log(c)=3;
  EXPECT_EQ(oraclePathReason(b,{{0,0},{1,0}},proof,10.2),"occupied");
}
TEST(StartupSafety, CertificateOnlyCoversInitialFootprintAndNeverForwardSweep) {
  BEV b(.1,12.,{-6.,-6.});
  Json p={{"provider","thermalnav_physx_startup_v1"},{"frame","odom"},{"xy",{0.,0.}},{"radius",.35},{"stamp",10.},{"collision_free",true},{"ground_supported",true},{"initial_pose",true},{"sim_pid",42},{"query_count",10}};
  ASSERT_TRUE(b.acceptCertificate(p,{0,0},10.));EXPECT_TRUE(b.footprintReason({0,0}).empty());
  EXPECT_EQ(b.pathReason({{0,0},{.2,0}}),"unknown_footprint");EXPECT_EQ(cv::countNonZero(b.seen),0);
  b.certificate.reset();p["ground_supported"]=false;EXPECT_FALSE(b.acceptCertificate(p,{0,0},10.));
}
TEST(StartupSafety, ProposalsRequireObservedFreeEndpointsAndStillNeedValidation) {
  BEV b(.1,12.,{-6.,-6.});Json rejected=Json::object();
  EXPECT_TRUE(finiteSafetyProposals(b,{0,0},{},rejected).empty());
  cv::Rect r(80,55,12,12);b.seen(r).setTo(255);b.log(r).setTo(-3);
  auto candidates=finiteSafetyProposals(b,{0,0},{},rejected);ASSERT_FALSE(candidates.empty());
  for(auto &c:candidates){EXPECT_EQ(b.occupancy()(b.cell(c.xy)),0);EXPECT_EQ(b.pathReason(c.path),"unknown_footprint");}
  auto count=footprintEvidence(b,{0,0});EXPECT_GT(count["unknown"].get<int>(),0);
}
TEST(BaselineStability, ArrivalUsesSruRadiusWithoutResidualDeadZone) {
  EXPECT_TRUE(sruArrived({0,0},{.74,0},0.,2.));
  EXPECT_FALSE(sruArrived({0,0},{.8,0},0.,2.));
  EXPECT_FALSE(sruArrived({0,0},{.74,0},.5,2.));
}
TEST(BaselineStability, SpatialHistorySeparatesRejectionsFromAttemptsAndDefersRepeats) {
  GoalHistory h;Candidate c;c.kind="frontier";c.xy={1,2};
  auto &r=h.associate(c);r.state="FAILED";r.failures=1;r.last_sim=10.;r.robot={0,0};r.known=200;
  c.xy={1.2,2.1};EXPECT_EQ(h.associate(c).id,r.id);
  EXPECT_FALSE(h.available(r,{0,0},205,20.));EXPECT_TRUE(h.available(r,{0,0},300,20.));
  EXPECT_TRUE(h.available(r,{2,0},205,20.));EXPECT_TRUE(h.available(r,{0,0},205,71.));
  r.failures=2;EXPECT_FALSE(h.available(r,{0,0},205,200.));EXPECT_EQ(r.attempts,0);
}
TEST(BaselineStability, GroundVisibilityUsesVerticalFovAndRejectsNearField) {
  ScanCamera c;c.width=640;c.height=360;c.K={733,0,320,0,733,180,0,0,1};
  c.R={0,0,1,-1,0,0,0,-1,0};c.t={.325,0,.43};c.base={0,0};
  EXPECT_FALSE(c.visible({1,0},{0,0},0));EXPECT_TRUE(c.visible({3,0},{0,0},0));
  EXPECT_FALSE(c.visible({3,3},{0,0},0));
}
TEST(BaselineStability, InspectionViewpointsHaveVisibleUnscannedTargetsAndDoNotUseUnknown) {
  auto b=freeMap();b.scanned.setTo(255);b.scanned(cv::Rect(95,55,10,10)).setTo(0);
  ScanCamera c;c.width=640;c.height=360;c.K={733,0,320,0,733,180,0,0,1};
  c.R={0,0,1,-1,0,0,0,-1,0};c.t={.325,0,.43};c.base={0,0};
  std::vector<ScanGap> gaps;auto candidates=inspectionViewpoints(b,{0,0},c,gaps);
  ASSERT_FALSE(candidates.empty());EXPECT_FALSE(gaps.empty());
  for(auto &v:candidates){EXPECT_EQ(v.kind,"inspection_viewpoint");ASSERT_TRUE(v.look_at);EXPECT_GT(cv::norm(v.xy-*v.look_at),1.);EXPECT_EQ(b.occupancy()(b.cell(v.xy)),0);}
  b.seen.setTo(0);b.log.setTo(0);EXPECT_TRUE(inspectionViewpoints(b,{0,0},c,gaps).empty());
}
TEST(BaselineStability, InspectionCanApproachAGapBehindTheTravelHeading) {
  auto b=freeMap();b.scanned.setTo(255);b.scanned(cv::Rect(56,56,8,8)).setTo(0);
  ScanCamera c;c.width=640;c.height=360;c.K={733,0,320,0,733,180,0,0,1};
  c.R={0,0,1,-1,0,0,0,-1,0};c.t={.325,0,.43};c.base={0,0};
  std::vector<ScanGap> gaps;auto views=inspectionViewpoints(b,{0,0},c,gaps);
  ASSERT_FALSE(views.empty());EXPECT_TRUE(std::any_of(views.begin(),views.end(),[](const auto &v){return v.approach.has_value();}));
  for(auto &v:views){EXPECT_LE(v.cost,5.9);EXPECT_FALSE(observedObstacleOnPath(b,v.path));}
}
TEST(BaselineInspection, RoutesAroundObservedObstacleWithoutChangingMap) {
  auto b=freeMap();b.log(cv::Rect(57,48,6,24)).setTo(3);
  auto before=b.checksum(b.occupancy());InspectionRoutes routes(b,{-2,0},false,{});
  auto path=routes.path({2,0});ASSERT_GT(path.size(),2u);
  EXPECT_FALSE(observedObstacleOnPath(b,path));EXPECT_GT(inspectionPathLength(path),4.);
  EXPECT_EQ(b.checksum(b.occupancy()),before);
}
TEST(BaselineInspection, UnknownRouteRequiresExplicitOracleAndIsStillOnlyAProposal) {
  BEV b(.1,8,{-4,-4});b.seen(b.cell({2,0}))=255;b.log(b.cell({2,0}))=-3;
  InspectionRoutes physical(b,{0,0},false,{});EXPECT_TRUE(physical.path({2,0}).empty());
  InspectionRoutes simulated(b,{0,0},true,{});auto path=simulated.path({2,0});ASSERT_FALSE(path.empty());
  EXPECT_EQ(b.pathReason(path),"unknown_footprint");EXPECT_EQ(cv::countNonZero(b.occupancy()==0),1);
}
TEST(BaselineInspection, ViewpointsSpecifyIntentYawAndRespectRealCameraArrivalInterval) {
  auto b=freeMap();b.scanned.setTo(255);b.scanned(cv::Rect(56,56,8,8)).setTo(0);
  ScanCamera camera;camera.width=640;camera.height=360;camera.K={733,0,320,0,733,180,0,0,1};
  camera.R={0,0,1,-1,0,0,0,-1,0};camera.t={.325,0,.43};
  std::vector<ScanGap> gaps;auto views=inspectionViewpoints(b,{0,0},camera,gaps);
  ASSERT_FALSE(views.empty());for(auto &v:views){ASSERT_TRUE(v.view_yaw);ASSERT_TRUE(v.look_at);
    EXPECT_TRUE(camera.visible(*v.look_at,v.xy,*v.view_yaw));EXPECT_LE(v.cost,5.9);
    for(size_t i=1;i<v.path.size();++i)EXPECT_LE(cv::norm(v.path[i]-v.path[i-1]),.10001);
    EXPECT_FALSE(observedObstacleOnPath(b,v.path));}
}
