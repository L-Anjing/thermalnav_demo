#include "thermalnav/core.hpp"
#include "thermalnav/perception.hpp"
#include <gtest/gtest.h>
#include <limits>
#include <opencv2/imgproc.hpp>

using namespace thermalnav;
namespace {
const cv::Matx33d K(40, 0, 20, 0, 40, 15, 0, 0, 1);
const cv::Matx33d R(0, 0, 1, -1, 0, 0, 0, -1, 0);
Json proof() {
  return {{"provider", "thermalnav_physx_startup_v1"},
          {"frame", "odom"},
          {"collision_free", true},
          {"ground_supported", true},
          {"initial_pose", true},
          {"xy", {0., 0.}},
          {"radius", .35},
          {"stamp", 1.},
          {"sim_pid", 99},
          {"query_count", 10}};
}
cv::Mat1f floorImage() {
  // Dense ground observation with <8cm neighbour depth spread near the bottom.
  cv::Mat1f d(120, 160, std::numeric_limits<float>::quiet_NaN());
  for (int v = 66; v < 120; ++v)
    d.row(v).setTo(.6 * 80 / (v - 60));
  return d;
}
const cv::Matx33d floorK(80, 0, 80, 0, 80, 60, 0, 0, 1);
void freeRect(BEV &b, cv::Rect roi) {
  b.seen(roi).setTo(255);
  b.log(roi).setTo(-3);
}
std::vector<cv::Vec3d> endpoints(double x = 1.5) {
  return {{x, 0, 0}, {x, .01, 0}, {x, .02, 0}, {x, .03, 0}};
}
} // namespace

TEST(Map, GridCoordinates) {
  BEV b(.1, 4, {-2, -2});
  auto c = b.cell({.25, -.35});
  EXPECT_EQ(c, cv::Point(22, 16));
  EXPECT_NEAR(cv::norm(b.world(c) - XY(.25, -.35)), 0, 1e-9);
}
TEST(Map, TranslationAccumulatesWorldHistory) {
  BEV b(.1, 10, {-5, -5});
  cv::Mat1f d(30, 40, 2.f);
  b.integrate(d, K, R, {0, 0, .6}, {0, 0});
  auto before = b.occupancy();
  b.integrate(d, K, R, {1, 0, .6}, {1, 0});
  auto after = b.occupancy();
  EXPECT_GT(cv::countNonZero(after != 255), cv::countNonZero(before != 255));
  EXPECT_GT(cv::countNonZero((before != 255) & (after != 255)), 0);
  EXPECT_DOUBLE_EQ(b.origin.x, -5.);
}
TEST(Map, RotationChangesWorldProjection) {
  BEV b(.1, 10, {-5, -5});
  cv::Mat1f d(30, 40, 2.f);
  b.integrate(d, K, R, {0, 0, .6}, {0, 0});
  auto before = b.occupancy();
  cv::Matx33d yaw(0, -1, 0, 1, 0, 0, 0, 0, 1);
  b.integrate(d, K, yaw * R, {0, 0, .6}, {0, 0});
  EXPECT_GT(cv::countNonZero(before != b.occupancy()), 0);
}
TEST(Map, ExpansionPreservesEveryLayer) {
  BEV b(.1, 2, {0, 0});
  b.seen(5, 5) = b.scanned(5, 5) = b.inferred(5, 5) = b.floor_evidence(5, 5) =
      255;
  b.log(5, 5) = -2;
  b.visits(5, 5) = 5;
  b.expand({{-5, -5, 0}, {5, 5, 0}});
  auto c = b.cell({.55, .55});
  EXPECT_EQ(b.occupancy()(c), 0);
  EXPECT_TRUE(b.scanned(c));
  EXPECT_TRUE(b.inferred(c));
  EXPECT_TRUE(b.floor_evidence(c));
  EXPECT_EQ(b.visits(c), 5);
}
TEST(Map, InvalidDepthLeavesUnknown) {
  BEV b;
  b.integrate(cv::Mat1f(30, 40, std::numeric_limits<float>::quiet_NaN()), K, R,
              {0, 0, .6}, {0, 0});
  EXPECT_EQ(cv::countNonZero(b.occupancy() != 255), 0);
  EXPECT_EQ(b.diagnostics["valid_depth"], 0);
}
TEST(Map, AboveFloorRaysNeverClearGround) {
  BEV b;
  b.integrate(cv::Mat1f(30, 40, 2.f), K, R, {0, 0, 4}, {0, 0});
  EXPECT_EQ(b.occupancy()(b.cell({.5, 0})), 255);
}
TEST(Map, GroundEndpointsDoNotClearCameraBlindZone) {
  BEV b;
  b.integrate(floorImage(), floorK, R, {0, 0, .6}, {0, 0});
  auto occ = b.occupancy();
  EXPECT_GT(cv::countNonZero(occ == 0), 0);
  EXPECT_EQ(occ(b.cell({.25, 0})), 255);
  EXPECT_FALSE(b.floor_evidence(b.cell({.25, 0})));
  EXPECT_EQ(b.diagnostics["ray_cleared_cells"], 0);
}
TEST(Map, OddPixelThinObstacleSurvivesStride) {
  BEV b;
  cv::Mat1f d(30, 40, std::numeric_limits<float>::quiet_NaN());
  d(15, 21) = 2.;
  b.integrate(d, K, R, {0, 0, .6}, {0, 0}, 0, 2);
  EXPECT_EQ(b.occupancy()(b.cell({2., -.05})), 100);
  EXPECT_EQ(b.diagnostics["obstacle_points"], 1);
}
TEST(Map, ObstacleWinsOverVisibleFloorInSameColumn) {
  BEV b;
  b.integrate(cv::Mat1f(30, 40, 2.f), K, R, {0, 0, .6}, {0, 0}, 0, 1);
  EXPECT_EQ(b.occupancy()(b.cell({2.05, 0})), 100);
  EXPECT_EQ(b.occupancy()(b.cell({3., 0})), 255);
  EXPECT_FALSE(b.scanned(b.cell({3., 0})));
}
TEST(Map, CoverageRejectsMissingDepthNeighbour) {
  BEV b;
  auto c = b.cell({2.05, -.05});
  b.seen(c) = 255;
  b.log(c) = -2;
  // Centre projects near (21,27). Invalid foreground must not certify coverage.
  cv::Mat1f d(30, 40, 2.1f);
  d(27, 21) = 0;
  b.integrate(d, K, R, {0, 0, .6}, {0, 0});
  EXPECT_FALSE(b.scanned(c));
}
TEST(Map, KnownObstacleHistoryIsConservative) {
  BEV b;
  auto c = b.cell({1, 0});
  b.seen(c) = 255;
  b.log(c) = 3;
  b.integrate(floorImage(), floorK, R, {0, 0, .6}, {0, 0});
  EXPECT_EQ(b.occupancy()(c), 100);
}
TEST(Map, InvalidGeometryRejected) {
  EXPECT_THROW(BEV(0), std::invalid_argument);
  BEV b;
  EXPECT_THROW(b.integrate(cv::Mat1f(1, 1, 1.f), cv::Matx33d::zeros(), R,
                           {0, 0, 0}, {0, 0}),
               std::invalid_argument);
}
TEST(Map, QuaternionValidation) {
  EXPECT_THROW(rotation(0, 0, 0, 0), std::invalid_argument);
  EXPECT_NEAR(cv::norm(cv::Mat(rotation(0, 0, 0, 2) - cv::Matx33d::eye())), 0,
              1e-9);
}
TEST(Nearfield, InferenceNeverMakesFree) {
  BEV b;
  auto before = b.occupancy();
  auto stats = b.complete(endpoints(), {0, 0});
  EXPECT_GT(stats["inferred_added"].get<int>(), 5);
  EXPECT_EQ(cv::countNonZero(before != b.occupancy()), 0);
  EXPECT_EQ(cv::countNonZero(b.scanned), 0);
  EXPECT_EQ(b.footprintReason({0, 0}), "unknown_footprint");
}
TEST(Nearfield, StopsAtObstacle) {
  BEV b;
  auto c = b.cell({.7, 0});
  b.seen(c) = 255;
  b.log(c) = 3;
  b.complete(endpoints(), {0, 0});
  EXPECT_FALSE(b.inferred(b.cell({1., 0})));
}
TEST(Nearfield, DiagonalSupercover) {
  auto cells = ray({0, 0}, {1, 1});
  EXPECT_NE(std::find(cells.begin(), cells.end(), cv::Point(1, 0)),
            cells.end());
  EXPECT_NE(std::find(cells.begin(), cells.end(), cv::Point(0, 1)),
            cells.end());
}
TEST(Nearfield, RejectsFarSparseAndNoisyEndpoints) {
  for (auto points : std::vector<std::vector<cv::Vec3d>>{
           {},
           endpoints(2.1),
           {{1, 0, 0}},
           {{1, 0, 0}, {1.6, 0, 0}, {1.9, 0, 0}}}) {
    BEV b;
    b.complete(points, {0, 0});
    EXPECT_EQ(cv::countNonZero(b.inferred), 0);
  }
}
TEST(Nearfield, BlindZoneUsesActualExtrinsics) {
  BEV b;
  cv::Matx33d k(733, 0, 320, 0, 733, 180, 0, 0, 1);
  auto stats = b.blindZone(k, R, {.32487, 0, .4}, 640, 360, {0, 0}, 0);
  EXPECT_NEAR(stats["floor_blind_left_center_right_m"][1].get<double>(),
              .32487 + .4 * 733 / 179, 1e-6);
}
TEST(Safety, CertificateIsFixedAndBounded) {
  BEV b;
  EXPECT_TRUE(b.acceptCertificate(proof(), {0, 0}, 1.1));
  EXPECT_EQ(b.footprintReason({0, 0}), "");
  EXPECT_EQ(b.footprintReason({.1, 0}), "unknown_footprint");
  EXPECT_NE(b.pathReason({{0, 0}, {.1, 0}}), "");
  EXPECT_EQ(cv::countNonZero(b.seen), 0);
}
TEST(Safety, CertificateCannotOverrideObstacle) {
  BEV b;
  auto c = b.cell({.1, 0});
  b.seen(c) = 255;
  b.log(c) = 3;
  EXPECT_FALSE(b.acceptCertificate(proof(), {0, 0}, 1.1));
  EXPECT_EQ(b.footprintReason({0, 0}), "occupied");
}
TEST(Safety, RejectsInvalidCertificates) {
  for (auto change : std::vector<Json>{{{"collision_free", false}},
                                       {{"ground_supported", false}},
                                       {{"initial_pose", false}},
                                       {{"radius", 2.}},
                                       {{"stamp", -9}},
                                       {{"xy", {1, 0}}},
                                       {{"provider", "assume_safe"}},
                                       {{"query_count", 0}},
                                       {{"xy", {0}}}}) {
    BEV b;
    auto p = proof();
    p.update(change);
    EXPECT_FALSE(b.acceptCertificate(p, {0, 0}, 1.1));
    EXPECT_FALSE(b.certificate);
  }
}
TEST(Safety, CertificateCannotBridgeUnknownGap) {
  BEV b;
  freeRect(b, {165, 140, 25, 20});
  EXPECT_TRUE(b.acceptCertificate(proof(), {0, 0}, 1.1));
  EXPECT_TRUE(b.candidates({0, 0}, {}, {}).empty());
}
TEST(Planning, SafeConnectedMapProducesCandidates) {
  BEV b(.1, 10, {0, 0});
  freeRect(b, {10, 10, 50, 50});
  auto cs = b.candidates({3, 3}, {}, {});
  ASSERT_FALSE(cs.empty());
  for (const auto &c : cs) {
    EXPECT_GT(c.cost, .8);
    EXPECT_EQ(b.pathReason(c.path), "");
  }
}
TEST(Planning, WallDisconnectsCandidates) {
  BEV b(.1, 10, {0, 0});
  freeRect(b, {10, 10, 50, 50});
  b.log(cv::Rect(35, 10, 5, 50)).setTo(3);
  auto cs = b.candidates({2, 3}, {}, {});
  for (const auto &c : cs)
    EXPECT_LT(c.xy.x, 3.5);
}
TEST(Planning, UnknownStartupCannotMove) {
  BEV b;
  EXPECT_TRUE(b.candidates({0, 0}, {}, {}).empty());
  EXPECT_EQ(b.startup_check, "unknown_footprint");
}
TEST(Detection, DepthLocalizationAndTracking) {
  cv::Matx33d k(10, 0, 10, 0, 10, 10, 0, 0, 1);
  auto p = localizeBox({0, 0, 20, 20}, cv::Mat1f(20, 20, 2.f), k,
                       cv::Matx33d::eye(), {1, 0, 0});
  ASSERT_TRUE(p);
  EXPECT_NEAR(cv::norm(*p - cv::Vec3d(1, 0, 2)), 0, 1e-9);
  std::vector<Track> tracks;
  for (double s : {0., .6, 1.2})
    addTrack(tracks, {(*p)[0], (*p)[1]}, s);
  ASSERT_EQ(tracks.size(), 1u);
  EXPECT_TRUE(tracks[0].verified);
}
TEST(Renderer, PreservesUnknownAndOccupancyPalette) {
  BEV b(.1, 10, {-5, -5});
  b.seen(70, 70) = b.seen(70, 71) = 255;
  b.log(70, 70) = -2;
  b.log(70, 71) = 2;
  Renderer r;
  auto image = r.render(b, {0, 0}, 0, {}, {}, std::nullopt, {});
  EXPECT_EQ(image(r.pixel(b.world({70, 70}), {0, 0})),
            cv::Vec3b(255, 255, 255));
  EXPECT_EQ(image(r.pixel(b.world({71, 70}), {0, 0})), cv::Vec3b(0, 0, 0));
  EXPECT_EQ(image(r.pixel(b.world({69, 70}), {0, 0})),
            cv::Vec3b(128, 128, 128));
}
TEST(Renderer, HeadingChangesWithoutMutatingMap) {
  BEV b;
  Renderer r;
  auto before = b.occupancy();
  auto a = r.render(b, {0, 0}, 0, {}, {}, std::nullopt, {}),
       n = r.render(b, {0, 0}, CV_PI / 2, {}, {}, std::nullopt, {});
  EXPECT_GT(cv::norm(a, n), 0);
  r.coverage(b, {0, 0}, {});
  EXPECT_EQ(cv::countNonZero(before != b.occupancy()), 0);
}
TEST(Policy, MissingServiceHasExplicitFallback) {
  unsetenv("THERMALNAV_VLM_URL");
  unsetenv("THERMALNAV_VLM_MODEL");
  Policy p("vlm");
  EXPECT_FALSE(p.remote());
  EXPECT_NE(p.status.find("VLM unavailable"), std::string::npos);
  EXPECT_EQ(p.calls, 0u);
}

TEST(Map, TraversalRequiresExactExecutedPoseAndPhysicsProof) {
  BEV b; auto p=proof();p["mode"]="sim_only_safety_oracle";
  EXPECT_FALSE(b.recordVerifiedTraversal(p,{1,0},1.)["accepted"].get<bool>());
  EXPECT_FALSE(b.recordVerifiedTraversal(p,{0,0},1.01)["accepted"].get<bool>());
  p["ground_supported"]=false;
  EXPECT_FALSE(b.recordVerifiedTraversal(p,{0,0},1.)["accepted"].get<bool>());
  EXPECT_EQ(cv::countNonZero(b.traversed),0);
  p["ground_supported"]=true;
  EXPECT_TRUE(b.recordVerifiedTraversal(p,{0,0},1.)["accepted"].get<bool>());
  EXPECT_GT(cv::countNonZero(b.traversed),0);
  EXPECT_EQ(cv::countNonZero(b.seen),0);
  EXPECT_EQ(cv::countNonZero(b.floor_evidence),0);
  EXPECT_EQ(cv::countNonZero(b.scanned),0);
  EXPECT_EQ(b.occupancy()(b.cell({.15,.05})),0);
  EXPECT_EQ(b.occupancy()(b.cell({.45,.05})),255);
  for(int y=0;y<b.log.rows;++y)for(int x=0;x<b.log.cols;++x)if(b.traversed(y,x)) {
    auto lo=b.origin+XY(x*b.res,y*b.res);
    for(XY c:{lo,lo+XY(b.res,0),lo+XY(0,b.res),lo+XY(b.res,b.res)})EXPECT_LE(cv::norm(c),.35+1e-9);
  }
  // Disjoint actual states do not certify the unsampled gap between them.
  p["xy"]={2.,0.};p["stamp"]=2.;b.recordVerifiedTraversal(p,{2,0},2.);
  EXPECT_EQ(b.occupancy()(b.cell({1.,0.})),255);
}
TEST(Map, TraversalNeverOverridesObstacleAndSurvivesExpansion) {
  BEV b;auto p=proof();p["mode"]="sim_only_safety_oracle";
  auto c=b.cell({.05,.05});b.seen(c)=255;b.log(c)=3;
  b.recordVerifiedTraversal(p,{0,0},1.);
  EXPECT_EQ(b.occupancy()(c),100);
  EXPECT_FALSE(b.traversed(c));
  b.expand({{-20,-20,0}});
  EXPECT_TRUE(b.traversed(b.cell({-.15,-.05})));
  EXPECT_EQ(b.occupancy()(b.cell({.05,.05})),100);
}
