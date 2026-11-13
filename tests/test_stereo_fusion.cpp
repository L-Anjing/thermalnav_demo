#include "thermalnav/occupancy_fusion.hpp"
#include "thermalnav/ros_observation.hpp"
#include <gtest/gtest.h>
#include <limits>
#include <opencv2/imgproc.hpp>

using namespace thermalnav;
namespace {
StereoCalibration calibration() {
  StereoCalibration c;
  c.size = {320, 240};
  c.left_K = c.right_K = {240, 0, 160, 0, 240, 120, 0, 0, 1};
  return c;
}
std::pair<cv::Mat1b, cv::Mat1b> images(int disparity = 12) {
  cv::Mat1b left(240, 320);
  cv::RNG rng(42);
  rng.fill(left, cv::RNG::UNIFORM, 0, 255);
  cv::GaussianBlur(left, left, {3, 3}, .5);
  cv::Mat1b right(left.size(), uint8_t(0));
  left(cv::Rect(disparity, 0, left.cols - disparity, left.rows))
      .copyTo(right(cv::Rect(0, 0, left.cols - disparity, left.rows)));
  return {left, right};
}
Observation observation(std::string name, double stamp = 1.) {
  Observation o;
  o.source = name;
  o.R = {0, 0, 1, -1, 0, 0, 0, -1, 0};
  o.stamp = stamp;
  o.origin = {0, 0, 1.};
  return o;
}
DepthPoint ground(double x, double y) {
  return {{x, -y, 1.}, 1.f, .0001f, true};
}
void verticalGround(Observation &o) {
  o.R = {1, 0, 0, 0, -1, 0, 0, 0, -1};
  for (double x = .05; x < 1.; x += .1)
    for (double y = .05; y < 1.; y += .1)
      o.points.push_back(ground(x, y));
}
} // namespace
TEST(Stereo, TexturedPlaneHasMetricDepthWithoutDepthInput) {
  auto [left, right] = images();
  auto result = StereoFrontend().estimate(left, right, calibration());
  ASSERT_GT(result.points.size(), 100u);
  cv::Mat1f truth(left.size(), 2.4f);
  auto metrics = evaluateStereo(result.points, truth, calibration().left_K);
  EXPECT_LT(metrics["median_relative_error"].get<double>(), .03);
  EXPECT_LT(metrics["p95_relative_error"].get<double>(), .1);
  EXPECT_EQ(result.depth.type(), CV_32FC1);
  EXPECT_GT(cv::countNonZero(result.confidence), 100);
  for (const auto &p : result.points) {
    EXPECT_GT(p.p[2], 0);
    EXPECT_GT(p.variance, 0);
    EXPECT_GT(p.confidence, 0);
  }
}
TEST(Stereo, BlankImagesProduceNoFabricatedDepth) {
  auto c = calibration();
  auto r = StereoFrontend().estimate(cv::Mat1b(c.size, uint8_t(128)),
                                     cv::Mat1b(c.size, uint8_t(128)), c);
  EXPECT_TRUE(r.points.empty());
  EXPECT_EQ(cv::countNonZero(r.confidence), 0);
}
TEST(Stereo, RejectsWrongBaselineAndImageGeometry) {
  auto [l, r] = images();
  auto c = calibration();
  c.right_from_left_t = {0, 0, 0};
  EXPECT_THROW(StereoFrontend().estimate(l, r, c), std::invalid_argument);
  c = calibration();
  c.right_from_left_t = {.12, 0, 0};
  EXPECT_THROW(StereoFrontend().estimate(l, r, c), std::invalid_argument);
  c = calibration();
  EXPECT_THROW(StereoFrontend().estimate(l, r(cv::Rect(0, 0, 100, 100)), c),
               std::invalid_argument);
}
TEST(Stereo, DepthScalesWithBaseline) {
  auto [l, r] = images();
  auto c = calibration();
  auto a = StereoFrontend().estimate(l, r, c);
  c.right_from_left_t = {-.06, 0, 0};
  auto b = StereoFrontend().estimate(l, r, c);
  ASSERT_FALSE(a.points.empty());
  ASSERT_EQ(a.points.size(), b.points.size());
  EXPECT_NEAR(a.points[0].p[2], 2 * b.points[0].p[2], 1e-6);
}
TEST(Stereo, ReferenceIsUsedOnlyForEvaluation) {
  auto [l, r] = images();
  auto c = calibration();
  auto a = StereoFrontend().estimate(l, r, c);
  auto good = evaluateStereo(a.points, cv::Mat1f(c.size, 2.4f), c.left_K);
  auto bad = evaluateStereo(a.points, cv::Mat1f(c.size, 5.f), c.left_K);
  EXPECT_LT(good["mae_m"].get<double>(), .1);
  EXPECT_GT(bad["mae_m"].get<double>(), 2.);
}
TEST(Fusion, ThreeDimensionalRayDoesNotClearWholeColumn) {
  BEV b;
  OccupancyFusion f;
  auto o = observation("depth");
  o.origin = {0, 0, .4};
  o.points = {{{0, 0, 2.}, 1, .0004f, false}};
  f.integrate({o}, b);
  EXPECT_EQ(b.occupancy()(b.cell({.5, 0})), 255);
  auto node = f.tree().search(.5, 0, .4);
  ASSERT_NE(node, nullptr);
  EXPECT_LT(node->getOccupancy(), .5);
}
TEST(Fusion, GroundAndFullHeightEvidenceAllowFreeCells) {
  BEV b;
  OccupancyFusion f;
  auto o = observation("depth");
  verticalGround(o);
  f.integrate({o}, b);
  EXPECT_GT(cv::countNonZero(b.occupancy() == 0), 0);
}
TEST(Fusion, ThinObstacleWinsOverFreeRayInSameBatch) {
  BEV b;
  OccupancyFusion f;
  auto o = observation("depth");
  o.origin = {0, 0, .4};
  o.points = {{{0, 0, 1.}, 1, .0001f, false}, {{0, 0, 2.}, 1, .0001f, false}};
  f.integrate({o}, b);
  EXPECT_EQ(b.occupancy()(b.cell({1.05, .05})), 100);
}
TEST(Fusion, MissingDepthCanBeSupplementedByStereo) {
  BEV b;
  OccupancyFusion f;
  auto empty = observation("depth");
  f.integrate({empty}, b);
  auto before = f.tree().size();
  auto s = observation("stereo");
  verticalGround(s);
  auto status = f.integrate({s}, b);
  EXPECT_GT(f.tree().size(), before);
  EXPECT_GT(cv::countNonZero(b.occupancy() == 0), 0);
  EXPECT_GT(status["sources"]["stereo"]["points"].get<int>(), 0);
}
TEST(Fusion, DuplicateFrameNeverAccumulatesEvidence) {
  BEV b;
  OccupancyFusion f;
  auto o = observation("depth");
  o.origin = {0, 0, .4};
  o.points = {{{0, 0, 1.}, 1, .0001f, false}};
  f.integrate({o}, b);
  double probability = f.tree().search(1., 0, .4)->getOccupancy();
  auto result = f.integrate({o}, b);
  EXPECT_EQ(result["updated_voxels"], 0);
  EXPECT_DOUBLE_EQ(f.tree().search(1., 0, .4)->getOccupancy(), probability);
}
TEST(Fusion, DensePixelsAndCorrelatedSourcesCannotOvercount) {
  BEV b;
  OccupancyFusion f;
  auto a = observation("depth");
  a.origin = {0, 0, .4};
  a.points = {{{0, 0, 1.}, 1, 0, false}};
  for (int i = 0; i < 100; ++i)
    a.points.push_back(a.points.front());
  auto s = a;
  s.source = "stereo";
  f.integrate({a, s}, b);
  EXPECT_NEAR(f.tree().search(1., 0, .4)->getLogOdds(), .85, 1e-5);
}
TEST(Fusion, UncertainSparseHitNeedsMoreEvidence) {
  BEV b;
  OccupancyFusion f;
  auto o = observation("stereo");
  o.origin = {0, 0, .4};
  o.points = {{{0, 0, 1.}, .2f, .04f, false}};
  f.integrate({o}, b);
  EXPECT_EQ(b.occupancy()(b.cell({1.05, .05})), 255);
}
TEST(Fusion, SubsequentReliableFreeRaysCanClearRemovedObstacle) {
  BEV b;
  OccupancyFusion f;
  auto o = observation("depth");
  o.origin = {0, 0, .4};
  o.points = {{{0, 0, 1.}, 1, 0, false}};
  f.integrate({o}, b);
  for (int i = 2; i < 9; ++i) {
    o.stamp = i;
    o.points = {{{0, 0, 2.}, 1, 0, false}};
    f.integrate({o}, b);
  }
  EXPECT_LT(f.tree().search(1., 0, .4)->getOccupancy(), .45);
}
TEST(Fusion, InvalidSamplesAreRejected) {
  BEV b;
  OccupancyFusion f;
  auto o = observation("depth");
  o.points = {{{NAN, 0, 1}, 1, 0, false},
              {{0, 0, 1}, 1, -1, false},
              {{0, 0, 0}, 1, 0, false}};
  auto status = f.integrate({o}, b);
  EXPECT_EQ(status["rejected_points"], 3);
  EXPECT_EQ(f.tree().size(), 0u);
}
TEST(Fusion, TimeResetClearsAllEvidence) {
  BEV b;
  OccupancyFusion f;
  auto o = observation("depth");
  verticalGround(o);
  f.integrate({o}, b);
  f.clear();
  f.project(b);
  EXPECT_EQ(f.tree().size(), 0u);
  EXPECT_EQ(cv::countNonZero(b.occupancy() != 255), 0);
  o.stamp = .1;
  f.integrate({o}, b);
  EXPECT_GT(f.tree().size(), 0u);
}
TEST(Fusion, MappingDegradationDoesNotMutateRawDepth) {
  cv::Mat1f raw(40, 60, 2.f);
  auto d = degradeDepth(raw, 1., 0, 8, 42);
  EXPECT_EQ(cv::countNonZero(raw != 2.f), 0);
  EXPECT_FALSE(std::isfinite(d(0, 0)));
  EXPECT_THROW(degradeDepth(raw, -1, 0, 8, 0), std::invalid_argument);
}
TEST(Fusion, DegradationIsRepeatable) {
  cv::Mat1f raw(40, 60, 2.f);
  auto a = degradeDepth(raw, .2, .01, 8, 42),
       b = degradeDepth(raw, .2, .01, 8, 42);
  EXPECT_EQ(std::memcmp(a.data, b.data, a.total() * sizeof(float)), 0);
}
TEST(Fusion, CloudRoundTripAndMalformedLayout) {
  std_msgs::msg::Header h;
  std::vector<DepthPoint> points{{{1, 2, 3}, .8f, .01f, true}};
  auto cloud = pointCloud(points, h);
  auto decoded = readPoints(cloud);
  ASSERT_EQ(decoded.size(), 1u);
  EXPECT_EQ(decoded[0].p, points[0].p);
  EXPECT_FLOAT_EQ(decoded[0].confidence, .8f);
  cloud.fields.clear();
  EXPECT_THROW(readPoints(cloud), std::invalid_argument);
}
TEST(Fusion, ReferenceMetricsCountUnsafeFreeCells) {
  BEV ref, est;
  auto c = ref.cell({1, 1});
  ref.seen(c) = est.seen(c) = 255;
  ref.log(c) = 3;
  est.log(c) = -3;
  auto metrics = compareOccupancy(est, ref);
  EXPECT_EQ(metrics["false_free_cells"], 1);
  EXPECT_EQ(metrics["occupied_recall"], 0.);
}

TEST(Fusion, InjectedDepthNoiseVarianceIsAvailableToFusion) {
  auto o = depthObservation(cv::Mat1f(10, 10, 2.f),
                            cv::Matx33d(10, 0, 5, 0, 10, 5, 0, 0, 1),
                            cv::Matx33d::eye(), {0, 0, 0}, 1., 8., .01);
  ASSERT_FALSE(o.points.empty());
  EXPECT_FLOAT_EQ(o.points.front().variance, .01f);
  EXPECT_THROW(depthObservation(cv::Mat1f(1, 1, 1.f), cv::Matx33d::eye(),
                                cv::Matx33d::eye(), {0, 0, 0}, 1., 8., -1),
               std::invalid_argument);
}

TEST(Fusion, StereoRecoversKnownCellsInsidePersistentDepthHole) {
  cv::Mat1f raw(360, 640, 2.4f), masked = raw.clone();
  masked(cv::Rect(160, 90, 320, 180))
      .setTo(std::numeric_limits<float>::quiet_NaN());
  cv::Matx33d depth_K(640. / (2 * std::tan(87 * CV_PI / 360)), 0, 320, 0,
                      360. / (2 * std::tan(58 * CV_PI / 360)), 180, 0, 0, 1);
  cv::Matx33d down(1, 0, 0, 0, -1, 0, 0, 0, -1);
  auto dense = depthObservation(masked, depth_K, down, {0, 0, 2.4}, 1.);
  cv::Mat1b left(512, 640);
  cv::RNG rng(42);
  rng.fill(left, cv::RNG::UNIFORM, 0, 255);
  cv::GaussianBlur(left, left, {3, 3}, .5);
  cv::Mat1b right;
  cv::warpAffine(left, right, cv::Matx23d(1, 0, -100., 0, 1, 0), left.size());
  StereoCalibration c;
  c.size = left.size();
  c.left_K = c.right_K = {25. / .012, 0, 320, 0, 25. / .012, 256, 0, 0, 1};
  auto result = StereoFrontend().estimate(left, right, c);
  ASSERT_GT(result.points.size(), 100u);
  Observation stereo;
  stereo.source = "stereo";
  stereo.stamp = 1.;
  stereo.R = down;
  stereo.origin = {0, 0, 2.5};
  stereo.points = result.points;
  BEV baseline, fused;
  OccupancyFusion depth_only, combined;
  depth_only.integrate({dense}, baseline);
  combined.integrate({dense, stereo}, fused);
  int before = cv::countNonZero(baseline.occupancy() != 255),
      after = cv::countNonZero(fused.occupancy() != 255);
  EXPECT_GT(after, before);
  std::cout << "CONTROLLED_DEPTH_HOLE known_cells depth_only=" << before
            << " fused=" << after << " stereo_points=" << result.points.size()
            << std::endl;
}

TEST(Fusion, KnownWallStopsClearingBehindIt) {
  BEV b;OccupancyFusion f;auto o=observation("depth");o.origin={0,0,.4};
  o.points={{{0,0,1},1,0,false}};f.integrate({o},b);
  o.stamp=2;o.points={{{0,0,2},1,0,false}};
  auto status=f.integrate({o},b);
  EXPECT_GT(status["occlusion_stopped_rays"].get<int>(),0);
  EXPECT_EQ(f.tree().search(1.5,0,.4),nullptr);
  EXPECT_EQ(b.occupancy()(b.cell({1.05,.05})),100);
}
TEST(Fusion, HistoricalEvidenceAndThermalMaskStayIndependent) {
  BEV b;OccupancyFusion f;auto o=observation("depth");verticalGround(o);
  f.integrate({o},b);auto before=b.occupancy();
  o.stamp=2;o.points.clear();f.integrate({o},b);
  EXPECT_EQ(cv::countNonZero(before!=b.occupancy()),0);
  EXPECT_EQ(cv::countNonZero(b.scanned),0);
  auto inferred=b.complete({{1,0,0},{1,.01,0},{1,.02,0},{1,.03,0}},{0,0});
  EXPECT_GT(inferred["valid_bins"].get<int>(),0);
  auto inf=b.inferred.clone();o.stamp=3;f.integrate({o},b);
  EXPECT_EQ(cv::countNonZero(inf!=b.inferred),0);
  EXPECT_EQ(cv::countNonZero(before!=b.occupancy()),0);
}
TEST(Fusion, GroundDoesNotCertifyUnobservedUpperClearance) {
  BEV b;OccupancyFusion f;auto o=observation("depth");o.origin={0,0,.4};
  o.points={{{0,.4,2},1,0,true}};f.integrate({o},b);
  auto c=b.cell({2.05,.05});
  EXPECT_TRUE(b.floor_evidence(c));
  EXPECT_EQ(b.occupancy()(c),255);
}
TEST(Fusion, TranslationAndRotationAccumulateSeparateWorldGround) {
  BEV b;OccupancyFusion f;auto o=observation("depth");verticalGround(o);f.integrate({o},b);
  auto old=b.occupancy();int free0=cv::countNonZero(old==0);
  o.stamp=2;o.origin[0]=2;f.integrate({o},b);
  EXPECT_GE(cv::countNonZero(b.occupancy()==0),2*free0);
  EXPECT_EQ(cv::countNonZero((old==0)&(b.occupancy()!=0)),0);
  o.stamp=3;cv::Matx33d yaw(0,-1,0,1,0,0,0,0,1);o.R=yaw*o.R;o.origin={0,2,1};
  f.integrate({o},b);EXPECT_GE(cv::countNonZero(b.occupancy()==0),3*free0);
}
