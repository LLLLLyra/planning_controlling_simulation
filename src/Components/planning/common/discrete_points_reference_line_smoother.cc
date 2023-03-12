#include "discrete_points_reference_line_smoother.h"

#include <algorithm>

#include "common/math/discrete_points_math.h"
#include "common/math/discrete_points_smoothing/fem_pos_deviation_smoother.h"
#include "glog/logging.h"

bool DiscretePointsReferenceLineSmoother::Smooth(
    std::vector<ReferencePoint>* ref_points) {
  std::vector<std::pair<double, double>> raw_point2d;
  std::vector<double> anchorpoints_lateralbound;

  for (const auto& anchor_point : anchor_points_) {
    raw_point2d.emplace_back(anchor_point.path_point.x(),
                             anchor_point.path_point.y());
    anchorpoints_lateralbound.emplace_back(anchor_point.lateral_bound);
  }

  // fix front and back points to avoid end states deviate from the center of
  // road
  anchorpoints_lateralbound.front() = 0.0;
  anchorpoints_lateralbound.back() = 0.0;

  NormalizePoints(&raw_point2d);

  bool status = false;

  std::vector<std::pair<double, double>> smoothed_point2d;
  status =
      FemPosSmooth(raw_point2d, anchorpoints_lateralbound, &smoothed_point2d);

  if (!status) {
    LOG(ERROR) << "discrete_points reference line smoother fails";
    return false;
  }

  DeNormalizePoints(&smoothed_point2d);

  GenerateRefPointProfile(smoothed_point2d, ref_points);

  ReferencePoint::RemoveDuplicates(ref_points);

  if (ref_points->size() < 2) {
    LOG(ERROR) << "Fail to generate smoothed reference line.";
    return false;
  }

  return true;
}

bool DiscretePointsReferenceLineSmoother::FemPosSmooth(
    const std::vector<std::pair<double, double>>& raw_point2d,
    const std::vector<double>& bounds,
    std::vector<std::pair<double, double>>* ptr_smoothed_point2d) {
  const auto& fem_pos_config = smoother::FemPosDeviationSmootherConfig();

  FemPosDeviationSmoother smoother(fem_pos_config);

  // box contraints on pos are used in fem pos smoother, thus shrink the
  // bounds by 1.0 / sqrt(2.0)
  std::vector<double> box_bounds = bounds;
  const double box_ratio = 1.0 / std::sqrt(2.0);
  for (auto& bound : box_bounds) {
    bound *= box_ratio;
  }

  std::vector<double> opt_x;
  std::vector<double> opt_y;
  bool status = smoother.Solve(raw_point2d, box_bounds, &opt_x, &opt_y);

  if (!status) {
    LOG(ERROR) << "Fem Pos reference line smoothing failed";
    return false;
  }

  if (opt_x.size() < 2 || opt_y.size() < 2) {
    LOG(ERROR) << "Return by fem pos smoother is wrong. Size smaller than 2 ";
    return false;
  }

  CHECK_EQ(opt_x.size(), opt_y.size()) << "x and y result size not equal";

  size_t point_size = opt_x.size();
  for (size_t i = 0; i < point_size; ++i) {
    ptr_smoothed_point2d->emplace_back(opt_x[i], opt_y[i]);
  }

  return true;
}

void DiscretePointsReferenceLineSmoother::SetAnchorPoints(
    const std::vector<AnchorPoint>& anchor_points) {
  CHECK_GT(anchor_points.size(), 1U);
  anchor_points_ = anchor_points;
}

void DiscretePointsReferenceLineSmoother::NormalizePoints(
    std::vector<std::pair<double, double>>* xy_points) {
  zero_x_ = xy_points->front().first;
  zero_y_ = xy_points->front().second;
  std::for_each(xy_points->begin(), xy_points->end(),
                [this](std::pair<double, double>& point) {
                  auto curr_x = point.first;
                  auto curr_y = point.second;
                  std::pair<double, double> xy(curr_x - zero_x_,
                                               curr_y - zero_y_);
                  point = std::move(xy);
                });
}

void DiscretePointsReferenceLineSmoother::DeNormalizePoints(
    std::vector<std::pair<double, double>>* xy_points) {
  std::for_each(xy_points->begin(), xy_points->end(),
                [this](std::pair<double, double>& point) {
                  auto curr_x = point.first;
                  auto curr_y = point.second;
                  std::pair<double, double> xy(curr_x + zero_x_,
                                               curr_y + zero_y_);
                  point = std::move(xy);
                });
}

bool DiscretePointsReferenceLineSmoother::GenerateRefPointProfile(
    const std::vector<std::pair<double, double>>& xy_points,
    std::vector<ReferencePoint>* reference_points) {
  // Compute path profile
  std::vector<double> headings;
  std::vector<double> kappas;
  std::vector<double> dkappas;
  std::vector<double> accumulated_s;
  if (!DiscretePointsMath::ComputePathProfile(
          xy_points, &headings, &accumulated_s, &kappas, &dkappas)) {
    return false;
  }

  // Load into ReferencePoints
  size_t points_size = xy_points.size();
  for (size_t i = 0; i < points_size; ++i) {
    reference_points->emplace_back(ReferencePoint(
        MapPathPoint(Vec2d(xy_points[i].first, xy_points[i].second),
                     headings[i]),
        kappas[i], dkappas[i]));
  }
  return true;
}
