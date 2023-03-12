#include "cruise_trajectory.h"

#include "common/math/Bspline.h"
#include "common/math/local_outlier_factor.h"
#include "glog/logging.h"

bool CruiseTrajectory::LoadPathPoints(
    std::vector<std::pair<double, double>>& xy_points, bool smoothing,
    RFPS* ref_points) {
  CHECK(ref_points) << "ref_points is nullptr";
  std::vector<double> headings;
  std::vector<double> kappas;
  std::vector<double> dkappas;
  std::vector<double> accumulated_s;

  std::vector<double> xpoints, ypoints;
  for (size_t i = 0; i < xy_points.size(); i += 5) {
    xpoints.push_back(xy_points[i].first);
    ypoints.push_back(xy_points[i].second);
  }
  const int k = 2;
  const int num = xpoints.size();
  std::vector<double> t;
  for (int i = 0; i < num + k + 1; i++) {
    if (i <= k)
      t.push_back(0);
    else if (i >= num)
      t.push_back(num - k);
    else
      t.push_back(i - k);
  }

  BSpline<double, double> spl_x = BSpline<double, double>(t, xpoints, k);
  BSpline<double, double> spl_y = BSpline<double, double>(t, ypoints, k);
  xy_points_.clear();
  const int samples = 200;
  const double kDenseStep = static_cast<double>(num - k + 1) / samples;
  for (double s = 0.0; s <= num - k; s += kDenseStep) {
    xy_points_.push_back({spl_x.bspline(s), spl_y.bspline(s)});
  }
  xy_points_.push_back({spl_x.bspline(num - k - kMathEpsilon),
                        spl_y.bspline(num - k - kMathEpsilon)});

  if (!DiscretePointsMath::ComputePathProfile(
          xy_points_, &headings, &accumulated_s, &kappas, &dkappas)) {
    return false;
  }

  if (!smoothing) {
    size_t points_size = xy_points_.size();
    for (size_t i = 0; i < points_size; ++i) {
      ref_points->emplace_back(ReferencePoint(
          MapPathPoint(Vec2d(xy_points_[i].first, xy_points_[i].second),
                       headings[i]),
          kappas[i], dkappas[i]));
    }
  } else {
    std::vector<AnchorPoint> anchor_points;
    size_t points_size = xy_points.size();
    for (size_t i = 0; i < points_size; ++i) {
      points::PathPoint p;
      AnchorPoint ap;
      p.set_x(xy_points[i].first);
      p.set_y(xy_points[i].second);
      p.set_kappa(kappas[i]);
      p.set_dkappa(dkappas[i]);
      p.set_theta(headings[i]);
      p.set_s(accumulated_s[i]);
      ap.path_point = p;
      anchor_points.push_back(ap);
    }
    // smooth begins
    DiscretePointsReferenceLineSmoother Smoother;
    Smoother.SetAnchorPoints(anchor_points);
    return Smoother.Smooth(ref_points);
    // smooth ends
  }

  return true;
}

bool CruiseTrajectory::LoadPathPoints(const std::string file_name,
                                      bool smoothing, RFPS* ref_points) {
  CHECK(ref_points) << "ref_points is nullptr";
  CHECK(LoadDataFromFile(file_name)) << "fail to load from file";

  // drop outliers
  LocalOutlierFactor lof(xy_points_);
  std::vector<int> outlier_indeces;
  lof.GetOutliers(FLAGS_k_LOF, &outlier_indeces);
  for (size_t i = 0; i < outlier_indeces.size(); i++) {
    xy_points_.erase(xy_points_.begin() + outlier_indeces[i] - i);
  }
  CutOverlap(&xy_points_);
  return LoadPathPoints(xy_points_, smoothing, ref_points);
}

bool CruiseTrajectory::GenerateTrajectory(RFPS ref_points) {
  ref_points_ = ref_points;
  return GenerateTrajectory();
}

bool CruiseTrajectory::GenerateTrajectory() {
  CHECK(ref_points_.size()) << "Unable to get reference points";

  kinetic_result_.x.clear();
  kinetic_result_.y.clear();
  kinetic_result_.phi.clear();
  for (auto& point : ref_points_) {
    auto path_point = point.ToPathPoint(0);
    kinetic_result_.x.push_back(path_point.x());
    kinetic_result_.y.push_back(path_point.y());
    kinetic_result_.phi.push_back(path_point.theta());
  }

  return SpeedGenerator::GenerateSCurveSpeedAcceleration(
      &kinetic_result_, vehicle_param_, piecewise_conf_);
}

bool CruiseTrajectory::LoadDataFromFile(const std::string file_name) {
  double time_idx, type, x, y, z, qw, qx, qy, qz;

  FILE* fptr = fopen(file_name.data(), "r");
  while (fscanf(fptr, "%lf %lf %lf %lf %lf %lf %lf %lf %lf", &time_idx, &type,
                &x, &y, &z, &qw, &qx, &qy, &qz) == 9) {
    xy_points_.push_back({x, y});
  }
  fclose(fptr);
  return true;
}

bool CruiseTrajectory::CutOverlap(
    std::vector<std::pair<double, double>>* xy_points) {
  CHECK(xy_points) << "nullptr xy_points";
  const size_t n = xy_points->size();
  if (n <= 3) {
    LOG(INFO) << "the number of points is too small";
    return false;
  }

  // !Note: For circled paths, there should be overlaps;
  // !otherwise, paths would be treated as uncircled
  return CutFromTail(xy_points) || CutFromHead(xy_points);
}

double CruiseTrajectory::GetDistanceSquared(std::pair<double, double>& a,
                                            std::pair<double, double>& b) {
  double dx, dy;
  dx = a.first - b.first;
  dy = a.second - b.second;
  return dx * dx + dy * dy;
}

bool CruiseTrajectory::CutFromTail(
    std::vector<std::pair<double, double>>* xy_points) {
  const size_t n = xy_points->size();
  Vec2d path_dirction =
      Vec2d(xy_points->at(1).first - xy_points->at(0).first,
            xy_points->at(1).second - xy_points->at(0).second);
  // avoid stops at the head
  for (size_t i = 2; i < n; ++i) {
    if (path_dirction.x() != 0 && path_dirction.y() != 0) {
      break;
    }
    path_dirction.set_x(xy_points->at(i).first - xy_points->at(i - 1).first);
    path_dirction.set_y(xy_points->at(i).second - xy_points->at(i - 1).second);
  }

  double d_squared = GetDistanceSquared(xy_points->front(), xy_points->back());
  Vec2d tail_direction =
      Vec2d(xy_points->front().first - xy_points->back().first,
            xy_points->front().second - xy_points->back().second);
  size_t cut_index = 0;
  for (size_t i = n - 2; i > 0; i--) {
    if (d_squared < 3 && path_dirction.InnerProd(tail_direction) > 0) {
      cut_index = i;
      break;
    }
    d_squared = GetDistanceSquared(xy_points->front(), xy_points->at(i));
    tail_direction.set_x(xy_points->front().first - xy_points->at(i).first);
    tail_direction.set_y(xy_points->front().second - xy_points->at(i).second);
  }

  if (cut_index == 0) {
    LOG(INFO) << "the path is uncircled from tail";
    return false;
  }
  LOG(INFO) << "the cut index is " << cut_index;
  if (cut_index == n - 1) {
    return true;
  }
  // protect to not drop all path
  if (static_cast<double>(cut_index) / n <= 0.5) {
    return false;
  }
  xy_points->erase(xy_points->begin() + cut_index + 1, xy_points->end());
  return true;
}

bool CruiseTrajectory::CutFromHead(
    std::vector<std::pair<double, double>>* xy_points) {
  const size_t n = xy_points->size();
  Vec2d path_dirction =
      Vec2d(xy_points->at(n - 1).first - xy_points->at(n - 2).first,
            xy_points->at(n - 1).second - xy_points->at(n - 2).second);
  // avoid stops at the tail
  for (size_t i = n - 2; i > 0; --i) {
    if (path_dirction.x() != 0 && path_dirction.y() != 0) {
      break;
    }
    path_dirction.set_x(xy_points->at(i).first - xy_points->at(i - 1).first);
    path_dirction.set_y(xy_points->at(i).second - xy_points->at(i - 1).second);
  }

  double d_squared = GetDistanceSquared(xy_points->front(), xy_points->back());
  Vec2d head_direction =
      Vec2d(xy_points->front().first - xy_points->back().first,
            xy_points->front().second - xy_points->back().second);
  size_t cut_index = n;
  for (size_t i = 0; i < n; i++) {
    if (d_squared < 3 && path_dirction.InnerProd(head_direction) > 0) {
      cut_index = i;
      break;
    }
    d_squared = GetDistanceSquared(xy_points->back(), xy_points->at(i));
    head_direction.set_x(xy_points->at(i).first - xy_points->back().first);
    head_direction.set_y(xy_points->at(i).second - xy_points->back().second);
  }

  if (cut_index == n) {
    LOG(INFO) << "the path is uncircled from head";
    return false;
  }
  LOG(INFO) << "the cut index is " << cut_index;
  if (cut_index == 0) {
    return true;
  }
  // protect to not drop all path
  if (static_cast<double>(cut_index) / n >= 0.5) {
    return false;
  }
  xy_points->erase(xy_points->begin(), xy_points->begin() + cut_index);
  return true;
}