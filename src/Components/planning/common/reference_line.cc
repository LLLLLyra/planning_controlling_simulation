#include "reference_line.h"

#include <algorithm>
#include <limits>
#include <unordered_set>

#include "common/math/angle.h"
#include "common/math/cartesian_frenet_conversion.h"
#include "common/math/linear_interpolation.h"
#include "common/math/vec2d.h"
#include "boost/math/tools/minima.hpp"
#include "glog/logging.h"

MapPath::MapPath(const std::vector<MapPathPoint>& path_points)
    : path_points_(path_points) {
  InitPoints();
};

MapPath::MapPath(std::vector<MapPathPoint>&& path_points)
    : path_points_(std::move(path_points)) {
  InitPoints();
  InitPointIndex();
  InitWidth();
};

void MapPath::InitPoints() {
  num_points_ = static_cast<int>(path_points_.size());
  CHECK_GE(num_points_, 2);

  accumulated_s_.clear();
  accumulated_s_.reserve(num_points_);
  segments_.clear();
  segments_.reserve(num_points_);
  unit_directions_.clear();
  unit_directions_.reserve(num_points_);
  double s = 0.0;
  for (int i = 0; i < num_points_; ++i) {
    accumulated_s_.push_back(s);
    Vec2d heading;
    if (i + 1 >= num_points_) {
      heading = path_points_[i] - path_points_[i - 1];
    } else {
      segments_.emplace_back(path_points_[i], path_points_[i + 1]);
      heading = path_points_[i + 1] - path_points_[i];
      // TODO(All): use heading.length when all adjacent lanes are guarantee to
      // be connected.
      s += heading.Length();
    }
    heading.Normalize();
    unit_directions_.push_back(heading);
  }
  length_ = s;
  num_sample_points_ = static_cast<int>(length_ / kSampleDistance) + 1;
  num_segments_ = num_points_ - 1;

  CHECK_EQ(accumulated_s_.size(), static_cast<size_t>(num_points_));
  CHECK_EQ(unit_directions_.size(), static_cast<size_t>(num_points_));
  CHECK_EQ(segments_.size(), static_cast<size_t>(num_segments_));
};

void MapPath::InitPointIndex() {
  last_point_index_.clear();
  last_point_index_.reserve(num_sample_points_);
  double s = 0.0;
  int last_index = 0;
  for (int i = 0; i < num_sample_points_; ++i) {
    while (last_index + 1 < num_points_ &&
           accumulated_s_[last_index + 1] <= s) {
      ++last_index;
    }
    last_point_index_.push_back(last_index);
    s += kSampleDistance;
  }
  CHECK_EQ(last_point_index_.size(), static_cast<size_t>(num_sample_points_));
}

void MapPath::InitWidth() {
  lane_left_width_.clear();
  lane_left_width_.reserve(num_sample_points_);
  lane_right_width_.clear();
  lane_right_width_.reserve(num_sample_points_);

  road_left_width_.clear();
  road_left_width_.reserve(num_sample_points_);
  road_right_width_.clear();
  road_right_width_.reserve(num_sample_points_);

  double s = 0;
  for (int i = 0; i < num_sample_points_; ++i) {
    const MapPathPoint point = GetSmoothPoint(s);
    if (point.get_lane_left_width() <= 0 || point.get_lane_right_width() <= 0) {
      lane_left_width_.push_back(FLAGS_default_lane_width / 2.0);
      lane_right_width_.push_back(FLAGS_default_lane_width / 2.0);

      road_left_width_.push_back(FLAGS_default_lane_width / 2.0);
      road_right_width_.push_back(FLAGS_default_lane_width / 2.0);
      LOG(INFO) << "path point:" << point.DebugString()
                << " has invalid width.";
    } else {
      lane_left_width_.push_back(point.get_lane_left_width());
      lane_right_width_.push_back(point.get_lane_right_width());
      road_left_width_.push_back(point.get_lane_left_width());
      road_right_width_.push_back(point.get_lane_right_width());
    }
    s += kSampleDistance;
  }

  auto num_sample_points = static_cast<size_t>(num_sample_points_);
  CHECK_EQ(lane_left_width_.size(), num_sample_points);
  CHECK_EQ(lane_right_width_.size(), num_sample_points);

  CHECK_EQ(road_left_width_.size(), num_sample_points);
  CHECK_EQ(road_right_width_.size(), num_sample_points);
}

MapPathPoint MapPath::GetSmoothPoint(const InterpolatedIndex& index) const {
  CHECK_GE(index.id, 0);
  CHECK_LT(index.id, num_points_);

  const MapPathPoint& ref_point = path_points_[index.id];
  if (std::abs(index.offset) > kMathEpsilon) {
    const Vec2d delta = unit_directions_[index.id] * index.offset;
    MapPathPoint point({ref_point.x() + delta.x(), ref_point.y() + delta.y()},
                       ref_point.heading());
    point.set_lane_left_width(ref_point.get_lane_left_width());
    point.set_lane_right_width(ref_point.get_lane_right_width());
    return point;
  } else {
    return ref_point;
  }
}

MapPathPoint MapPath::GetSmoothPoint(double s) const {
  return GetSmoothPoint(GetIndexFromS(s));
}

bool MapPath::GetProjection(const Vec2d& point, double* accumulate_s,
                            double* lateral) const {
  double distance = 0.0;
  return GetProjection(point, accumulate_s, lateral, &distance);
}

bool MapPath::GetProjection(const Vec2d& point, double* accumulate_s,
                            double* lateral, double* min_distance) const {
  if (segments_.empty()) {
    return false;
  }
  if (accumulate_s == nullptr || lateral == nullptr ||
      min_distance == nullptr) {
    return false;
  }
  // if (use_path_approximation_) {
  //   return approximation_.GetProjection(*this, point, accumulate_s, lateral,
  //                                       min_distance);
  // }
  CHECK_GE(num_points_, 2);
  *min_distance = std::numeric_limits<double>::infinity();
  int min_index = 0;
  for (int i = 0; i < num_segments_; ++i) {
    const double distance = segments_[i].DistanceSquareTo(point);
    if (distance < *min_distance) {
      min_index = i;
      *min_distance = distance;
    }
  }
  *min_distance = std::sqrt(*min_distance);
  const auto& nearest_seg = segments_[min_index];
  const auto prod = nearest_seg.ProductOntoUnit(point);
  const auto proj = nearest_seg.ProjectOntoUnit(point);
  if (min_index == 0) {
    *accumulate_s = std::min(proj, nearest_seg.length());
    if (proj < 0) {
      *lateral = prod;
    } else {
      *lateral = (prod > 0.0 ? 1 : -1) * *min_distance;
    }
  } else if (min_index == num_segments_ - 1) {
    *accumulate_s = accumulated_s_[min_index] + std::max(0.0, proj);
    if (proj > 0) {
      *lateral = prod;
    } else {
      *lateral = (prod > 0.0 ? 1 : -1) * *min_distance;
    }
  } else {
    *accumulate_s = accumulated_s_[min_index] +
                    std::max(0.0, std::min(proj, nearest_seg.length()));
    *lateral = (prod > 0.0 ? 1 : -1) * *min_distance;
  }
  return true;
}

double MapPath::GetSample(const std::vector<double>& samples,
                          const double s) const {
  if (samples.empty()) {
    return 0.0;
  }
  if (s <= 0.0) {
    return samples[0];
  }
  const int idx = static_cast<int>(s / kSampleDistance);
  if (idx >= num_sample_points_ - 1) {
    return samples.back();
  }
  const double ratio = (s - idx * kSampleDistance) / kSampleDistance;
  return samples[idx] * (1.0 - ratio) + samples[idx + 1] * ratio;
};

InterpolatedIndex MapPath::GetIndexFromS(double s) const {
  if (s <= 0.0) {
    return {0, 0.0};
  }
  CHECK_GT(num_points_, 0);
  if (s >= length_) {
    return {num_points_ - 1, 0.0};
  }
  const int sample_id = static_cast<int>(s / kSampleDistance);
  if (sample_id >= num_sample_points_) {
    return {num_points_ - 1, 0.0};
  }
  const int next_sample_id = sample_id + 1;
  int low = last_point_index_[sample_id];
  int high = (next_sample_id < num_sample_points_
                  ? std::min(num_points_, last_point_index_[next_sample_id] + 1)
                  : num_points_);
  while (low + 1 < high) {
    const int mid = (low + high) / 2;
    if (accumulated_s_[mid] <= s) {
      low = mid;
    } else {
      high = mid;
    }
  }
  return {low, s - accumulated_s_[low]};
};

bool MapPath::GetLaneWidth(const double s, double* lane_left_width,
                           double* lane_right_width) const {
  CHECK_NOTNULL(lane_left_width);
  CHECK_NOTNULL(lane_right_width);

  if (s < 0.0 || s > length_) {
    return false;
  }
  *lane_left_width = GetSample(lane_left_width_, s);
  *lane_right_width = GetSample(lane_right_width_, s);
  return true;
};

bool MapPath::OverlapWith(const Box2d& box, double width) const {
  const Vec2d center = box.center();
  const double radius_sqr = Sqr(box.diagonal() / 2.0 + width) + kMathEpsilon;
  for (const auto& segment : segments_) {
    if (segment.DistanceSquareTo(center) > radius_sqr) {
      continue;
    }
    if (box.DistanceTo(segment) <= width + kMathEpsilon) {
      return true;
    }
  }
  return false;
};

ReferenceLine::ReferenceLine(
    const std::vector<ReferencePoint>& reference_points)
    : reference_points_(reference_points),
      map_path_(std::move(std::vector<MapPathPoint>(reference_points.begin(),
                                                    reference_points.end()))) {
  CHECK_EQ(static_cast<size_t>(map_path_.num_points()),
           reference_points_.size());
};

ReferenceLine::ReferenceLine(const MapPath& hdmap_path)
    : map_path_(hdmap_path) {
  for (const auto& point : hdmap_path.path_points()) {
    DCHECK(!point.lane_waypoints().empty());
    const auto& lane_waypoint = point.lane_waypoints()[0];
    reference_points_.emplace_back(
        MapPathPoint(point, point.heading(), lane_waypoint), 0.0, 0.0);
  }
  CHECK_EQ(static_cast<size_t>(map_path_.num_points()),
           reference_points_.size());
}

const MapPath& ReferenceLine::map_path() const { return map_path_; }

bool ReferenceLine::Stitch(const ReferenceLine& other) {
  if (other.reference_points().empty()) {
    LOG(INFO) << "The other reference line is empty.";
    return true;
  }
  auto first_point = reference_points_.front();
  points::SLPoint first_sl;
  if (!other.XYToSL(first_point, &first_sl)) {
    LOG(INFO)
        << "Failed to project the first point to the other reference line.";
    return false;
  }
  bool first_join = first_sl.s() > 0 && first_sl.s() < other.Length();

  auto last_point = reference_points_.back();
  points::SLPoint last_sl;
  if (!other.XYToSL(last_point, &last_sl)) {
    LOG(INFO)
        << "Failed to project the last point to the other reference line.";
    return false;
  }
  bool last_join = last_sl.s() > 0 && last_sl.s() < other.Length();

  if (!first_join && !last_join) {
    LOG(ERROR) << "These reference lines are not connected.";
    return false;
  }

  const auto& accumulated_s = other.map_path().accumulated_s();
  const auto& other_points = other.reference_points();
  auto lower = accumulated_s.begin();
  static constexpr double kStitchingError = 1e-1;
  if (first_join) {
    if (first_sl.l() > kStitchingError) {
      LOG(ERROR)
          << "lateral stitching error on first join of reference line too "
             "big, stitching fails";
      return false;
    }
    lower = std::lower_bound(accumulated_s.begin(), accumulated_s.end(),
                             first_sl.s());
    size_t start_i = std::distance(accumulated_s.begin(), lower);
    reference_points_.insert(reference_points_.begin(), other_points.begin(),
                             other_points.begin() + start_i);
  }
  if (last_join) {
    if (last_sl.l() > kStitchingError) {
      LOG(ERROR)
          << "lateral stitching error on first join of reference line too "
             "big, stitching fails";
      return false;
    }
    auto upper = std::upper_bound(lower, accumulated_s.end(), last_sl.s());
    auto end_i = std::distance(accumulated_s.begin(), upper);
    reference_points_.insert(reference_points_.end(),
                             other_points.begin() + end_i, other_points.end());
  }
  map_path_ = MapPath(std::move(std::vector<MapPathPoint>(
      reference_points_.begin(), reference_points_.end())));
  return true;
}

ReferencePoint ReferenceLine::GetReferencePoint(const double s) const {
  const auto& accumulated_s = map_path_.accumulated_s();
  if (s < accumulated_s.front() - 1e-2) {
    LOG(INFO) << "The requested s: " << s << " < 0.";
    return reference_points_.front();
  }
  if (s > accumulated_s.back() + 1e-2) {
    LOG(INFO) << "The requested s: " << s
              << " > reference line length: " << accumulated_s.back();
    return reference_points_.back();
  }

  auto interpolate_index = map_path_.GetIndexFromS(s);

  size_t index = interpolate_index.id;
  size_t next_index = index + 1;
  if (next_index >= reference_points_.size()) {
    next_index = reference_points_.size() - 1;
  }

  const auto& p0 = reference_points_[index];
  const auto& p1 = reference_points_[next_index];

  const double s0 = accumulated_s[index];
  const double s1 = accumulated_s[next_index];
  return InterpolateWithMatchedIndex(p0, s0, p1, s1, interpolate_index);
}

points::FrenetFramePoint ReferenceLine::GetFrenetPoint(
    const points::PathPoint& path_point) const {
  if (reference_points_.empty()) {
    return points::FrenetFramePoint();
  }

  points::SLPoint sl_point;
  XYToSL(path_point, &sl_point);
  points::FrenetFramePoint frenet_frame_point;
  frenet_frame_point.set_s(sl_point.s());
  frenet_frame_point.set_l(sl_point.l());

  const double theta = path_point.theta();
  const double kappa = path_point.kappa();
  const double l = frenet_frame_point.l();

  ReferencePoint ref_point = GetReferencePoint(frenet_frame_point.s());

  const double theta_ref = ref_point.heading();
  const double kappa_ref = ref_point.kappa();
  const double dkappa_ref = ref_point.dkappa();

  const double dl = CartesianFrenetConverter::CalculateLateralDerivative(
      theta_ref, theta, l, kappa_ref);
  const double ddl =
      CartesianFrenetConverter::CalculateSecondOrderLateralDerivative(
          theta_ref, theta, kappa_ref, kappa, dkappa_ref, l);
  frenet_frame_point.set_dl(dl);
  frenet_frame_point.set_ddl(ddl);
  return frenet_frame_point;
}

std::pair<std::array<double, 3>, std::array<double, 3>>
ReferenceLine::ToFrenetFrame(const points::TrajectoryPoint& traj_point) const {
  CHECK(!reference_points_.empty());

  points::SLPoint sl_point;
  XYToSL(traj_point.path_point(), &sl_point);

  std::array<double, 3> s_condition;
  std::array<double, 3> l_condition;
  ReferencePoint ref_point = GetReferencePoint(sl_point.s());
  CartesianFrenetConverter::cartesian_to_frenet(
      sl_point.s(), ref_point.x(), ref_point.y(), ref_point.heading(),
      ref_point.kappa(), ref_point.dkappa(), traj_point.path_point().x(),
      traj_point.path_point().y(), traj_point.v(), traj_point.a(),
      traj_point.path_point().theta(), traj_point.path_point().kappa(),
      &s_condition, &l_condition);

  return std::make_pair(s_condition, l_condition);
}

ReferencePoint ReferenceLine::InterpolateWithMatchedIndex(
    const ReferencePoint& p0, const double s0, const ReferencePoint& p1,
    const double s1, const InterpolatedIndex& index) const {
  if (std::fabs(s0 - s1) < kMathEpsilon) {
    return p0;
  }
  double s = s0 + index.offset;
  DCHECK_LE(s0 - 1.0e-6, s) << "s: " << s << " is less than s0 : " << s0;
  DCHECK_LE(s, s1 + 1.0e-6) << "s: " << s << " is larger than s1: " << s1;

  auto map_path_point = map_path_.GetSmoothPoint(index);
  const double kappa = lerp(p0.kappa(), s0, p1.kappa(), s1, s);
  const double dkappa = lerp(p0.dkappa(), s0, p1.dkappa(), s1, s);

  return ReferencePoint(map_path_point, kappa, dkappa);
}

ReferencePoint ReferenceLine::GetNearestReferencePoint(const Vec2d& xy) const {
  double min_dist = std::numeric_limits<double>::max();
  size_t min_index = 0;
  for (size_t i = 0; i < reference_points_.size(); ++i) {
    const double distance = DistanceXY(xy, reference_points_[i]);
    if (distance < min_dist) {
      min_dist = distance;
      min_index = i;
    }
  }
  return reference_points_[min_index];
}

bool ReferenceLine::Segment(const Vec2d& point, const double look_backward,
                            const double look_forward) {
  points::SLPoint sl;
  if (!XYToSL(point, &sl)) {
    LOG(ERROR) << "Failed to project point: " << point.DebugString();
    return false;
  }
  return Segment(sl.s(), look_backward, look_forward);
}

bool ReferenceLine::Segment(const double s, const double look_backward,
                            const double look_forward) {
  const auto& accumulated_s = map_path_.accumulated_s();

  // inclusive
  auto start_index =
      std::distance(accumulated_s.begin(),
                    std::lower_bound(accumulated_s.begin(), accumulated_s.end(),
                                     s - look_backward));

  // exclusive
  auto end_index =
      std::distance(accumulated_s.begin(),
                    std::upper_bound(accumulated_s.begin(), accumulated_s.end(),
                                     s + look_forward));

  if (end_index - start_index < 2) {
    LOG(ERROR) << "Too few reference points after shrinking.";
    return false;
  }

  reference_points_ =
      std::vector<ReferencePoint>(reference_points_.begin() + start_index,
                                  reference_points_.begin() + end_index);

  map_path_ = MapPath(std::vector<MapPathPoint>(reference_points_.begin(),
                                                reference_points_.end()));
  return true;
}

ReferencePoint ReferenceLine::GetNearestReferencePoint(const double s) const {
  const auto& accumulated_s = map_path_.accumulated_s();
  if (s < accumulated_s.front() - 1e-2) {
    LOG(INFO) << "The requested s: " << s << " < 0.";
    return reference_points_.front();
  }
  if (s > accumulated_s.back() + 1e-2) {
    LOG(INFO) << "The requested s: " << s
              << " > reference line length: " << accumulated_s.back();
    return reference_points_.back();
  }
  auto it_lower =
      std::lower_bound(accumulated_s.begin(), accumulated_s.end(), s);
  if (it_lower == accumulated_s.begin()) {
    return reference_points_.front();
  }
  auto index = std::distance(accumulated_s.begin(), it_lower);
  if (std::fabs(accumulated_s[index - 1] - s) <
      std::fabs(accumulated_s[index] - s)) {
    return reference_points_[index - 1];
  }
  return reference_points_[index];
}

size_t ReferenceLine::GetNearestReferenceIndex(const double s) const {
  const auto& accumulated_s = map_path_.accumulated_s();
  if (s < accumulated_s.front() - 1e-2) {
    LOG(INFO) << "The requested s: " << s << " < 0.";
    return 0;
  }
  if (s > accumulated_s.back() + 1e-2) {
    LOG(INFO) << "The requested s: " << s << " > reference line length "
              << accumulated_s.back();
    return reference_points_.size() - 1;
  }
  auto it_lower =
      std::lower_bound(accumulated_s.begin(), accumulated_s.end(), s);
  return std::distance(accumulated_s.begin(), it_lower);
}

std::vector<ReferencePoint> ReferenceLine::GetReferencePoints(
    double start_s, double end_s) const {
  if (start_s < 0.0) {
    start_s = 0.0;
  }
  if (end_s > Length()) {
    end_s = Length();
  }
  std::vector<ReferencePoint> ref_points;
  auto start_index = GetNearestReferenceIndex(start_s);
  auto end_index = GetNearestReferenceIndex(end_s);
  if (start_index < end_index) {
    ref_points.assign(reference_points_.begin() + start_index,
                      reference_points_.begin() + end_index);
  }
  return ref_points;
}

bool ReferenceLine::XYToSL(const Vec2d& xy_point,
                           points::SLPoint* const sl_point) const {
  double s = 0.0;
  double l = 0.0;
  if (!map_path_.GetProjection(xy_point, &s, &l)) {
    LOG(ERROR) << "Cannot get nearest point from path.";
    return false;
  }
  sl_point->set_s(s);
  sl_point->set_l(l);
  return true;
}

double ReferenceLine::GetSpeedLimitFromS(const double s) const {
  for (const auto& speed_limit : speed_limit_) {
    if (s >= speed_limit.start_s && s <= speed_limit.end_s) {
      return speed_limit.speed_limit;
    }
  }
  double speed_limit = 2.78;
  return speed_limit;
}

bool ReferenceLine::GetLaneWidth(const double s, double* const lane_left_width,
                                 double* const lane_right_width) const {
  if (map_path_.path_points().empty()) {
    return false;
  }

  if (!map_path_.GetLaneWidth(s, lane_left_width, lane_right_width)) {
    return false;
  }
  return true;
}

bool ReferenceLine::IsBlockRoad(const Box2d& box2d, double gap) const {
  return map_path_.OverlapWith(box2d, gap);
}

double ReferenceLine::GetDrivingWidth(
    const boundary::SLBoundary& sl_boundary) const {
  double lane_left_width = 0.0;
  double lane_right_width = 0.0;
  GetLaneWidth(sl_boundary.start_s(), &lane_left_width, &lane_right_width);

  double driving_width = std::max(lane_left_width - sl_boundary.end_l(),
                                  lane_right_width + sl_boundary.start_l());
  driving_width = std::min(lane_left_width + lane_right_width, driving_width);
  LOG(INFO) << "Driving width [" << driving_width << "].";
  return driving_width;
}

bool ReferenceLine::IsOnLane(const boundary::SLBoundary& sl_boundary) const {
  if (sl_boundary.end_s() < 0 || sl_boundary.start_s() > Length()) {
    return false;
  }
  double middle_s = (sl_boundary.start_s() + sl_boundary.end_s()) / 2.0;
  double lane_left_width = 0.0;
  double lane_right_width = 0.0;
  map_path_.GetLaneWidth(middle_s, &lane_left_width, &lane_right_width);
  return sl_boundary.start_l() <= lane_left_width &&
         sl_boundary.end_l() >= -lane_right_width;
}
