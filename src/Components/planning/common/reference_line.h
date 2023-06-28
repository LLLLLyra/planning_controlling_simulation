#ifndef _REFERENCE_LINE_H_
#define _REFERENCE_LINE_H_

#include <string>
#include <utility>
#include <vector>

#include "common/math/box2d.h"
#include "common/math/cartesian_frenet_conversion.h"
#include "common/math/line_segment2d.h"
#include "common/math/math_utils.h"
#include "common/math/vec2d.h"
#include "common/proto/lattice_structure.pb.h"
#include "common/proto/pnc_point.pb.h"
#include "common/proto/sl_boundary.pb.h"
#include "discretized_trajectory.h"
#include "glog/logging.h"
#include "reference_point.h"

class InterpolatedIndex {
 public:
  InterpolatedIndex(int id, double offset) : id(id), offset(offset) {}
  int id = 0;
  double offset = 0.0;
};

class MapPath {
 public:
  MapPath() = default;
  explicit MapPath(const std::vector<MapPathPoint>& path_points);
  explicit MapPath(std::vector<MapPathPoint>&& path_points);

  double length() const { return length_; };
  const std::vector<MapPathPoint>& path_points() const { return path_points_; }
  const std::vector<double>& accumulated_s() const { return accumulated_s_; }
  int num_points() const { return num_points_; };
  bool GetLaneWidth(const double s, double* lane_left_width,
                    double* lane_right_width) const;
  InterpolatedIndex GetIndexFromS(double s) const;
  bool GetProjection(const Vec2d& point, double* accumulate_s,
                     double* lateral) const;
  bool GetProjection(const Vec2d& point, double* accumulate_s, double* lateral,
                     double* distance) const;
  bool OverlapWith(const Box2d& box, double width) const;

  // Return smooth coordinate by interpolated index or accumulate_s.
  MapPathPoint GetSmoothPoint(const InterpolatedIndex& index) const;
  MapPathPoint GetSmoothPoint(double s) const;

 protected:
  double GetSample(const std::vector<double>& samples, const double s) const;
  void InitPoints();
  void InitWidth();
  void InitPointIndex();
  int num_points_ = 0;
  int num_segments_ = 0;
  double length_ = 0.0;
  double kSampleDistance = 0.25;
  std::vector<MapPathPoint> path_points_;
  std::vector<LineSegment2d> segments_;
  std::vector<double> accumulated_s_;
  std::vector<Vec2d> unit_directions_;

  int num_sample_points_ = 0;
  std::vector<double> lane_left_width_;
  std::vector<double> lane_right_width_;
  std::vector<double> road_left_width_;
  std::vector<double> road_right_width_;
  std::vector<int> last_point_index_;
};

class ReferenceLine {
 public:
  ReferenceLine() = default;
  explicit ReferenceLine(const ReferenceLine& reference_line) = default;
  template <typename Iterator>
  ReferenceLine(const Iterator begin, const Iterator end)
      : reference_points_(begin, end),
        map_path_(std::move(std::vector<MapPathPoint>(begin, end))) {}
  explicit ReferenceLine(const std::vector<ReferencePoint>& reference_points);
  explicit ReferenceLine(const MapPath& hdmap_path);
  // explicit ReferenceLine(const hdmap::Path& hdmap_path);

  /** Stitch current reference line with the other reference line
   * The stitching strategy is to use current reference points as much as
   * possible. The following two examples show two successful stitch cases.
   *
   * Example 1
   * this:   |--------A-----x-----B------|
   * other:                 |-----C------x--------D-------|
   * Result: |------A-----x-----B------x--------D-------|
   * In the above example, A-B is current reference line, and C-D is the other
   * reference line. If part B and part C matches, we update current reference
   * line to A-B-D.
   *
   * Example 2
   * this:                  |-----A------x--------B-------|
   * other:  |--------C-----x-----D------|
   * Result: |--------C-----x-----A------x--------B-------|
   * In the above example, A-B is current reference line, and C-D is the other
   * reference line. If part A and part D matches, we update current reference
   * line to C-A-B.
   *
   * @return false if these two reference line cannot be stitched
   */
  bool Stitch(const ReferenceLine& other);

  bool Segment(const Vec2d& point, const double distance_backward,
               const double distance_forward);

  bool Segment(const double s, const double distance_backward,
               const double distance_forward);

  const MapPath& map_path() const;
  const std::vector<ReferencePoint>& reference_points() const {
    return reference_points_;
  };

  ReferencePoint GetReferencePoint(const double s) const;

  points::FrenetFramePoint GetFrenetPoint(
      const points::PathPoint& path_point) const;

  std::pair<std::array<double, 3>, std::array<double, 3>> ToFrenetFrame(
      const points::TrajectoryPoint& traj_point) const;

  std::vector<ReferencePoint> GetReferencePoints(double start_s,
                                                 double end_s) const;

  size_t GetNearestReferenceIndex(const double s) const;

  ReferencePoint GetNearestReferencePoint(const Vec2d& xy) const;

  bool GetLaneWidth(const double s, double* const lane_left_width,
                    double* const lane_right_width) const;

  ReferencePoint GetNearestReferencePoint(const double s) const;

  double GetDrivingWidth(const boundary::SLBoundary& sl_boundary) const;

  bool IsBlockRoad(const Box2d& box2d, double gap) const;
  bool IsOnLane(const boundary::SLBoundary& sl_boundary) const;

  bool XYToSL(const Vec2d& xy_point, points::SLPoint* const sl_point) const;
  template <class XYPoint>
  bool XYToSL(const XYPoint& xy, points::SLPoint* const sl_point) const {
    return XYToSL(Vec2d(xy.x(), xy.y()), sl_point);
  }

  double Length() const { return map_path_.length(); }

  double GetSpeedLimitFromS(const double s) const;

 private:
  static double FindMinDistancePoint(const ReferencePoint& p0, const double s0,
                                     const ReferencePoint& p1, const double s1,
                                     const double x, const double y);

  ReferencePoint InterpolateWithMatchedIndex(
      const ReferencePoint& p0, const double s0, const ReferencePoint& p1,
      const double s1, const InterpolatedIndex& index) const;

 private:
  struct SpeedLimit {
    double start_s = 0.0;
    double end_s = 0.0;
    double speed_limit = 0.0;  // unit m/s
    SpeedLimit() = default;
    SpeedLimit(double _start_s, double _end_s, double _speed_limit)
        : start_s(_start_s), end_s(_end_s), speed_limit(_speed_limit) {}
  };
  /**
   * This speed limit overrides the lane speed limit
   **/
  std::vector<SpeedLimit> speed_limit_;
  std::vector<ReferencePoint> reference_points_;
  MapPath map_path_;
  uint32_t priority_ = 0;
};

class ReferenceLineInfo {
 public:
  enum class LaneType { LeftForward, LeftReverse, RightForward, RightReverse };
  ReferenceLineInfo() = default;

  ReferenceLineInfo(const ReferenceLine& reference_line)
      : reference_line_(reference_line){};
  const ReferenceLine& reference_line() const { return reference_line_; };

  void set_is_on_reference_line() { is_on_reference_line_ = true; };

  void SetLatticeCruiseSpeed(double speed) {
    planning_target_.set_cruise_speed(speed);
  };

  const lattice_structure::PlanningTarget& planning_target() const {
    return planning_target_;
  };

  void SetTrajectory(const DiscretizedTrajectory& trajectory) {
    discretized_trajectory_ = trajectory;
  };

  void AddCost(double cost) { cost_ += cost; };
  void SetCost(double cost) { cost_ = cost; };
  double PriorityCost() const { return priority_cost_; };

  void SetDrivable(bool drivable) { is_drivable_ = drivable; };

  void SetPriorityCost(double cost) { priority_cost_ = cost; };

  const DiscretizedTrajectory& trajectory() const {
    return discretized_trajectory_;
  };

 private:
  ReferenceLine const reference_line_;
  bool is_on_reference_line_ = false;
  lattice_structure::PlanningTarget planning_target_;
  DiscretizedTrajectory discretized_trajectory_;
  double cost_, priority_cost_;
  bool is_drivable_ = true;
};

#endif
