#ifndef _REFERENCE_POINT_H_
#define _REFERENCE_POINT_H_

#include <string>
#include <vector>

#include "common/config/flags.h"
#include "common/math/vec2d.h"
#include "common/proto/pnc_point.pb.h"
#include "glog/logging.h"

class MapPathPoint : public Vec2d {
 public:
  MapPathPoint() = default;
  MapPathPoint(const Vec2d& point, double heading)
      : Vec2d(point.x(), point.y()), heading_(heading){};
  MapPathPoint(const Vec2d& point, double heading, MapPathPoint lane_waypoint)
      : Vec2d(point.x(), point.y()), heading_(heading) {
    lane_waypoints_.emplace_back(std::move(lane_waypoint));
  };
  MapPathPoint(const Vec2d& point, double heading,
               std::vector<MapPathPoint> lane_waypoints)
      : Vec2d(point.x(), point.y()),
        heading_(heading),
        lane_waypoints_(std::move(lane_waypoints)){};

  double heading() const { return heading_; };
  void set_heading(const double heading) { heading_ = heading; };

  const std::vector<MapPathPoint>& lane_waypoints() const {
    return lane_waypoints_;
  };

  void add_lane_waypoint(MapPathPoint lane_waypoint) {
    lane_waypoints_.emplace_back(std::move(lane_waypoint));
  };
  void add_lane_waypoints(const std::vector<MapPathPoint>& lane_waypoints) {
    lane_waypoints_.insert(lane_waypoints_.end(), lane_waypoints.begin(),
                           lane_waypoints.end());
  };

  void set_lane_left_width(double lane_left_width) {
    lane_left_width_ = lane_left_width;
  };
  void set_lane_right_width(double lane_right_width) {
    lane_right_width_ = lane_right_width;
  };

  const double get_lane_left_width() const { return lane_left_width_; };
  const double get_lane_right_width() const { return lane_right_width_; };

 protected:
  double heading_ = 0.0;
  std::vector<MapPathPoint> lane_waypoints_;
  double lane_left_width_ = 0.0;
  double lane_right_width_ = 0.0;
};

class ReferencePoint : public MapPathPoint {
 public:
  ReferencePoint() = default;

  ReferencePoint(const MapPathPoint& map_path_point, const double kappa,
                 const double dkappa)
      : MapPathPoint(map_path_point), kappa_(kappa), dkappa_(dkappa){};

  points::PathPoint ToPathPoint(double s) const;

  double kappa() const;
  double dkappa() const;

  std::string DebugString() const;

  static void RemoveDuplicates(std::vector<ReferencePoint>* points);

 private:
  double kappa_ = 0.0;
  double dkappa_ = 0.0;
};

#endif
