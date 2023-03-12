#ifndef _COLLISION_CHECKER_H_
#define _COLLISION_CHECKER_H_

#include <memory>
#include <vector>

#include "common/math/box2d.h"
#include "discretized_trajectory.h"
#include "obstacle.h"
#include "planning/lattice/behavior/path_time_graph.h"

class CollisionChecker {
 public:
  CollisionChecker(
      const std::vector<const Obstacle*>& obstacles, const double ego_vehicle_s,
      const double ego_vehicle_d,
      const std::vector<points::PathPoint>& discretized_reference_line,
      const ReferenceLineInfo* ptr_reference_line_info,
      const std::shared_ptr<PathTimeGraph>& ptr_path_time_graph);

  bool InCollision(const DiscretizedTrajectory& discretized_trajectory);

  static bool InCollision(const std::vector<const Obstacle*>& obstacles,
                          const DiscretizedTrajectory& ego_trajectory,
                          const double ego_length, const double ego_width,
                          const double ego_edge_to_center);

 private:
  void BuildPredictedEnvironment(
      const std::vector<const Obstacle*>& obstacles, const double ego_vehicle_s,
      const double ego_vehicle_d,
      const std::vector<points::PathPoint>& discretized_reference_line);

  bool IsEgoVehicleInLane(const double ego_vehicle_s,
                          const double ego_vehicle_d);

  bool IsObstacleBehindEgoVehicle(
      const Obstacle* obstacle, const double ego_vehicle_s,
      const std::vector<points::PathPoint>& discretized_reference_line);

 private:
  const ReferenceLineInfo* ptr_reference_line_info_;
  std::shared_ptr<PathTimeGraph> ptr_path_time_graph_;
  std::vector<std::vector<Box2d>> predicted_bounding_rectangles_;
};

#endif
