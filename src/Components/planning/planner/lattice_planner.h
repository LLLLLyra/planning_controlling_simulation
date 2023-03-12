#ifndef _LATTICE_PLANNER_H_
#define _LATTICE_PLANNER_H_

#include <vector>

#include "common/math/path_matcher.h"
#include "common/proto/lattice_structure.pb.h"
#include "common/proto/planning.pb.h"
#include "common/proto/pnc_point.pb.h"
#include "glog/logging.h"
#include "planning/common/collision_checker.h"
#include "planning/common/discretized_trajectory.h"
#include "planning/common/obstacle.h"
#include "planning/common/reference_line.h"
#include "planning/lattice/behavior/path_time_graph.h"
#include "planning/lattice/behavior/prediction_querier.h"
#include "planning/lattice/trajectory_generation/backup_trajectory_generator.h"
#include "planning/lattice/trajectory_generation/trajectory1d_generator.h"
#include "planning/lattice/trajectory_generation/trajectory_combiner.h"
#include "planning/lattice/trajectory_generation/trajectory_evaluator.h"

class ConstraintChecker {
 public:
  enum class Result {
    VALID,
    LON_VELOCITY_OUT_OF_BOUND,
    LON_ACCELERATION_OUT_OF_BOUND,
    LON_JERK_OUT_OF_BOUND,
    LAT_VELOCITY_OUT_OF_BOUND,
    LAT_ACCELERATION_OUT_OF_BOUND,
    LAT_JERK_OUT_OF_BOUND,
    CURVATURE_OUT_OF_BOUND,
  };
  ConstraintChecker() = delete;
  static Result ValidTrajectory(const DiscretizedTrajectory& trajectory);
};

class LatticePlanner {
 public:
  LatticePlanner() = default;
  LatticePlanner(const std::vector<const Obstacle*>& obstacles,
                 std::vector<ReferenceLineInfo>& reference_line_infos);
  bool Plan(const points::TrajectoryPoint& planning_init_point);
  std::vector<DiscretizedTrajectory> GetTrajectory();
  static void AppendTrajectory(DiscretizedTrajectory& trajectory,
                               planning::ADCTrajectory* pub_trajectory);

 private:
  const std::vector<const Obstacle*> obstacles_;
  std::vector<ReferenceLineInfo> reference_line_infos_;
  std::vector<DiscretizedTrajectory> trajectories_;
  bool PlanOnReferenceLine(const points::TrajectoryPoint& planning_init_point,
                           ReferenceLineInfo* reference_line_info);
};

#endif