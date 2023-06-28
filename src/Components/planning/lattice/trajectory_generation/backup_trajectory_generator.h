#ifndef _BACKUP_TRAJECTORY_GENERATOR_H_
#define _BACKUP_TRAJECTORY_GENERATOR_H_

#include <functional>
#include <memory>
#include <queue>
#include <utility>
#include <vector>

#include "common/math/curve1d.h"
#include "planning/common/collision_checker.h"
#include "planning/common/constant_deceleration_trajectory1d.h"
#include "planning/common/discretized_trajectory.h"
#include "trajectory1d_generator.h"

class BackupTrajectoryGenerator {
 public:
  typedef std::pair<std::shared_ptr<Curve1d>, std::shared_ptr<Curve1d>>
      Trajectory1dPair;
  typedef std::pair<Trajectory1dPair, double> PairCost;

  BackupTrajectoryGenerator(
      const std::array<double, 3>& init_s, const std::array<double, 3>& init_d,
      const double init_relative_time,
      const std::shared_ptr<CollisionChecker>& ptr_collision_checker,
      const Trajectory1dGenerator* trajectory1d_generator);

  DiscretizedTrajectory GenerateTrajectory(
      const std::vector<points::PathPoint>& discretized_ref_points);

 private:
  void GenerateTrajectory1dPairs(const std::array<double, 3>& init_s,
                                 const std::array<double, 3>& init_d);

  double init_relative_time_;

  std::shared_ptr<CollisionChecker> ptr_collision_checker_;

  const Trajectory1dGenerator* ptr_trajectory1d_generator_;

  struct CostComparator
      : public std::binary_function<const Trajectory1dPair&,
                                    const Trajectory1dPair&, bool> {
    bool operator()(const Trajectory1dPair& left,
                    const Trajectory1dPair& right) const {
      auto lon_left = left.first;
      auto lon_right = right.first;
      auto s_dot_left = lon_left->Evaluate(1, FLAGS_trajectory_time_length);
      auto s_dot_right = lon_right->Evaluate(1, FLAGS_trajectory_time_length);
      if (s_dot_left < s_dot_right) {
        return true;
      }
      return false;
    }
  };

  std::priority_queue<Trajectory1dPair, std::vector<Trajectory1dPair>,
                      CostComparator>
      trajectory_pair_pqueue_;
};

#endif
