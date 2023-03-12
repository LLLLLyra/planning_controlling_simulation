#ifndef _TRAJECTORY_EVALUATOR_H_
#define _TRAJECTORY_EVALUATOR_H_

#include <functional>
#include <memory>
#include <queue>
#include <utility>
#include <vector>

#include "common/math/curve1d.h"
#include "common/proto/lattice_structure.pb.h"
#include "planning/lattice/behavior/path_time_graph.h"

class TrajectoryEvaluator {
  // normal use
  typedef std::pair<
      std::pair<std::shared_ptr<Curve1d>, std::shared_ptr<Curve1d>>, double>
      PairCost;

  // auto tuning
  typedef std::pair<
      std::pair<std::shared_ptr<Curve1d>, std::shared_ptr<Curve1d>>,
      std::pair<std::vector<double>, double>>
      PairCostWithComponents;

 public:
  TrajectoryEvaluator(
      const std::array<double, 3>& init_s,
      const lattice_structure::PlanningTarget& planning_target,
      const std::vector<std::shared_ptr<Curve1d>>& lon_trajectories,
      const std::vector<std::shared_ptr<Curve1d>>& lat_trajectories,
      std::shared_ptr<PathTimeGraph> path_time_graph,
      std::shared_ptr<std::vector<points::PathPoint>> reference_line);

  virtual ~TrajectoryEvaluator() = default;

  bool has_more_trajectory_pairs() const;

  size_t num_of_trajectory_pairs() const;

  std::pair<std::shared_ptr<Curve1d>, std::shared_ptr<Curve1d>>
  next_top_trajectory_pair();

  double top_trajectory_pair_cost() const;

  std::vector<double> top_trajectory_pair_component_cost() const;

 private:
  double Evaluate(const lattice_structure::PlanningTarget& planning_target,
                  const std::shared_ptr<Curve1d>& lon_trajectory,
                  const std::shared_ptr<Curve1d>& lat_trajectory,
                  std::vector<double>* cost_components = nullptr) const;

  double LatOffsetCost(const std::shared_ptr<Curve1d>& lat_trajectory,
                       const std::vector<double>& s_values) const;

  double LatComfortCost(const std::shared_ptr<Curve1d>& lon_trajectory,
                        const std::shared_ptr<Curve1d>& lat_trajectory) const;

  double LonComfortCost(const std::shared_ptr<Curve1d>& lon_trajectory) const;

  double LonCollisionCost(const std::shared_ptr<Curve1d>& lon_trajectory) const;

  double LonObjectiveCost(
      const std::shared_ptr<Curve1d>& lon_trajectory,
      const lattice_structure::PlanningTarget& planning_target,
      const std::vector<double>& ref_s_dot) const;

  double CentripetalAccelerationCost(
      const std::shared_ptr<Curve1d>& lon_trajectory) const;

  std::vector<double> ComputeLongitudinalGuideVelocity(
      const lattice_structure::PlanningTarget& planning_target) const;

  bool InterpolateDenseStPoints(
      const std::vector<points::SpeedPoint>& st_points, double t,
      double* traj_s) const;

  struct CostComparator
      : public std::binary_function<const PairCost&, const PairCost&, bool> {
    bool operator()(const PairCost& left, const PairCost& right) const {
      return left.second > right.second;
    }
  };

  std::priority_queue<PairCost, std::vector<PairCost>, CostComparator>
      cost_queue_;

  std::shared_ptr<PathTimeGraph> path_time_graph_;

  std::shared_ptr<std::vector<points::PathPoint>> reference_line_;

  std::vector<std::vector<std::pair<double, double>>> path_time_intervals_;

  std::array<double, 3> init_s_;

  std::vector<double> reference_s_dot_;
};

#endif