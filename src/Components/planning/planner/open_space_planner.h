#pragma once

#include "common/proto/planning.pb.h"
#include "planning/common/iterative_anchoring_smoother.h"
#include "planning/open_space/hybrid_a_star.h"

class OpenSpacePlanner {
 public:
  static bool Plan(double sx, double sy, double sphi, double ex, double ey,
                   double ephi, std::vector<double> XYbounds,
                   std::vector<std::vector<Vec2d>> obstacles,
                   std::vector<planning::ADCTrajectory>* trajectory);

 private:
  static void LoadHybridAStarResultInEigen(HybridAStarResult* result,
                                           Eigen::MatrixXd* xWS,
                                           Eigen::MatrixXd* uWS);
  static void LoadTrajectory(const Eigen::MatrixXd& state_result,
                             const Eigen::MatrixXd& control_result,
                             const Eigen::MatrixXd& time_result,
                             planning::ADCTrajectory* trajectoy);
  static void LoadResult(const DiscretizedTrajectory& discretized_trajectory,
                         Eigen::MatrixXd* state_result_dc,
                         Eigen::MatrixXd* control_result_dc,
                         Eigen::MatrixXd* time_result_dc);
};
