#ifndef _HYBRID_A_STAR_H_
#define _HYBRID_A_STAR_H_

#include <algorithm>
#include <memory>
#include <queue>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "common/math/discretized_path.h"
#include "common/math/speed_data.h"
#include "common/proto/pnc_point.pb.h"
#include "grid_search.h"
#include "node3d.h"
#include "reeds_shepp_path.h"

struct HybridAStarResult {
  std::vector<double> x;
  std::vector<double> y;
  std::vector<double> phi;
  std::vector<double> v;
  std::vector<double> a;
  std::vector<double> steer;
  std::vector<double> accumulated_s;
};

class HybridAStar {
 public:
  explicit HybridAStar(const planning::PlannerOpenSpaceConfig& open_space_conf);
  virtual ~HybridAStar() = default;
  bool Plan(double sx, double sy, double sphi, double ex, double ey,
            double ephi, const std::vector<double>& XYbounds,
            const std::vector<std::vector<Vec2d>>& obstacles_vertices_vec,
            HybridAStarResult* result);
  bool TrajectoryPartition(const HybridAStarResult& result,
                           std::vector<HybridAStarResult>* partitioned_result);

 private:
  bool AnalyticExpansion(std::shared_ptr<Node3d> current_node);
  // check collision and validity
  bool ValidityCheck(std::shared_ptr<Node3d> node);
  // check Reeds Shepp path collision and validity
  bool RSPCheck(const std::shared_ptr<ReedSheppPath> reeds_shepp_to_end);
  // load the whole RSP as nodes and add to the close set
  std::shared_ptr<Node3d> LoadRSPinCS(
      const std::shared_ptr<ReedSheppPath> reeds_shepp_to_end,
      std::shared_ptr<Node3d> current_node);
  std::shared_ptr<Node3d> Next_node_generator(
      std::shared_ptr<Node3d> current_node, size_t next_node_index);
  void CalculateNodeCost(std::shared_ptr<Node3d> current_node,
                         std::shared_ptr<Node3d> next_node);
  double TrajCost(std::shared_ptr<Node3d> current_node,
                  std::shared_ptr<Node3d> next_node);
  double HoloObstacleHeuristic(std::shared_ptr<Node3d> next_node);
  bool GetResult(HybridAStarResult* result);
  bool GetTemporalProfile(HybridAStarResult* result);
  bool GenerateSpeedAcceleration(HybridAStarResult* result);
  bool GenerateSCurveSpeedAcceleration(HybridAStarResult* result);

 private:
  planning::PlannerOpenSpaceConfig planner_open_space_config_;
  vehicle::VehicleParam vehicle_param_ = vehicle::VehicleParam();
  size_t next_node_num_ = 0;
  double max_steer_angle_ = 0.0;
  double step_size_ = 0.0;
  double xy_grid_resolution_ = 0.0;
  double delta_t_ = 0.0;
  double traj_forward_penalty_ = 0.0;
  double traj_back_penalty_ = 0.0;
  double traj_gear_switch_penalty_ = 0.0;
  double traj_steer_penalty_ = 0.0;
  double traj_steer_change_penalty_ = 0.0;
  double heu_rs_forward_penalty_ = 0.0;
  double heu_rs_back_penalty_ = 0.0;
  double heu_rs_gear_switch_penalty_ = 0.0;
  double heu_rs_steer_penalty_ = 0.0;
  double heu_rs_steer_change_penalty_ = 0.0;
  std::vector<double> XYbounds_;
  std::shared_ptr<Node3d> start_node_;
  std::shared_ptr<Node3d> end_node_;
  std::shared_ptr<Node3d> final_node_;
  std::vector<std::vector<LineSegment2d>> obstacles_linesegments_vec_;

  struct cmp {
    bool operator()(const std::pair<std::string, double>& left,
                    const std::pair<std::string, double>& right) const {
      return left.second >= right.second;
    }
  };
  std::priority_queue<std::pair<std::string, double>,
                      std::vector<std::pair<std::string, double>>, cmp>
      open_pq_;
  std::unordered_map<std::string, std::shared_ptr<Node3d>> open_set_;
  std::unordered_map<std::string, std::shared_ptr<Node3d>> close_set_;
  std::unique_ptr<ReedShepp> reed_shepp_generator_;
  std::unique_ptr<GridSearch> grid_a_star_heuristic_generator_;
};

#endif  // _HYBRID_A_STAR_H_