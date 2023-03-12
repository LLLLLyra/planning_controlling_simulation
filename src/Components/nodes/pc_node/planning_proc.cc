#include "planning_proc.h"

#include "deciders.h"

void PlanningProc::LatticeInit(const std::string& file_name) {
  std::vector<ReferenceLineInfo> rf_info;
  const std::string path = FLAGS_path_points_file_path + file_name + ".txt";
  Planning::GetReferenceLine(path, &rf_info);
  const std::vector<const Obstacle*> obstacles;
  lattice_.reset(new LatticePlanner(obstacles, rf_info));
  show_->ReferencePathToShow(rf_info[0].reference_line());
}

void PlanningProc::OpenSpaceInit(ObstacleInfo& obstacles) {
  Deciders::OpenSpaceROIDecider(obstacles, &obstacles_, &XYbounds_, &end_);
  show_->ObstaclePointToShow(obstacles_);
}

bool PlanningProc::LatticePlan(points::TrajectoryPoint& start_point,
                               planning::ADCTrajectory* trajectory) {
  CHECK_NOTNULL(trajectory);
  trajectory->mutable_trajectory_point()->Clear();
  start_point.set_relative_time(0);

  if (!lattice_->Plan(start_point)) {
    return false;
  }
  auto traj = lattice_->GetTrajectory()[0];
  lattice_->AppendTrajectory(traj, trajectory);
  show_->TrajectoryToShow(*trajectory);
  return true;
}

bool PlanningProc::OpenSpacePlan(
    std::array<double, 3>& start,
    std::vector<planning::ADCTrajectory>* trajectories) {
  CHECK_NOTNULL(trajectories);
  trajectories->clear();
  auto& [sx, sy, sphi] = start;
  auto& [ex, ey, ephi] = end_;
  if (!OpenSpacePlanner::Plan(sx, sy, sphi, ex, ey, ephi, XYbounds_, obstacles_,
                              trajectories)) {
    return false;
  }
  for (auto& trajectory : *trajectories) {
    trajectory.mutable_header()->set_sequence_num(cnt_++);
    show_->TrajectoryToShow(trajectory);
  }
  return true;
}

bool PlanningProc::CruisePlan(const std::string& file_name,
                              planning::ADCTrajectory* trajectory) {
  CHECK_NOTNULL(trajectory);
  trajectory->mutable_trajectory_point()->Clear();
  if (!Planning::PublishTrajectory(trajectory, file_name)) {
    return false;
  }
  trajectory->mutable_header()->set_sequence_num(cnt_++);
  show_->TrajectoryToShow(*trajectory);
  return true;
}

void PlanningProc::InitShow(Show* show) { show_ = show; }