#include "lattice_planner.h"

#include <ctime>

#include "common/config/flags.h"

namespace {
std::vector<points::PathPoint> ToDiscretizedReferenceLine(
    const std::vector<ReferencePoint>& ref_points) {
  double s = 0.0;
  std::vector<points::PathPoint> path_points;
  for (const auto& ref_point : ref_points) {
    points::PathPoint path_point;
    path_point.set_x(ref_point.x());
    path_point.set_y(ref_point.y());
    path_point.set_theta(ref_point.heading());
    path_point.set_kappa(ref_point.kappa());
    path_point.set_dkappa(ref_point.dkappa());

    if (!path_points.empty()) {
      double dx = path_point.x() - path_points.back().x();
      double dy = path_point.y() - path_points.back().y();
      s += std::sqrt(dx * dx + dy * dy);
    }
    path_point.set_s(s);
    path_points.push_back(std::move(path_point));
  }
  return path_points;
}

void ComputeInitFrenetState(const points::PathPoint& matched_point,
                            const points::TrajectoryPoint& cartesian_state,
                            std::array<double, 3>* ptr_s,
                            std::array<double, 3>* ptr_d) {
  CartesianFrenetConverter::cartesian_to_frenet(
      matched_point.s(), matched_point.x(), matched_point.y(),
      matched_point.theta(), matched_point.kappa(), matched_point.dkappa(),
      cartesian_state.path_point().x(), cartesian_state.path_point().y(),
      cartesian_state.v(), cartesian_state.a(),
      cartesian_state.path_point().theta(),
      cartesian_state.path_point().kappa(), ptr_s, ptr_d);
}

template <typename T>
bool WithinRange(const T v, const T lower, const T upper) {
  return lower <= v && v <= upper;
}
}  // namespace

ConstraintChecker::Result ConstraintChecker::ValidTrajectory(
    const DiscretizedTrajectory& trajectory) {
  const double kMaxCheckRelativeTime = FLAGS_trajectory_time_length;
  for (const auto& p : trajectory) {
    double t = p.relative_time();
    if (t > kMaxCheckRelativeTime) {
      break;
    }
    double lon_v = p.v();
    if (!WithinRange(lon_v, FLAGS_speed_lower_bound, FLAGS_speed_upper_bound)) {
      LOG(INFO) << "Velocity at relative time " << t
                << " exceeds bound, value: " << lon_v << ", bound ["
                << FLAGS_speed_lower_bound << ", " << FLAGS_speed_upper_bound
                << "].";
      return Result::LON_VELOCITY_OUT_OF_BOUND;
    }

    double lon_a = p.a();
    if (!WithinRange(lon_a, FLAGS_longitudinal_acceleration_lower_bound,
                     FLAGS_longitudinal_acceleration_upper_bound)) {
      LOG(INFO) << "Longitudinal acceleration at relative time " << t
                << " exceeds bound, value: " << lon_a << ", bound ["
                << FLAGS_longitudinal_acceleration_lower_bound << ", "
                << FLAGS_longitudinal_acceleration_upper_bound << "].";
      return Result::LON_ACCELERATION_OUT_OF_BOUND;
    }

    double kappa = p.path_point().kappa();
    if (!WithinRange(kappa, -FLAGS_kappa_bound, FLAGS_kappa_bound)) {
      LOG(INFO) << "Kappa at relative time " << t
                << " exceeds bound, value: " << kappa << ", bound ["
                << -FLAGS_kappa_bound << ", " << FLAGS_kappa_bound << "].";
      return Result::CURVATURE_OUT_OF_BOUND;
    }
  }

  for (size_t i = 1; i < trajectory.NumOfPoints(); ++i) {
    const auto& p0 = trajectory.TrajectoryPointAt(static_cast<uint32_t>(i - 1));
    const auto& p1 = trajectory.TrajectoryPointAt(static_cast<uint32_t>(i));

    if (p1.relative_time() > kMaxCheckRelativeTime) {
      break;
    }

    double t = p0.relative_time();

    double dt = p1.relative_time() - p0.relative_time();
    double d_lon_a = p1.a() - p0.a();
    double lon_jerk = d_lon_a / dt;
    if (!WithinRange(lon_jerk, FLAGS_longitudinal_jerk_lower_bound,
                     FLAGS_longitudinal_jerk_upper_bound)) {
      LOG(INFO) << "Longitudinal jerk at relative time " << t
                << " exceeds bound, value: " << lon_jerk << ", bound ["
                << FLAGS_longitudinal_jerk_lower_bound << ", "
                << FLAGS_longitudinal_jerk_upper_bound << "].";
      return Result::LON_JERK_OUT_OF_BOUND;
    }

    double lat_a = p1.v() * p1.v() * p1.path_point().kappa();
    if (!WithinRange(lat_a, -FLAGS_lateral_acceleration_bound,
                     FLAGS_lateral_acceleration_bound)) {
      LOG(INFO) << "Lateral acceleration at relative time " << t
                << " exceeds bound, value: " << lat_a << ", bound ["
                << -FLAGS_lateral_acceleration_bound << ", "
                << FLAGS_lateral_acceleration_bound << "].";
      return Result::LAT_ACCELERATION_OUT_OF_BOUND;
    }

    // TODO(zhangyajia): this is temporarily disabled
    // due to low quality reference line.
    /**
    double d_lat_a = p1.v() * p1.v() * p1.path_point().kappa() -
                     p0.v() * p0.v() * p0.path_point().kappa();
    double lat_jerk = d_lat_a / dt;
    if (!WithinRange(lat_jerk, -FLAGS_lateral_jerk_bound,
                     FLAGS_lateral_jerk_bound)) {
      LOG(INFO) << "Lateral jerk at relative time " << t
             << " exceeds bound, value: " << lat_jerk << ", bound ["
             << -FLAGS_lateral_jerk_bound << ", " << FLAGS_lateral_jerk_bound
             << "].";
      return Result::LAT_JERK_OUT_OF_BOUND;
    }
    **/
  }

  return Result::VALID;
};

LatticePlanner::LatticePlanner(
    const std::vector<const Obstacle*>& obstacles,
    std::vector<ReferenceLineInfo>& reference_line_infos)
    : obstacles_(obstacles), reference_line_infos_(reference_line_infos) {}

bool LatticePlanner::PlanOnReferenceLine(
    const points::TrajectoryPoint& planning_init_point,
    ReferenceLineInfo* reference_line_info) {
  static size_t num_planning_cycles = 0;
  static size_t num_planning_succeeded_cycles = 0;

  clock_t start_time = std::clock();
  clock_t current_time = start_time;

  LOG(INFO) << "Number of planning cycles: " << num_planning_cycles << " "
            << num_planning_succeeded_cycles;
  ++num_planning_cycles;

  reference_line_info->set_is_on_reference_line();
  // 1. obtain a reference line and transform it to the PathPoint format.
  auto ptr_reference_line = std::make_shared<std::vector<points::PathPoint>>(
      ToDiscretizedReferenceLine(
          reference_line_info->reference_line().reference_points()));

  // 2. compute the matched point of the init planning point on the reference
  // line.
  points::PathPoint matched_point = PathMatcher::MatchToPath(
      *ptr_reference_line, planning_init_point.path_point().x(),
      planning_init_point.path_point().y());

  // 3. according to the matched point, compute the init state in Frenet frame.
  std::array<double, 3> init_s;
  std::array<double, 3> init_d;
  ComputeInitFrenetState(matched_point, planning_init_point, &init_s, &init_d);

  LOG(INFO) << "ReferenceLine and Frenet Conversion Time = "
            << ((double)(std::clock() - current_time) / CLOCKS_PER_SEC) * 1000;
  current_time = std::clock();
  auto ptr_prediction_querier =
      std::make_shared<PredictionQuerier>(obstacles_, ptr_reference_line);

  // 4. parse the decision and get the planning target.
  auto ptr_path_time_graph = std::make_shared<PathTimeGraph>(
      ptr_prediction_querier->GetObstacles(), *ptr_reference_line,
      reference_line_info, init_s[0],
      init_s[0] + FLAGS_speed_lon_decision_horizon, 0.0,
      FLAGS_trajectory_time_length, init_d);

  double speed_limit =
      reference_line_info->reference_line().GetSpeedLimitFromS(init_s[0]);
  reference_line_info->SetLatticeCruiseSpeed(speed_limit);

  lattice_structure::PlanningTarget planning_target =
      reference_line_info->planning_target();
  if (planning_target.has_stop_point()) {
    LOG(INFO) << "Planning target stop s: " << planning_target.stop_point().s()
              << "Current ego s: " << init_s[0];
  }

  LOG(INFO) << "Decision_Time = "
            << ((double)(std::clock() - current_time) / CLOCKS_PER_SEC) * 1000;
  current_time = std::clock();

  // 5. generate 1d trajectory bundle for longitudinal and lateral respectively.
  Trajectory1dGenerator trajectory1d_generator(
      init_s, init_d, ptr_path_time_graph, ptr_prediction_querier);
  std::vector<std::shared_ptr<Curve1d>> lon_trajectory1d_bundle;
  std::vector<std::shared_ptr<Curve1d>> lat_trajectory1d_bundle;
  trajectory1d_generator.GenerateTrajectoryBundles(
      planning_target, &lon_trajectory1d_bundle, &lat_trajectory1d_bundle);

  LOG(INFO) << "Trajectory_Generation_Time = "
            << ((double)(std::clock() - current_time) / CLOCKS_PER_SEC) * 1000;
  current_time = std::clock();

  // 6. first, evaluate the feasibility of the 1d trajectories according to
  // dynamic constraints.
  //   second, evaluate the feasible longitudinal and lateral trajectory pairs
  //   and sort them according to the cost.
  TrajectoryEvaluator trajectory_evaluator(
      init_s, planning_target, lon_trajectory1d_bundle, lat_trajectory1d_bundle,
      ptr_path_time_graph, ptr_reference_line);

  LOG(INFO) << "Trajectory_Evaluator_Construction_Time = "
            << ((double)(std::clock() - current_time) / CLOCKS_PER_SEC) * 1000;
  current_time = std::clock();

  LOG(INFO) << "number of trajectory pairs = "
            << trajectory_evaluator.num_of_trajectory_pairs()
            << "  number_lon_traj = " << lon_trajectory1d_bundle.size()
            << "  number_lat_traj = " << lat_trajectory1d_bundle.size();

  // Get instance of collision checker and constraint checker
  CollisionChecker collision_checker(obstacles_, init_s[0], init_d[0],
                                     *ptr_reference_line, reference_line_info,
                                     ptr_path_time_graph);

  // 7. always get the best pair of trajectories to combine; return the first
  // collision-free trajectory.
  size_t constraint_failure_count = 0;
  size_t collision_failure_count = 0;
  size_t combined_constraint_failure_count = 0;

  size_t lon_vel_failure_count = 0;
  size_t lon_acc_failure_count = 0;
  size_t lon_jerk_failure_count = 0;
  size_t curvature_failure_count = 0;
  size_t lat_acc_failure_count = 0;
  size_t lat_jerk_failure_count = 0;

  size_t num_lattice_traj = 0;

  while (trajectory_evaluator.has_more_trajectory_pairs()) {
    double trajectory_pair_cost =
        trajectory_evaluator.top_trajectory_pair_cost();
    auto trajectory_pair = trajectory_evaluator.next_top_trajectory_pair();

    // combine two 1d trajectories to one 2d trajectory
    auto combined_trajectory = TrajectoryCombiner::Combine(
        *ptr_reference_line, *trajectory_pair.first, *trajectory_pair.second,
        planning_init_point.relative_time());

    // check longitudinal and lateral acceleration
    // considering trajectory curvatures
    auto result = ConstraintChecker::ValidTrajectory(combined_trajectory);
    if (result != ConstraintChecker::Result::VALID) {
      ++combined_constraint_failure_count;

      switch (result) {
        case ConstraintChecker::Result::LON_VELOCITY_OUT_OF_BOUND:
          lon_vel_failure_count += 1;
          break;
        case ConstraintChecker::Result::LON_ACCELERATION_OUT_OF_BOUND:
          lon_acc_failure_count += 1;
          break;
        case ConstraintChecker::Result::LON_JERK_OUT_OF_BOUND:
          lon_jerk_failure_count += 1;
          break;
        case ConstraintChecker::Result::CURVATURE_OUT_OF_BOUND:
          curvature_failure_count += 1;
          break;
        case ConstraintChecker::Result::LAT_ACCELERATION_OUT_OF_BOUND:
          lat_acc_failure_count += 1;
          break;
        case ConstraintChecker::Result::LAT_JERK_OUT_OF_BOUND:
          lat_jerk_failure_count += 1;
          break;
        case ConstraintChecker::Result::VALID:
        default:
          // Intentional empty
          break;
      }
      continue;
    }

    // check collision with other obstacles
    if (collision_checker.InCollision(combined_trajectory)) {
      ++collision_failure_count;
      continue;
    }

    // put combine trajectory into debug data
    const auto& combined_trajectory_points = combined_trajectory;
    num_lattice_traj += 1;
    reference_line_info->SetTrajectory(combined_trajectory);
    reference_line_info->SetCost(reference_line_info->PriorityCost() +
                                 trajectory_pair_cost);
    reference_line_info->SetDrivable(true);

    // Print the chosen end condition and start condition
    LOG(INFO) << "Starting Lon. State: s = " << init_s[0]
              << " ds = " << init_s[1] << " dds = " << init_s[2];
    // cast
    auto lattice_traj_ptr =
        std::dynamic_pointer_cast<LatticeTrajectory1d>(trajectory_pair.first);
    if (!lattice_traj_ptr) {
      LOG(INFO) << "Dynamically casting trajectory1d ptr. failed.";
    }

    if (lattice_traj_ptr->has_target_position()) {
      LOG(INFO) << "Ending Lon. State s = "
                << lattice_traj_ptr->target_position()
                << " ds = " << lattice_traj_ptr->target_velocity()
                << " t = " << lattice_traj_ptr->target_time();
    }

    LOG(INFO) << "InputPose";
    LOG(INFO) << "XY: " << planning_init_point.ShortDebugString();
    LOG(INFO) << "S: (" << init_s[0] << ", " << init_s[1] << "," << init_s[2]
              << ")";
    LOG(INFO) << "L: (" << init_d[0] << ", " << init_d[1] << "," << init_d[2]
              << ")";

    LOG(INFO) << "Reference_line_priority_cost = "
              << reference_line_info->PriorityCost();
    LOG(INFO) << "Total_Trajectory_Cost = " << trajectory_pair_cost;
    LOG(INFO) << "OutputTrajectory";
    for (uint i = 0; i < combined_trajectory_points.size(); ++i) {
      LOG(INFO) << combined_trajectory_points[i].ShortDebugString();
    }

    break;
    /*
    auto combined_trajectory_path =
        ptr_debug->mutable_planning_data()->add_trajectory_path();
    for (uint i = 0; i < combined_trajectory_points.size(); ++i) {
      combined_trajectory_path->add_trajectory_point()->CopyFrom(
          combined_trajectory_points[i]);
    }
    combined_trajectory_path->set_lattice_trajectory_cost(trajectory_pair_cost);
    */
  }

  LOG(INFO) << "Trajectory_Evaluation_Time = "
            << ((double)(std::clock() - current_time) / CLOCKS_PER_SEC) * 1000;

  LOG(INFO) << "Step CombineTrajectory Succeeded";

  LOG(INFO) << "1d trajectory not valid for constraint ["
            << constraint_failure_count << "] times";
  LOG(INFO) << "Combined trajectory not valid for ["
            << combined_constraint_failure_count << "] times";
  LOG(INFO) << "Trajectory not valid for collision [" << collision_failure_count
            << "] times";
  LOG(INFO) << "Total_Lattice_Planning_Frame_Time = "
            << ((double)(std::clock() - current_time) / CLOCKS_PER_SEC) * 1000;

  if (num_lattice_traj > 0) {
    LOG(INFO) << "Planning succeeded";
    num_planning_succeeded_cycles += 1;
    reference_line_info->SetDrivable(true);
    return true;
  } else {
    LOG(ERROR) << "Planning failed";
    if (FLAGS_enable_backup_trajectory) {
      LOG(ERROR) << "Use backup trajectory";
      BackupTrajectoryGenerator backup_trajectory_generator(
          init_s, init_d, planning_init_point.relative_time(),
          std::make_shared<CollisionChecker>(collision_checker),
          &trajectory1d_generator);
      DiscretizedTrajectory trajectory =
          backup_trajectory_generator.GenerateTrajectory(*ptr_reference_line);

      reference_line_info->AddCost(FLAGS_backup_trajectory_cost);
      reference_line_info->SetTrajectory(trajectory);
      reference_line_info->SetDrivable(true);
      return true;

    } else {
      reference_line_info->SetCost(std::numeric_limits<double>::infinity());
    }
    LOG(ERROR) << "No feasible trajectories";
    return false;
  }
}

bool LatticePlanner::Plan(const points::TrajectoryPoint& planning_init_point) {
  size_t success_line_count = 0;
  size_t index = 0;
  trajectories_.reserve(reference_line_infos_.size());
  trajectories_.clear();
  for (auto& reference_line_info : reference_line_infos_) {
    if (index != 0) {
      reference_line_info.SetPriorityCost(
          FLAGS_cost_non_priority_reference_line);
    } else {
      reference_line_info.SetPriorityCost(0.0);
    }
    bool status =
        PlanOnReferenceLine(planning_init_point, &reference_line_info);

    if (!status) {
      LOG(ERROR) << "Planner fail at the " << index << "segment";
    } else {
      trajectories_.emplace_back(reference_line_info.trajectory());
      ++success_line_count;
    }
    ++index;
  }
  return success_line_count > 0;
}

std::vector<DiscretizedTrajectory> LatticePlanner::GetTrajectory() {
  return trajectories_;
}

void LatticePlanner::AppendTrajectory(DiscretizedTrajectory& trajectory,
                                      planning::ADCTrajectory* pub_trajectory) {
  pub_trajectory->mutable_header()->set_sequence_num(
      pub_trajectory->header().sequence_num() + 1);
  double last_s;
  last_s =
      pub_trajectory->trajectory_point().empty()
          ? 0.0
          : (pub_trajectory->trajectory_point().end() - 1)->path_point().s();
  for (auto& t : trajectory) {
    auto p = pub_trajectory->add_trajectory_point();
    t.mutable_path_point()->set_s(last_s + t.path_point().s());
    p->CopyFrom(t);
  }
}