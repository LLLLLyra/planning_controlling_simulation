#include "pub_trajectory.h"

template <typename ResContrainer>
void Planning::WriteKineticResult(const std::string file_name,
                                  ResContrainer& result) {
  std::fstream f;
  f.open(file_name, std::fstream::out);
  f << std::fixed;

  // write header
  f << "x,"
    << "y,"
    << "phi,"
    << "t,"
    << "v,"
    << "a,"
    << "s" << std::endl;

  result.a.push_back(0);
  double t = 0.0;
  for (size_t i = 0; i < result.x.size(); ++i) {
    f << result.x[i] << ',' << result.y[i] << ',' << result.phi[i] << ',' << t
      << ',' << result.v[i] << ',' << result.a[i] << ','
      << result.accumulated_s[i] << std::endl;
    t += FLAGS_delta_t;
  }

  f.close();
}

void Planning::GetCruiseTrajectory(bool to_csv, KineticResult* result,
                                   const std::string& file_name) {
  const std::string path_file =
      FLAGS_path_points_file_path + file_name + ".txt";
  ;

  vehicle::VehicleParam vehicle_param;
  CruiseTrajectory cruise(vehicle_param);

  auto ref_points = cruise.get_ref_points();

  CHECK(cruise.LoadPathPoints(path_file, FLAGS_enable_smooth, &ref_points))
      << "fail to load path points";

  CHECK(cruise.GenerateTrajectory(ref_points));

  auto kinetic_result = cruise.get_kinetic_result();

  if (to_csv) {
    WriteKineticResult(FlAGS_trajectory_points_output_path, kinetic_result);
  }

  *result = kinetic_result;
}

template <typename ResContainer>
bool Planning::PublishTrajectory(planning::ADCTrajectory* pub_trajectory,
                                 const ResContainer& k_r) {
  std::vector<std::pair<double, double>> xypoints;
  std::vector<double> time;
  std::vector<double> headings;
  std::vector<double> accumulated_s;
  std::vector<double> kappas;
  std::vector<double> dkappas;
  std::vector<double> vs;
  std::vector<double> as;

  pub_trajectory->mutable_header()->set_timestamp_sec(Timer::Now());
  pub_trajectory->mutable_header()->set_sequence_num(1);
  headings = k_r.phi;
  vs = k_r.v;
  as = k_r.a;
  double t = 0.0;
  for (size_t i = 0; i < k_r.x.size(); ++i) {
    time.push_back(t);
    xypoints.push_back({k_r.x[i], k_r.y[i]});
    t += FLAGS_delta_t;
  }

  CHECK(DiscretePointsMath::ComputePathProfile(xypoints, &accumulated_s,
                                               &kappas, &dkappas));

  // vote for gear
  size_t v_pos = 0;
  for (auto& v : vs) {
    v_pos += v > 0;
  }

  pub_trajectory->set_gear(v_pos > vs.size() / 2
                               ? canbus::Chassis::GEAR_LOW
                               : canbus::Chassis::GEAR_REVERSE);
  double gear =
      pub_trajectory->gear() == canbus::Chassis::GEAR_REVERSE ? -1.0 : 1.0;
  const int n = xypoints.size();
  for (int i = 0; i < n; ++i) {
    auto* point = pub_trajectory->add_trajectory_point();
    point->mutable_path_point()->set_x(xypoints[i].first);
    point->mutable_path_point()->set_y(xypoints[i].second);
    point->mutable_path_point()->set_theta(headings[i]);
    point->mutable_path_point()->set_s(accumulated_s[i] * gear);
    point->mutable_path_point()->set_kappa(kappas[i] * gear);
    point->mutable_path_point()->set_dkappa(dkappas[i]);
    point->set_a(as[i]);
    point->set_v(vs[i]);
    point->set_relative_time(time[i]);
    point->set_steer(k_r.steer[i]);
  }
  return true;
}

bool Planning::PublishTrajectory(planning::ADCTrajectory* pub_trajectory,
                                 const std::string& file_name) {
  KineticResult k_r;
  Planning::GetCruiseTrajectory(true, &k_r, file_name);
  return PublishTrajectory<KineticResult>(pub_trajectory, k_r);
}

void Planning::GetReferenceLine(const std::string& file_name,
                                std::vector<ReferenceLineInfo>* rfl_info) {
  CHECK(rfl_info) << "reference line info is nullptr";
  CruiseTrajectory tr;
  RFPS reference_points;
  tr.LoadPathPoints(file_name, FLAGS_enable_smooth, &reference_points);
  rfl_info->push_back(ReferenceLineInfo(ReferenceLine(reference_points)));
  std::ofstream f;
  f.open(FlAGS_trajectory_points_output_path, std::fstream::out);
  f << std::fixed;
  f << "x,y,phi,kappa" << std::endl;
  for (auto&& p : reference_points) {
    f << p.x() << ',' << p.y() << ',' << p.heading() << ',' << p.kappa() << ','
      << std::endl;
  }
}

void Planning::OpenSpaceTrajectoryPub(std::string const& file_name,
                                      HybridAStarResult& res,
                                      planning::ADCTrajectory* trajectory) {
  WriteKineticResult<HybridAStarResult>(file_name, res);
  PublishTrajectory<HybridAStarResult>(trajectory, res);
}
