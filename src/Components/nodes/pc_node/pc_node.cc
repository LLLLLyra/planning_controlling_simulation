#include "pc_node.h"

#include <pthread.h>

#include <thread>

#include "common/math/quaternion.h"
#include "control/control/read_proto.h"
#include "deciders.h"

PCNode::PCNode(ros::NodeHandle *n) : n_(*n) {}

void PCNode::localisation_call_back(const nav_msgs::OdometryPtr odometry) {
  show_.RealPathToShow(odometry);

  double x, y, z;
  x = odometry->pose.pose.position.x;
  y = odometry->pose.pose.position.y;
  z = odometry->pose.pose.position.z;

  double vx = (x - location_.pose().position().x()) / FLAGS_dt;
  double vy = (y - location_.pose().position().y()) / FLAGS_dt;
  double vz = (z - location_.pose().position().z()) / FLAGS_dt;

  location_.mutable_pose()->mutable_position()->set_x(x);
  location_.mutable_pose()->mutable_position()->set_y(y);
  location_.mutable_pose()->mutable_position()->set_z(z);

  location_.mutable_pose()->mutable_orientation()->set_qw(
      odometry->pose.pose.orientation.w);
  location_.mutable_pose()->mutable_orientation()->set_qx(
      odometry->pose.pose.orientation.x);
  location_.mutable_pose()->mutable_orientation()->set_qy(
      odometry->pose.pose.orientation.y);
  location_.mutable_pose()->mutable_orientation()->set_qz(
      odometry->pose.pose.orientation.z);

  double heading = QuaternionToHeading(
      odometry->pose.pose.orientation.w, odometry->pose.pose.orientation.x,
      odometry->pose.pose.orientation.y, odometry->pose.pose.orientation.z);

  double v = std::sqrt(vx * vx + vy * vy + vz * vz);
  double a = (v - chassis_.speed_mps()) / FLAGS_dt;
  double omega = (heading - location_.pose().heading()) / FLAGS_dt;

  chassis_.set_speed_mps(v);

  location_.mutable_pose()->mutable_linear_acceleration()->set_y(a);
  location_.mutable_pose()->mutable_angular_velocity()->set_z(omega);
  location_.mutable_pose()->set_heading(heading);
  injector_->vehicle_state()->Update(location_, chassis_);
}

void PCNode::Run() {
  std::shared_ptr<LatLonController> lat_lon_ctr =
      std::make_shared<LatLonController>();
  ctr_ = lat_lon_ctr.get();
  show_.Init(&n_, &reference_path_, &trajectory_, &real_path_);
  obstacles_map_service_ =
      n_.serviceClient<simulation_msg::ObstacleService>("/get_obstacles");
  cmd_pub_ = n_.advertise<simulation_msg::CarControlMsg>("/car_control", 1000);
  location_sub_ = n_.subscribe("/location_odom", 1000,
                               &PCNode::localisation_call_back, this);

  std::thread P{&PCNode::Process, this};
  ros::spin();
  pthread_cancel(P.native_handle());
  P.join();
}

void PCNode::Process() {
  std::this_thread::sleep_for(Timer::MilliSeconds(500));
  std::string file_name;
  ros::param::get("file_name", file_name);
  controller::ControlConf ctr_conf;
  PlanningProc plan_proc;
  ControlProc control_proc;
  plan_proc.InitShow(&show_);
  control_proc.Init(&cmd_pub_, &show_);
  bool parking, lattice;
  ros::param::get("parking", parking);
  ros::param::get("lattice", lattice);
  if (parking) {
    CHECK(Controlling::GetProtoFromASCIIFile(FLAGS_controller_config_parking,
                                             &ctr_conf));

    obstacles_map_service_.call(obstacles_info_);
    Deciders::CoorTrans(obstacles_info_.response, injector_, &obstacles_);
    plan_proc.OpenSpaceInit(obstacles_);
    std::array<double, 3L> start = {injector_->vehicle_state()->x(),
                                    injector_->vehicle_state()->x(),
                                    injector_->vehicle_state()->heading()};
    std::vector<planning::ADCTrajectory> trajectories;
    plan_proc.OpenSpacePlan(start, &trajectories);
    for (auto &t : trajectories) {
      trajectory_queue_.push(t);
    }
    ctr_->Init(injector_, &ctr_conf);
    control_proc.ControllerInit(ctr_);
    control_proc.Control(&location_, &chassis_, &trajectory_queue_, injector_);
  } else {
    CHECK(Controlling::GetProtoFromASCIIFile(FLAGS_controller_config_lattice,
                                             &ctr_conf));
    planning::ADCTrajectory &trajectory = trajectory_queue_.emplace();
    ctr_->Init(injector_, &ctr_conf);
    control_proc.ControllerInit(ctr_);
    if (!lattice) {
      plan_proc.CruisePlan(file_name, &trajectory);
      control_proc.Control(&location_, &chassis_, &trajectory_queue_);
    } else {
      plan_proc.LatticeInit(file_name);
      points::TrajectoryPoint start_point;
      auto rate = ros::Rate(10);
      while (!ctr_->get_reached()) {
        if (!trajectory.trajectory_point_size() ||
            control_proc.LatticePlanningSignal(trajectory)) {
          start_point.mutable_path_point()->set_x(
              injector_->vehicle_state()->x());
          start_point.mutable_path_point()->set_y(
              injector_->vehicle_state()->y());
          start_point.mutable_path_point()->set_theta(
              injector_->vehicle_state()->heading());
          start_point.set_relative_time(0);
          start_point.set_v(chassis_.has_speed_mps() ? chassis_.speed_mps()
                                                     : 0);
          plan_proc.LatticePlan(start_point, &trajectory);
        }
        control_proc.Controlling(&location_, &chassis_, &trajectory);
        rate.sleep();
      }
    }
  }
}

int main(int args, char *argv[]) {
  ros::init(args, argv, "simulation");
  ros::NodeHandle node;
  PCNode n(&node);
  n.Run();
}