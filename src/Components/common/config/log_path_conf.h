#ifndef _LOG_PATH_CONF_H_
#define _LOG_PATH_CONF_H_

// controller configs
#define FLAGS_controller_config_lattice \
  "/Components/common/config/"          \
  "control_conf.pb.lattice.txt"

#define FLAGS_controller_config_parking \
  "/Components/common/config/"          \
  "control_conf.pb.parking.txt"

// controlled log file
#define FLAGS_controlled_log \
  "/Components/trajectory/"  \
  "traj_controlled.csv"

// input path points
#define FLAGS_path_points_file_path "/Components/common/config/"

// output trajectory points
#define FlAGS_trajectory_points_output_path \
  "/Components/trajectory/"                 \
  "demo_trajectory.csv"

// controller logs
#define FLAGS_controller_log "/Components/log/"

#endif  // _LOG_PATH_CONF_H_
