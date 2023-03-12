#ifndef _FLAGS_H_
#define _FLAGS_H_

#define FLAGS_dt 0.1

#define FLAGS_enable_parking true
#define FlAGS_enable_lattice true

#define FLAGS_v_LOF 50
#define FLAGS_k_LOF 5

#define FLAGS_query_forward_time_point_only true

#define FLAGS_reverse_heading_vehicle_state false
#define FLAGS_use_navigation_mode false
#define FLAGS_enable_map_reference_unify false
#define FLAGS_state_transform_to_com_reverse false
#define FLAGS_state_transform_to_com_drive true

#define FLAGS_enable_csv_debug true
#define FLAGS_enable_navigation_mode_position_update true
#define FLAGS_trajectory_transform_to_com_reverse false
#define FLAGS_trajectory_transform_to_com_drive false
#define FLAGS_enable_gain_scheduler false
#define FLAGS_enable_feedback_augment_on_high_speed false
#define FLAGS_set_steer_limit false
#define FLAGS_enable_maximum_steer_rate_limit false
#define FLAGS_lock_steer_speed 0.081
#define FLAGS_steer_angle_rate 100.0
#define FLAGS_query_time_nearest_point_only true
#define FLAGS_enable_navigation_mode_error_filter false
#define FLAGS_reverse_heading_control false
#define FLAGS_enable_navigation_mode_position_update true
#define FLAGS_enable_speed_station_preview true
#define FLAGS_enable_slope_offset false
#define FLAGS_use_preview_speed_for_table false

#define FLAGS_multithread_run false

// enable qp smoothing or not
#define FLAGS_enable_smooth false
// time interval between the smoothed trajectory points
#define FLAGS_delta_t 0.5

#define FLAGS_default_lane_width 3.048

#define FLAGS_enable_trajectory_stitcher true
#define FLAGS_replan_lateral_distance_threshold 0.5
#define FLAGS_replan_longitudinal_distance_threshold 2.5

#define FLAGS_default_reference_line_width 4.0
#define FLAGS_trajectory_time_length 8.0
#define FLAGS_trajectory_time_resolution 0.1
#define FLAGS_numerical_epsilon 1e-6
#define FLAGS_bound_buffer 0.1
#define FLAGS_nudge_buffer 0.3

#define FLAGS_longitudinal_acceleration_lower_bound -6.0
#define FLAGS_longitudinal_acceleration_upper_bound 4.0

#define FLAGS_polynomial_minimal_param 0.01
#define FLAGS_num_velocity_sample 6
#define FLAGS_time_min_density 1.0
#define FLAGS_default_lon_buffer 5.0
#define FLAGS_min_velocity_sample_gap 0.3
#define FLAGS_num_sample_follow_per_timestamp 3

#define FLAGS_lateral_third_order_derivative_max 0.1
#define FLAGS_weight_lateral_obstacle_distance 0.0
#define FLAGS_weight_lateral_offset 1.0
#define FLAGS_weight_lateral_derivative 500.0
#define FLAGS_weight_lateral_second_order_derivative 1000.0
#define FLAGS_default_delta_s_lateral_optimization 1.0

#define FLAGS_lattice_stop_buffer 0.02
#define FLAGS_trajectory_space_resolution 1.0
#define FLAGS_weight_lon_objective 10.0
#define FLAGS_weight_lon_jerk 1.0
#define FLAGS_weight_lon_collision 5.0
#define FLAGS_weight_centripetal_acceleration 1.5
#define FLAGS_weight_lat_offset 100.0
#define FLAGS_weight_lat_comfort 1.0
#define FLAGS_lat_offset_bound 3.0
#define FLAGS_weight_opposite_side_offset 10.0
#define FLAGS_weight_same_side_offset 1.0
#define FLAGS_longitudinal_jerk_upper_bound 2.0
#define FLAGS_weight_target_speed 1.0
#define FLAGS_weight_dist_travelled 10.0
#define FLAGS_lon_collision_cost_std 0.5
#define FLAGS_lon_collision_yield_buffer 1.0
#define FLAGS_lon_collision_overtake_buffer 5.0
#define FLAGS_comfort_acceleration_factor 0.5

#define FLAGS_max_s_lateral_optimization 60.0
#define FLAGS_lateral_optimization false

#define FLAGS_lon_collision_buffer 2.0
#define FLAGS_lat_collision_buffer 0.1

#define FLAGS_speed_lower_bound -0.1
#define FLAGS_speed_upper_bound 40.0
#define FLAGS_longitudinal_jerk_lower_bound -4.0
#define FLAGS_lateral_acceleration_bound 4.0
#define FLAGS_lateral_jerk_bound 4.0

#define FLAGS_virtual_stop_wall_height 2.0
#define FLAGS_max_stop_distance_obstacle 10.0
#define FLAGS_min_stop_distance_obstacle 6.0
#define FLAGS_st_max_t 8.0
#define FLAGS_st_max_s 100.0
#define FLAGS_nonstatic_obstacle_nudge_l_buffer 0.4
#define FLAGS_static_obstacle_nudge_l_buffer 0.3

#define FLAGS_speed_lon_decision_horizon 200.0

#define FLAGS_use_s_curve_speed_smooth true

#define FLAGS_kappa_bound 0.35
#define FLAGS_enable_backup_trajectory false
#define FLAGS_backup_trajectory_cost 1000.0
#define FLAGS_cost_non_priority_reference_line 5.0

#define FLAGS_enable_control_by_time false

#define FLAGS_enable_IAPS true
#define FLAGS_min_trajectory_distance 1.0
#define FLAGS_enable_fem_pos_smoother false
#endif  // _FLAGS_H_
