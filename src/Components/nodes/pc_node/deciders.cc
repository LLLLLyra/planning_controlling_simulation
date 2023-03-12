#include "deciders.h"

#include "common/math/quaternion.h"
#include "common/proto/vehicle_config.pb.h"
#include "glog/logging.h"

void Deciders::OpenSpaceROIDecider(
    ObstacleInfo& obstacles, std::vector<std::vector<Vec2d>>* obstacles_final,
    std::vector<double>* XYbounds, std::array<double, 3>* end) {
  CHECK_NOTNULL(end);
  CHECK_NOTNULL(XYbounds);
  CHECK_NOTNULL(obstacles_final);
  XYbounds->clear();
  obstacles_final->clear();

  // choose the first one
  auto selected_slot = obstacles.first.front();
  *XYbounds = {selected_slot[3].x() - 8, selected_slot[0].x() + 8,
               selected_slot[3].y() - 8, selected_slot[0].y() + 8};

  double x_min = XYbounds->at(0);
  double x_max = XYbounds->at(1);
  double y_min = XYbounds->at(2);
  double y_max = XYbounds->at(3);

  for (auto& rada_point : obstacles.second) {
    std::vector<Vec2d> ob;
    for (auto& vec : rada_point) {
      if (vec.x() > x_max || vec.x() < x_min || vec.y() > y_max ||
          vec.y() < y_min) {
        continue;
      }
      bool is_in_slot = false;
      for (auto& slot_corners : obstacles.first) {
        if (IsPointInRect(slot_corners[0], slot_corners[1], slot_corners[2],
                          slot_corners[3], vec)) {
          is_in_slot = true;
          break;
        }
      }
      // drop the rest radar points as this is the first one lying in the slot
      if (is_in_slot) break;
      ob.push_back(vec);
    }
    if (!ob.empty()) {
      obstacles_final->emplace_back(ob);
    }
  }
  obstacles_final->push_back(selected_slot);

  Vec2d mid_pt_03 = (selected_slot[0] + selected_slot[3]) / 2;
  Vec2d mid_pt_12 = (selected_slot[1] + selected_slot[2]) / 2;
  LineSegment2d mid_line(mid_pt_12, mid_pt_03);
  vehicle::VehicleParam veh_param;
  const double set_val = veh_param.back_edge_to_center() + 0.2;
  const double theta = mid_line.heading();
  *end = {set_val * std::cos(theta) + mid_pt_12.x(),
          set_val * std::sin(theta) + mid_pt_12.y(), theta};
}

void Deciders::CoorTrans(simulation_msg::ObstacleServiceResponse& obstacle_res,
                         std::shared_ptr<DependencyInjector> injector,
                         ObstacleInfo* obstacles) {
  CHECK_NOTNULL(obstacles);
  CHECK_NOTNULL(injector);
  auto& [slots, radars] = *obstacles;
  for (auto& slot : obstacle_res.slots) {
    std::vector<Vec2d> obs;
    for (size_t i = 0; i < 4; i++) {
      Eigen::Vector3d slot_vec3d;
      slot_vec3d(0) = static_cast<double>(slot.corner[i].x) / 1000.0;
      slot_vec3d(1) = static_cast<double>(slot.corner[i].y) / 1000.0;
      slot_vec3d(2) = 0.0;
      auto slot_vec3d_trans = InverseQuaternionRotate(
          injector->vehicle_state()->pose().orientation(), slot_vec3d);
      obs.push_back({slot_vec3d_trans(0) + injector->vehicle_state()->x(),
                     slot_vec3d_trans(1) + +injector->vehicle_state()->y()});
    }
    slots.emplace_back(obs);
  }

  for (auto& radar : obstacle_res.radar_points) {
    std::vector<Vec2d> obs;
    for (auto& coor : radar.radar) {
      Eigen::Vector3d obl_vec3d;
      obl_vec3d(0) = static_cast<double>(coor.x) / 1000.0;
      obl_vec3d(1) = static_cast<double>(coor.y) / 1000.0;
      obl_vec3d(2) = 0.0;
      auto obl_vec3d_trans = InverseQuaternionRotate(
          injector->vehicle_state()->pose().orientation(), obl_vec3d);
      obs.push_back({obl_vec3d_trans(0) + injector->vehicle_state()->x(),
                     obl_vec3d_trans(1) + +injector->vehicle_state()->y()});
    }
    radars.emplace_back(obs);
  }
}

bool Deciders::IsPointInRect(const Vec2d& p1, const Vec2d& p2, const Vec2d& p3,
                             const Vec2d& p4, const Vec2d& pt) {
  double a = (p2 - p1).CrossProd(pt - p1);
  double b = (p3 - p2).CrossProd(pt - p2);
  double c = (p4 - p3).CrossProd(pt - p3);
  double d = (p1 - p4).CrossProd(pt - p4);

  if ((a > 0 && b > 0 && c > 0 && d > 0) ||
      (a < 0 && b < 0 && c < 0 && d < 0)) {
    return true;
  }
  return false;
}