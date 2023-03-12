#ifndef DECIDERS_H_
#define DECIDERS_H_

#include <array>
#include <cmath>
#include <vector>

#include "common/math/vec2d.h"
#include "control/control/dependency_injector.h"
#include "simulation_msg/ObstacleServiceResponse.h"

/**
 * @brief Container for obstacles; [slots | radar points]
 */
using ObstacleInfo =
    std::pair<std::vector<std::vector<Vec2d>>, std::vector<std::vector<Vec2d>>>;

namespace Deciders {
/**
 * @brief Decide ROI for hybrid a star
 *
 * @param obstacles coordinate-transferred obstacles information from perception
 * module
 * @param obstacles_final obstacles for hybrid a star
 * @param XYbounds map boundary for hybrid a star
 * @param end end point
 */
void OpenSpaceROIDecider(ObstacleInfo& obstacles,
                         std::vector<std::vector<Vec2d>>* obstacles_final,
                         std::vector<double>* XYbounds,
                         std::array<double, 3>* end);

/**
 * @brief transfer original obstacles' coordinates to global coordinates
 *
 * @param obstacle_res original obstacles information from perception module
 * @param injector the ego pose
 * @param obstacles transferred obstacles
 */
void CoorTrans(simulation_msg::ObstacleServiceResponse& obstacle_res,
               std::shared_ptr<DependencyInjector> injector,
               ObstacleInfo* obstacles);

/**
 * @brief Check a point `pt` is the a rec or not
 *
 * @param p1 rectangular left-top point
 * @param p2 rectangular right-top point
 * @param p3 rectangular left-bottom point
 * @param p4 rectangular right-bottom point
 * @param pt
 * @return the point pt is in the rectangular or not
 */
bool IsPointInRect(const Vec2d& p1, const Vec2d& p2, const Vec2d& p3,
                   const Vec2d& p4, const Vec2d& pt);
}  // namespace Deciders

#endif  // DECIDERS_H_