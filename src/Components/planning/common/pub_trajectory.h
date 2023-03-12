#ifndef _PUB_TRAJECTORY_H_
#define _PUB_TRAJECTORY_H_

#include <glog/logging.h>

#include <fstream>
#include <vector>

#include "common/config/flags.h"
#include "common/config/log_path_conf.h"
#include "common/config/timer.h"
#include "common/math/discrete_points_math.h"
#include "common/proto/planning.pb.h"
#include "cruise_trajectory.h"
#include "planning/open_space/hybrid_a_star.h"
#include "reference_line.h"

namespace Planning {
/**
 * @brief write kinetic result ro csv
 *
 * @tparam ResContrainer
 * @param file_name to csv
 * @param result planning results
 */
template <typename ResContrainer>
void WriteKineticResult(const std::string file_name, ResContrainer& result);

/**
 * @brief Get the Cruise Trajectory object
 *
 * @param to_csv whether write to csv
 * @param result smoothed planning results
 * @param file_name csv file
 */
void GetCruiseTrajectory(bool to_csv, KineticResult* result,
                         const std::string& file_name);

/**
 * @brief publish a planned trajectory
 *
 * @tparam ResContainer
 * @param pub_trajectory a pointer to the published trajectory
 * @param k_r smoothed planned results
 */
template <typename ResContainer>
bool PublishTrajectory(planning::ADCTrajectory* pub_trajectory,
                       const ResContainer& k_r);

/**
 * @brief publish a planned trajectory
 *
 * @param pub_trajectory a pointer to the published trajectory
 * @param file_name path file
 */
bool PublishTrajectory(planning::ADCTrajectory* pub_trajectory,
                       const std::string& file_name);

/**
 * @brief Get the Reference Line object
 *
 * @param file_name to csv
 * @param rfl_info the target reference line info vector
 */
void GetReferenceLine(const std::string& file_name,
                      std::vector<ReferenceLineInfo>* rfl_info);

/**
 * @brief publish a smoothed trajectory from hybrid a star planning
 *
 * @param file_name csv file
 * @param res HybridAStarResult
 * @param trajectory a pointer to the a target trajectory to be published
 */
void OpenSpaceTrajectoryPub(std::string const& file_name,
                            HybridAStarResult& res,
                            planning::ADCTrajectory* trajectory);
}  // namespace Planning

#endif
