#ifndef _OBSTACLE_H_
#define _OBSTACLE_H_

#include <list>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "common/math/box2d.h"
#include "common/math/polygon2d.h"
#include "common/proto/decision.pb.h"
#include "common/proto/perception_obstacle.pb.h"
#include "common/proto/prediction_obstacle.pb.h"
#include "common/proto/sl_boundary.pb.h"
#include "common/proto/vehicle_config.pb.h"
#include "reference_line.h"
#include "st_boundary.h"

/**
 * @class Obstacle
 * @brief This is the class that associates an Obstacle with its path
 * properties. An obstacle's path properties relative to a path.
 * The `s` and `l` values are examples of path properties.
 * The decision of an obstacle is also associated with a path.
 *
 * The decisions have two categories: lateral decision and longitudinal
 * decision.
 * Lateral decision includes: nudge, ignore.
 * Lateral decision safety priority: nudge > ignore.
 * Longitudinal decision includes: stop, yield, follow, overtake, ignore.
 * Decision safety priorities order: stop > yield >= follow > overtake > ignore
 *
 * Ignore decision belongs to both lateral decision and longitudinal decision,
 * and it has the lowest priority.
 */
class Obstacle {
 public:
  Obstacle() = default;
  Obstacle(const std::string& id,
           const perception::PerceptionObstacle& perception_obstacle,
           const prediction::ObstaclePriority::Priority& obstacle_priority,
           const bool is_static);
  Obstacle(const std::string& id,
           const perception::PerceptionObstacle& perception_obstacle,
           const prediction::Trajectory& trajectory,
           const prediction::ObstaclePriority::Priority& obstacle_priority,
           const bool is_static);

  const std::string& Id() const { return id_; }
  void SetId(const std::string& id) { id_ = id; }

  double speed() const { return speed_; }

  int32_t PerceptionId() const { return perception_id_; }

  bool IsStatic() const { return is_static_; }
  bool IsVirtual() const { return is_virtual_; }

  points::TrajectoryPoint GetPointAtTime(const double time) const;

  Box2d GetBoundingBox(const points::TrajectoryPoint& point) const;

  const Box2d& PerceptionBoundingBox() const {
    return perception_bounding_box_;
  }
  const Polygon2d& PerceptionPolygon() const { return perception_polygon_; }
  const prediction::Trajectory& Trajectory() const { return trajectory_; }
  points::TrajectoryPoint* AddTrajectoryPoint() {
    return trajectory_.add_trajectory_point();
  }
  bool HasTrajectory() const {
    return !(trajectory_.trajectory_point().empty());
  }

  const perception::PerceptionObstacle& Perception() const {
    return perception_obstacle_;
  }

  /**
   * @brief This is a helper function that can create obstacles from prediction
   * data.  The original prediction may have multiple trajectories for each
   * obstacle. But this function will create one obstacle for each trajectory.
   * @param predictions The prediction results
   * @return obstacles The output obstacles saved in a list of unique_ptr.
   */
  static std::list<std::unique_ptr<Obstacle>> CreateObstacles(
      const prediction::PredictionObstacles& predictions);

  static std::unique_ptr<Obstacle> CreateStaticVirtualObstacles(
      const std::string& id, const Box2d& obstacle_box);

  static bool IsValidPerceptionObstacle(
      const perception::PerceptionObstacle& obstacle);

  static bool IsValidTrajectoryPoint(const points::TrajectoryPoint& point);

  inline bool IsCautionLevelObstacle() const {
    return is_caution_level_obstacle_;
  }

  // const Obstacle* obstacle() const;

  /**
   * return the merged lateral decision
   * Lateral decision is one of {Nudge, Ignore}
   **/
  const decision::ObjectDecisionType& LateralDecision() const;

  /**
   * @brief return the merged longitudinal decision
   * Longitudinal decision is one of {Stop, Yield, Follow, Overtake, Ignore}
   **/
  const decision::ObjectDecisionType& LongitudinalDecision() const;

  std::string DebugString() const;

  const boundary::SLBoundary& PerceptionSLBoundary() const;

  const STBoundary& reference_line_st_boundary() const;

  const STBoundary& path_st_boundary() const;

  const std::vector<std::string>& decider_tags() const;

  const std::vector<decision::ObjectDecisionType>& decisions() const;

  void AddLongitudinalDecision(const std::string& decider_tag,
                               const decision::ObjectDecisionType& decision);

  void AddLateralDecision(const std::string& decider_tag,
                          const decision::ObjectDecisionType& decision);
  bool HasLateralDecision() const;

  void set_path_st_boundary(const STBoundary& boundary);

  bool is_path_st_boundary_initialized() {
    return path_st_boundary_initialized_;
  }

  void SetStBoundaryType(const STBoundary::BoundaryType type);

  void EraseStBoundary();

  void SetReferenceLineStBoundary(const STBoundary& boundary);

  void SetReferenceLineStBoundaryType(const STBoundary::BoundaryType type);

  void EraseReferenceLineStBoundary();

  bool HasLongitudinalDecision() const;

  bool HasNonIgnoreDecision() const;

  /**
   * @brief Calculate stop distance with the obstacle using the ADC's minimum
   * turning radius
   */
  double MinRadiusStopDistance(
      const vehicle::VehicleParam& vehicle_param) const;

  /**
   * @brief Check if this object can be safely ignored.
   * The object will be ignored if the lateral decision is ignore and the
   * longitudinal decision is ignore
   *  return longitudinal_decision_ == ignore && lateral_decision == ignore.
   */
  bool IsIgnore() const;
  bool IsLongitudinalIgnore() const;
  bool IsLateralIgnore() const;

  void SetPerceptionSlBoundary(const boundary::SLBoundary& sl_boundary);

  /**
   * @brief check if an decision::ObjectDecisionType is a longitudinal decision.
   **/
  static bool IsLongitudinalDecision(
      const decision::ObjectDecisionType& decision);

  /**
   * @brief check if an decision::ObjectDecisionType is a lateral decision.
   **/
  static bool IsLateralDecision(const decision::ObjectDecisionType& decision);

  void SetBlockingObstacle(bool blocking) { is_blocking_obstacle_ = blocking; }
  bool IsBlockingObstacle() const { return is_blocking_obstacle_; }

  /*
   * @brief IsLaneBlocking is only meaningful when IsStatic() == true.
   */
  bool IsLaneBlocking() const { return is_lane_blocking_; }
  void CheckLaneBlocking(const ReferenceLine& reference_line);
  bool IsLaneChangeBlocking() const { return is_lane_change_blocking_; }
  void SetLaneChangeBlocking(const bool is_distance_clear);

 private:
  // FRIEND_TEST(MergeLongitudinalDecision, AllDecisions);
  static decision::ObjectDecisionType MergeLongitudinalDecision(
      const decision::ObjectDecisionType& lhs,
      const decision::ObjectDecisionType& rhs);
  // FRIEND_TEST(MergeLateralDecision, AllDecisions);
  static decision::ObjectDecisionType MergeLateralDecision(
      const decision::ObjectDecisionType& lhs,
      const decision::ObjectDecisionType& rhs);

  bool IsValidObstacle(
      const perception::PerceptionObstacle& perception_obstacle);

 private:
  std::string id_;
  int32_t perception_id_ = 0;
  bool is_static_ = false;
  bool is_virtual_ = false;
  double speed_ = 0.0;

  bool path_st_boundary_initialized_ = false;

  prediction::Trajectory trajectory_;
  perception::PerceptionObstacle perception_obstacle_;
  Box2d perception_bounding_box_;
  Polygon2d perception_polygon_;

  std::vector<decision::ObjectDecisionType> decisions_;
  std::vector<std::string> decider_tags_;
  boundary::SLBoundary sl_boundary_;

  STBoundary reference_line_st_boundary_;
  STBoundary path_st_boundary_;

  decision::ObjectDecisionType lateral_decision_;
  decision::ObjectDecisionType longitudinal_decision_;

  // for keep_clear usage only
  bool is_blocking_obstacle_ = false;

  bool is_lane_blocking_ = false;

  bool is_lane_change_blocking_ = false;

  bool is_caution_level_obstacle_ = false;

  double min_radius_stop_distance_ = -1.0;

  struct ObjectTagCaseHash {
    size_t operator()(
        const decision::ObjectDecisionType::ObjectTagCase tag) const {
      return static_cast<size_t>(tag);
    }
  };

  static const std::unordered_map<decision::ObjectDecisionType::ObjectTagCase,
                                  int, ObjectTagCaseHash>
      s_lateral_decision_safety_sorter_;
  static const std::unordered_map<decision::ObjectDecisionType::ObjectTagCase,
                                  int, ObjectTagCaseHash>
      s_longitudinal_decision_safety_sorter_;
};

// typedef IndexedList<std::string, Obstacle> IndexedObstacles;
// typedef ThreadSafeIndexedList<std::string, Obstacle>
// ThreadSafeIndexedObstacles;

template <class Collection>
const typename Collection::value_type::second_type& FindOrDie(
    Collection& collection,  // NOLINT
    typename Collection::value_type::first_type& key) {
  auto it = collection.find(key);
  GOOGLE_CHECK(it != collection.end()) << "Map key not found: " << key;
  return it->second;
}

#endif
