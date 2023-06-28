#ifndef _DISCRETIZED_TRAJECTORY_H_
#define _DISCRETIZED_TRAJECTORY_H_

#include <vector>


#include "common/math/vec2d.h"
#include "common/proto/pnc_point.pb.h"
#include "glog/logging.h"

class DiscretizedTrajectory : public std::vector<points::TrajectoryPoint> {
 public:
  DiscretizedTrajectory() = default;


  explicit DiscretizedTrajectory(
      const std::vector<points::TrajectoryPoint>& trajectory_points);

  void SetTrajectoryPoints(
      const std::vector<points::TrajectoryPoint>& trajectory_points);

  virtual ~DiscretizedTrajectory() = default;

  virtual points::TrajectoryPoint StartPoint() const;

  virtual double GetTemporalLength() const;

  virtual double GetSpatialLength() const;

  virtual points::TrajectoryPoint Evaluate(const double relative_time) const;

  virtual size_t QueryLowerBoundPoint(const double relative_time,
                                      const double epsilon = 1.0e-5) const;

  virtual size_t QueryNearestPoint(const Vec2d& position) const;

  size_t QueryNearestPointWithBuffer(const Vec2d& position,
                                     const double buffer) const;

  virtual void AppendTrajectoryPoint(
      const points::TrajectoryPoint& trajectory_point);

  void PrependTrajectoryPoints(
      const std::vector<points::TrajectoryPoint>& trajectory_points) {
    if (!empty() && trajectory_points.size() > 1) {
      CHECK(trajectory_points.back().relative_time() <
             front().relative_time());
    }
    insert(begin(), trajectory_points.begin(), trajectory_points.end());
  }

  const points::TrajectoryPoint& TrajectoryPointAt(const size_t index) const;

  size_t NumOfPoints() const;

  virtual void Clear();
};

inline size_t DiscretizedTrajectory::NumOfPoints() const { return size(); }

inline void DiscretizedTrajectory::Clear() { clear(); }


#endif
