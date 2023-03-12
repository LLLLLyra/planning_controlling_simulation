#ifndef _DISCRETIZED_PATH_H_
#define _DISCRETIZED_PATH_H_

#include <utility>
#include <vector>

#include "common/proto/pnc_point.pb.h"

class DiscretizedPath : public std::vector<points::PathPoint> {
 public:
  DiscretizedPath() = default;

  explicit DiscretizedPath(std::vector<points::PathPoint> path_points);

  double Length() const;

  points::PathPoint Evaluate(const double path_s) const;

  points::PathPoint EvaluateReverse(const double path_s) const;

 protected:
  std::vector<points::PathPoint>::const_iterator QueryLowerBound(
      const double path_s) const;
  std::vector<points::PathPoint>::const_iterator QueryUpperBound(
      const double path_s) const;
};

#endif  // _DISCRETIZED_PATH_H_
