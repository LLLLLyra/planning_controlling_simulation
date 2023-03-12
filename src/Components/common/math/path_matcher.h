#ifndef _PATH_MATCHER_H_
#define _PATH_MATCHER_H_

#include <utility>
#include <vector>

#include "common/proto/pnc_point.pb.h"

class PathMatcher {
 public:
  PathMatcher() = delete;

  static points::PathPoint MatchToPath(
      const std::vector<points::PathPoint>& reference_line, const double x,
      const double y);

  static std::pair<double, double> GetPathFrenetCoordinate(
      const std::vector<points::PathPoint>& reference_line, const double x,
      const double y);

  static points::PathPoint MatchToPath(
      const std::vector<points::PathPoint>& reference_line, const double s);

 private:
  static points::PathPoint FindProjectionPoint(const points::PathPoint& p0,
                                               const points::PathPoint& p1,
                                               const double x, const double y);
};

#endif
