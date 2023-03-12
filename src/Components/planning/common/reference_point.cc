#include "reference_point.h"

#include "point_factory.h"

namespace {
// Minimum distance to remove duplicated points.
const double kDuplicatedPointsEpsilon = 1e-7;
}  // namespace

// ReferencePoint::ReferencePoint(const MapPathPoint& map_path_point,
//                                const double kappa, const double dkappa)
//     : hdmap::MapPathPoint(map_path_point), kappa_(kappa), dkappa_(dkappa) {}

points::PathPoint ReferencePoint::ToPathPoint(double s) const {
  return PointFactory::ToPathPoint(x(), y(), 0.0, s, heading(), kappa_,
                                   dkappa_);
}

double ReferencePoint::kappa() const { return kappa_; }

double ReferencePoint::dkappa() const { return dkappa_; }

std::string ReferencePoint::DebugString() const {
  return "{x: " + std::to_string(x()) + ", y: " + std::to_string(y()),
         ", theta: " + std::to_string(heading()) +
             ", kappa: " + std::to_string(kappa()) +
             ", dkappa: " + std::to_string(dkappa()) + "}";
}

void ReferencePoint::RemoveDuplicates(std::vector<ReferencePoint>* points) {
  CHECK_NOTNULL(points);
  int count = 0;
  const double limit = kDuplicatedPointsEpsilon * kDuplicatedPointsEpsilon;
  for (size_t i = 0; i < points->size(); ++i) {
    if (count == 0 ||
        (*points)[i].DistanceSquareTo((*points)[count - 1]) > limit) {
      (*points)[count++] = (*points)[i];
    } else {
      (*points)[count - 1].add_lane_waypoints((*points)[i].lane_waypoints());
    }
  }
  points->resize(count);
}
