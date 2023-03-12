#ifndef _PREDICTION_QUERIER_H_
#define _PREDICTION_QUERIER_H_

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "planning/common/obstacle.h"

class PredictionQuerier {
 public:
  PredictionQuerier(const std::vector<const Obstacle*>& obstacles,
                    const std::shared_ptr<std::vector<points::PathPoint>>&
                        ptr_reference_line);

  virtual ~PredictionQuerier() = default;

  std::vector<const Obstacle*> GetObstacles() const;

  double ProjectVelocityAlongReferenceLine(const std::string& obstacle_id,
                                           const double s,
                                           const double t) const;

 private:
  std::unordered_map<std::string, const Obstacle*> id_obstacle_map_;

  std::vector<const Obstacle*> obstacles_;

  std::shared_ptr<std::vector<points::PathPoint>> ptr_reference_line_;
};

template <class Collection>
bool InsertIfNotPresent(Collection* const collection,
                        const typename Collection::value_type& vt) {
  return collection->insert(vt).second;
}
// Same as above except the key and value are passed separately.
template <class Collection>
bool InsertIfNotPresent(
    Collection* const collection,
    const typename Collection::value_type::first_type& key,
    const typename Collection::value_type::second_type& value) {
  return InsertIfNotPresent(collection,
                            typename Collection::value_type(key, value));
}

#endif
