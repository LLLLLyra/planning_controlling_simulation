#include "local_outlier_factor.h"

#include "common/config/flags.h"
#include "glog/logging.h"

LocalOutlierFactor::LocalOutlierFactor(std::vector<Sample*>& points)
    : points_(points) {
  CHECK(points.size());
  CHECK(points[0]->sample.size());
  Ndimensions_ = points[0]->sample.size();
}

LocalOutlierFactor::LocalOutlierFactor(
    std::vector<std::pair<double, double>>& xypoints) {
  CHECK(xypoints.size());
  for (auto& p : xypoints) {
    Sample* s = new Sample();
    s->sample = {p.first, p.second};
    points_.emplace_back(s);
  }
  Ndimensions_ = 2;
  newed_ = true;
}

LocalOutlierFactor::LocalOutlierFactor(
    std::vector<std::vector<double>>& points) {
  CHECK(points.size());
  CHECK(points[0].size());
  for (auto& p : points) {
    CHECK_EQ(p.size(), points[0].size());
    Sample* s = new Sample();
    s->sample = p;
    points_.emplace_back(s);
  }
  Ndimensions_ = points[0].size();
  newed_ = true;
}

LocalOutlierFactor::~LocalOutlierFactor() {
  if (!newed_) return;
  for (auto& p : points_) {
    delete p;
  }
}

double LocalOutlierFactor::ManhattanDistance(Sample& s1, Sample& s2) {
  CHECK(s1.sample.size());
  CHECK(s2.sample.size());

  double distance = 0.0;
  for (size_t i = 0; i < Ndimensions_; i++) {
    distance += std::abs(s1.sample[i] - s2.sample[i]);
  }
  return distance;
}

double LocalOutlierFactor::K_Distance(int k, Sample* s) {
  std::map<double, std::vector<Sample*>> map_distance;
  for (auto& p : points_) {
    double d = ManhattanDistance(*s, *p);
    if (d == 0) continue;
    map_distance[d].emplace_back(p);
  }

  double kdist;
  for (auto&& iter : map_distance) {
    k -= iter.second.size();
    s->neighbours.insert(s->neighbours.end(), iter.second.begin(),
                         iter.second.end());
    kdist = iter.first;
    if (k <= 0) break;
  }
  s->k_dist = kdist;
  return kdist;
}

double LocalOutlierFactor::ReachableDistance(int k, Sample& s1, Sample& s2) {
  double d = ManhattanDistance(s1, s2);
  return std::max(s2.k_dist, d);
}

double LocalOutlierFactor::LocalReachableDensity(int k, Sample* s) {
  double acc_reachable_dist = 0.0;
  for (auto& p : s->neighbours) {
    acc_reachable_dist += ReachableDistance(k, *p, *s);
  }
  double lrd =
      acc_reachable_dist == 0
          ? 1e9
          : static_cast<double>(s->neighbours.size()) / acc_reachable_dist;
  s->lrd = lrd;
  return lrd;
}

double LocalOutlierFactor::Factor(int k, Sample& s) {
  double s_dist = 0.0;
  for (auto& nei : s.neighbours) {
    s_dist += nei->lrd;
  }
  return s_dist / s.lrd / s.neighbours.size();
}

bool LocalOutlierFactor::GetOutliers(int k, std::vector<int>* outlier_indeces) {
  for (size_t i = 0; i < points_.size(); i++) {
    K_Distance(k, points_[i]);
    LocalReachableDensity(k, points_[i]);
  }

  for (size_t i = 0; i < points_.size(); i++) {
    double v = Factor(k, *points_[i]);
    if (v > FLAGS_v_LOF) {
      outlier_indeces->push_back(i);
    }
  }
  return true;
}