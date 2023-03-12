#ifndef LOCAL_OUTLIER_FACTOR_H_
#define LOCAL_OUTLIER_FACTOR_H_

#include <math.h>

#include <map>
#include <vector>

struct Sample {
  std::vector<double> sample;
  std::vector<Sample*> neighbours;
  double k_dist;
  double lrd;
};

class LocalOutlierFactor {
 public:
  LocalOutlierFactor() = default;
  LocalOutlierFactor(std::vector<std::vector<double>>& points);
  LocalOutlierFactor(std::vector<std::pair<double, double>>& xypoints);
  LocalOutlierFactor(std::vector<Sample*>& points);
  ~LocalOutlierFactor();

  /**
   * @brief Get the indeces of Outliers
   * @param k number of neighbors
   * @param indeces a pointer to indeces of Outliers
   * @return success or not
   */
  bool GetOutliers(int k, std::vector<int>* indeces);

 private:
  double ManhattanDistance(Sample& s1, Sample& s2);

  double K_Distance(int k, Sample* s);

  double ReachableDistance(int k, Sample& s1, Sample& s2);

  double LocalReachableDensity(int k, Sample* s);

  double Factor(int k, Sample& s);

 private:
  std::vector<Sample*> points_;
  size_t Ndimensions_;
  bool newed_ = false;
};

#endif  // LOCAL_OUTLIER_FACTOR_H_