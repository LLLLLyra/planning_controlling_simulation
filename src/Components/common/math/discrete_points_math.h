#ifndef _DISCRETE_POINTS_H_
#define _DISCRETE_POINTS_H_

#include <utility>
#include <vector>

class DiscretePointsMath {
 public:
  DiscretePointsMath() = delete;

  // @brief for known headings
  static bool ComputePathProfile(
      const std::vector<std::pair<double, double>>& xy_points,
      std::vector<double>* accumulated_s, std::vector<double>* kappas,
      std::vector<double>* dkappas);

  // @brief for unknown headings
  static bool ComputePathProfile(
      const std::vector<std::pair<double, double>>& xy_points,
      std::vector<double>* headings, std::vector<double>* accumulated_s,
      std::vector<double>* kappas, std::vector<double>* dkappas);
};

#endif