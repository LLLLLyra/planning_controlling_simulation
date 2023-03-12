#include "search.h"

#include <cmath>

double GoldenSectionSearch(const std::function<double(double)> &func,
                           const double lower_bound, const double upper_bound,
                           const double tol) {
  static constexpr double gr = 1.618033989;  // (sqrt(5) + 1) / 2

  double a = lower_bound;
  double b = upper_bound;

  double t = (b - a) / gr;
  double c = b - t;
  double d = a + t;

  while (std::abs(c - d) > tol) {
    if (func(c) < func(d)) {
      b = d;
    } else {
      a = c;
    }
    t = (b - a) / gr;
    c = b - t;
    d = a + t;
  }
  return (a + b) * 0.5;
}
