#ifndef BSPLINE_H_
#define BSPLINE_H_

#include <vector>

/**
 * @brief B-Spline interpolater
 *
 * @tparam U double | int
 * @tparam V double | int
 */
template <typename U, typename V>
class BSpline {
 public:
  /**
   * @brief Construct a new BSpline object
   *
   * @param t knots
   * @param c spline coefficients
   * @param k B-spline degree
   */
  BSpline(std::vector<U>& t, std::vector<V>& c, int k);

  /**
   * @brief interpolated value of x
   *
   * @param x the interpolation location
   * @return double B-spline value
   */
  double bspline(double x);
  /**
   * @brief interpolated values of xs
   *
   * @param xs the interpolation location
   * @param vals B-spline values
   */
  void bspline(std::vector<double>& xs, std::vector<double>* vals);

 private:
  double B(double x, int k, int i);

 private:
  std::vector<U> t_;
  std::vector<V> c_;
  int k_;
};

#endif  // BSPLINE_H_