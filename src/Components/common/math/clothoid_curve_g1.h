#ifndef CLOTHOID_CURVE_G1_H_
#define CLOTHOID_CURVE_G1_H_

#include <array>
#include <vector>

/**
 * @brief a class to generate G1 Hermite interpolation with a single clothoid
 * segment.
 * @brief Reference:
 * https://atsushisakai.github.io/PythonRobotics/modules/path_planning/clothoid_path/clothoid_path.html
 *
 */
class ClothoidCurveG1 {
 public:
  ClothoidCurveG1() = default;
  ~ClothoidCurveG1() = default;

  /**
   * @brief overload operator()
   *
   * @param start start state [x, y, phi]
   * @param end  end state [x, y, phi]
   * @param num number of waypoints
   * @return a pair of xypoints
   */
  std::vector<std::pair<double, double>> operator()(
      std::array<double, 3>& start, std::array<double, 3>& end, size_t num);

  /**
   * @brief overload operator()
   *
   * @param start start state [x, y, phi]
   * @param end end state [x, y, phi]
   * @param num number of waypoints
   * @param xpoints
   * @param ypoints
   */
  void operator()(std::array<double, 3>& start, std::array<double, 3>& end,
                  size_t num, std::vector<double>* xpoints,
                  std::vector<double>* ypoints);

 protected:
  // Fresnel integrals
  double X(double a, double b, double c);
  // Fresnel integrals
  double Y(double a, double b, double c);
  double ComputePathLength(double r, double theta1, double delta, double A);
  double ComputeCurvature(double delta, double A, double L);
  double ComputeCurvatureRate(double A, double L);
  double SolveGforRoot(double theta1, double theta2, double delta);
};

#endif  // CLOTHOID_CURVE_G1_H_