#include "clothoid_curve_g1.h"

#include <cmath>

#include "glog/logging.h"
#include "integral.h"
#include "math_utils.h"

double ClothoidCurveG1::X(double a, double b, double c) {
  std::vector<double> xs;
  size_t steps = 1001;
  double dx = 1.0 / steps;
  for (double x = 0; x <= 1; x += dx) {
    xs.push_back(std::cos((a / 2.0) * x * x + b * x + c));
  }
  return IntegrateBySimpson(xs, dx, steps);
}

double ClothoidCurveG1::Y(double a, double b, double c) {
  std::vector<double> ys;
  size_t steps = 1001;
  double dy = 1.0 / steps;
  for (double y = 0; y <= 1; y += dy) {
    ys.push_back(std::sin((a / 2.0) * y * y + b * y + c));
  }
  return IntegrateBySimpson(ys, dy, steps);
}

double ClothoidCurveG1::ComputePathLength(double r, double theta1, double delta,
                                          double A) {
  return r / X(2.0 * A, delta - A, theta1);
}

double ClothoidCurveG1::ComputeCurvature(double delta, double A, double L) {
  return (delta - A) / L;
}

double ClothoidCurveG1::ComputeCurvatureRate(double A, double L) {
  return 2.0 * A / (L * L);
}

double ClothoidCurveG1::SolveGforRoot(double theta1, double theta2,
                                      double delta) {
  auto f = [&](double A) -> double { return Y(2.0 * A, delta - A, theta1); };

  double init = 3.0 * (theta1 + theta2);
  return NewtonRaphson(f, init, 1e-10, 2000);
}

void ClothoidCurveG1::operator()(std::array<double, 3>& start,
                                 std::array<double, 3>& end, size_t num,
                                 std::vector<double>* xpoints,
                                 std::vector<double>* ypoints) {
  CHECK_NOTNULL(xpoints);
  CHECK_NOTNULL(ypoints);

  double dx = end[0] - start[0];
  double dy = end[1] - start[1];
  double r = hypot(dx, dy);

  double phi = std::atan2(dy, dx);
  double phi1 = NormalizeAngle(start[2] - phi);
  double phi2 = NormalizeAngle(end[2] - phi);
  double delta = phi2 - phi1;

  double A = SolveGforRoot(phi1, phi2, delta);
  double L = ComputePathLength(r, phi1, delta, A);
  double curvature = ComputeCurvature(delta, A, L);
  double curvature_rate = ComputeCurvatureRate(A, L);

  for (double s = 0; s < L; s += L / num) {
    double x =
        start[0] + s * X(curvature_rate * s * s, curvature * s, start[2]);
    double y =
        start[1] + s * Y(curvature_rate * s * s, curvature * s, start[2]);
    xpoints->push_back(x);
    ypoints->push_back(y);
  }
}

std::vector<std::pair<double, double>> ClothoidCurveG1::operator()(
    std::array<double, 3>& start, std::array<double, 3>& end, size_t num) {
  std::vector<double> x, y;
  std::vector<std::pair<double, double>> xypoints;
  operator()(start, end, num, &x, &y);
  for (size_t i = 0; i < x.size(); i++) {
    xypoints.push_back({x[i], y[i]});
  }
  return xypoints;
}