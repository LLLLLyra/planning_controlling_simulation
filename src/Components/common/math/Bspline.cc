#include "Bspline.h"

#include "glog/logging.h"

template <typename U, typename V>
BSpline<U, V>::BSpline(std::vector<U>& t, std::vector<V>& c, int k)
    : t_(t), c_(c), k_(k) {}

template <typename U, typename V>
double BSpline<U, V>::bspline(double x) {
  const int n = t_.size() - k_ - 1;
  CHECK_GE(n, k_ + 1);
  CHECK_GE(c_.size(), n);
  double s = 0.0;
  for (int i = 0; i < n; i++) {
    s += c_[i] * B(x, k_, i);
  }
  return s;
}

template <typename U, typename V>
void BSpline<U, V>::bspline(std::vector<double>& xs,
                            std::vector<double>* vals) {
  CHECK_NOTNULL(vals);
  vals->clear();

  for (auto& x : xs) {
    vals->push_back(bspline(x));
  }
}

template <typename U, typename V>
double BSpline<U, V>::B(double x, int k, int i) {
  if (k == 0) {
    return t_[i] <= x && x < t_[i + 1] ? 1.0 : 0.0;
  }

  double c1, c2;
  if (t_[i + k] == t_[i]) {
    c1 = 0.0;
  } else {
    c1 = (x - t_[i]) / (t_[i + k] - t_[i]) * B(x, k - 1, i);
  }

  if (t_[i + k + 1] == t_[i + 1]) {
    c2 = 0.0;
  } else {
    c2 = (t_[i + k + 1] - x) / (t_[i + k + 1] - t_[i + 1]) * B(x, k - 1, i + 1);
  }
  return c1 + c2;
}

template class BSpline<double, double>;
template class BSpline<double, int>;
template class BSpline<int, int>;
template class BSpline<int, double>;