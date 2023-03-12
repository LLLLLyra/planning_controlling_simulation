#ifndef _ST_POINT_H_
#define _ST_POINT_H_

#include <string>

#include "common/math/vec2d.h"


class STPoint : public Vec2d {
  // x-axis: t; y-axis: s.
 public:
  STPoint() = default;
  STPoint(const double s, const double t);
  explicit STPoint(const Vec2d& vec2d_point);

  double x() const = delete;
  double y() const = delete;

  double s() const;
  double t() const;
  void set_s(const double s);
  void set_t(const double t);
};


#endif
