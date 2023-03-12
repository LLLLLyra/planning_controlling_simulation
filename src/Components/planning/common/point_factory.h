#ifndef _POINT_FACTORY_H_
#define _POINT_FACTORY_H_

#include "common/math/vec2d.h"
#include "common/proto/pnc_point.pb.h"

class PointFactory {
 public:
  template <typename XY>
  static inline Vec2d ToVec2d(const XY& xy) {
    return Vec2d(xy.x(), xy.y());
  }

  static inline points::SpeedPoint ToSpeedPoint(const double s, const double t,
                                                const double v = 0,
                                                const double a = 0,
                                                const double da = 0) {
    points::SpeedPoint speed_point;
    speed_point.set_s(s);
    speed_point.set_t(t);
    speed_point.set_v(v);
    speed_point.set_a(a);
    speed_point.set_da(da);
    return speed_point;
  }

  static inline points::PathPoint ToPathPoint(
      const double x, const double y, const double z = 0, const double s = 0,
      const double theta = 0, const double kappa = 0, const double dkappa = 0,
      const double ddkappa = 0) {
    points::PathPoint path_point;
    path_point.set_x(x);
    path_point.set_y(y);
    path_point.set_z(z);
    path_point.set_s(s);
    path_point.set_theta(theta);
    path_point.set_kappa(kappa);
    path_point.set_dkappa(dkappa);
    path_point.set_ddkappa(ddkappa);
    return path_point;
  }
};

#endif  // _POINT_FACTORY_H_