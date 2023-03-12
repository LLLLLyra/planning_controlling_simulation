#include "st_point.h"


STPoint::STPoint(const double s, const double t) : Vec2d(t, s) {}

STPoint::STPoint(const Vec2d& vec2d_point) : Vec2d(vec2d_point) {}

double STPoint::s() const { return y_; }

double STPoint::t() const { return x_; }

void STPoint::set_s(const double s) { y_ = s; }

void STPoint::set_t(const double t) { x_ = t; }


