#ifndef _SPEED_DATE_H_
#define _SPEED_DATE_H_

#include <string>
#include <vector>

#include "common/config/flags.h"
#include "common/proto/pnc_point.pb.h"

#define UNIQUE_LOCK_MULTITHREAD(mutex_type)                         \
  std::unique_ptr<std::unique_lock<std::mutex>> lock_ptr = nullptr; \
  if (FLAGS_multithread_run) {                                      \
    lock_ptr.reset(new std::unique_lock<std::mutex>(mutex_type));   \
  }

class SpeedData : public std::vector<points::SpeedPoint> {
 public:
  SpeedData() = default;

  virtual ~SpeedData() = default;

  explicit SpeedData(std::vector<points::SpeedPoint> speed_points);

  void AppendSpeedPoint(const double s, const double time, const double v,
                        const double a, const double da);

  bool EvaluateByTime(const double time,
                      points::SpeedPoint* const speed_point) const;

  // Assuming spatial traversed distance is monotonous, which is the case for
  // current usage on city driving scenario
  bool EvaluateByS(const double s, points::SpeedPoint* const speed_point) const;

  double TotalTime() const;

  // Assuming spatial traversed distance is monotonous
  double TotalLength() const;

  virtual std::string DebugString() const;
};

#endif  // _SPEED_DATE_H_
