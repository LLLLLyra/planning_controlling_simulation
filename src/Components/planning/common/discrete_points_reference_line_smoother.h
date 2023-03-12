#ifndef _DISCRETE_POINTS_REFERENCE_LINE_SMOOTHER_H_
#define _DISCRETE_POINTS_REFERENCE_LINE_SMOOTHER_H_

#include <utility>
#include <vector>

#include "reference_point.h"

struct AnchorPoint {
  points::PathPoint path_point;
  double lateral_bound = 1.0;
  double longitudinal_bound = 1.0;
  // enforce smoother to strictly follow this reference point
  bool enforced = false;
};

class ReferenceLineSmoother {
 public:
  explicit ReferenceLineSmoother() {}

  /**
   * Smoothing constraints
   */
  virtual void SetAnchorPoints(
      const std::vector<AnchorPoint>& achor_points) = 0;

  /**
   * Smooth a given reference line consisting of reference points
   */
  virtual bool Smooth(std::vector<ReferencePoint>* ref_points) = 0;

  virtual ~ReferenceLineSmoother() = default;
};

class DiscretePointsReferenceLineSmoother : public ReferenceLineSmoother {
 public:
  explicit DiscretePointsReferenceLineSmoother() = default;

  virtual ~DiscretePointsReferenceLineSmoother() = default;

  bool Smooth(std::vector<ReferencePoint>* ref_points) override;

  void SetAnchorPoints(const std::vector<AnchorPoint>&) override;

 private:
  bool FemPosSmooth(
      const std::vector<std::pair<double, double>>& raw_point2d,
      const std::vector<double>& bounds,
      std::vector<std::pair<double, double>>* ptr_smoothed_point2d);

  void NormalizePoints(std::vector<std::pair<double, double>>* xy_points);

  void DeNormalizePoints(std::vector<std::pair<double, double>>* xy_points);

  bool GenerateRefPointProfile(
      const std::vector<std::pair<double, double>>& xy_points,
      std::vector<ReferencePoint>* reference_points);

  std::vector<AnchorPoint> anchor_points_;

  double zero_x_ = 0.0;

  double zero_y_ = 0.0;
};

#endif