#ifndef BEAM_MODEL_H_
#define BEAM_MODEL_H_

#include <eigen_conversions/eigen_msg.h>
#include <geometry_msgs/Transform.h>
#include <vector>

class BeamModel
{
public:
  BeamModel();

  BeamModel(double w, unsigned int num_beams);

  void initialize(double w, unsigned int num_beams);

  inline int CalculateBeam(double x, double depth) const
  {
    return (fx_ * x) / depth + half_num_beams_;
  }

  inline Eigen::Vector2d CalculatePoint(int i, double depth) const
  {
    return rays_[i] * depth;
  }

  inline void CalculatePoints(const std::vector<double>& ranges, std::vector<Eigen::Vector2d>& points)
  {
    static double nan = 0.0 / 0.0;
    points.resize(ranges.size());
    for (unsigned int i = 0; i < points.size(); ++i)
    {
      if (ranges[i] <= 0)
        points[i] = Eigen::Vector2d(nan, nan);
      else
        points[i] = CalculatePoint(i, ranges[i]);
    }
  }

  void RenderModel(const std::vector<std::vector<Eigen::Vector2d> >& contours, const geometry_msgs::Transform& pose,
                   int identifier, std::vector<double>& ranges, std::vector<int>& identifiers) const;

  inline unsigned int num_beams() const
  {
    return rays_.size();
  }
  inline float fx() const
  {
    return fx_;
  }

  inline const std::vector<Eigen::Vector2d>& rays() const
  {
    return rays_;
  }

private:
  double fx_;
  unsigned int half_num_beams_;
  std::vector<Eigen::Vector2d> rays_;
};

#endif
