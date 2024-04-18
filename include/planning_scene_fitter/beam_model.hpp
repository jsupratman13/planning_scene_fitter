#ifndef BEAM_MODEL_HPP_
#define BEAM_MODEL_HPP_

#include <eigen_conversions/eigen_msg.h>
#include <limits>
#include <vector>

class BeamModel
{
public:
  BeamModel();

  BeamModel(double w, unsigned int num_beams);

  void initialize(double w, unsigned int num_beams);

  inline int calculateBeam(double x, double depth) const
  {
    return static_cast<int>((fx_ * x) / depth + half_num_beams_);
  }

  inline Eigen::Vector2d calculatePoint(int i, double depth) const
  {
    return rays_[i] * depth;
  }

  inline void calculatePoints(const std::vector<double>& ranges, std::vector<Eigen::Vector2d>& points)
  {
    points.resize(ranges.size());
    for (std::size_t i = 0; i < points.size(); ++i)
    {
      if (ranges[i] <= 0)
        points[i] = Eigen::Vector2d(std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::quiet_NaN());
      else
        points[i] = calculatePoint(i, ranges[i]);
    }
  }

  void renderModel(const std::vector<std::vector<Eigen::Vector2d> >& contours, const Eigen::Affine2d& pose,
                   int identifier, std::vector<double>& ranges, std::vector<int>& identifiers) const;

  inline unsigned int numBeams() const
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
