#include "planning_scene_fitter/beam_model.hpp"

BeamModel::BeamModel() : half_num_beams_(0)
{
}

BeamModel::BeamModel(double w, unsigned int num_beams)
{
  initialize(w, num_beams);
}

void BeamModel::initialize(double w, unsigned int num_beams)
{
  half_num_beams_ = num_beams / 2;
  fx_ = 2 * num_beams / w;

  rays_.resize(num_beams);
  for (std::size_t i = 0; i < num_beams; ++i)
  {
    rays_[i] = Eigen::Vector2d(static_cast<double>(i - half_num_beams_) / fx_, 1.0);
  }
}

void BeamModel::renderModel(const std::vector<std::vector<Eigen::Vector2d> >& contours, const Eigen::Affine2d& pose,
                            int identifier, std::vector<double>& ranges, std::vector<int>& identifiers) const
{
  double near_plane = 0.01;

  Eigen::Vector2d p1_temp, p2_temp;

  for (const auto& model : contours)
  {
    std::vector<Eigen::Vector2d> t_vertices(model.size());
    std::transform(model.begin(), model.end(), t_vertices.begin(),
                   [&](const Eigen::Vector2d& vertex) { return pose * vertex; });

    int nbeams = numBeams();
    for (std::size_t i = 0; i < model.size(); i++)
    {
      std::size_t j = (i + 1) % model.size();
      const Eigen::Vector2d* p1 = &t_vertices[i];
      const Eigen::Vector2d* p2 = &t_vertices[j];

      // If p1 is behind the near plane, clip it
      if (p1->y() < near_plane)
      {
        // if p2 is also behind the near plane, skip the line
        if (p2->y() < near_plane)
          continue;

        double r = (near_plane - p1->y()) / (p2->y() - p1->y());
        p1_temp.x() = p1->x() + r * (p2->x() - p1->x());
        p1_temp.y() = near_plane;
        p1 = &p1_temp;
      }

      // If p2 is behind the near plane, clip it
      if (p2->y() < near_plane)
      {
        double r = (near_plane - p2->y()) / (p1->y() - p2->y());
        p2_temp.x() = p2->x() + r * (p1->x() - p2->x());
        p2_temp.y() = near_plane;
        p2 = &p2_temp;
      }

      // Calculate the beam numbers corresponding to p1 and p2
      int i1 = calculateBeam(p1->x(), p1->y()) + 1;
      int i2 = calculateBeam(p2->x(), p2->y());

      // If i2 < i1, we are looking at the back face of the line, so skip it (back face culling)
      // If i2 < 0 or i1 >= nbeams, the whole line is out of view, so skip it
      if (i2 < i1 || i2 < 0 || i1 >= nbeams)
        continue;

      // Clip i1 and i2 to be between 0 and nbeams
      i1 = std::max(0, i1);
      i2 = std::min(i2, nbeams - 1);

      Eigen::Vector2d s = *p2 - *p1;
      double t = p1->x() * s.y() - p1->y() * s.x();

      for (std::size_t i_beam = i1; i_beam <= i2; i_beam++)
      {
        const Eigen::Vector2d& r = rays_[i_beam];

        // calculate depth of intersection between line (p1, p2) and r
        double d = t / (r.x() * s.y() - r.y() * s.x());

        double& depth_old = ranges[i_beam];
        if (d > 0 && (d < depth_old || depth_old == 0))
        {
          depth_old = d;
          identifiers[i_beam] = identifier;
        }
      }
    }
  }
}
