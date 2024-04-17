#include "planning_scene_fitter/beam_model.h"

// ----------------------------------------------------------------------------------------------------

BeamModel::BeamModel() : half_num_beams_(0)
{
}

// ----------------------------------------------------------------------------------------------------

BeamModel::BeamModel(double w, unsigned int num_beams)
{
  initialize(w, num_beams);
}

// ----------------------------------------------------------------------------------------------------

void BeamModel::initialize(double w, unsigned int num_beams)
{
  half_num_beams_ = num_beams / 2;
  fx_ = 2 * num_beams / w;

  rays_.resize(num_beams);
  for (unsigned int i = 0; i < num_beams; ++i)
    rays_[i] = Eigen::Vector2d(((double)(i)-half_num_beams_) / fx_, 1);
}

// ----------------------------------------------------------------------------------------------------

void BeamModel::RenderModel(const std::vector<std::vector<Eigen::Vector2d> >& contours,
                            const geometry_msgs::Transform& pose, int identifier, std::vector<double>& ranges,
                            std::vector<int>& identifiers) const
{
  double near_plane = 0.01;

  Eigen::Vector2d p1_temp, p2_temp;

  for (std::vector<std::vector<Eigen::Vector2d> >::const_iterator it_contour = contours.begin();
       it_contour != contours.end(); ++it_contour)
  {
    const std::vector<Eigen::Vector2d>& model = *it_contour;

    std::vector<Eigen::Vector2d> t_vertices(model.size());
    for (unsigned int i = 0; i < model.size(); ++i)
    {
      Eigen::Affine3d p;
      tf::transformMsgToEigen(pose, p);
      t_vertices[i] = p * model[i];
    }

    int nbeams = num_beams();

    for (unsigned int i = 0; i < model.size(); ++i)
    {
      unsigned int j = (i + 1) % model.size();
      const Eigen::Vector2d* p1 = &t_vertices[i];
      const Eigen::Vector2d* p2 = &t_vertices[j];

      // If p1 is behind the near plane, clip it
      if ((*p1)[1] < near_plane)
      {
        // if p2 is also behind the near plane, skip the line
        if ((*p2)[1] < near_plane)
          continue;

        double r = (near_plane - (*p1)[1]) / ((*p2)[1] - (*p1)[1]);
        p1_temp(0) = (*p1)[0] + r * ((*p2)[0] - (*p1)[0]);
        p1_temp(1) = near_plane;
        p1 = &p1_temp;
      }

      // If p2 is behind the near plane, clip it
      if ((*p2)[1] < near_plane)
      {
        double r = (near_plane - (*p2)[1]) / ((*p1)[1] - (*p2)[1]);
        p2_temp(0) = (*p2)[0] + r * ((*p1)[0] - (*p2)[0]);
        p2_temp(1) = near_plane;
        p2 = &p2_temp;
      }

      // Calculate the beam numbers corresponding to p1 and p2
      int i1 = CalculateBeam((*p1)[0], (*p1)[1]) + 1;
      int i2 = CalculateBeam((*p2)[0], (*p2)[1]);

      // If i2 < i1, we are looking at the back face of the line, so skip it (back face culling)
      // If i2 < 0 or i1 >= nbeams, the whole line is out of view, so skip it
      if (i2 < i1 || i2 < 0 || i1 >= nbeams)
        continue;

      // Clip i1 and i2 to be between 0 and nbeams
      i1 = std::max(0, i1);
      i2 = std::min(i2, nbeams - 1);

      Eigen::Vector2d s = *p2 - *p1;
      double t = (*p1)[0] * s(1) - (*p1)[1] * s(0);

      for (int i_beam = i1; i_beam <= i2; ++i_beam)
      {
        const Eigen::Vector2d& r = rays_[i_beam];

        // calculate depth of intersection between line (p1, p2) and r
        double d = t / (r(0) * s(1) - r(1) * s(0));

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
