#include <ros/ros.h>
#include <ros/service_server.h>
#include <sensor_msgs/Image.h>
#include <std_srvs/Empty.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_scene/planning_scene.h>
#include <rgbd/view.h>
#include <rgbd/image_buffer/image_buffer.h>

#include <memory>
#include "geolib/datatypes.h"
#include "planning_scene_fitter/fitter.h"
#include "rgbd/types.h"

class PlanningSceneFitter
{
public:
  PlanningSceneFitter(ros::NodeHandle& nh)
  {
    nh_ = nh;
    sub_ = nh_.subscribe("image", 1, &PlanningSceneFitter::imageCallback, this);
    server_ = nh_.advertiseService("service", &PlanningSceneFitter::serviceCallback, this);
    robot_model_loader_ = robot_model_loader::RobotModelLoader("robot_description");
    auto robot_model = robot_model_loader_.getModel();
    planning_scene_ = std::make_shared<planning_scene::PlanningScene>(robot_model);
    std::string topic_name;
    nh_.getParam("topic_name", topic_name);
    image_buffer_.initialize(topic_name, "map");
  }

private:
  void imageCallback(const sensor_msgs::ImageConstPtr& msg)
  {
    ROS_INFO("Received image");
  }

  bool serviceCallback(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
  {
    ROS_INFO("Received service call");
    rgbd::ImageConstPtr image;
    geo::Pose3D sensor_pose;
    if (!image_buffer_.waitForRecentImage(image, sensor_pose, 2.0))
    {
      ROS_ERROR("Could not get image");
      return false;
    }
    if (!fitter_.isConfigured())
    {
      fitter_.configureBeamModel(image->getCameraModel());
    }
    FitterData fitter_data;
    fitter_.processSensorData(*image, sensor_pose, fitter_data);

    auto box_1 = planning_scene_->getWorld()->getObject("Box_1");
    geo::Vec3d translation(box_1->pose_.translation().x(), box_1->pose_.translation().y(),
                           box_1->pose_.translation().z());
    geo::Mat3d rotation;
    for (int i = 0; i < 3; i++)
    {
      for (int j = 0; j < 3; j++)
      {
        rotation(i, j) = box_1->pose_.rotation()(i, j);
      }
    }
    geo::Pose3D expected_pose(rotation, translation);

    double max_yaw_change = M_PI / 4;
    geo::Pose3D new_pose;
    if (!fitter_.estimateEntityPose(fitter_data, box_1, expected_pose, new_pose, max_yaw_change))
    {
      ROS_ERROR("Could not estimate entity pose");
      return false;
    }
    return true;
  }

  ros::NodeHandle nh_;
  ros::Subscriber sub_;
  ros::ServiceServer server_;
  robot_model_loader::RobotModelLoader robot_model_loader_;
  planning_scene::PlanningScenePtr planning_scene_;
  Fitter fitter_;
  rgbd::ImageBuffer image_buffer_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "planning_scene_fitter");
  ros::NodeHandle nh;

  PlanningSceneFitter fitter(nh);
  ros::spin();
  return 0;
}
