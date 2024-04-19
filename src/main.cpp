#include <ros/ros.h>
#include <ros/service_server.h>
#include <sensor_msgs/Image.h>
#include <std_srvs/Empty.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_scene/planning_scene.h>
#include <rgbd/view.h>
#include "planning_scene_fitter/fitter.h"

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
    planning_scene_ = planning_scene::PlanningScenePtr(new planning_scene::PlanningScene(robot_model));
  }

private:
  void imageCallback(const sensor_msgs::ImageConstPtr& msg)
  {
    ROS_INFO("Received image");
  }
  bool serviceCallback(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
  {
    ROS_INFO("Received service call");
    if (!fitter_.isConfigured())
    {
      // fitter_.configureBeamModel(image->getCameraModel());
    }
    FitterData fitter_data;
    return true;
  }

  ros::NodeHandle nh_;
  ros::Subscriber sub_;
  ros::ServiceServer server_;
  robot_model_loader::RobotModelLoader robot_model_loader_;
  planning_scene::PlanningScenePtr planning_scene_;
  Fitter fitter_;
  rgbd::ImageConstPtr image_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "planning_scene_fitter");
  ros::NodeHandle nh;

  PlanningSceneFitter fitter(nh);
  ros::spin();
  return 0;
}
