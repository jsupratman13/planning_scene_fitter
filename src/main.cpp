#include <ros/ros.h>
#include <ros/service_server.h>
#include <sensor_msgs/Image.h>
#include <std_srvs/Empty.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <rgbd/view.h>
#include <rgbd/image_buffer/image_buffer.h>

#include <memory>
#include "Eigen/src/Geometry/Quaternion.h"
#include "Eigen/src/Geometry/Transform.h"
#include "eigen_stl_containers/eigen_stl_vector_container.h"
#include "geolib/datatypes.h"
#include "moveit/collision_detection/world.h"
#include "moveit/planning_scene_monitor/planning_scene_monitor.h"
#include "planning_scene_fitter/fitter.h"
#include "rgbd/types.h"

class PlanningSceneFitter
{
public:
  PlanningSceneFitter(ros::NodeHandle& nh)
  {
    nh_ = nh;
    // sub_ = nh_.subscribe("image", 1, &PlanningSceneFitter::imageCallback, this);
    server_ = nh_.advertiseService("service", &PlanningSceneFitter::serviceCallback, this);

    planning_scene_monitor_ = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>("robot_description");
    planning_scene_monitor_->startSceneMonitor();
    planning_scene_monitor_->startStateMonitor();
    planning_scene_monitor_->startWorldGeometryMonitor();
    planning_scene_monitor_->startPublishingPlanningScene(planning_scene_monitor::PlanningSceneMonitor::UPDATE_SCENE,
                                                          "/move_group/monitored_planning_scene");
    ros::WallDuration(1.0).sleep();
    // planning_scene_interface_ = moveit::planning_interface::PlanningSceneInterface();
    std::string topic_name;
    nh_.getParam("topic_name", topic_name);
    std::cout << "topic_name: " << topic_name << std::endl;
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
    if (!image_buffer_.waitForRecentImage(image, sensor_pose, 5.0))
    {
      ROS_ERROR("Could not get image");
      return false;
    }
    ROS_INFO("Got RGBD image");
    if (!fitter_.isConfigured())
    {
      fitter_.configureBeamModel(image->getCameraModel());
    }
    ROS_INFO("configured beam model");
    FitterData fitter_data;
    fitter_.processSensorData(*image, sensor_pose, fitter_data);

    ROS_INFO("processed sensor data");
    // std::vector<std::string> object_ids = { "Box_1" };
    // auto box_1 = planning_scene_interface_.getObjects(object_ids);
    planning_scene_monitor::LockedPlanningSceneRW locked_scene(planning_scene_monitor_);
    planning_scene::PlanningScenePtr planning_scene = locked_scene.operator->();
    collision_detection::WorldPtr world = planning_scene->getWorldNonConst();
    auto box_1 = world->getObject("Box_1");
    for (auto& obj : world->getObjectIds())
    {
      ROS_INFO("object: %s", obj.c_str());
    }
    if (!box_1)
    {
      ROS_ERROR("Could not find object Box_1 in planning scene");
      return false;
    }
    ROS_INFO("found object Box_1 in planning scene");
    geo::Vec3d translation(box_1->pose_.translation().x(), box_1->pose_.translation().y(),
                           box_1->pose_.translation().z());
    geo::Mat3d rotation;
    Eigen::Quaterniond e_q(box_1->pose_.linear());
    geo::Quaternion expected_orientation(e_q.coeffs().x(), e_q.coeffs().y(), e_q.coeffs().z(), e_q.coeffs().w());
    rotation.setRotation(expected_orientation);
    geo::Pose3D expected_pose(rotation, translation);

    double max_yaw_change = M_PI / 4;
    geo::Pose3D new_pose;
    ROS_INFO("estimating entity pose");
    if (!fitter_.estimateEntityPose(fitter_data, box_1, expected_pose, new_pose, max_yaw_change))
    {
      ROS_ERROR("Could not estimate entity pose");
      return false;
    }
    double roll, pitch, yaw;
    expected_pose.getRPY(roll, pitch, yaw);
    std::cout << "expected pose: " << expected_pose.getOrigin() << " " << roll << " " << pitch << " " << yaw
              << std::endl;
    new_pose.getRPY(roll, pitch, yaw);
    std::cout << "new pose: " << new_pose.getOrigin() << " " << roll << " " << pitch << " " << yaw << std::endl;
    Eigen::Vector3d position(new_pose.t.x, new_pose.t.y, new_pose.t.z);
    auto q = new_pose.getQuaternion();
    Eigen::Quaterniond orientation(q.w, q.x, q.y, q.z);
    Eigen::Isometry3d new_pose_eigen = Eigen::Isometry3d::Identity();
    new_pose_eigen.translate(position);
    new_pose_eigen.rotate(orientation);

    world->removeObject("Box_2");
    world->addToObject("Box_2", new_pose_eigen, box_1->shapes_, box_1->shape_poses_);
    planning_scene_monitor_->triggerSceneUpdateEvent(planning_scene_monitor::PlanningSceneMonitor::UPDATE_SCENE);
    return true;
  }

  ros::NodeHandle nh_;
  ros::Subscriber sub_;
  ros::ServiceServer server_;
  robot_model_loader::RobotModelLoader robot_model_loader_;
  planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;
  // moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;
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
