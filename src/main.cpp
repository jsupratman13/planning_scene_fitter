#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/registration/ndt.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <eigen_conversions/eigen_msg.h>
#include <geometry_msgs/Pose.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <iostream>
#include <geometry_msgs/TransformStamped.h>
#include <ros/service.h>
#include <std_srvs/Empty.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <Eigen/Geometry>
#include "Eigen/src/Geometry/Transform.h"
#include <open3d/Open3D.h>
#include <open3d_conversions/open3d_conversions.h>
#include <open3d/geometry/TriangleMesh.h>
#include <pclomp/ndt_omp.h>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloudXYZ;

class PlanningSceneFitter
{
public:
  PlanningSceneFitter(ros::NodeHandle& nh) : tf_listener_(tf_buffer_)
  {
    nh_ = nh;
    sub_ = nh_.subscribe("input_cloud", 1, &PlanningSceneFitter::cloudCallback, this);
    server_ = nh_.advertiseService("service", &PlanningSceneFitter::serviceCallback, this);
    pub_ = nh.advertise<sensor_msgs::PointCloud2>("output_cloud", 1);
    std::string filename;
    nh_.param<std::string>("filename", filename, "mesh.stl");
    auto mesh = open3d::io::CreateMeshFromFile(filename);
    if (!mesh)
    {
      ROS_WARN_STREAM("Could not load mesh file");
    }
    // auto mesh_point_cloud = mesh->SamplePointsPoissonDisk(10000);
    // mesh = open3d::geometry::TriangleMesh::CreateBox(1.0, 1.0, 1.0);
    auto mesh_point_cloud = mesh->SamplePointsUniformly(10000);
    open3d_conversions::open3dToRos(*mesh_point_cloud, mesh_point_cloud_msg_, "map");
  }

private:
  ros::NodeHandle nh_;
  ros::Subscriber sub_;
  ros::Publisher pub_;
  PointCloudXYZ::Ptr input_cloud_;
  PointCloudXYZ::Ptr target_cloud_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  ros::ServiceServer server_;
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;
  sensor_msgs::PointCloud2 mesh_point_cloud_msg_;

  void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
  {
    sensor_msgs::PointCloud2 cloud_msg_transformed;
    try
    {
      {
        geometry_msgs::TransformStamped transform =
            tf_buffer_.lookupTransform("map", cloud_msg->header.frame_id, cloud_msg->header.stamp);
        tf2::doTransform(*cloud_msg, cloud_msg_transformed, transform);
      }
    }
    catch (tf2::TransformException& ex)
    {
      ROS_WARN_STREAM("Could not transform point cloud: " << ex.what());
      return;
    }
    target_cloud_ = PointCloudXYZ::Ptr(new PointCloudXYZ);
    pcl::fromROSMsg(cloud_msg_transformed, *target_cloud_);
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(target_cloud_);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(0.05, FLT_MAX);
    pass.filter(*target_cloud_);
    // pcl::ApproximateVoxelGrid<pcl::PointXYZ> approximate_voxel_filter;
    // approximate_voxel_filter.setLeafSize(0.1, 0.1, 0.1);
    // approximate_voxel_filter.setInputCloud(target_cloud_);
    // approximate_voxel_filter.filter(*target_cloud_);
  }

  bool serviceCallback(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
  {
    auto co_vec = planning_scene_interface_.getObjects({ "shelf" });
    if (co_vec.empty())
    {
      ROS_WARN_STREAM("Could not find shelf in planning scene");
      return false;
    }
    geometry_msgs::Pose shelf_pose = co_vec["shelf"].pose;
    Eigen::Affine3d shelf_mat;
    tf::poseMsgToEigen(shelf_pose, shelf_mat);
    // shelf_mat = shelf_mat * Eigen::Translation3d(-0.5, -0.5, -0.5);
    input_cloud_ = PointCloudXYZ::Ptr(new PointCloudXYZ);
    pcl::fromROSMsg(mesh_point_cloud_msg_, *input_cloud_);
    pcl::transformPointCloud(*input_cloud_, *input_cloud_, shelf_mat.matrix().cast<float>());
    // pcl::ApproximateVoxelGrid<pcl::PointXYZ> approximate_voxel_filter;
    // approximate_voxel_filter.setLeafSize(0.2, 0.2, 0.2);
    // approximate_voxel_filter.setInputCloud(input_cloud_);
    // approximate_voxel_filter.filter(*input_cloud_);
    sensor_msgs::PointCloud2Ptr input_msg(new sensor_msgs::PointCloud2);
    pcl::toROSMsg(*input_cloud_, *input_msg);
    input_msg->header.frame_id = "map";
    pub_.publish(*input_msg);

    pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt;
    ndt.setTransformationEpsilon(0.01);
    ndt.setStepSize(0.1);
    ndt.setResolution(1.0);
    ndt.setMaximumIterations(35);
    ndt.setInputSource(target_cloud_);
    ndt.setInputTarget(input_cloud_);

    pclomp::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt2;
    ndt2.setResolution(1.0);
    ndt2.setStepSize(0.1);
    ndt2.setNeighborhoodSearchMethod(pclomp::DIRECT7);
    ndt2.setInputSource(target_cloud_);
    ndt2.setInputTarget(input_cloud_);

    Eigen::Matrix4f initial_guess = shelf_mat.matrix().cast<float>();
    pcl::PointCloud<pcl::PointXYZ> output_cloud;
    // ndt.align(output_cloud, initial_guess);
    // ndt2.align(output_cloud, initial_guess);
    ndt2.align(output_cloud);

    auto diff_x = ndt2.getFinalTransformation()(0, 3);
    auto diff_y = ndt2.getFinalTransformation()(1, 3);
    auto diff_yaw = std::atan2(ndt2.getFinalTransformation()(1, 0), ndt2.getFinalTransformation()(0, 0));
    Eigen::Matrix4f diff_mat = Eigen::Matrix4f::Identity();
    diff_mat(0, 3) = diff_x;
    diff_mat(1, 3) = diff_y;
    diff_mat.block<3, 3>(0, 0) = Eigen::AngleAxisf(diff_yaw, Eigen::Vector3f::UnitZ()).toRotationMatrix();
    pcl::transformPointCloud(*target_cloud_, output_cloud, diff_mat);

    auto source = std::make_shared<open3d::geometry::PointCloud>();
    auto target = std::make_shared<open3d::geometry::PointCloud>();
    auto result = std::make_shared<open3d::geometry::PointCloud>();
    sensor_msgs::PointCloud2Ptr source_msg(new sensor_msgs::PointCloud2);
    sensor_msgs::PointCloud2Ptr target_msg(new sensor_msgs::PointCloud2);
    sensor_msgs::PointCloud2Ptr result_msg(new sensor_msgs::PointCloud2);
    pcl::toROSMsg(*ndt.getInputSource(), *source_msg);
    pcl::toROSMsg(*ndt.getInputTarget(), *target_msg);
    pcl::toROSMsg(output_cloud, *result_msg);
    open3d_conversions::rosToOpen3d(source_msg, *source);
    open3d_conversions::rosToOpen3d(target_msg, *target);
    open3d_conversions::rosToOpen3d(result_msg, *result);
    source->PaintUniformColor(Eigen::Vector3d(1, 0, 0));
    target->PaintUniformColor(Eigen::Vector3d(0, 1, 0));
    result->PaintUniformColor(Eigen::Vector3d(0, 0, 1));

    open3d::visualization::DrawGeometries({ source, target, result });

    if (ndt.hasConverged())
    {
      ROS_INFO_STREAM("NDT converged, score: " << ndt.getFitnessScore());
      auto transformation = ndt.getFinalTransformation();
      Eigen::Affine3d diff_mat = Eigen::Affine3d(transformation.cast<double>());
      geometry_msgs::Pose pose;
      tf::poseEigenToMsg(shelf_mat * diff_mat, pose);
      std::cout << "Pose: " << pose << std::endl;
      auto fit_co = co_vec["shelf"];
      fit_co.pose = pose;
      fit_co.id = "shelf2";
      planning_scene_interface_.addCollisionObjects({ fit_co });
    }
    else
    {
      ROS_WARN_STREAM("NDT did not converge");
      return false;
    }
    return true;
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "planning_scene_fitter");
  ros::NodeHandle nh;
  PlanningSceneFitter fitter(nh);
  ros::spin();
  return 0;
}
