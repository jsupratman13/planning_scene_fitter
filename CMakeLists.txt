cmake_minimum_required(VERSION 3.0.2)
project(planning_scene_fitter)

add_compile_options(-std=c++17)

find_package(
  catkin REQUIRED
  COMPONENTS actionlib
             moveit_ros_planning_interface
             pcl_ros
             geometry_msgs
             roscpp
             tf2_sensor_msgs
             tf2_ros
             tf2_geometry_msgs
             cv_bridge
             eigen_conversions
             open3d_conversions
             ndt_omp
)

find_package(OpenCV REQUIRED)
find_package(Open3D REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
    actionlib
    moveit_ros_planning_interface
    pcl_ros
    geometry_msgs
    roscpp
    tf2_ros
    tf2_geometry_msgs
    cv_bridge
    eigen_conversions
    open3d_conversions
    ndt_omp
  DEPENDS OpenCV Open3D
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Open3D_LIBRARY_DIRS}
)

add_executable(${PROJECT_NAME} src/main.cpp)
target_link_libraries(
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${Open3D_LIBRARIES}
)
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})

install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)
