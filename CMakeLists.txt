cmake_minimum_required(VERSION 3.0.2)
project(planning_scene_fitter)

add_compile_options(-std=c++17)

find_package(
  catkin REQUIRED
  COMPONENTS actionlib
             geometry_msgs
             roscpp
             tf2_ros
             tf2_geometry_msgs
             cv_bridge
             eigen_conversions
)

find_package(OpenCV REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
    actionlib
    geometry_msgs
    roscpp
    tf2_ros
    tf2_geometry_msgs
    cv_bridge
    eigen_conversions
  DEPENDS OpenCV
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME} src/beam_model.cpp)
target_link_libraries(
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})

install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

#install(
#  TARGETS point_frame
#  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
#)
