cmake_minimum_required(VERSION 3.5)
project(drawing)

find_package(catkin REQUIRED
  COMPONENTS
    interactive_markers
    moveit_core
    moveit_ros_planning
    moveit_ros_planning_interface
    moveit_ros_perception
    rviz_visual_tools
    moveit_visual_tools
    pluginlib
    geometric_shapes
    pcl_ros
    pcl_conversions
    rosbag
    tf2_ros
    tf2_eigen
    tf2_geometry_msgs
    actionlib_msgs
    control_msgs
    trac_ik_lib
)

find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED system filesystem date_time thread)

generate_messages(
  DEPENDENCIES actionlib_msgs std_msgs trajectory_msgs control_msgs  # Or other packages containing msgs
)

catkin_package(
  LIBRARIES
  INCLUDE_DIRS
  CATKIN_DEPENDS
    moveit_core
    moveit_visual_tools
    moveit_ros_planning_interface
    interactive_markers
    tf2_geometry_msgs
    actionlib_msgs
    control_msgs
  DEPENDS
    EIGEN3
)

ADD_LIBRARY(drawingInput
  src/drawing_input.cpp
)

include_directories(SYSTEM ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIR} ${EIGEN3_INCLUDE_DIRS} include)

add_executable(drawing_manager src/drawing_manager.cpp)
target_link_libraries(drawing_manager drawingInput ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(drawing_manager_2 src/drawing_manager_mult.cpp)
target_link_libraries(drawing_manager_2 drawingInput ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(init_pose src/init_pose.cpp)
target_link_libraries(init_pose ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(marker_publisher src/marker_publisher.cpp)
target_link_libraries(marker_publisher ${catkin_LIBRARIES})

add_executable(marker_publisher_2 src/marker_publisher_2.cpp)
target_link_libraries(marker_publisher_2 ${catkin_LIBRARIES})

