#include "drawing_input.h"

#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_msgs/ApplyPlanningScene.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

#include <std_msgs/Bool.h>

#include <string>
#include <vector>

#define Stroke std::vector<geometry_msgs::Pose>
using moveit::planning_interface::MoveItErrorCode;
#define SAVE 0
#define READ 1

class DrawingManager {
  public:
    DrawingManager(ros::NodeHandle* nh);

    int manager_state;

    std::vector<DrawingInput> drawings;

    visualization_msgs::Marker marker;
    visualization_msgs::Marker target_range_marker;

    ros::Publisher marker_pub;
    ros::Publisher drawing_line_pub;
    ros::Publisher drawing_color_pub;
    ros::Publisher trajectory_pub;

    // iiwa
    geometry_msgs::Pose init_drawing_pose;

    // problem
    std::string drawing_file_name;
    std::vector<std::string> colors;

    void visualizeStrokes(std::vector<Stroke> &strokes, char color);

  private:
    ros::NodeHandle nh_;

    void initPublisher();
    void initMarker();

};
