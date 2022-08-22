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
#include <std_msgs/Int32.h>

#include <string>
#include <vector>

#define Stroke std::vector<geometry_msgs::Pose>
using moveit::planning_interface::MoveItErrorCode;
#define SAVE 0
#define READ 1

class DrawingManager {
  public:
    DrawingManager(ros::NodeHandle* nh);

    // moveit
    moveit::planning_interface::MoveGroupInterface *rightArm;
    moveit::planning_interface::MoveGroupInterface *rightGripper;
    moveit::planning_interface::MoveGroupInterface *leftArm;
    moveit::planning_interface::MoveGroupInterface *leftGripper;
    moveit::planning_interface::PlanningSceneInterface *planning_scene_interface;
    const std::string EE_LINK_R = "left_gripper_tool0";
    const std::string EE_LINK_L = "right_gripper_tool0";

    const double jump_threshold = 0.0; // 0.0
    const double eef_step = 0.001; // 0.001

    int manager_state;

    std::vector<DrawingInput> drawings;

    visualization_msgs::Marker marker;
    visualization_msgs::Marker target_range_marker;

    ros::Publisher marker_pub;
    ros::Publisher drawing_line_pub;
    ros::Publisher drawing_color_pub;
    ros::Publisher trajectory_pub;
    ros::Publisher arm_num_pub;

    geometry_msgs::Pose init_drawing_pose_r;
    geometry_msgs::Pose init_drawing_pose_l;

    // problem
    std::string drawing_file_name;
    std::vector<std::string> colors;

    void visualizeStrokes(std::vector<Stroke> &strokes, char color);
    void setJointValue (double q[], int arm_num);
    void initPose ();

  private:
    ros::NodeHandle nh_;

    const std::string PLANNING_GROUP_ARM_R = "left_arm";
    const std::string PLANNING_GROUP_GRIPPER_R = "left_gripper";
    const std::string PLANNING_GROUP_ARM_L = "right_arm";
    const std::string PLANNING_GROUP_GRIPPER_L = "right_gripper";

    void initMoveGroup();
    void initPublisher();
    void initMarker();

};
