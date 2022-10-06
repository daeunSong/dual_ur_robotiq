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

#include <moveit/moveit_cpp/moveit_cpp.h>
#include <eigen_conversions/eigen_msg.h>

#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Char.h>

#include <string>
#include <vector>

#define Stroke std::vector<geometry_msgs::Pose>
using moveit::planning_interface::MoveItErrorCode;

class DrawingManager {
  public:
    DrawingManager(ros::NodeHandle* nh);

    // moveit
    moveit::planning_interface::MoveGroupInterface *arms;
    moveit::planning_interface::MoveGroupInterface *rightArm;
    moveit::planning_interface::MoveGroupInterface *rightGripper;
    moveit::planning_interface::MoveGroupInterface *leftArm;
    moveit::planning_interface::MoveGroupInterface *leftGripper;
//    moveit::planning_interface::PlanningSceneInterface *planning_scene_interface;
    const robot_state::JointModelGroup *right_arm_;
    const robot_state::JointModelGroup *left_arm_;
    std::vector<std::string> right_joint_names;
    std::vector<std::string> left_joint_names;
    const std::string PLANNING_GROUP_ARMS = "arms";
    const std::string PLANNING_GROUP_ARM_R = "right_arm";
    const std::string PLANNING_GROUP_GRIPPER_R = "right_gripper";
    const std::string PLANNING_GROUP_ARM_L = "left_arm";
    const std::string PLANNING_GROUP_GRIPPER_L = "left_gripper";
    const std::string EE_LINK_R = "right_gripper_tool0";
    const std::string EE_LINK_L = "left_gripper_tool0";

    moveit::core::RobotModelConstPtr kinematic_model;
    moveit::core::RobotStatePtr kinematic_state;

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
    ros::Publisher drawing_line_pub_2;
    ros::Publisher drawing_color_pub_2;
//    ros::Publisher arm_num_pub_2;
    ros::Publisher gripper_pub_left;
    ros::Publisher gripper_pub_right;

    geometry_msgs::Pose init_drawing_pose_r;
    geometry_msgs::Pose init_drawing_pose_l;

    // problem
    std::string drawing_file_name;
    std::vector<std::string> colors;

    // functions
    void visualizeStrokes(std::vector<Stroke> &strokes, char color);
    std::map<std::string, double> vector2map (const std::vector<std::string> &joint_names, std::vector<double> &joint_values);
    void setJointValue (std::vector<double> &q, int arm_num);
    void initPose ();
    void initPose_r ();
    void initPose_l ();
    static bool compareTime(std::pair<trajectory_msgs::JointTrajectoryPoint,int> p1, std::pair<trajectory_msgs::JointTrajectoryPoint,int> p2);
    void mergeVectors(std::vector<double> &v1, std::vector<double> &v2, std::vector<double> &temp_point);
    void mergeTrajectories(moveit_msgs::RobotTrajectory& traj_l, moveit_msgs::RobotTrajectory& traj_r, moveit_msgs::RobotTrajectory& traj);

  private:
    ros::NodeHandle nh_;

    void initMoveGroup();
    void initPublisher();
    void initMarker();

};
