#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_msgs/ApplyPlanningScene.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

#include <iostream>
#include <fstream>
#include <string>
#include <vector>

#include "drawing_input.h"

#define TXT_FILE "/input/heart/heart_path_c.txt"

using moveit::planning_interface::MoveItErrorCode;

DrawingManager::DrawingManager(ros::NodeHandle* nh):nh_(*nh) {
  initPublisher();
  initMarker();
}

void DrawingManager::initPublisher() {
  marker_pub = nh_.advertise<visualization_msgs::Marker>("/target_drawing", 100);
}

// Init marker for target drawing
void DrawingManager::initMarker() {
  marker.header.frame_id = "/world";
  marker.header.stamp = ros::Time::now();
  marker.ns = "target";
  marker.action = visualization_msgs::Marker::ADD;
  marker.type = visualization_msgs::Marker::LINE_STRIP;

  marker.pose.orientation.w = 1.0;
  marker.scale.x = 0.001;
  marker.color.r = 0.0;
  marker.color.g = 0.0;
  marker.color.b = 0.0;
  marker.color.a = 0.8;
}

void DrawingManager::visualizeStrokes(std::vector<Stroke> &strokes, char color){
  int id = 0;

  if(color == 'c'){
    marker.color.r = 0.0; marker.color.g = 1.0; marker.color.b = 1.0;   // cyan (0, 255, 255)
  }else if(color == 'm'){
    marker.color.r = 1.0; marker.color.g = 0.0; marker.color.b = 1.0;   // magenta (255, 0, 255)
  }else if(color == 'y'){
    marker.color.r = 1.0; marker.color.g = 1.0; marker.color.b = 0.0;   // yellow (255, 255, 0)
  }else{ // black
    marker.color.r = 0.0; marker.color.g = 0.0; marker.color.b = 0.0;   // black (0, 0, 0)
  }

  for (int i = 0; i < strokes.size(); i++) { // storkes
    marker.header.stamp = ros::Time::now();
    marker.id = id * int(color); id++;
    for (int j = 0; j < strokes[i].size(); j++) { // points
      marker.points.push_back(strokes[i][j].position);
    }
    marker_pub.publish(marker);
    ros::Duration(0.05).sleep();
    marker.points.clear();
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "drawingManager");
  ros::NodeHandle nh("~");

  DrawingManager dm(&nh);

  // MoveIt operates on sets of joints called "planning groups" and stores them in an object called
  // the `JointModelGroup`. Throughout MoveIt the terms "planning group" and "joint model group"
  // are used interchangably.
  static const std::string PLANNING_GROUP_ARM_R = "left_arm";
  static const std::string PLANNING_GROUP_GRIPPER_R = "left_gripper";
  static const std::string PLANNING_GROUP_ARM_L = "right_arm";
  static const std::string PLANNING_GROUP_GRIPPER_L = "right_gripper";
  static const std::string EE_LINK_R = "left_ee_link";
  static const std::string EE_LINK_L = "right_ee_link";

  // The :planning_interface:`MoveGroupInterface` class can be easily
  // setup using just the name of the planning group you would like to control and plan for.
  moveit::planning_interface::MoveGroupInterface rightArm(PLANNING_GROUP_ARM_R);
  moveit::planning_interface::MoveGroupInterface rightGripper(PLANNING_GROUP_GRIPPER_R);
  moveit::planning_interface::MoveGroupInterface leftArm(PLANNING_GROUP_ARM_L);
  moveit::planning_interface::MoveGroupInterface leftGripper(PLANNING_GROUP_GRIPPER_L);

  rightArm.setEndEffectorLink(EE_LINK_R);
  leftArm.setEndEffectorLink(EE_LINK_L);

  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  moveit::planning_interface::MoveGroupInterface::Plan my_plan_arm_r;
  moveit::planning_interface::MoveGroupInterface::Plan my_plan_arm_l;
  moveit_msgs::RobotTrajectory trajectory;
  const double jump_threshold = 0.0; // 0.0
  const double eef_step = 0.001; // 0.001


  bool success;
  MoveItErrorCode success_plan = MoveItErrorCode::FAILURE, motion_done = MoveItErrorCode::FAILURE;

  // init pose
  // set all the joint values to the init joint position
  rightArm.setStartStateToCurrentState();
  rightArm.setJointValueTarget("left_shoulder_pan_joint", -0.26179); //-15
  rightArm.setJointValueTarget("left_shoulder_lift_joint", -1.3962634); //-80
  rightArm.setJointValueTarget("left_elbow_joint", 1.91986); //110
  rightArm.setJointValueTarget("left_wrist_1_joint", -1.3962634); //-80
  rightArm.setJointValueTarget("left_wrist_2_joint", 2.35619); //135
  rightArm.setJointValueTarget("left_wrist_3_joint", -0.523599);  //-30
  success = (rightArm.plan(my_plan_arm_r) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  rightArm.execute(my_plan_arm_r);

  leftArm.setStartStateToCurrentState();
  leftArm.setJointValueTarget("right_shoulder_pan_joint", 0.26179); //15
  leftArm.setJointValueTarget("right_shoulder_lift_joint", -1.74532925); //-100
  leftArm.setJointValueTarget("right_elbow_joint", -1.91986); //-110
  leftArm.setJointValueTarget("right_wrist_1_joint", -1.3962634); //-80
  leftArm.setJointValueTarget("right_wrist_2_joint", -2.35619); //-135
  leftArm.setJointValueTarget("right_wrist_3_joint", -1.0472);  //-60
  success = (leftArm.plan(my_plan_arm_l) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  leftArm.execute(my_plan_arm_l);
  ros::Duration(5).sleep(); // wait for 3 sec



  geometry_msgs::PoseStamped current_cartesian_position, command_cartesian_position;
  std::vector<geometry_msgs::Pose> drawing_stroke;
  std::vector<geometry_msgs::Pose> linear_path;
  geometry_msgs::Pose drawing_point;


//  rightArm.setStartStateToCurrentState();
//  rightArm.setJointValueTarget("left_shoulder_pan_joint", -0.994838); //-57
//  rightArm.setJointValueTarget("left_shoulder_lift_joint", -1.27409); //-73
//  rightArm.setJointValueTarget("left_elbow_joint", 1.76278); //101
//  rightArm.setJointValueTarget("left_wrist_1_joint", 2.6529); //152
//  rightArm.setJointValueTarget("left_wrist_2_joint", -0.541052); //-31
//  rightArm.setJointValueTarget("left_wrist_3_joint", -3.07178);  //-176
//  success = (rightArm.plan(my_plan_arm_r) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
//
//  rightArm.execute(my_plan_arm_r);


  //save init pose
  current_cartesian_position = rightArm.getCurrentPose(EE_LINK_R);
  drawing_point = current_cartesian_position.pose;



  // start reading input file
  DrawingInput drawing("heart_path", 'c', current_cartesian_position.pose);
  dm.drawings.push_back(drawing);
  dm.visualizeStrokes(drawing.strokes, drawing.color);

  ros::Duration(5).sleep(); // wait for 3 sec

  ros::shutdown();
  return 0;
}
