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

shape_msgs::SolidPrimitive setPrim(int d, float x, float y, float z);
geometry_msgs::Pose setGeomPose(float x, float y, float z, float ox, float oy, float oz, float ow);

int main(int argc, char** argv)
{
  ros::init(argc, argv, "drawing");
  ros::NodeHandle n;
  ros::AsyncSpinner spinner(1);
  spinner.start();

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


  bool success;

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



  geometry_msgs::PoseStamped current_cartesian_position, command_cartesian_position
  current_cartesian_position = rightArm.getCurrentPose(EE_LINK_R);

  std::vector<geometry_msgs::Pose> drawing_stroke;
  std::vector<geometry_msgs::Pose> linear_path;
  geometry_msgs::Pose drawing_point;

  /*

  // The package MoveItVisualTools provides many capabilities for visualizing objects, robots,
  // and trajectories in RViz as well as debugging tools such as step-by-step introspection of a script.
  namespace rvt = rviz_visual_tools;
  moveit_visual_tools::MoveItVisualTools visual_tools(rightArm.getPlanningFrame().c_str()); //here might be the problem?
  visual_tools.deleteAllMarkers();

  // Remote control is an introspection tool that allows users to step through a high level script
  // via buttons and keyboard shortcuts in RViz
  visual_tools.loadRemoteControl();

  // RViz provides many types of markers, in this demo we will use text, cylinders, and spheres
  Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
  text_pose.translation().z() = 1.0;
  visual_tools.publishText(text_pose, "MoveGroupInterface Demo", rvt::WHITE, rvt::XLARGE);

  // Batch publishing is used to reduce the number of messages being sent to RViz for large visualizations
  visual_tools.trigger();

    // We can get a list of all the groups in the robot:
    ROS_INFO_NAMED("tutorial", "Available Planning Groups:");
    std::copy(rightArm.getJointModelGroupNames().begin(),
    rightArm.getJointModelGroupNames().end(), std::ostream_iterator<std::string>(std::cout, ", "));
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;

    ROS_INFO_NAMED("tutorial", "\nEnd effector link: %s", rightArm.getEndEffectorLink().c_str());
    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");

    // 1. Move to home position
    rightArm.setJointValueTarget(rightArm.getNamedTargetValues("left_arm_zero"));
    success = (rightArm.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

    rightArm.move();

    visual_tools.publishText(text_pose, "move left arm zero pose", rvt::WHITE, rvt::XLARGE);
    visual_tools.trigger();
    visual_tools.prompt("Press 'next' to continue the demo");

    // 1-2. Add Collision object
    // add table
    std::vector<moveit_msgs::CollisionObject> collision_objects;

    moveit_msgs::CollisionObject collision_object;
    collision_object.header.frame_id = rightArm.getPlanningFrame();
    collision_object.id = "table";
    shape_msgs::SolidPrimitive primitive = setPrim(3, 1.7, 4.0, 0.03);
    geometry_msgs::Pose table_pose = setGeomPose(-1.0, 0, 0.95, 0.0, 1.0, 0.0, 0.0);
    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(table_pose);
    collision_object.operation = collision_object.ADD;
    collision_objects.push_back(collision_object);

    ROS_INFO_NAMED("tutorial", "Add a table into the world");
    //planning_scene_interface.applyCollisionObjects(collision_objects);

    // add block
    moveit_msgs::CollisionObject object_to_attach;
    object_to_attach.header.frame_id = rightArm.getPlanningFrame();
    object_to_attach.id = "block";
    primitive = setPrim(3, 0.07, 0.07, 0.07);
    geometry_msgs::Pose block_pose = setGeomPose(-0.764824, 0.734106, 1.00, 0.0, 0.7071068, 0, 0.7071068);
    object_to_attach.primitives.push_back(primitive);
    object_to_attach.primitive_poses.push_back(block_pose);
    object_to_attach.operation = object_to_attach.ADD;
    collision_objects.push_back(object_to_attach);

    planning_scene_interface.addCollisionObjects(collision_objects);

    visual_tools.publishText(text_pose, "Add table", rvt::WHITE, rvt::XLARGE);
    visual_tools.trigger();
    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to once the table appears in RViz");

    // 2. Place the EE above the wood block
    geometry_msgs::Pose pre_grasp_pose = block_pose;//setGeomPose(-0.764824, 0.734106, 1.26638, 0.0, 0.7071068, 0, 0.7071068);
    pre_grasp_pose.position.z += 0.2;
    rightArm.setPoseTarget(pre_grasp_pose);
    success = (rightArm.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    rightArm.move();

    ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");
    visual_tools.trigger();
    visual_tools.prompt("Press 'next' once the plan is complete and then it will move the arm");

    //geometry_msgs::Pose block_pose = setGeomPose(-0.743945, 0.721829, 1.03214,  0.00533024, 0.717904, -0.00352154, 0.696113);
    //geometry_msgs::Pose grasp_pose = pre_grasp_pose;
    pre_grasp_pose.position.z -= 0.04;
    rightArm.setPoseTarget(pre_grasp_pose);
    success = (rightArm.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    rightArm.move();

    pre_grasp_pose.position.z -= 0.04;
    rightArm.setPoseTarget(pre_grasp_pose);
    success = (rightArm.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    rightArm.move();

    pre_grasp_pose.position.z -= 0.04;
    rightArm.setPoseTarget(pre_grasp_pose);
    success = (rightArm.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    rightArm.move();

    pre_grasp_pose.position.z -= 0.03;
    rightArm.setPoseTarget(pre_grasp_pose);
    success = (rightArm.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    rightArm.move();

    pre_grasp_pose.position.z -= 0.02;
    rightArm.setPoseTarget(pre_grasp_pose);
    success = (rightArm.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    rightArm.move();

    ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");
    visual_tools.trigger();
    visual_tools.prompt("Press 'next' once the plan is complete and then it will move the arm");
    rightArm.move();
    */

/*
    geometry_msgs::PoseStamped current_pose = rightArm.getCurrentPose("left_gripper_tool0");
    std::cout << std::endl <<"pos: " << current_pose.pose.position.x << ", " << current_pose.pose.position.y << ", " << current_pose.pose.position.z << std::endl;
    std::cout <<"ori: " << current_pose.pose.orientation.x << ", " << current_pose.pose.orientation.y << ", " << current_pose.pose.orientation.z << ", " << current_pose.pose.orientation.w << std::endl;
*/

/*
    // 3. Open the gripper
    moveit::planning_interface::MoveGroupInterface::Plan my_plan_gripper;

    move_group_interface_gripper.setJointValueTarget(move_group_interface_gripper.getNamedTargetValues("open"));

    success = (move_group_interface_gripper.plan(my_plan_gripper) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

    move_group_interface_gripper.move();

    // 4. Move the TCP close to the object
    target_pose1.position.z = target_pose1.position.z - 0.2;
    rightArm.setPoseTarget(target_pose1);

    success = (rightArm.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

    rightArm.move();

    // 5. Close the  gripper
    move_group_interface_gripper.setJointValueTarget(move_group_interface_gripper.getNamedTargetValues("closed"));

    success = (move_group_interface_gripper.plan(my_plan_gripper) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

    move_group_interface_gripper.move();

    // 6. Move the TCP above the plate
    target_pose1.position.z = target_pose1.position.z + 0.2;
    target_pose1.position.x = target_pose1.position.x - 0.6;
    rightArm.setPoseTarget(target_pose1);

    success = (rightArm.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

    rightArm.move();

    // 7. Lower the TCP above the plate
    target_pose1.position.z = target_pose1.position.z - 0.14;
    rightArm.setPoseTarget(target_pose1);

    success = (rightArm.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

    rightArm.move();

    // 8. Open the gripper
    move_group_interface_gripper.setJointValueTarget(move_group_interface_gripper.getNamedTargetValues("open"));

    success = (move_group_interface_gripper.plan(my_plan_gripper) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

    move_group_interface_gripper.move();

    ROS_INFO_NAMED("tutorial", "Remove the object from the world");
    std::vector<std::string> object_ids;
    object_ids.push_back(collision_object.id);
    planning_scene_interface.removeCollisionObjects(object_ids);
*/
  ros::shutdown();
  return 0;
}


shape_msgs::SolidPrimitive setPrim(int d, float x, float y, float z)
{
    shape_msgs::SolidPrimitive pr;
    pr.type = pr.BOX;
    pr.dimensions.resize(d);
    pr.dimensions[pr.BOX_X] = x;
    pr.dimensions[pr.BOX_Y] = y;
    pr.dimensions[pr.BOX_Z] = z;

    return pr;
}

geometry_msgs::Pose setGeomPose(float x, float y, float z, float ox, float oy, float oz, float ow)
{
    geometry_msgs::Pose p;

    p.position.x = x;
    p.position.y = y;
    p.position.z = z;
    p.orientation.x = ox;
    p.orientation.y = oy;
    p.orientation.z = oz;
    p.orientation.w = ow;

    return p;
}
