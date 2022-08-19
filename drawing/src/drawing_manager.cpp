#include "drawing_manager.h"

DrawingManager::DrawingManager(ros::NodeHandle* nh):nh_(*nh) {
  initPublisher();
  initMarker();
}

void DrawingManager::initPublisher() {
  marker_pub = nh_.advertise<visualization_msgs::Marker>("/target_drawing", 100);
  drawing_line_pub = nh_.advertise<std_msgs::Bool>("/ready_to_draw", 1);
  drawing_color_pub = nh_.advertise<geometry_msgs::Point>("/drawing_color", 1);
  arm_num_pub = nh_.advertise<std_msgs::Int32>("/arm_number", 1);
  trajectory_pub = nh_.advertise<moveit_msgs::RobotTrajectory>("/trajectory", 1);
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
  int id = (int)ros::Time::now().toSec()*1000;

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
  //*********** Initialize ROSc
  ros::init(argc, argv, "drawingManager");
  ros::NodeHandle nh("~");

  DrawingManager dm(&nh);

  //*********** ROS spinner.
  ros::AsyncSpinner spinner(1);
  spinner.start();

  // MoveIt operates on sets of joints called "planning groups" and stores them in an object called
  // the `JointModelGroup`. Throughout MoveIt the terms "planning group" and "joint model group"
  // are used interchangably.
  static const std::string PLANNING_GROUP_ARM_R = "left_arm";
  static const std::string PLANNING_GROUP_GRIPPER_R = "left_gripper";
  static const std::string PLANNING_GROUP_ARM_L = "right_arm";
  static const std::string PLANNING_GROUP_GRIPPER_L = "right_gripper";
  static const std::string EE_LINK_R = "left_gripper_tool0";
  static const std::string EE_LINK_L = "right_gripper_tool0";

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
  rightArm.setJointValueTarget("left_shoulder_pan_joint", 172*D2R); //-15
  rightArm.setJointValueTarget("left_shoulder_lift_joint", -97*D2R); //-80
  rightArm.setJointValueTarget("left_elbow_joint", -106*D2R); //110
  rightArm.setJointValueTarget("left_wrist_1_joint", -73*D2R); //-80
  rightArm.setJointValueTarget("left_wrist_2_joint", -44*D2R); //135
  rightArm.setJointValueTarget("left_wrist_3_joint", -35*D2R);  //-30
  success = (rightArm.plan(my_plan_arm_r) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  rightArm.execute(my_plan_arm_r);

  leftArm.setStartStateToCurrentState();
  leftArm.setJointValueTarget("right_shoulder_pan_joint", -178*D2R); //15
  leftArm.setJointValueTarget("right_shoulder_lift_joint", -88*D2R); //-100
  leftArm.setJointValueTarget("right_elbow_joint", 97*D2R); //-110
  leftArm.setJointValueTarget("right_wrist_1_joint", -102*D2R); //-80
  leftArm.setJointValueTarget("right_wrist_2_joint", 46*D2R); //-135
  leftArm.setJointValueTarget("right_wrist_3_joint", 137*D2R);  //-60
  success = (leftArm.plan(my_plan_arm_l) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  leftArm.execute(my_plan_arm_l);
  ros::Duration(3).sleep(); // wait for 3 sec

  //save init pose
  dm.init_drawing_pose_r = rightArm.getCurrentPose(EE_LINK_R).pose;
  dm.init_drawing_pose_l = leftArm.getCurrentPose(EE_LINK_L).pose;


  geometry_msgs::PoseStamped current_cartesian_position, command_cartesian_position;
  std::vector<geometry_msgs::Pose> drawing_stroke;
  std::vector<geometry_msgs::Pose> linear_path;
  geometry_msgs::Pose drawing_point;

  // start reading input file
  dm.colors.push_back("c"); dm.colors.push_back("m"); dm.colors.push_back("y"); dm.colors.push_back("k");
  dm.drawing_file_name = "heart_path_";

  for (int i = 0; i < dm.colors.size(); i++){
    char ch[1];
    strcpy(ch, dm.colors[i].c_str());
    ROS_INFO("Drawing init");
    DrawingInput drawing(dm.drawing_file_name, ch[0], dm.init_drawing_pose_r, dm.init_drawing_pose_l);
    dm.drawings.push_back(drawing);
//    for (int j = 0; j < drawing.strokes_by_range.size(); j++)
//      dm.visualizeStrokes(drawing.strokes_by_range[j], drawing.color);
//    ros::Duration(0.5).sleep();
  }

  // draw
  int stroke_num = 0;
  std_msgs::Bool ready;
  ready.data = false;
  dm.drawing_line_pub.publish(ready);
  MoveItErrorCode executed = MoveItErrorCode::SUCCESS;

  // left arm
  std_msgs::Int32 arm_num;
  arm_num.data = 1;
  dm.arm_num_pub.publish(arm_num);
  for (int i = 0; i < dm.colors.size(); i ++)
  {
    dm.drawing_color_pub.publish(dm.drawings[i].color_);
    stroke_num = 0;
    for (auto stroke : dm.drawings[i].strokes_by_range[1])
    {
      // move to first position
      command_cartesian_position.pose = stroke[0];
      linear_path.push_back(command_cartesian_position.pose);
      double fraction = leftArm.computeCartesianPath(linear_path, eef_step, jump_threshold, trajectory);
      ROS_INFO("PLANNING DONE");
      my_plan_arm_r.trajectory_ = trajectory;
      leftArm.execute(my_plan_arm_r);  //ros::Duration(0.1).sleep();
      if (fraction < 0.5) ROS_WARN_STREAM("MOVE READY POSITION ERROR");
      ROS_INFO("MOVE READY POSITION");
      linear_path.clear();

      std::cout << "Drawing " << dm.drawings[i].color << " " << stroke_num << "th stroke ... " << std::endl;
      fraction = leftArm.computeCartesianPath(stroke, eef_step, jump_threshold, trajectory);
      ROS_INFO("PLANNING DONE");
      my_plan_arm_r.trajectory_ = trajectory;
      dm.trajectory_pub.publish(trajectory);

      // publish
      ready.data = true;
      dm.drawing_line_pub.publish(ready);
      ros::Duration(0.1).sleep();
      // execute
      ROS_INFO("EXECUTING ...");
      executed = leftArm.execute(my_plan_arm_r);
      ROS_INFO("EXECUTION DONE");
      ros::Duration(0.1).sleep();
      // publish
      ready.data = false;
      dm.drawing_line_pub.publish(ready);
      stroke_num++;
    }
  }

  // right arm
  arm_num.data = 0;
  dm.arm_num_pub.publish(arm_num);
  for (int i = 0; i < dm.colors.size(); i ++)
  {
    dm.drawing_color_pub.publish(dm.drawings[i].color_);
    stroke_num = 0;
    for (auto stroke : dm.drawings[i].strokes_by_range[0])
    {
      // move to first position
      command_cartesian_position.pose = stroke[0];
      linear_path.push_back(command_cartesian_position.pose);
      double fraction = rightArm.computeCartesianPath(linear_path, eef_step, jump_threshold, trajectory);
      ROS_INFO("PLANNING DONE");
      my_plan_arm_r.trajectory_ = trajectory;
      rightArm.execute(my_plan_arm_r);  //ros::Duration(0.1).sleep();
      if (fraction < 0.5) ROS_WARN_STREAM("MOVE READY POSITION ERROR");
      ROS_INFO("MOVE READY POSITION");
      linear_path.clear();

      std::cout << "Drawing " << dm.drawings[i].color << " " << stroke_num << "th stroke ... " << std::endl;
      fraction = rightArm.computeCartesianPath(stroke, eef_step, jump_threshold, trajectory);
      ROS_INFO("PLANNING DONE");
      my_plan_arm_r.trajectory_ = trajectory;
      dm.trajectory_pub.publish(trajectory);

      // publish
      ready.data = true;
      dm.drawing_line_pub.publish(ready);
      ros::Duration(0.1).sleep();
      // execute
      ROS_INFO("EXECUTING ...");
      executed = rightArm.execute(my_plan_arm_r);
      ROS_INFO("EXECUTION DONE");
      ros::Duration(0.1).sleep();
      // publish
      ready.data = false;
      dm.drawing_line_pub.publish(ready);
      stroke_num++;
    }
  }

  ros::Duration(3).sleep(); // wait for 3 sec

  ros::shutdown();
  return 0;
}
