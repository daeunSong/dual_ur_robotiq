#include "drawing_manager.h"

DrawingManager::DrawingManager(ros::NodeHandle* nh):nh_(*nh) {
//  initMoveGroup();
  initPublisher();
  initMarker();
}

void DrawingManager::initMoveGroup() {
  this->arms = new moveit::planning_interface::MoveGroupInterface(PLANNING_GROUP_ARMS);
  this->rightArm = new moveit::planning_interface::MoveGroupInterface(PLANNING_GROUP_ARM_R);
  this->rightGripper = new moveit::planning_interface::MoveGroupInterface(PLANNING_GROUP_GRIPPER_R);
  this->leftArm = new moveit::planning_interface::MoveGroupInterface(PLANNING_GROUP_ARM_L);
  this->leftGripper = new moveit::planning_interface::MoveGroupInterface(PLANNING_GROUP_GRIPPER_L);

  this->rightArm->setEndEffectorLink(EE_LINK_R);
  this->leftArm->setEndEffectorLink(EE_LINK_L);

  this->kinematic_model = this->arms->getRobotModel();
  this->kinematic_state = this->arms->getCurrentState();
  this->right_arm_ = this->kinematic_model->getJointModelGroup(PLANNING_GROUP_ARM_R);
  this->left_arm_ = this->kinematic_model->getJointModelGroup(PLANNING_GROUP_ARM_L);

  this->right_joint_names = this->right_arm_->getVariableNames();
  this->left_joint_names = this->left_arm_->getVariableNames();
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

std::map<std::string, double> DrawingManager::vector2map (const std::vector<std::string> &joint_names, std::vector<double> &joint_values){
  std::map<std::string, double> m;
  for (int i = 0; i < joint_names.size(); i++)
    m.insert({joint_names[i], joint_values[i]});
  return m;
}

void DrawingManager::setJointValue (std::vector<double> &q, int arm_num = 0){
  std::map<std::string, double> variable_values;
  if (arm_num == 0) // right
    variable_values = vector2map(this->right_joint_names, q);
  else if (arm_num == 1) // left
    variable_values = vector2map(this->left_joint_names, q);

  this->arms->setJointValueTarget(variable_values);
}

void DrawingManager::initPose (){
  std::vector<double> init_r = {172*D2R, -97*D2R, -106*D2R, -73*D2R, -44*D2R, -35*D2R};
  std::vector<double> init_l = {-178*D2R, -88*D2R, 97*D2R, -102*D2R, 46*D2R, -43*D2R};

  this->setJointValue(init_r, 0);
  this->setJointValue(init_l, 1);

  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  bool success = (this->arms->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  if(!success){
    ROS_INFO("Plan did not successed");
  }
  this->arms->execute(my_plan);
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

  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  moveit_msgs::RobotTrajectory trajectory;
  bool success;

  // init pose
  // set all the joint values to the init joint position
//  dm.initPose();
//  ros::Duration(3).sleep(); // wait for 3 sec

  //save init pose
  dm.init_drawing_pose_r = dm.rightArm->getCurrentPose(dm.EE_LINK_R).pose;
  dm.init_drawing_pose_l = dm.leftArm->getCurrentPose(dm.EE_LINK_L).pose;


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
      double fraction = dm.leftArm->computeCartesianPath(linear_path, dm.eef_step, dm.jump_threshold, trajectory);
      ROS_INFO("PLANNING DONE");
      my_plan.trajectory_ = trajectory;
      dm.leftArm->execute(my_plan);  //ros::Duration(0.1).sleep();
      if (fraction < 0.5) ROS_WARN_STREAM("MOVE READY POSITION ERROR");
      ROS_INFO("MOVE READY POSITION");
      linear_path.clear();

      std::cout << "Drawing " << dm.drawings[i].color << " " << stroke_num << "th stroke ... " << std::endl;
      ready.data = true;
      for (int j = 0; j < stroke.size(); j++){
//        std::vector<double> joint_values;
//        bool found = kinematic_state.setFromIK(left_arm_, stroke[j], dm.EE_LINK_L, 0.1);
//        while (!found)
//          found = kinematic_state.setFromIK(left_arm_, stroke[j], dm.EE_LINK_L, 0.1);
//        if (found){
//          kinematic_state.copyJointGroupPositions(left_arm_, joint_values);
//          dm.setJointValue(joint_values, 1);
//        }
//        else ROS_INFO("Did not find IK solution");
      }

//      fraction = dm.leftArm->computeCartesianPath(stroke, dm.eef_step, dm.jump_threshold, trajectory);
//      ROS_INFO("PLANNING DONE");
//      my_plan.trajectory_ = trajectory;
//      dm.trajectory_pub.publish(trajectory);

      // publish
//      dm.drawing_line_pub.publish(ready);
//      ros::Duration(0.1).sleep();
      // execute
//      ROS_INFO("EXECUTING ...");
//      executed = dm.leftArm->execute(my_plan);
      ROS_INFO("EXECUTION DONE");
      ros::Duration(0.1).sleep();
      // publish
      ready.data = false;
      dm.drawing_line_pub.publish(ready);
      stroke_num++;
    }
  }

//  dm.initPose();
//  ros::Duration(3).sleep(); // wait for 3 sec

  ros::shutdown();
  return 0;
}
