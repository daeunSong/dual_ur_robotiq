#include "drawing_manager.h"

DrawingManager::DrawingManager(ros::NodeHandle* nh):nh_(*nh) {
  initMoveGroup();
  initPublisher();
  initMarker();
}

void DrawingManager::initMoveGroup() {
  this->arms = new moveit::planning_interface::MoveGroupInterface(PLANNING_GROUP_ARMS);
  this->rightArm = new moveit::planning_interface::MoveGroupInterface(PLANNING_GROUP_ARM_R);
//  this->rightGripper = new moveit::planning_interface::MoveGroupInterface(PLANNING_GROUP_GRIPPER_R);
  this->leftArm = new moveit::planning_interface::MoveGroupInterface(PLANNING_GROUP_ARM_L);
//  this->leftGripper = new moveit::planning_interface::MoveGroupInterface(PLANNING_GROUP_GRIPPER_L);

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
  drawing_line_pub_2 = nh_.advertise<std_msgs::Bool>("/ready_to_draw_2", 1);
  drawing_color_pub_2 = nh_.advertise<geometry_msgs::Point>("/drawing_color_2", 1);
//  arm_num_pub_2 = nh_.advertise<std_msgs::Int32>("/arm_number_2", 1);
  trajectory_pub = nh_.advertise<moveit_msgs::RobotTrajectory>("/trajectory", 1);
  gripper_pub_left = nh_.advertise<std_msgs::Char>("/left_gripper/gripper_left", 10);
  gripper_pub_right = nh_.advertise<std_msgs::Char>("/right_gripper/gripper_right", 10);

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

}

//void DrawingManager::initPose (){
//  std::vector<double> init_r = {-18*D2R, -41*D2R, 64*D2R, -130*D2R, -132*D2R, 111*D2R};
////  std::vector<double> init_l = {-178*D2R, -88*D2R, 97*D2R, -102*D2R, 46*D2R, -43*D2R};
//
//  this->setJointValue(init_r, 0);
////  this->setJointValue(init_l, 1);
//
//  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
//  bool success = (this->arms->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
//  if(!success){
//    ROS_INFO("Plan did not successed");
//  }
//  this->arms->execute(my_plan);
//
//}

//void DrawingManager::initPose (){
//  std::vector<double> init_r = {-170*D2R, -120*D2R, -80*D2R, -70*D2R, 45*D2R, 140*D2R};
//  std::vector<double> init_l = {170*D2R, -65*D2R, 80*D2R, -110*D2R, -45*D2R, 130*D2R};
//
//  std::map<std::string, double> variable_values_r = vector2map(this->right_joint_names, init_r);
//  std::map<std::string, double> variable_values_l = vector2map(this->left_joint_names, init_l);
//
//  this->arms->setJointValueTarget(variable_values_r);
//  this->arms->setJointValueTarget(variable_values_l);
//
//  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
//  bool success = (this->arms->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
//  if(!success){
//    ROS_INFO("Plan did not successed");
//  }
//  this->arms->execute(my_plan);
////  this->rightArm->execute(my_plan);
////  this->leftArm->execute(my_plan);
//}

void DrawingManager::initPose() {
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  moveit_msgs::RobotTrajectory trajectory;

  // right arm
  std::vector<geometry_msgs::Pose> linear_path;
  geometry_msgs::Pose command_pose;
  command_pose.position.x = -0.65;
  command_pose.position.y = 0.35;
  command_pose.position.z = 1.50;
  command_pose.orientation.x = -0.7071068;
  command_pose.orientation.y = 0.7071068;
  command_pose.orientation.z = 0.0;
  command_pose.orientation.w = 0.0;
  linear_path.push_back(command_pose);
  this->rightArm->computeCartesianPath(linear_path, this->eef_step, this->jump_threshold, trajectory);
  my_plan.trajectory_ = trajectory;
  this->rightArm->execute(my_plan);
  linear_path.clear();

  // left arm
  command_pose.position.y = -0.35;
  command_pose.orientation.x = 0.7071068;
  linear_path.push_back(command_pose);
  this->leftArm->computeCartesianPath(linear_path, this->eef_step, this->jump_threshold, trajectory);
  my_plan.trajectory_ = trajectory;
  this->leftArm->execute(my_plan);
  linear_path.clear();
}

void DrawingManager::initPose_r (){
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  moveit_msgs::RobotTrajectory trajectory;

  // right arm
  std::vector<geometry_msgs::Pose> linear_path;
  geometry_msgs::Pose command_pose;
  command_pose.position.x = -0.65;
  command_pose.position.y = 0.3;
  command_pose.position.z = 1.50;
  command_pose.orientation.x = -0.7071068;
  command_pose.orientation.y = 0.7071068;
  command_pose.orientation.z = 0.0;
  command_pose.orientation.w = 0.0;
  linear_path.push_back(command_pose);
  this->rightArm->computeCartesianPath(linear_path, this->eef_step, this->jump_threshold, trajectory);
  my_plan.trajectory_ = trajectory;
  this->trajectory_pub.publish(trajectory);
  ros::Duration(1).sleep(); // wait for 3 sec
//  this->rightArm->execute(my_plan);
  linear_path.clear();
}

void DrawingManager::initPose_l (){
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  moveit_msgs::RobotTrajectory trajectory;

  // right arm
  std::vector<geometry_msgs::Pose> linear_path;
  geometry_msgs::Pose command_pose;
  command_pose.position.x = -0.65;
  command_pose.position.y = -0.3;
  command_pose.position.z = 1.50;
  command_pose.orientation.x = 0.7071068;
  command_pose.orientation.y = 0.7071068;
  command_pose.orientation.z = 0.0;
  command_pose.orientation.w = 0.0;
  this->leftArm->computeCartesianPath(linear_path, this->eef_step, this->jump_threshold, trajectory);
  my_plan.trajectory_ = trajectory;
  this->trajectory_pub.publish(trajectory);
  ros::Duration(1).sleep(); // wait for 3 sec
//  this->leftArm->execute(my_plan);
  linear_path.clear();
}

/*
void DrawingManager::initPose_r (){
  std::vector<double> init_r = {-170*D2R, -120*D2R, -80*D2R, -70*D2R, 45*D2R, 140*D2R};
//  std::vector<double> init_l = {170*D2R, -65*D2R, 80*D2R, -110*D2R, -45*D2R, 130*D2R};

  std::map<std::string, double> variable_values_r = vector2map(this->right_joint_names, init_r);
//  std::map<std::string, double> variable_values_l = vector2map(this->left_joint_names, init_l);

  this->rightArm->setJointValueTarget(variable_values_r);
//  this->arms->setJointValueTarget(variable_values_l);

  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  bool success = (this->rightArm->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  if(!success){
    ROS_INFO("Plan did not successed");
  }
//  this->arms->execute(my_plan);
  this->rightArm->execute(my_plan);
//  this->leftArm->execute(my_plan);
}

void DrawingManager::initPose_l (){
//  std::vector<double> init_r = {-170*D2R, -120*D2R, -80*D2R, -70*D2R, 45*D2R, 140*D2R};
  std::vector<double> init_l = {170*D2R, -65*D2R, 80*D2R, -110*D2R, -45*D2R, 130*D2R};

//  std::map<std::string, double> variable_values_r = vector2map(this->right_joint_names, init_r);
  std::map<std::string, double> variable_values_l = vector2map(this->left_joint_names, init_l);

//  this->rightArm->setJointValueTarget(variable_values_r);
  this->leftArm->setJointValueTarget(variable_values_l);

  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  bool success = (this->leftArm->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  if(!success){
    ROS_INFO("Plan did not successed");
  }
//  this->arms->execute(my_plan);
//  this->rightArm->execute(my_plan);
  this->leftArm->execute(my_plan);
}
*/

bool DrawingManager::compareTime(std::pair<trajectory_msgs::JointTrajectoryPoint,int> p1, std::pair<trajectory_msgs::JointTrajectoryPoint,int> p2) {
  if (p1.first.time_from_start == p2.first.time_from_start)
    return p1.second < p2.second;
  return p1.first.time_from_start < p2.first.time_from_start;
}

void DrawingManager::mergeVectors(std::vector<double> &v1, std::vector<double> &v2, std::vector<double> &temp) {
    for (int i = 0; i < v1.size(); i++)
      temp.push_back(v1[i]);
    for (int i = 0; i < v2.size(); i++)
      temp.push_back(v2[i]);
}

void DrawingManager::mergeTrajectories(moveit_msgs::RobotTrajectory& traj_r, moveit_msgs::RobotTrajectory& traj_l, moveit_msgs::RobotTrajectory& traj) {
    // Use the left trajectory as template
    traj.joint_trajectory.header = traj_l.joint_trajectory.header;
    traj.joint_trajectory.joint_names = traj_l.joint_trajectory.joint_names;
    traj.joint_trajectory.joint_names.insert(traj.joint_trajectory.joint_names.end(),
            traj_r.joint_trajectory.joint_names.begin(), traj_r.joint_trajectory.joint_names.end());
    traj.joint_trajectory.points.clear();

    // sort trajectory points according to execution time
    int num_points_l = traj_l.joint_trajectory.points.size();
    int num_points_r = traj_r.joint_trajectory.points.size();
    int num_points = num_points_l + num_points_r;
    std::vector<std::pair<trajectory_msgs::JointTrajectoryPoint, int>> points;
//    std::vector<trajectory_msgs::JointTrajectoryPoint> points;
    for (int i=0; i<num_points_l; i++)
        points.push_back(std::make_pair(traj_l.joint_trajectory.points[i], i));
    for (int i=0; i<num_points_r; i++)
        points.push_back(std::make_pair(traj_r.joint_trajectory.points[i], i+num_points_l));
    std::sort(points.begin(), points.end(), this->compareTime);

    // merge the points
    trajectory_msgs::JointTrajectoryPoint last_l = traj_l.joint_trajectory.points[0];
    trajectory_msgs::JointTrajectoryPoint last_r = traj_r.joint_trajectory.points[0];
    trajectory_msgs::JointTrajectoryPoint cur;

    for (int i=0; i<num_points; i++) {
        // merge the most recent trajectory of counter arm to current point
        trajectory_msgs::JointTrajectoryPoint temp_point;
        cur = points[i].first;
        if (points[i].second < num_points_l) { // if current timestep is a left point
            this->mergeVectors(cur.positions, last_r.positions, temp_point.positions);
            this->mergeVectors(cur.velocities, last_r.velocities, temp_point.velocities);
            this->mergeVectors(cur.accelerations, last_r.accelerations, temp_point.accelerations);
            this->mergeVectors(cur.effort, last_r.effort, temp_point.effort);
            temp_point.time_from_start = cur.time_from_start;
            last_l = cur;
        }
        else {
            this->mergeVectors(last_l.positions, cur.positions, temp_point.positions);
            this->mergeVectors(last_l.velocities, cur.velocities, temp_point.velocities);
            this->mergeVectors(last_l.accelerations, cur.accelerations, temp_point.accelerations);
            this->mergeVectors(last_l.effort, cur.effort, temp_point.effort);
            temp_point.time_from_start = cur.time_from_start;
            last_r = cur;
        }
        traj.joint_trajectory.points.push_back(temp_point);
    }

    // remove redundant points
    auto point_it = ++traj.joint_trajectory.points.begin();
    while (point_it != traj.joint_trajectory.points.end()) {
        auto last = std::prev(point_it);
        if (last->time_from_start.toSec() == point_it->time_from_start.toSec())
            traj.joint_trajectory.points.erase(last);
        ++point_it;
    }
}

shape_msgs::SolidPrimitive setPrim(int d, float x, float y, float z);
geometry_msgs::Pose setGeomPose(float x, float y, float z, float ox, float oy, float oz, float ow);

std::map<std::string, double> vector_to_map (const std::vector<std::string> &joint_names, std::vector<double> &joint_values)
{
  std::map<std::string, double> m;
  for (int i = 0; i < joint_names.size(); i++)
    m.insert({joint_names[i], joint_values[i]});
  return m;
}

int main(int argc, char** argv)
{
  //*********** Initialize ROSc
  ros::init(argc, argv, "drawingManager");
  ros::NodeHandle nh("~");

  //*********** ROS spinner.
  ros::AsyncSpinner spinner(1);
  spinner.start();

  DrawingManager dm(&nh);
  ros::Duration(3.0).sleep();


  //---------------- Moveit
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  moveit_msgs::RobotTrajectory trajectory;
  bool success; double fraction;
//  const moveit::core::RobotModelPtr& kinematic_model = robot_model_loader.getModel();
//  moveit::core::RobotState& kinematic_state = planning_scene.getCurrentStateNonConst();


  //---------------- RViz visual tools
  namespace rvt = rviz_visual_tools;
  moveit_visual_tools::MoveItVisualTools visual_tools(dm.rightArm->getPlanningFrame().c_str());
  visual_tools.deleteAllMarkers();
  visual_tools.loadRemoteControl();
  Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
  text_pose.translation().z() = 1.0;

  //---------------- Gripper
  std_msgs::Char gripper_msg;


  //---------------- joints initialization
//  dm.initPose();
//  dm.initPose_r();
//  dm.initPose_l();

  //save init pose
  dm.init_drawing_pose_r = dm.rightArm->getCurrentPose(dm.EE_LINK_R).pose;
  dm.init_drawing_pose_l = dm.leftArm->getCurrentPose(dm.EE_LINK_L).pose;
//  dm.init_drawing_pose_r.orientation.x = -0.7071068;
//  dm.init_drawing_pose_r.orientation.y = 0.7071068;
//  dm.init_drawing_pose_r.orientation.z = 0.0;
//  dm.init_drawing_pose_r.orientation.w = 0.0;
//  dm.init_drawing_pose_l.orientation.x = 0.7071068;
//  dm.init_drawing_pose_l.orientation.y = 0.7071068;
//  dm.init_drawing_pose_l.orientation.z = 0.0;
//  dm.init_drawing_pose_l.orientation.w = 0.0;

  geometry_msgs::PoseStamped current_cartesian_position, command_cartesian_position;
  std::vector<geometry_msgs::Pose> drawing_stroke;
  std::vector<geometry_msgs::Pose> linear_path;
  geometry_msgs::Pose drawing_point;

  // start reading input file
  // y,m left // c,k right
  dm.colors.push_back("y"); dm.colors.push_back("c");
  dm.colors.push_back("m"); dm.colors.push_back("k");
//  dm.drawing_file_name = "heart_path_";
  dm.drawing_file_name = "Starry_Night_path_";
//  dm.drawing_file_name = "bigben_path_";
//  dm.drawing_file_name = "dog_path_";
//  dm.drawing_file_name = "red_flower_path_";
//  dm.drawing_file_name = "bigben_1_path_";
//  dm.drawing_file_name = "sunflower_path_";

  for (int i = 0; i < dm.colors.size(); i++){
//  for (int i = 3; i < dm.colors.size(); i++){
    char ch[1];
    strcpy(ch, dm.colors[i].c_str());
    ROS_INFO("Drawing init");
    DrawingInput drawing(dm.drawing_file_name, ch[0], dm.init_drawing_pose_r, dm.init_drawing_pose_l);
    dm.drawings.push_back(drawing);
//    dm.visualizeStrokes(drawing.strokes, drawing.color);
//    ros::Duration(0.1).sleep();
  }

  // draw
  int stroke_num = 0;
  std_msgs::Int32 arm_num;
  arm_num.data = 0;

  for (int i = 0; i < dm.colors.size(); i ++)
  {
    dm.drawing_color_pub.publish(dm.drawings[i].color_);
    if (i%2 == 0) arm_num.data = 0; // right
    else          arm_num.data = 1; // left
    dm.arm_num_pub.publish(arm_num);

    stroke_num = 0;
    for (auto stroke : dm.drawings[i].strokes)
    {
      // move to ready position
      command_cartesian_position.pose = stroke[0];
      command_cartesian_position.pose.position.z = 1.312 + 0.025;
      linear_path.push_back(command_cartesian_position.pose);
      if (i%2 == 0) fraction = dm.rightArm->computeCartesianPath(linear_path, dm.eef_step, dm.jump_threshold, trajectory);
      else          fraction = dm.leftArm->computeCartesianPath(linear_path, dm.eef_step, dm.jump_threshold, trajectory);

      if (fraction < 0.5)
        ROS_WARN_STREAM("MOVE READY POSITION ERROR");
      else {
        ROS_INFO("PLANNING DONE");
        my_plan.trajectory_ = trajectory;
        dm.trajectory_pub.publish(trajectory);
//        visual_tools.trigger();
//        visual_tools.prompt("MOVE READY POSITION");
        ROS_INFO("MOVE READY POSITION");
        if (i%2 == 0) dm.rightArm->execute(my_plan);
        else          dm.leftArm->execute(my_plan);
        linear_path.clear();
      }

      std::cout << "Drawing " << dm.drawings[i].color << " " << stroke_num << "th stroke ... " << std::endl;
      if (i%2 == 0) fraction = dm.rightArm->computeCartesianPath(stroke, dm.eef_step, dm.jump_threshold, trajectory);
      else          fraction = dm.leftArm->computeCartesianPath(stroke, dm.eef_step, dm.jump_threshold, trajectory);

      if (fraction < 0.5)
        ROS_WARN_STREAM("DRAWING ERROR");
      else {
        ROS_INFO("PLANNING DONE");
        my_plan.trajectory_ = trajectory;
        dm.trajectory_pub.publish(trajectory);
//        visual_tools.trigger();
//        visual_tools.prompt("DRAWING");
        ROS_INFO("DRAWING");
//        if (i%2 == 0) dm.rightArm->execute(my_plan);
//        else          dm.leftArm->execute(my_plan);
        linear_path.clear();
      }

      ROS_INFO("MOVE TO LAST POSE");
      command_cartesian_position.pose = stroke[stroke.size()-1];
      linear_path.push_back(command_cartesian_position.pose);
      if (i%2 == 0) fraction = dm.rightArm->computeCartesianPath(linear_path, dm.eef_step, dm.jump_threshold, trajectory);
      else          fraction = dm.leftArm->computeCartesianPath(linear_path, dm.eef_step, dm.jump_threshold, trajectory);

      if (fraction < 0.5)
        ROS_WARN_STREAM("MOVE READY POSITION ERROR");
      else {
        ROS_INFO("PLANNING DONE");
        my_plan.trajectory_ = trajectory;

      }
      linear_path.clear();
      if (i%2 == 0) dm.rightArm->execute(my_plan);
      else          dm.leftArm->execute(my_plan);

      stroke_num++;
    }

    ROS_INFO("BACK TO INIT");
    geometry_msgs::Pose command_pose;
    if (i%2 == 0) {
//      std::vector<geometry_msgs::Pose> linear_path;
      command_pose.position.x = -0.65;
      command_pose.position.y = 0.35;
      command_pose.position.z = 1.50;
      command_pose.orientation.x = -0.7071068;
      command_pose.orientation.y = 0.7071068;
      command_pose.orientation.z = 0.0;
      command_pose.orientation.w = 0.0;
      linear_path.push_back(command_pose);
      dm.rightArm->computeCartesianPath(linear_path, dm.eef_step, dm.jump_threshold, trajectory);
      my_plan.trajectory_ = trajectory;
      dm.trajectory_pub.publish(trajectory);
      ros::Duration(1).sleep(); // wait for 3 sec
      dm.rightArm->execute(my_plan);
      linear_path.clear();
    }
    else {
      command_pose.position.x = -0.65;
      command_pose.position.y = -0.35;
      command_pose.position.z = 1.50;
      command_pose.orientation.x = 0.7071068;
      command_pose.orientation.y = 0.7071068;
      command_pose.orientation.z = 0.0;
      command_pose.orientation.w = 0.0;
      linear_path.push_back(command_pose);
      dm.leftArm->computeCartesianPath(linear_path, dm.eef_step, dm.jump_threshold, trajectory);
      my_plan.trajectory_ = trajectory;
      dm.trajectory_pub.publish(trajectory);
      ros::Duration(1).sleep(); // wait for 3 sec
      dm.leftArm->execute(my_plan);
      linear_path.clear();
    }


//    visual_tools.trigger();
//    gripper_msg.data = 'h';
//    if (i%2 == 0) dm.gripper_pub_right.publish(gripper_msg);
//    else          dm.gripper_pub_left.publish(gripper_msg);
//    visual_tools.trigger();
//    gripper_msg.data = 'c';
//    visual_tools.prompt("CLOSE");
//    if (i%2 == 0) dm.gripper_pub_right.publish(gripper_msg);
//    else          dm.gripper_pub_left.publish(gripper_msg);
  }

//  dm.initPose_r();
//  dm.initPose_l();
  dm.initPose();
  ros::Duration(3).sleep(); // wait for 3 sec

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