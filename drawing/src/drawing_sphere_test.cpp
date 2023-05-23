#include "drawing_manager_sphere.h"

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
//  this->planning_scene_interface = moveit::planning_interface::PlanningSceneInterface;

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
//  marker_pub = nh_.advertise<visualization_msgs::Marker>("/target_drawing", 100);
 marker_pub = nh_.advertise<geometry_msgs::PoseArray>("/target_drawing", 1);
  drawing_line_pub = nh_.advertise<std_msgs::Bool>("/ready_to_draw", 1);
  drawing_color_pub = nh_.advertise<geometry_msgs::Point>("/drawing_color", 1);
  color_pub = nh_.advertise<std_msgs::Int32>("/drawing_color_", 1);
  arm_num_pub = nh_.advertise<std_msgs::Int32>("/arm_number", 1);
//  drawing_line_pub_2 = nh_.advertise<std_msgs::Bool>("/ready_to_draw_2", 1);
//  drawing_color_pub_2 = nh_.advertise<geometry_msgs::Point>("/drawing_color_2", 1);
//  arm_num_pub_2 = nh_.advertise<std_msgs::Int32>("/arm_number_2", 1);
  trajectory_pub = nh_.advertise<moveit_msgs::RobotTrajectory>("/trajectory", 1);
  drawing_traj_pub_r = nh_.advertise<std_msgs::Int32>("/right_arm/traj_num", 1);
  drawing_traj_pub_l = nh_.advertise<std_msgs::Int32>("/left_arm/traj_num", 1);
  trajectory_pub_r = nh_.advertise<moveit_msgs::RobotTrajectory>("/right_arm/trajectory", 1);
  trajectory_pub_l = nh_.advertise<moveit_msgs::RobotTrajectory>("/left_arm/trajectory", 1);
  gripper_pub_left = nh_.advertise<std_msgs::Char>("/left_gripper/gripper_left", 10);
  gripper_pub_right = nh_.advertise<std_msgs::Char>("/right_gripper/gripper_right", 10);

  drawing_comm_r = nh_.advertise<std_msgs::Int32>("/right_arm/drawing_command", 1);
  drawing_comm_l = nh_.advertise<std_msgs::Int32>("/left_arm/drawing_command", 1);
  drawing_state_r = nh_.subscribe("/right_arm/drawing_state", 10, &DrawingManager::drawCallback_r, this);
  drawing_state_l = nh_.subscribe("/left_arm/drawing_state", 10, &DrawingManager::drawCallback_l, this);
  tool_comm_r = nh_.advertise<std_msgs::Int32>("/right_arm/tool_command", 1);
  tool_comm_l = nh_.advertise<std_msgs::Int32>("/left_arm/tool_command", 1);
  tool_state_r = nh_.subscribe("/right_arm/tool_state", 10, &DrawingManager::toolCallback_r, this);
  tool_state_l = nh_.subscribe("/left_arm/tool_state", 10, &DrawingManager::toolCallback_l, this);
}

void DrawingManager::drawCallback_r(const std_msgs::Int32::ConstPtr& msg){
  this->drawing_r_done = msg->data;
  std::cout << "msg recieved" << std::endl;
}

void DrawingManager::drawCallback_l(const std_msgs::Int32::ConstPtr& msg){
  this->drawing_l_done = msg->data;
  std::cout << "msg recieved" << std::endl;
}

void DrawingManager::toolCallback_r(const std_msgs::Int32::ConstPtr& msg){
  this->tool_change_r_done = msg->data;
  std::cout << "msg recieved" << std::endl;
}

void DrawingManager::toolCallback_l(const std_msgs::Int32::ConstPtr& msg){
  this->tool_change_l_done = msg->data;
  std::cout << "msg recieved" << std::endl;
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

void DrawingManager::initPose (){
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;

  this->arms->setStartStateToCurrentState();
  this->arms->setJointValueTarget("left_shoulder_pan_joint", 0*D2R); //
  this->arms->setJointValueTarget("left_shoulder_lift_joint", -180*D2R); //
  this->arms->setJointValueTarget("left_elbow_joint", 0*D2R); //
  this->arms->setJointValueTarget("left_wrist_1_joint", 0*D2R); //
  this->arms->setJointValueTarget("left_wrist_2_joint", 0*D2R); //
  this->arms->setJointValueTarget("left_wrist_3_joint", 0*D2R);  //

  // [RIGHT]: init joint pose
  this->arms->setJointValueTarget("right_shoulder_pan_joint", 0*D2R); //
  this->arms->setJointValueTarget("right_shoulder_lift_joint", 0*D2R); //
  this->arms->setJointValueTarget("right_elbow_joint", 0*D2R); //
  this->arms->setJointValueTarget("right_wrist_1_joint", 0*D2R); //
  this->arms->setJointValueTarget("right_wrist_2_joint", 0*D2R); //
  this->arms->setJointValueTarget("right_wrist_3_joint", 0*D2R);  //
  bool success = (this->arms->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  if(!success){
    ROS_INFO("Plan did not successed");
  }
  this->arms->execute(my_plan);
}

void DrawingManager::left_drawing_initPose (){
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;

  this->arms->setStartStateToCurrentState();
  this->arms->setJointValueTarget("left_shoulder_pan_joint", 41*D2R); //
  this->arms->setJointValueTarget("left_shoulder_lift_joint", -130*D2R); //
  this->arms->setJointValueTarget("left_elbow_joint", -101*D2R); //
  this->arms->setJointValueTarget("left_wrist_1_joint", -6*D2R); //
  this->arms->setJointValueTarget("left_wrist_2_joint", 122*D2R); //
  this->arms->setJointValueTarget("left_wrist_3_joint", 6*D2R);  //

  // [RIGHT]: init joint pose
  this->arms->setJointValueTarget("right_shoulder_pan_joint", -206*D2R); //
  this->arms->setJointValueTarget("right_shoulder_lift_joint", -129*D2R); //
  this->arms->setJointValueTarget("right_elbow_joint", -50*D2R); //
  this->arms->setJointValueTarget("right_wrist_1_joint", -67*D2R); //
  this->arms->setJointValueTarget("right_wrist_2_joint", 50*D2R); //
  this->arms->setJointValueTarget("right_wrist_3_joint", 98*D2R);  //
  bool success = (this->arms->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  if(!success){
    ROS_INFO("Plan did not successed");
  }
  this->arms->execute(my_plan);
}


void DrawingManager::initPose_r (){
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;

  // [RIGHT]: init joint pose
  this->rightArm->setJointValueTarget("right_shoulder_pan_joint", -35*D2R); //
  this->rightArm->setJointValueTarget("right_shoulder_lift_joint", -82*D2R); //
  this->rightArm->setJointValueTarget("right_elbow_joint", 100*D2R); //
  this->rightArm->setJointValueTarget("right_wrist_1_joint", -138*D2R); //
  this->rightArm->setJointValueTarget("right_wrist_2_joint", -125*D2R); //
  this->rightArm->setJointValueTarget("right_wrist_3_joint", 90*D2R);  //
  bool success = (this->rightArm->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  if(!success){
    ROS_INFO("Plan did not successed");
  }
  this->rightArm->execute(my_plan);
//  moveit_msgs::RobotTrajectory trajectory;
//
//  // right arm
//  std::vector<geometry_msgs::Pose> linear_path;
//  geometry_msgs::Pose command_pose;
//  command_pose = this->init_drawing_pose_r;
////  command_pose.position.x = -0.65;
////  command_pose.position.y = 0.35;
////  command_pose.position.z = 1.35;
////  command_pose.orientation.x = -0.7071068;
////  command_pose.orientation.y = 0.7071068;
////  command_pose.orientation.z = 0.0;
////  command_pose.orientation.w = 0.0;
//  linear_path.push_back(command_pose);
//  this->rightArm->computeCartesianPath(linear_path, this->eef_step, this->jump_threshold, trajectory);
//  my_plan.trajectory_ = trajectory;
////  this->trajectory_pub.publish(trajectory);
//  ros::Duration(1).sleep(); // wait for 3 sec
//  this->rightArm->execute(my_plan);
//  linear_path.clear();
}

void DrawingManager::initPose_l (){
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  moveit_msgs::RobotTrajectory trajectory;

  // right arm
  std::vector<geometry_msgs::Pose> linear_path;
  geometry_msgs::Pose command_pose;
  command_pose = this->init_drawing_pose_l;
//  command_pose.position.x = -0.65;
//  command_pose.position.y = -0.35;
//  command_pose.position.z = 1.50;
//  command_pose.orientation.x = 0.7071068;
//  command_pose.orientation.y = 0.7071068;
//  command_pose.orientation.z = 0.0;
//  command_pose.orientation.w = 0.0;
  this->leftArm->computeCartesianPath(linear_path, this->eef_step, this->jump_threshold, trajectory);
  my_plan.trajectory_ = trajectory;
//  this->trajectory_pub.publish(trajectory);
  ros::Duration(1).sleep(); // wait for 3 sec
  this->leftArm->execute(my_plan);
  linear_path.clear();
}

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

shape_msgs::SolidPrimitive setPrim(float r);
geometry_msgs::Pose setGeomPose(float x, float y, float z, float ox, float oy, float oz, float ow);

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
  moveit::planning_interface::MoveGroupInterface::Plan my_plan, ready_plan;
  moveit_msgs::RobotTrajectory trajectory;
  moveit_msgs::RobotTrajectory trajectory_;
  std::vector<moveit_msgs::RobotTrajectory> trajectory_full;
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
  char input;

//  geometry_msgs::Pose init_sphere_pose = dm.rightArm->getCurrentPose(dm.EE_LINK_R).pose;
//  init_sphere_pose.position.x += 0.35;
////  init_sphere_pose.position.z -= 0.1;
//
  std::vector<geometry_msgs::Pose> linear_path;
//
//  linear_path.push_back(init_sphere_pose);
//  fraction = dm.rightArm->computeCartesianPath(linear_path, dm.eef_step, dm.jump_threshold, trajectory, false);
//  my_plan.trajectory_ = trajectory;
//  std::cin >> input;
//  if (fraction == 1.0 ) dm.rightArm->execute(my_plan);
//  linear_path.clear();

  geometry_msgs::Pose command_pose = dm.rightArm->getCurrentPose(dm.EE_LINK_R).pose;
  command_pose.position.z -= 0.15;

  linear_path.push_back(command_pose);
  fraction = dm.rightArm->computeCartesianPath(linear_path, dm.eef_step, dm.jump_threshold, trajectory, false);
  my_plan.trajectory_ = trajectory;
  std::cin >> input;
  if (fraction == 1.0 ) dm.rightArm->execute(my_plan);
  linear_path.clear();
  dm.trajectory_pub.publish(trajectory);

  ros::Duration(3).sleep(); // wait for 3 sec
//
//  gripper_msg.data = 'o';
//  dm.gripper_pub_right.publish(gripper_msg);

  ros::shutdown();
  return 0;
}

shape_msgs::SolidPrimitive setPrim(float r)
{
    shape_msgs::SolidPrimitive pr;
    pr.type = pr.SPHERE;
    pr.dimensions.resize(1);
    pr.dimensions[pr.SPHERE_RADIUS] = r;

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