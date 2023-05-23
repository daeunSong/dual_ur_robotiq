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
  command_pose = this->init_drawing_pose_r;
//  command_pose.position.x = -0.65;
//  command_pose.position.y = 0.35;
//  command_pose.position.z = 1.35;
//  command_pose.orientation.x = -0.7071068;
//  command_pose.orientation.y = 0.7071068;
//  command_pose.orientation.z = 0.0;
//  command_pose.orientation.w = 0.0;
  linear_path.push_back(command_pose);
  this->rightArm->computeCartesianPath(linear_path, this->eef_step, this->jump_threshold, trajectory);
  my_plan.trajectory_ = trajectory;
//  this->trajectory_pub.publish(trajectory);
  ros::Duration(1).sleep(); // wait for 3 sec
  this->rightArm->execute(my_plan);
  linear_path.clear();
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

shape_msgs::SolidPrimitive setPrim(int d, float x, float y, float z);
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
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
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

  //---------------- joints initialization
//  dm.initPose();
//  dm.initPose_r();
//  dm.initPose_l();

  //save init pose
  dm.init_drawing_pose_r = dm.rightArm->getCurrentPose(dm.EE_LINK_R).pose;
  dm.init_drawing_pose_l = dm.leftArm->getCurrentPose(dm.EE_LINK_L).pose;
  geometry_msgs::Pose move_right = dm.init_drawing_pose_r;
  geometry_msgs::Pose move_left = dm.init_drawing_pose_l;
  move_right.position.x = -0.55;
  move_right.position.z = 1.40;
  move_right.position.y = 0.27;
  move_left.position.x = -0.55;
  move_left.position.z = 1.40;
  move_left.position.y = -0.27;

  geometry_msgs::PoseStamped current_cartesian_position, command_cartesian_position;
  std::vector<geometry_msgs::Pose> drawing_stroke;
  std::vector<geometry_msgs::Pose> linear_path;
  std::vector<geometry_msgs::Pose> linear_path_r;
  std::vector<geometry_msgs::Pose> linear_path_l;
  geometry_msgs::Pose drawing_point;

  linear_path_r.push_back(dm.init_drawing_pose_r);
  linear_path_l.push_back(dm.init_drawing_pose_l);

  // start reading input file
  dm.drawing_file_name = "actions_path.csv";
  DrawingInput drawing(dm.drawing_file_name, dm.init_drawing_pose_r, dm.init_drawing_pose_l);
  ROS_INFO("Drawing init");
  ROS_INFO("stroke num: " + drawing.stroke_num);
  ros::Duration(1).sleep(); // wait for 3 sec



  // draw
  int stroke_num = 802;
  std_msgs::Int32 arm_num;
  arm_num.data = 0;
  std_msgs::Int32 color;
  std_msgs::Int32 gripper_ready;
  gripper_ready.data = 0;
  dm.ready_to_draw = 0;

  std_msgs::Int32 tool_num;
  std_msgs::Int32 traj_num;

  int right_index_bound = drawing.strokes_r.size();
  int left_index_bound = drawing.strokes_l.size();
  int right_index = 440;
  int left_index = 364;
  Stroke stroke_r, stroke_l;
  int color_r = 0;
  int color_l = 1;
  int curr_color_r = 0;
  int curr_color_l = 1;
  int stroke_num_r = 803;
  int stroke_num_l = 802;
  bool right_sent = true;
  bool left_sent = true;
  double min_y = 1000;
  double max_y = -1000;
  int arm_num_curr = -1;
  int arm_num_prev = -2;

  dm.tool_change_r_done = true;
  dm.tool_change_l_done = true;

/*
  // planning first
  ROS_INFO("START PLANNING");
  int i = 0;
  int arm_prev = 1;
  for (auto stroke_ : drawing.strokes_full) {

    std::cout << "Planning " << i << "th stroke \n";
    int arm = std::get<1>(stroke_);
    Stroke stroke = std::get<0>(stroke_);

    // move to init position
    if (arm != arm_prev){
      if (arm_prev == 0) {
        ROS_INFO("RIGHT ARM MOVE A BIT");
        linear_path.push_back(move_right);
        fraction = dm.rightArm->computeCartesianPath(linear_path, dm.eef_step, dm.jump_threshold, trajectory, false);
        my_plan.trajectory_ = trajectory;
        dm.rightArm->execute(my_plan);
        linear_path.clear();
      }
      else{
        ROS_INFO("LEFT ARM MOVE A BIT");
        linear_path.push_back(move_left);
        fraction = dm.leftArm->computeCartesianPath(linear_path, dm.eef_step, dm.jump_threshold, trajectory, false);
        my_plan.trajectory_ = trajectory;
        dm.leftArm->execute(my_plan);
        linear_path.clear();
      }
    }

    // move to ready position
    command_cartesian_position.pose = stroke[0];
    command_cartesian_position.pose.position.z = 1.35;
    linear_path.push_back(command_cartesian_position.pose);
    if (arm == 0) fraction = dm.rightArm->computeCartesianPath(linear_path, dm.eef_step, dm.jump_threshold, trajectory, false);
    else          fraction = dm.leftArm->computeCartesianPath(linear_path, dm.eef_step, dm.jump_threshold, trajectory, false);

    if (fraction < 0.5)
      ROS_WARN_STREAM("MOVE READY POSITION ERROR");
    else {
      my_plan.trajectory_ = trajectory;
//      dm.trajectory_pub.publish(trajectory);
      ROS_INFO("MOVE READY POSITION");
      if (arm == 0) dm.rightArm->execute(my_plan);
      else          dm.leftArm->execute(my_plan);
      linear_path.clear();
    }

    // draw stroke
    if (arm == 0) fraction = dm.rightArm->computeCartesianPath(stroke, dm.eef_step, dm.jump_threshold, trajectory, false);
    else          fraction = dm.leftArm->computeCartesianPath(stroke, dm.eef_step, dm.jump_threshold, trajectory, false);

    if (fraction < 0.5)
      ROS_WARN_STREAM("DRAW STROKE ERROR");
    else {
      my_plan.trajectory_ = trajectory;
      dm.trajectory_pub.publish(trajectory);
      ROS_INFO("DRAW STROKE");
//      if (i%2 == 0) dm.rightArm->execute(my_plan);
//      else          dm.leftArm->execute(my_plan);
    }

    trajectory_full.push_back(trajectory);
    arm_prev = arm;
    i ++;
  }

  std::string input;
  std::cout << "Wait for user input: \n";
  std::cin >> input;
*/
  int count = 0;


  // execution
  for (int i = 803; i < drawing.stroke_num; i ++) {
//    ros::Duration(1).sleep(); // wait for 3 sec
    // read right
    if (right_index < right_index_bound && right_sent){
      stroke_num_r = std::get<0>(drawing.strokes_r[right_index]);
      stroke_r = std::get<1>(drawing.strokes_r[right_index]);
      color_r = std::get<2>(drawing.strokes_r[right_index]);
      min_y = std::get<3>(drawing.strokes_r[right_index]);
      right_index++;
      right_sent = false;
    }
    // read left
    if (left_index < left_index_bound && left_sent) {
      stroke_num_l = std::get<0>(drawing.strokes_l[left_index]);
      stroke_l = std::get<1>(drawing.strokes_l[left_index]);
      color_l = std::get<2>(drawing.strokes_l[left_index]);
      max_y = std::get<3>(drawing.strokes_r[left_index]);
      left_index++;
      left_sent = false;
    }

    std::cout << "i: " << i << std::endl;
    std::cout << "right_index: " << right_index << std::endl;
    std::cout << "left_index: " << left_index << std::endl;
    std::cout << "stroke_num_r: " << stroke_num_r << std::endl;
    std::cout << "stroke_num_l: " << stroke_num_l << std::endl;
    std::cout << "color_r: " << color_r << std::endl;
    std::cout << "color_l: " << color_l << std::endl;
    std::cout << "min_y: " << min_y << std::endl;
    std::cout << "max_y: " << max_y << std::endl;

    //////////////////////////////////// tool change
    ///////////////////// make sure arm is in init pose before executing tool change !!!
    if (curr_color_r != color_r){ // right
      if (i == 0)  curr_color_r = color_r;
      // move to init pose
      fraction = dm.rightArm->computeCartesianPath(linear_path_r, dm.eef_step, dm.jump_threshold, trajectory, false);
      my_plan.trajectory_ = trajectory;
      dm.rightArm->execute(my_plan);
      // place
      if (i != 0){ // no place for the first index
        ROS_INFO("RIGHT TOOL PLACE");
        tool_num.data = curr_color_r + 10;
        std::cout << tool_num.data << std::endl;
        dm.tool_comm_r.publish(tool_num);
      }
      // pick
      ROS_INFO("RIGHT TOOL PICK");
      tool_num.data = color_r;
      std::cout << tool_num.data << std::endl;
      dm.tool_comm_r.publish(tool_num);
      curr_color_r = color_r;
      dm.tool_change_r_done = 0;
    }
    if (curr_color_l != color_l){ // left
      if (i == 0)  curr_color_l = color_l;
      // move to init pose
      fraction = dm.leftArm->computeCartesianPath(linear_path_l, dm.eef_step, dm.jump_threshold, trajectory, false);
      my_plan.trajectory_ = trajectory;
      dm.leftArm->execute(my_plan);
      // place
      if (i != 0){ // no place for the first index
        ROS_INFO("LEFT TOOL PLACE");
        tool_num.data = curr_color_l + 10;
        std::cout << tool_num.data << std::endl;
        dm.tool_comm_l.publish(tool_num);
      }
      // pick
      ROS_INFO("LEFT TOOL PICK");
      tool_num.data = color_l;
      std::cout << tool_num.data << std::endl;
      dm.tool_comm_l.publish(tool_num);
      curr_color_l = color_l;
      dm.tool_change_l_done = 0;
    }

    while (!dm.tool_change_r_done || !dm.tool_change_l_done){
      // wait for tool change
    }
    ROS_INFO("DONE");

    //////////////////////////////////// draw
    ///////////// right arm
    std::cout << "Drawing " << i << "th stroke " << std::endl;
    if (stroke_num_r == i){
      arm_num_curr = 0;
      if (i!= 0 && arm_num_prev != arm_num_curr) {
        // move the left arm (previous arm)
        ROS_INFO("LEFT ARM MOVE A BIT");
        linear_path.push_back(move_left);
        fraction = dm.leftArm->computeCartesianPath(linear_path, dm.eef_step, dm.jump_threshold, trajectory_, false);
//        dm.trajectory_pub_l.publish(trajectory_);
        linear_path.clear();
      }

      // plan right arm
      // move to ready position
      dm.drawing_r_done = 0;
      command_cartesian_position.pose = stroke_r[0];
      command_cartesian_position.pose.position.z = 1.35;
      linear_path.push_back(command_cartesian_position.pose);
//      linear_path.push_back(stroke_r[0]);
      fraction = dm.rightArm->computeCartesianPath(linear_path, dm.eef_step, dm.jump_threshold, trajectory, false);
      if (i!= 0 && arm_num_prev != arm_num_curr)
        dm.trajectory_pub_l.publish(trajectory_);

      dm.trajectory_pub_r.publish(trajectory);
      linear_path.clear();
      while (!dm.drawing_r_done) {
        // wait until finish
      }
      fraction = dm.rightArm->computeCartesianPath(stroke_r, dm.eef_step, dm.jump_threshold, trajectory, false);
      dm.trajectory_pub_r.publish(trajectory);
      dm.drawing_r_done = 0;
//      dm.trajectory_pub_r.publish(trajectory_full[i]);
//      traj_num.data = i;
//      dm.drawing_traj_pub_r.publish(traj_num);
      ROS_INFO("RIGHT ARM DRAW");

      arm_num_prev = arm_num_curr;
//      if (stroke_num_l - stroke_num_r == 1){  // if index diff is only one
//        if (min_y - max_y >= 0.13){ // bigger than 13cm
//          // draw simultaneously
//          // plan left arm
//          // move to ready position
//          dm.drawing_l_done = 0;
//
////          linear_path.push_back(stroke_l[0]);
//          command_cartesian_position.pose = stroke_l[0];
//          command_cartesian_position.pose.position.z = 1.35;
//          linear_path.push_back(command_cartesian_position.pose);
//          fraction = dm.leftArm->computeCartesianPath(linear_path, dm.eef_step, dm.jump_threshold, trajectory, false);
//          dm.trajectory_pub_l.publish(trajectory);
//          linear_path.clear();
//          while (!dm.drawing_l_done) {
//            // wait until finish
//          }
////          fraction = dm.leftArm->computeCartesianPath(stroke_l, dm.eef_step, dm.jump_threshold, trajectory);
////          dm.trajectory_pub_l.publish(trajectory);
//          dm.drawing_l_done = 0;
////          dm.trajectory_pub_l.publish(trajectory_full[i+1]);
//          traj_num.data = i+1;
//          dm.drawing_traj_pub_l.publish(traj_num);
//          ROS_INFO("SS LEFT ARM DRAW");
//
//          left_sent = true;
//          i++;
//          count++;
//          arm_num_prev = -1;
//        }
//      }
      right_sent = true;
    }
    ///////////// left arm
    else if (stroke_num_l == i){
      arm_num_curr = 1;
      if (i!= 0 && arm_num_prev != arm_num_curr) {
        // move the right arm (previous arm)
        ROS_INFO("RIGHT ARM MOVE A BIT");
        linear_path.push_back(move_right);
        fraction = dm.rightArm->computeCartesianPath(linear_path, dm.eef_step, dm.jump_threshold, trajectory_, false);
//        dm.trajectory_pub_r.publish(trajectory_);
        linear_path.clear();
      }

      // plan left arm
      // move to ready position
      dm.drawing_l_done = 0;
//      linear_path.push_back(stroke_l[0]);
      command_cartesian_position.pose = stroke_l[0];
      command_cartesian_position.pose.position.z = 1.35;
      linear_path.push_back(command_cartesian_position.pose);
      fraction = dm.leftArm->computeCartesianPath(linear_path, dm.eef_step, dm.jump_threshold, trajectory, false);
      if (i!= 0 && arm_num_prev != arm_num_curr)
        dm.trajectory_pub_r.publish(trajectory_);
      dm.trajectory_pub_l.publish(trajectory);
      linear_path.clear();
      while (!dm.drawing_l_done) {
        // wait until finish
      }
      fraction = dm.leftArm->computeCartesianPath(stroke_l, dm.eef_step, dm.jump_threshold, trajectory);
      dm.trajectory_pub_l.publish(trajectory);
      dm.drawing_l_done = 0;
//      dm.trajectory_pub_l.publish(trajectory_full[i]);
//      traj_num.data = i;
//      dm.drawing_traj_pub_l.publish(traj_num);
      ROS_INFO("LEFT ARM DRAW");

      arm_num_prev = arm_num_curr;
//      if (stroke_num_r - stroke_num_l == 1){ // if index diff is only one
//        if (min_y - max_y >= 0.13){
//          // draw simultaneously
//          // plan right arm
//          // move to ready position
//          dm.drawing_r_done = 0;
////          linear_path.push_back(stroke_r[0]);
//          command_cartesian_position.pose = stroke_r[0];
//          command_cartesian_position.pose.position.z = 1.35;
//          linear_path.push_back(command_cartesian_position.pose);
//          fraction = dm.rightArm->computeCartesianPath(linear_path, dm.eef_step, dm.jump_threshold, trajectory, false);
//          dm.trajectory_pub_r.publish(trajectory);
//          linear_path.clear();
//          while (!dm.drawing_r_done) {
//            // wait until finish
//          }
////          fraction = dm.rightArm->computeCartesianPath(stroke_r, dm.eef_step, dm.jump_threshold, trajectory);
////          dm.trajectory_pub_r.publish(trajectory);
//          dm.drawing_r_done = 0;
////          dm.trajectory_pub_r.publish(trajectory_full[i+1]);
//          traj_num.data = i+1;
//          dm.drawing_traj_pub_r.publish(traj_num);
//          ROS_INFO("SS RIGHT ARM DRAW");
//
//          right_sent = true;
//          i++;
//          count++;
//          arm_num_prev = -1;
//        }
//      }
      // send left arm trajectory
      left_sent = true;
    }

    while (!dm.drawing_r_done || !dm.drawing_l_done){
      // wait for drawing stroke
    }

  }

  fraction = dm.leftArm->computeCartesianPath(linear_path_l, dm.eef_step, dm.jump_threshold, trajectory, false);
  dm.trajectory_pub_l.publish(trajectory);
//  my_plan.trajectory_ = trajectory;
//  dm.leftArm->execute(my_plan);

  // TODO: ERROR
  fraction = dm.rightArm->computeCartesianPath(linear_path_r, dm.eef_step, dm.jump_threshold, trajectory, false);
  dm.trajectory_pub_r.publish(trajectory);
//  my_plan.trajectory_ = trajectory;
//  dm.rightArm->execute(my_plan);


  while (!dm.drawing_r_done || !dm.drawing_l_done){
    // wait for drawing stroke
  }

  ROS_INFO("LEFT TOOL PLACE");
  tool_num.data = curr_color_l + 10;
  dm.tool_comm_l.publish(tool_num);

  ROS_INFO("RIGHT TOOL PLACE");
  tool_num.data = curr_color_r + 10;
  dm.tool_comm_r.publish(tool_num);

  std::cout << "simultaneouls drawing: " << count << std::endl;

/*
  for (int i = 0; i < dm.colors.size(); i ++)
  {
//    dm.drawing_color_pub.publish(dm.drawings[i].color_);
    color.data = i;
    dm.color_pub.publish(color);
    if (i%2 == 0) arm_num.data = 0; // right
    else          arm_num.data = 1; // left
    dm.arm_num_pub.publish(arm_num);

    gripper_ready.data = 1;
    if (i%2 == 0) dm.drawing_comm_r.publish(gripper_ready);
    else          dm.drawing_comm_l.publish(gripper_ready);

    while (dm.ready_to_draw == 0) {
      // wait for gripper
//      std::cout << "waiting" << std::endl;
    }

    stroke_num = 0;
    for (auto stroke : dm.drawings[i].strokes)
//    for (int j = 0; j < dm.drawings[i].strokes.size(); j+2)
    {
//      Stroke stroke = dm.drawings[i].strokes[j];
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
        if (i%2 == 0) dm.rightArm->execute(my_plan);
        else          dm.leftArm->execute(my_plan);
        linear_path.clear();
      }

//      ROS_INFO("MOVE TO LAST POSE");
//      command_cartesian_position.pose = stroke[stroke.size()-1];
//      linear_path.push_back(command_cartesian_position.pose);
//      if (i%2 == 0) fraction = dm.rightArm->computeCartesianPath(linear_path, dm.eef_step, dm.jump_threshold, trajectory);
//      else          fraction = dm.leftArm->computeCartesianPath(linear_path, dm.eef_step, dm.jump_threshold, trajectory);

//      if (fraction < 0.5)
//        ROS_WARN_STREAM("MOVE READY POSITION ERROR");
//      else {
//        ROS_INFO("PLANNING DONE");
//        my_plan.trajectory_ = trajectory;
//
//      }
//      linear_path.clear();
//      if (i%2 == 0) dm.rightArm->execute(my_plan);
//      else          dm.leftArm->execute(my_plan);

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

    gripper_ready.data = 1;
    if (i%2 == 0) dm.drawing_comm_r.publish(gripper_ready);
    else          dm.drawing_comm_l.publish(gripper_ready);

    dm.ready_to_draw = 0;
    while (dm.ready_to_draw == 0) {
      // wait for gripper
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
*/
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