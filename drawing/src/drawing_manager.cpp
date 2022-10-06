#include "drawing_manager.h"

DrawingManager::DrawingManager(ros::NodeHandle* nh):nh_(*nh) {
  initMoveGroup();
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
  drawing_line_pub_2 = nh_.advertise<std_msgs::Bool>("/ready_to_draw_2", 1);
  drawing_color_pub_2 = nh_.advertise<geometry_msgs::Point>("/drawing_color_2", 1);
//  arm_num_pub_2 = nh_.advertise<std_msgs::Int32>("/arm_number_2", 1);
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

/*
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
*/

void DrawingManager::initPose() {
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  moveit_msgs::RobotTrajectory trajectory;

  // right arm
  std::vector<geometry_msgs::Pose> linear_path;
  geometry_msgs::Pose command_pose;
  command_pose.position.x = -0.5;
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

//    std::string hello;
//    for (int i=0; i <10; i ++){
//      std::cout << traj_l.joint_trajectory.points[i].time_from_start.nsec << std::endl;
//      std::cout << traj_r.joint_trajectory.points[i].time_from_start.nsec << std::endl;
//      std::cout << points[i].first << ", " << points[i].second << std::endl;
//      std::cin >> hello ;
//    }

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

int main(int argc, char** argv)
{
  //*********** Initialize ROSc
  ros::init(argc, argv, "drawingManager_2");
  ros::NodeHandle nh("~");

  DrawingManager dm(&nh);
  ros::Duration(1).sleep(); // wait for 1 sec

  //*********** ROS spinner.
  ros::AsyncSpinner spinner(1);
  spinner.start();

  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  moveit_msgs::RobotTrajectory trajectory_r;
  moveit_msgs::RobotTrajectory trajectory_l;
  moveit_msgs::RobotTrajectory trajectory;
  bool success; double fraction;

  robot_model_loader::RobotModelLoader robot_model_loader;
  const moveit::core::RobotModelPtr& kinematic_model = robot_model_loader.getModel();
//  ROS_INFO("Model frame: %s", kinematic_model->getModelFrame().c_str());
  planning_scene::PlanningScene planning_scene(kinematic_model);
  moveit::core::RobotState& kinematic_state = planning_scene.getCurrentStateNonConst();
  collision_detection::CollisionRequest collision_request;
  collision_detection::CollisionResult collision_result;


  // init pose
  // set all the joint values to the init joint position
  dm.initPose();
  ros::Duration(3).sleep(); // wait for 3 sec

  //save init pose
  dm.init_drawing_pose_r = dm.rightArm->getCurrentPose(dm.EE_LINK_R).pose;
  dm.init_drawing_pose_l = dm.leftArm->getCurrentPose(dm.EE_LINK_L).pose;

  geometry_msgs::PoseStamped current_cartesian_position, command_cartesian_position;
  std::vector<geometry_msgs::Pose> drawing_stroke;
  std::vector<geometry_msgs::Pose> linear_path;
  geometry_msgs::Pose drawing_point;

  // start reading input file
  dm.colors.push_back("y"); dm.colors.push_back("c"); dm.colors.push_back("m"); dm.colors.push_back("k");
  dm.drawing_file_name = "Starry_Night_path_";

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
  std::stringstream ss;

  std_msgs::Int32 arm_num;
  arm_num.data = -1;
  dm.arm_num_pub.publish(arm_num);

  bool done = false ,done_r = false, done_l = false;
  int color_i_r = 0, color_i_l = 0;
  int stroke_i_r = 0, stroke_i_l = 0;
  int point_i_r = 0, point_i_l = 0;
  int index = 0;

  dm.drawing_color_pub.publish(dm.drawings[color_i_r].color_);
  dm.drawing_color_pub_2.publish(dm.drawings[color_i_l].color_);
  while (!done) {
    if (!done_r) {
//      geometry_msgs::Pose point_r = dm.drawings[color_i_r].strokes_by_range[0][stroke_i_r][point_i_r];
      int cur_stroke_size_r = dm.drawings[color_i_r].strokes_by_range[0].size();
      int cur_point_size_r = dm.drawings[color_i_r].strokes_by_range[0][stroke_i_r].size();
      // check index
      if (point_i_r == cur_point_size_r) {
        point_i_r = 0;
        stroke_i_r++;
      }
      if (stroke_i_r == cur_stroke_size_r){
        stroke_i_r = 0;
        color_i_r++;
        dm.drawing_color_pub.publish(dm.drawings[color_i_r].color_);
      }
      // once more
      cur_stroke_size_r = dm.drawings[color_i_r].strokes_by_range[0].size();
      if (stroke_i_r == cur_stroke_size_r){
        stroke_i_r = 0;
        color_i_r++;
        dm.drawing_color_pub.publish(dm.drawings[color_i_r].color_);
      }

      if (color_i_r == dm.colors.size()){
        done_r = true;
      }
    }
    if (!done_l) {
//      geometry_msgs::Pose point_l = dm.drawings[color_i_l].strokes_by_range[2][stroke_i_l][point_i_l];
      int cur_stroke_size_l = dm.drawings[color_i_l].strokes_by_range[2].size();
      int cur_point_size_l = dm.drawings[color_i_l].strokes_by_range[2][stroke_i_l].size();

      // check index
      /////////////////////////////////////// done one stroke
      if (point_i_l == cur_point_size_l) {
        point_i_l = 0;
        stroke_i_l++;
      }
      /////////////////////////////////////// done one color
      if (stroke_i_l == cur_stroke_size_l){
        stroke_i_l = 0;
        color_i_l++;
        dm.drawing_color_pub_2.publish(dm.drawings[color_i_l].color_);
      }
      // check once more
      cur_stroke_size_l = dm.drawings[color_i_l].strokes_by_range[2].size();
      if (stroke_i_l == cur_stroke_size_l){
        stroke_i_l = 0;
        color_i_l++;
        dm.drawing_color_pub_2.publish(dm.drawings[color_i_l].color_);
      }
      if (color_i_l == dm.colors.size()){
        done_l = true;
      }
    }

    /////////////////////////////////////// done everything
    if (done_r && done_l){
      done = true;
      break;
    }

    /////////////////////////////////////// plan right
    if (!done_r) {
      command_cartesian_position.pose = dm.drawings[color_i_r].strokes_by_range[0][stroke_i_r][point_i_r];
      linear_path.push_back(command_cartesian_position.pose);
      fraction = dm.rightArm->computeCartesianPath(linear_path, dm.eef_step, dm.jump_threshold, trajectory_r);
      linear_path.clear();
    }
    /////////////////////////////////////// plan left
    if (!done_l) {
      command_cartesian_position.pose = dm.drawings[color_i_l].strokes_by_range[2][stroke_i_l][point_i_l];
      linear_path.push_back(command_cartesian_position.pose);
      fraction = dm.leftArm->computeCartesianPath(linear_path, dm.eef_step, dm.jump_threshold, trajectory_l);
      linear_path.clear();
    }

    if (!done_r && !done_l) { // merge plan
      dm.mergeTrajectories(trajectory_r, trajectory_l, trajectory);
      arm_num.data = -1;
//      dm.arm_num_pub.publish(arm_num);
      dm.trajectory_pub.publish(trajectory);
      my_plan.trajectory_ = trajectory;
      ready.data = true;
      if (point_i_r == 0) ready.data = false;
      dm.drawing_line_pub.publish(ready);
      if (point_i_l == 0) ready.data = false;
      dm.drawing_line_pub_2.publish(ready);
      ros::Duration(0.1).sleep();
      dm.arms->execute(my_plan);
      ROS_INFO("DRAWING ... ");
      ROS_INFO("R: %d, %d, %d", color_i_r, stroke_i_r, point_i_r);
      ROS_INFO("L: %d, %d, %d", color_i_l, stroke_i_l, point_i_l);
      ready.data = false;
      dm.drawing_line_pub.publish(ready);
      dm.drawing_line_pub_2.publish(ready);

      ss << index << " " << arm_num.data << " "
          << color_i_r << " "  << stroke_i_r << " "  <<point_i_r << " "
          << color_i_l << " "  << stroke_i_l << " "  <<point_i_l << "\n";
    }
    else if (done_l) {
      trajectory = trajectory_r;
      arm_num.data = 1;
//      dm.arm_num_pub.publish(arm_num);
      dm.trajectory_pub.publish(trajectory);
      my_plan.trajectory_ = trajectory;
      ready.data = true;
      if (point_i_r == 0) ready.data = false;
      dm.drawing_line_pub.publish(ready);
      ros::Duration(0.1).sleep();
      dm.rightArm->execute(my_plan);
      ROS_INFO("DRAWING ...");
      ROS_INFO("R: %d, %d, %d", color_i_r, stroke_i_r, point_i_r);
      ready.data = false;
      dm.drawing_line_pub.publish(ready);

      ss << index << " " << arm_num.data << " "
          << color_i_r << " "  << stroke_i_r << " "  <<point_i_r << "\n";
    }
    else if (done_r) {
      trajectory = trajectory_l;
      arm_num.data = 0;
//      dm.arm_num_pub.publish(arm_num);
      dm.trajectory_pub.publish(trajectory);
      my_plan.trajectory_ = trajectory;
      ready.data = true;
      if (point_i_l == 0) ready.data = false;
      dm.drawing_line_pub_2.publish(ready);
      ros::Duration(0.1).sleep();
      dm.leftArm->execute(my_plan);
      ROS_INFO("DRAWING ...");
      ROS_INFO("L: %d, %d, %d", color_i_l, stroke_i_l, point_i_l);
      ready.data = false;
      dm.drawing_line_pub_2.publish(ready);

      ss << index << " " << arm_num.data << " "
          << color_i_l << " "  << stroke_i_l << " "  <<point_i_l << "\n";
    }

    point_i_r++;
    point_i_l++;
    index++;
  }


  std::ofstream outfile (ros::package::getPath("drawing") + "/data/trajectory/bimanual/"+dm.drawing_file_name+"bimanual.txt",std::ios::app);
  if(!outfile.is_open())
  {
    ROS_INFO("open failed");
  }
  else
  {
    outfile<<ss.str()<<std::endl;
    outfile.close();
    ROS_INFO("File created");
  }

  // right arm
//  std_msgs::Int32 arm_num;
//  arm_num.data = -1;
//  dm.arm_num_pub.publish(arm_num);
/*
  for (int i = 0; i < dm.colors.size(); i ++)
//  for (int i = 0; i < 1; i ++)
  {
    dm.drawing_color_pub.publish(dm.drawings[i].color_);
    stroke_num = 0;

    int stroke_num_r = dm.drawings[i].strokes_by_range[0].size();
    int stroke_num_l = dm.drawings[i].storkes_by_range[2].size();

    Stroke stroke_r = dm.drawings[i].strokes_by_range[0][];
    Stroke stroke_l;

    // plan right
    command_cartesian_position.pose = dm.drawings[i].strokes_by_range[0][0][0];
    linear_path.push_back(command_cartesian_position.pose);
    fraction = dm.rightArm->computeCartesianPath(linear_path, dm.eef_step, dm.jump_threshold, trajectory_r);
    linear_path.clear();

    // plan left
    command_cartesian_position.pose = dm.drawings[i].strokes_by_range[2][0][0];
    linear_path.push_back(command_cartesian_position.pose);
    fraction = dm.leftArm->computeCartesianPath(linear_path, dm.eef_step, dm.jump_threshold, trajectory_l);
    linear_path.clear();

    // merge plan
    dm.mergeTrajectories(trajectory_r, trajectory_l, trajectory);
    dm.trajectory_pub.publish(trajectory);

    my_plan.trajectory_ = trajectory;
    dm.arms->execute(my_plan);
    ROS_INFO("MOVE READY POSITION");
*/
    /*
    for (auto stroke : dm.drawings[i].strokes_by_range[0])
    {
      // move to first position
      command_cartesian_position.pose = stroke[0];
      linear_path.push_back(command_cartesian_position.pose);
      double fraction = dm.rightArm->computeCartesianPath(linear_path, dm.eef_step, dm.jump_threshold, trajectory);
      ROS_INFO("PLANNING DONE");
      my_plan.trajectory_ = trajectory;
      dm.rightArm->execute(my_plan);  //ros::Duration(0.1).sleep();
      if (fraction < 0.5) ROS_WARN_STREAM("MOVE READY POSITION ERROR");
      ROS_INFO("MOVE READY POSITION");
      linear_path.clear();

      std::cout << "Drawing " << dm.drawings[i].color << " " << stroke_num << "th stroke ... " << std::endl;
      ready.data = true;
      dm.drawing_line_pub.publish(ready);
      for (int j = 0; j < stroke.size(); j++){
//        std::vector<double> joint_values;
//        bool found = kinematic_state.setFromIK(right_arm_, stroke[j], dm.EE_LINK_R, 0.1);
//        if (found){
//          kinematic_state.copyJointGroupPositions(right_arm_, joint_values);
//          dm.setJointValue(joint_values, 0);
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
    */
//  }

  dm.initPose();
  ros::Duration(3).sleep(); // wait for 3 sec

  ros::shutdown();
  return 0;
}
