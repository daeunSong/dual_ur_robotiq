#include "drawing_input_sphere.h"

DrawingInput::DrawingInput(const std::string &drawing_file_name,
                                   const geometry_msgs::Pose &init_drawing_pose_r,
                                   const geometry_msgs::Pose &init_drawing_pose_l) {
  this->drawing_file_name = drawing_file_name;
  this->init_drawing_pose_r = init_drawing_pose_r;
  this->init_drawing_pose_l = init_drawing_pose_l;
  readDrawingFile();
//  removeLongLine();
//  splitDrawing();
}

void DrawingInput::setColor() {
  if(this->color == 'c'){
    this->color_.x = 0.0; this->color_.y = 1.0; this->color_.z = 1.0;   // cyan (0, 255, 255)
  }else if(this->color == 'm'){
    this->color_.x = 1.0; this->color_.y = 0.0; this->color_.z = 1.0;   // magenta (255, 0, 255)
  }else if(this->color == 'y'){
    this->color_.x = 1.0; this->color_.y = 1.0; this->color_.z = 0.0;   // yellow (255, 255, 0)
  }else{ // black
    this->color_.x = 0.0; this->color_.y = 0.0; this->color_.z = 0.0;   // black (0, 0, 0)
  }
}


// read file line by line and save it in strokes
void DrawingInput::readDrawingFile() {
  std::string line;

  std::ifstream txt(ros::package::getPath("drawing") + "/data/input/sphere/" + this->drawing_file_name);
  // check if text file is well opened
  if(!txt.is_open()){
    ROS_ERROR("FILE NOT FOUND");
    return;
  }
  // first line indicates the size
  this->radius = 0.05;

  std::vector<std::string> tempSplit;
  double x, y, z;
  double xp, yp, zp;
  double sn_x, sn_y, sn_z;
  point_t pt, sn, orientation;

  Stroke stroke;
  geometry_msgs::Pose init_pose = this->init_drawing_pose_l;
  geometry_msgs::Pose drawing_pose;
  char input;
  int i = 0;

  while(std::getline(txt, line)) {  // first control point
    if (line == "End") { // end of stroke
      xp = x + sn_x * 0.05;
      yp = y + sn_y * 0.05;
      zp = z + sn_z * 0.05;

      drawing_pose.position.x = xp;
      drawing_pose.position.y = yp;
      drawing_pose.position.z = zp;
      stroke.push_back(drawing_pose);

      this->strokes.push_back(stroke);
      Stroke().swap(stroke);
      i = 0;
    }
    else {
      point_t().swap(sn);
      point_t().swap(pt);

      tempSplit = split(line, ' ');
      x = stod(tempSplit[0]) * this->radius + init_pose.position.x;
      y = stod(tempSplit[1]) * this->radius + init_pose.position.y;
      z = stod(tempSplit[2]) * this->radius + init_pose.position.z;

      sn_x = stod(tempSplit[3]);
      sn_y = stod(tempSplit[4]);
      sn_z = stod(tempSplit[5]);
      sn.push_back(sn_x); sn.push_back(sn_y); sn.push_back(sn_z);
      orientation = getQuaternion(stod(tempSplit[0]),sn);

      drawing_pose.orientation.x = orientation[0];
      drawing_pose.orientation.y = orientation[1];
      drawing_pose.orientation.z = orientation[2];
      drawing_pose.orientation.w = orientation[3];

      if (i == 0){
        xp = x + sn_x * 0.015;
        yp = y + sn_y * 0.015;
        zp = z + sn_z * 0.015;

        drawing_pose.position.x = xp;
        drawing_pose.position.y = yp;
        drawing_pose.position.z = zp;
        stroke.push_back(drawing_pose);
      }

      drawing_pose.position.x = x;
      drawing_pose.position.y = y;
      drawing_pose.position.z = z;
      stroke.push_back(drawing_pose);
      i++;
    }
  }
}


std::vector<std::string> DrawingInput::split(const std::string input, const char delimiter){
  std::vector<std::string> dat;
  std::stringstream str(input);
  std::string temp;
  while(getline(str, temp, delimiter)){
      dat.push_back(temp);
  }
  return dat;
}


point_t DrawingInput::getQuaternion(double x, point_t &sn){

  Eigen::Vector3d n (sn[0], sn[1], sn[2]);
  n.normalize();

  Eigen::Vector3d axis = Eigen::Vector3d::UnitZ().cross(n); // sin theta
  double theta = std::asin(std::min(std::max(axis.norm(),-1.0),1.0)); // theta in radius
  Eigen::Quaterniond q(Eigen::AngleAxisd(theta, axis));

  point_t orientation;
  orientation.push_back(q.x());
  orientation.push_back(q.y());
  orientation.push_back(q.z());
  orientation.push_back(q.w());

  return orientation;
}
