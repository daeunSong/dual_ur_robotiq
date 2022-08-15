#include "drawing_input.h"

DrawingInput::DrawingInput(const std::string &drawing_file_name,
                                  const char &color, const geometry_msgs::Pose &init_drawing_pose) {
  setFileName(drawing_file_name, color);
  this->drawing_pose = init_drawing_pose;
  readDrawingFile();
  removeLongLine();
}

void DrawingInput::setFileName(const std::string drawing_file_name, const char color) {
  this->drawing_file_name = drawing_file_name;
  this->color = color;
  setColor();
  this->drawing_file_name_full = drawing_file_name + color + ".txt";
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


void DrawingInput::setTargetSize (const double target_size) {
  this->target_size = target_size;
}

void DrawingInput::setDrawingSize (const double ratio) {
  this->ratio = ratio;
  double width = ratio * this->target_size;
  double height = this->target_size;
  this->size.push_back(width);
  this->size.push_back(height);
}

// read file line by line and save it in strokes
void DrawingInput::readDrawingFile() {
  std::string line;

  std::ifstream txt(ros::package::getPath("drawing") + "/data/input/" + this->drawing_file_name_full);
  // check if text file is well opened
  if(!txt.is_open()){
    ROS_ERROR("FILE NOT FOUND");
    return;
  }
  // first line indicates the size
  std::getline(txt, line); // drawing size
  std::vector<std::string> tempSplit = split(line, ' ');
  double width = stod(tempSplit[0]);
  double height = stod(tempSplit[1]);
  setDrawingSize(width/height);

  double x, y;
  Stroke stroke;
  geometry_msgs::Pose drawing_pose = this->drawing_pose;

  while(std::getline(txt, line)) {
    if(line == "End") { // stroke finished
      this->strokes.push_back(stroke);
      //stroke.clear();
      Stroke().swap(stroke);
    }
    else { // start reading strokes
      tempSplit = split(line, ' ');
      y = (-stod(tempSplit[0])+0.5) * this->ratio * this->target_size + this->drawing_pose.position.y;
      x = (stod(tempSplit[1])-0.5) * this->target_size + this->drawing_pose.position.x;
      drawing_pose.position.x = x;
      drawing_pose.position.y = y;
      drawing_pose.position.z = this->drawing_pose.position.z - 0.15;
      stroke.push_back(drawing_pose);
    }
  }
}

// remove the long distanced lines
void DrawingInput::removeLongLine() {
  std::vector<Stroke> new_strokes;
  int num_strokes = this->strokes.size();
  for (int i = 0; i < this->strokes.size(); i++) {  // strokes
    Stroke stroke;
    geometry_msgs::Pose p1 = this->strokes[i][0];
    for (int j = 1; j < this->strokes[i].size(); j++) { // points
      geometry_msgs::Pose p2 = this->strokes[i][j];
      double dist = this->getDist(p1, p2);
      if (dist > 0.03) { // 3cm
        if (stroke.size() > 5)
          new_strokes.push_back(stroke);
        Stroke().swap(stroke);
        stroke.push_back(p2);
      }
      else {
        stroke.push_back(p2);
      }
      p1 = p2;
    }
    new_strokes.push_back(stroke);
  }
  this->strokes = new_strokes;
}

double DrawingInput::getDist(geometry_msgs::Pose p1, geometry_msgs::Pose p2) {
  return sqrt(pow(p1.position.x - p2.position.x, 2)+ pow(p1.position.y - p2.position.y, 2));
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

/*
std::tuple<double, point_t> DrawingInput::getXAndQuaternion(point_t &pt){
  double x; point_t sn;

  tie(x, sn) = getXAndSurfaceNormal(pt);

  // std::cout<< "5) Got Surface Normal\n";

  point_t zaxis{0,0,1};

  // calc cosine
  double vSize = getVectorSize(sn);
  if(vSize != 1){
    for(int i = 0; i < 3; i++){
      sn[i] = sn[i]/vSize;
    }
  }
  double cosPsi = -1*sn[2];

  // calc axis
  point_t vecA = getCrossProduct(sn, zaxis);
  vSize = getVectorSize(vecA);
  if(vSize != 1){
    for(int i = 0; i < vecA.size(); i++){
      vecA[i] = vecA[i]/vSize;
    }
  }

  // calc roation matrix
  tf::Matrix3x3 rotMat = getRotationMatrix(vecA, cosPsi);
  tf::Quaternion quat;
  rotMat.getRotation(quat);

  // std::cout<< "6) Got Rotation Matrix\n";


  point_t orientation;
  for(int i = 0; i < 4; i++){
    orientation.push_back(quat[i]);
  }

  // std::cout<< "7) Got Orientation\n";


  return {x, orientation};
}

point_t DrawingInput::getCrossProduct(point_t &a, point_t &b){
  point_t temp;

  temp.push_back(a[1]*b[2]-a[2]*b[1]);
  temp.push_back(a[2]*b[0]-a[0]*b[2]);
  temp.push_back(a[0]*b[1]-a[1]*b[0]);

  return temp;
}

tf::Matrix3x3 DrawingInput::getRotationMatrix(point_t &axis, double c){
  double s = sqrt(1-(c*c));
  double ux = axis[0];
  double uy = axis[1];
  double uz = axis[2];

  tf::Matrix3x3 temp(c+ux*ux*(1-c),ux*uy*(1-c)-uz*s,ux*uz*(1-c)+uy*s,
                        uy*ux*(1-c)+uz*s, c+uy*uy*(1-c), uy*uz*(1-c)-ux*s,
                        uz*ux*(1-c)-uy*s, uz*uy*(1-c)+ux*s, c+uz*uz*(1-c));

  return temp;
}
*/
