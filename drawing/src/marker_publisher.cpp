#include "marker_publisher.h"

MarkerPublisher::MarkerPublisher(ros::NodeHandle* nh):nh_(*nh) {
  // init visual tools
  visual_tools_.reset(new rvt::RvizVisualTools("/world", "/axes_marker"));
  visual_tools_->loadMarkerPub();
  // Clear messages
  visual_tools_->deleteAllMarkers();
  visual_tools_->enableBatchPublishing();

  initSubscriber();
  initPublisher();
  initMarker();
}

// Init subscriber
void MarkerPublisher::initSubscriber() {
  drawing_sub = nh_.subscribe("/ready_to_draw", 10, &MarkerPublisher::drawCallback, this);
  color_sub = nh_.subscribe("/drawing_color", 10, &MarkerPublisher::colorCallback, this);
}

// Init publisher
void MarkerPublisher::initPublisher() {
  marker_pub = nh_.advertise<visualization_msgs::Marker>("/drawing_marker", 100);
}

// Init marker
void MarkerPublisher::initMarker() {
  line_strip.header.frame_id = "/world";
  line_strip.header.stamp = ros::Time::now();
  line_strip.ns = "points_and_lines";
  line_strip.action = visualization_msgs::Marker::ADD;
  line_strip.type = visualization_msgs::Marker::LINE_STRIP;

  line_strip.pose.orientation.w = 1.0;
  line_strip.scale.x = 0.001;
}



// Callback function to know whether if ready to visualize drawing
void MarkerPublisher::drawCallback(const std_msgs::Bool::ConstPtr& msg){
  ready_to_draw = msg->data;
}

// Callback function to get the drawing color
void MarkerPublisher::colorCallback(const geometry_msgs::Point::ConstPtr& msg){
  line_color = *msg;
  setColor();
}

// Get the end-effector pose
geometry_msgs::Point MarkerPublisher::getEEPoint(){
  try {
    listener.waitForTransform("/world", "/left_ee_link", ros::Time(0), ros::Duration(3.0));
    listener.lookupTransform("/world", "/left_ee_link", ros::Time(0), transform);
  }
  catch (tf::TransformException ex){
    ROS_ERROR("%s",ex.what());
    ros::Duration(1.0).sleep();
  }

  geometry_msgs::Point p;
  p.x = transform.getOrigin().x();
  p.y = transform.getOrigin().y();
  p.z = transform.getOrigin().z();
  return p;
}

// Set color
void MarkerPublisher::setColor(){
  line_strip.color.a = 0.5;
  line_strip.color.r = line_color.x;
  line_strip.color.g = line_color.y;
  line_strip.color.b = line_color.z;
}

void MarkerPublisher::publishLine(float id) {
  geometry_msgs::Point p;

  p = getEEPoint();
  line_strip.header.stamp = ros::Time::now();
  line_strip.id = id;

  if (ready_to_draw) {
    line_strip.points.push_back(p);
    if(line_strip.points.size() > 2){
      marker_pub.publish(line_strip);
      line_strip.points.erase(line_strip.points.begin());
      line_strip.points.push_back(p);
    }
  }
  else {
    line_strip.points.clear();
  }
}

int main( int argc, char** argv ) {
  ros::init(argc, argv, "markerPublisher");
  ros::NodeHandle nh("~");

  MarkerPublisher markerPublisher(&nh);

  ros::Rate loop_rate(10);
  float id = 0.0;

  while (ros::ok()) {
    markerPublisher.publishLine(id);

    id++;
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
