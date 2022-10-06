#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseArray.h>
#include <visualization_msgs/Marker.h>
#include <rviz_visual_tools/rviz_visual_tools.h>
#include <tf/transform_listener.h>

// Eigen
#include <Eigen/Geometry>

namespace rvt = rviz_visual_tools;

#define D2R M_PI/180
#define Stroke std::vector<geometry_msgs::Pose>

class MarkerPublisher {
  public:
    MarkerPublisher(ros::NodeHandle* nh);

    bool ready_to_draw = false;
    visualization_msgs::Marker line_strip;
    visualization_msgs::Marker wall;

    std::string wall_file_name;
    std::vector<double> wall_pose;
    void initWall();

    ros::Publisher marker_pub;

    geometry_msgs::Point getEEPoint();
    void setColor();
    void publishLine(float id);
    void publishAxes();

  private:
    ros::NodeHandle nh_;
    rvt::RvizVisualToolsPtr visual_tools_;

    geometry_msgs::Point line_color;
    geometry_msgs::PoseArray trajectories;
    tf::TransformListener listener;
    tf::StampedTransform transform;
    int arm_num;

    void initSubscriber();
    void initPublisher();
    void initMarker();

    ros::Subscriber drawing_sub;
    ros::Subscriber color_sub;
    ros::Subscriber arm_num_sub;
    ros::Subscriber drawing_sub_2;
    ros::Subscriber color_sub_2;
    ros::Subscriber arm_num_sub_2;
    ros::Subscriber traj_sub;
    ros::Subscriber coord_sub; //

    void drawCallback(const std_msgs::Bool::ConstPtr& msg);
    void colorCallback(const geometry_msgs::Point::ConstPtr& msg);
    void armNumCallback(const std_msgs::Int32::ConstPtr& msg);
    void trajCallback(const geometry_msgs::PoseArray::ConstPtr& msg);
    void coordCallback(const geometry_msgs::Pose::ConstPtr& msg); //
};
