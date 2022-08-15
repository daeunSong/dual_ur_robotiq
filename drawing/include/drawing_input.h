#include <geometry_msgs/Pose.h>
#include <ros/package.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf/tf.h>

#include <stdio.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include <cmath>


#define Stroke std::vector<geometry_msgs::Pose>

class DrawingInput {
  public:
    DrawingInput(const std::string &, const char &, const geometry_msgs::Pose &);

    std::vector<Stroke> strokes;

    // regarding input file
    std::string drawing_file_name;
    char color;
    geometry_msgs::Point color_;
    std::string drawing_file_name_full;

    // regarding drawing size
    std::vector<double> size; // width, height
    double target_size = 0.5; // target height
    double ratio;
    geometry_msgs::Pose drawing_pose;


    // functions
    void setFileName(const std::string file_name, const char color);
    void setColor();
    void setTargetSize (const double target_size);
    void setDrawingSize (const double ratio);
    void readDrawingFile ();
    void removeLongLine ();
    double getDist(geometry_msgs::Pose p1, geometry_msgs::Pose p2);
    std::vector<std::string> split(const std::string input, const char delimiter);

};
