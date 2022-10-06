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


#define D2R M_PI/180
#define R2D 180/M_PI
#define Stroke std::vector<geometry_msgs::Pose>

class DrawingInput {
  public:
    DrawingInput(const std::string &, const geometry_msgs::Pose &, const geometry_msgs::Pose &);

    std::vector<Stroke> strokes;
//    std::vector<Stroke> strokes_r;
//    std::vector<Stroke> strokes_l;
    std::vector<std::tuple<int, Stroke, int, double>> strokes_r;  // stroke_index, stroke, color, miny
    std::vector<std::tuple<int, Stroke, int, double>> strokes_l;  // stroke_index, stroke, color, maxy
    std::vector<std::vector<Stroke>> strokes_by_range;
    std::vector<std::array<double, 2>> ranges;


    // regarding input file
    std::string drawing_file_name;
    int stroke_num;
    char color;
    geometry_msgs::Point color_;
    std::string drawing_file_name_full;


    // regarding drawing size
    std::vector<double> size; // width, height
    double target_size = 0.160; // target height
    double ratio;
    geometry_msgs::Pose drawing_pose;
    geometry_msgs::Pose init_drawing_pose_r;
    geometry_msgs::Pose init_drawing_pose_l;

    // regarding drawing spliting
    void splitDrawing ();
    int detectRange(geometry_msgs::Pose p);


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
