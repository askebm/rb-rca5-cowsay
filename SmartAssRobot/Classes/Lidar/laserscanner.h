#ifndef LASERSCANNER_H
#define LASERSCANNER_H

#include <gazebo/gazebo_client.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>


#include <iostream>
#include <vector>
#include <cmath>
#include <time.h>

// boost headers
#include <boost/random.hpp>
#include <boost/math/distributions.hpp> // import all distributions
#include <boost/random/normal_distribution.hpp>
#include <boost/random/mersenne_twister.hpp>

using namespace std;
using namespace cv;

struct rays
{
      double distance;
      double dir;

};
class Laserscanner
{
public:
    Laserscanner();
    bool hasUpdated(bool flag);
    vector<rays> rayCasting(double x,  double y, double betaa);
    double drawLines(double x, double x1, double y, double y1);
    double calDistance(vector<double> x, vector<double> y);
    void updateLidar(int nr, float angle_mi, double angle_ma, float angle_i, float range_m,  vector<float> ranges);
    void updatePose(vector<double> x, vector<double> y, vector<double> beta);
    void updateSpeed(double speed, double dir);
    double vel;
    double angle_vel;
    double robot_angle;
    double robot_x;
    double robot_y;
    vector<double> range;
    vector<double> angle;
private:
    float angle_min;
    double angle_max;
    float angle_increment;
    float range_min;
    float range_max;
    int nranges;
    int nintensities;
    Mat map = imread("/home/annie/git_repo/rb-rca5-cowsay/models/bigworld/meshes/floor_plan.png", CV_LOAD_IMAGE_GRAYSCALE);

};

#endif // LASERSCANNER_H
