#ifndef LOCALIZATION_H
#define LOCALIZATION_H

#include <gazebo/gazebo_client.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <iostream>
#include <vector>
#include <cmath>
#include <time.h>

// boost headers
#include <boost/random.hpp>
#include <boost/math/distributions.hpp> // import all distributions
#include <boost/random/normal_distribution.hpp>
#include <boost/random/mersenne_twister.hpp>

#include "laserscanner.h"


using namespace std;
using namespace cv;


class particle
{
public:
    double weight;
    double last_time;
    double x;
    double y;
    double beta;

    double getY();
    double getX();
    double getBeta();
    double getWeight();
    double getLastTime();
    void updateState(double x_n, double y_n, double beta_n, double time);
};

class Localization
{
public:
    Localization(int sample_size, Mat m);
    ~Localization();
    void init();
    void prediction(Laserscanner s);
    void updatePos();
    vector<particle> resampling(vector<particle> M);
    bool checkCoordinates(double x, double y);
    // variables
    vector<particle> samples;

private:
    //float ray(Mat* map, Point2f p, float angle);
    float _range_max;
    int N; // Number of particles.
    boost::mt19937 gen; // random number generator
    time_t timer;
    double current_time;
    bool first_flag;
    Mat map;


};

#endif // LOCALIZATION_H
