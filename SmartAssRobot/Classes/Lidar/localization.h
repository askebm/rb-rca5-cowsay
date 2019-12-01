#ifndef LOCALIZATION_H
#define LOCALIZATION_H

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
    Localization(int sample_size, Laserscanner *ls);
    ~Localization();
    void init(Laserscanner *ls);
    void prediction(Laserscanner *sh);
    void updatePos();
    vector<particle> resampling(vector<particle> M);
    bool checkCoordinates(double x, double y);
    // variables
    vector<particle> samples = {};

private:
    float _range_max;
    int N; // Number of particles.
    boost::mt19937 gen; // random number generator
    time_t timer;
    double current_time;
    bool first_flag;


};

#endif // LOCALIZATION_H
