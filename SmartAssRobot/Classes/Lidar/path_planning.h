#ifndef PATH_PLANNING_H
#define PATH_PLANNING_H

#include <gazebo/gazebo_client.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>

#include <iostream>
#include <vector>
#include <math.h>       /* atan2 */

using namespace std;

class Path_planning
{
public:
    Path_planning();
    ~Path_planning();
    void localization();
    vector<vector<float>> lineExtraction(vector<float> p, vector<float> tetha, int n ); // Incremental algorithm
    void leastSquare(vector<float> p, vector<float> tetha, int n );
    void rawData(float range, float angle, int counter);

private:
    vector<vector<float>> points;
    float r;
    float alpha;
    double threshhold_d = 5;

};

#endif // PATH_PLANNING_H
