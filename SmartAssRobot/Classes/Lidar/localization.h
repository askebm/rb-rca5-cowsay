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
#include <math.h>       /* atan2 */

using namespace std;
using namespace cv;

struct sample{
    vector<int> state;
    int weight;

};

class Localization
{
public:
    Localization();
    ~Localization();
    vector<sample> resampling(vector<sample> M);


private:


};

#endif // LOCALIZATION_H
