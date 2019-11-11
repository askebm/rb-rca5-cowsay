#ifndef BRUSHFIREAL_H
#define BRUSHFIREAL_H

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <gazebo/gazebo_client.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>

#include <array>
#include <vector>

using namespace std;
using namespace cv;

class BrushfireAl
{
private:
    Mat binary_image;
    void convertToBinary(Mat img);
    Mat grey_scale;
public:
    BrushfireAl();
    void brushfire();


};

#endif // BRUSHFIREAL_H
