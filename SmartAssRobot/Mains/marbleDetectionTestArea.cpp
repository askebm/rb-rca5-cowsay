// This is a testarea for testing atomic methods

#include <gazebo/gazebo_client.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>
#include <opencv2/opencv.hpp>
#include <iostream>
//#include "../Classes/Camera/Vision.h"
#include <Camera/Vision.h>

//#include <Camera/Vision.h>
#include "../Classes/Camera/Vision.h"

int main()
{

    Vision vision;

    cv::Mat picture     = cv::imread("Photos/marbles.jpg",1);
    //cv::Mat red         = cv::imread("Photos/red.png",1);
    //cv::Mat bin         = vision.cvt2Bin(picture); 

    //cv::imshow("Non manipulated image",picture);
    //cv::imshow("Hls image",vision.cvt2Hls(picture));
    //cv::imshow("Binary image",bin);
    //cv::imshow("red => hls",vision.cvt2Hls(red));
    //vision.findContour(picture);
    //cv::imshow("Image with Contours",vision.findContour(picture));

    cv::imshow("Original image",picture);
    //cv::imshow("HLS image", vision.cvt2Hls(picture));
    //cv::imshow("Hough circle image",vision.findContour(picture));
    //cv::imshow("Binary image",vision.cvt2Bin(picture));
    cv::waitKey(0);

}
