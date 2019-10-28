//
// Created by victor on 10/25/19.
//

#pragma once
#include <opencv2/opencv.hpp>


class Vision {

public:

    Vision();
    ~Vision();


    cv::Mat findContour(cv::Mat& im);
    bool    isAllBlack(cv::Mat& im);




protected:




};
