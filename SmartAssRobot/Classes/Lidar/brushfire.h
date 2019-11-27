#pragma once

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#define BRUSHFIRE_STEP_SIZE 1
#define BRUSHFIRE_BEGIN 10 //100

#define DEBUG_BRUSHFIRE  0

using namespace std;
using namespace cv;

struct pixel_t {

        int row;
        int col;
};

void check_neighbors(Mat & image,  vector<pixel_t> & neighbors, int row, int col);
int generate_brushfire(Mat & image, Mat & dst);
