#pragma once

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#define value_step 1
#define starting_value 1


using namespace std;
using namespace cv;

struct pixel {

        int row;
        int col;
};

void check_neighbours(Mat & image,  vector<pixel> & neighbors, int row, int col);
int generate_brushfire(Mat & image, Mat & dst);
