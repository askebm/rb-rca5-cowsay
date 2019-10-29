// Vision.h

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <array>
#include <cmath>
#include <time.h>
#include <vector>
#include <cv.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/viz/types.hpp>
#include <opencv2/imgproc.hpp>

class Vision {
  private:
	
  public:
	
	Vision();
	~Vision();

	cv::Mat findContour(cv::Mat& im);
	bool 	isAllBlack(cv::Mat& im);
    cv::Mat cvt2Hue(cv::Mat& im);
    cv::Mat cvt2Hls(cv::Mat& im);
    void    takePhoto(cv::Mat& im);    



};

