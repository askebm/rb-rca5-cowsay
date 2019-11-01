// Vision.h

#ifndef VISION_H_28FXIT71
#define VISION_H_28FXIT71




#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <array>
#include <cmath>
#include <time.h>
#include <vector>
//#include <cv.hpp>
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
    cv::Mat cvt2Hls(cv::Mat& im);
    void    takePhoto(cv::Mat& im,std::string name); 
    cv::Mat cvt2Bin(cv::Mat& im);

    bool isRed(cv::Point p, cv::Mat& im);   



};

#endif /* end of include guard: VISION_H_28FXIT71 */
