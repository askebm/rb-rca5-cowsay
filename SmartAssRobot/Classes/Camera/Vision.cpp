// Vision.cpp
#include <gazebo/gazebo_client.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>

#include <opencv2/opencv.hpp>
#include "Vision.h"

cv::Mat Vision::findContour(cv::Mat& im)
{
	
	cv::Mat im_hue;
	cv::Mat im_hls;
	cv::Mat im_bin;	

	cv::cvtColor(im,im_hls,CV_BGR2HLS);
	
	
	return im;





}


bool isAllBlack(cv::Mat& im)
{

	for(int y = 0; y < im.rows; y++)
	{
		for(int x = 0; x < im.cols; x++)
		{
			if(im.at<uchar>(cv::Point(x,y)) == 255)
			{
				return false;
			}
		}
	}

	return true;
}











