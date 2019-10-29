// Vision.cpp
#include <gazebo/gazebo_client.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>

#include <opencv2/opencv.hpp>
#include "Vision.h"

cv::Mat Vision::cvt2Hue(cv::Mat& im)
{
    cv::Mat im_hue;
    cv::cvtColor(im,im_hue,CV_8UC1);
    return im_hue;
}

cv::Mat Vision::cvt2Hls(cv::Mat& im)
{
    cv::Mat im_hls;
    cv::cvtColor(im,im_hls,cv::COLOR_BGR2HLS);
    return im_hls; 
}

void Vision::takePhoto(cv::Mat& im)
{
    int key = cv::waitKey(1);
    if(key == 32)
    {
        cv::imwrite( "../../Photos/marbles.jpg", im );
    }
}

cv::Mat Vision::findContour(cv::Mat& im)
{
	// Define the needed iamges.
	cv::Mat im_hue;
	cv::Mat im_hls;
	cv::Mat im_bin;	
    
    // Make the im_hls.
	cv::cvtColor(im,im_hls,cv::COLOR_BGR2HLS);

    // Make the im_hue.
    cv::cvtColor(im_hls,im_hue,CV_8UC1);
    


	return im_hls;

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











