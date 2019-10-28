//
// Created by victor on 10/25/19.
//

#include "Vision.h"


Vision::Vision() {};

Vision::~Vision() {};

cv::Mat Vision::findContour(cv::Mat& im)
{

    std::cout << "2" << std::endl;

    cv::Mat im_hls;
    std::cout << "3" << std::endl;

    cv::Mat im_hue;
    std::cout << "4" << std::endl;

    cv::Mat binary(im.rows,im.cols,CV_8UC1);
    std::cout << "5" << std::endl;


    int ch[2] = {0};

    std::cout << "6" << std::endl;

    cv::cvtColor(im,im_hls,CV_BGR2HLS);
    std::cout << "7" << std::endl;

    im_hue.create( im_hls.size(),im_hls.depth() );
    std::cout << "8" << std::endl;

    cv::mixChannels(&im_hls,1,&im_hue,1,ch,1);
    std::cout << "9" << std::endl;

    cv::imshow("im_hue",im_hue);
    std::cout << im_hue.empty() << std::endl;

    if (isAllBlack(binary))
    {
	std::cout << "The Image is all black" << std::endl;
    	return im;
    }

    for(int y = 0; y < im_hue.cols - 1; y++)
    {
        for(int x = 0; x < im_hue.rows - 1; x++)
        {
            if(im_hue.at<uchar>(cv::Point( x , y )) > 170 || im_hue.at<uchar>(cv::Point( x , y )) < 10 )
            {
                binary.at<uchar>(cv::Point(x,y)) == 255;
            }
            else
            {
                binary.at<uchar>(cv::Point(x,y)) == 0;
            }
        }
    }

    return binary;

}

bool Vision::isAllBlack(cv::Mat& im)
{
    if (im.empty())
    {
        std::cerr << "isAllBlack has been given an empty image" << std::endl;
        return true;
    }
	

    int n = 0;

    for(int y = 0; y < im.cols - 1; y++)
    {
        for(int x = 0; x < im.rows - 1; x++)
        {

            if(im.at<uchar>(cv::Point( x , y )) != 0)
            {
		n++;
		if(n > 20)
		{
               	     return false;
		}
            }
        }
    }

    return true;

}
