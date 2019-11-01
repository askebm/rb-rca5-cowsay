// Vision.cpp
#include <gazebo/gazebo_client.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>
#include <opencv2/opencv.hpp>

#include "Vision.h"

// TodoList
//      Interface to robot []
//      Camshift           []
//      



Vision::Vision(){}

Vision::~Vision(){}

// Has been tested and works: takes a Mat (BGR). 
bool Vision::isRed(cv::Point p, cv::Mat& im)
{
    cv::Mat image = cvt2Hls(im);

    auto hue = image.at<cv::Vec3b>(p)[0];

    if( hue < 10  || hue > 230 ) { return true; }

    return false;
}

// Converts to Hls format.
cv::Mat Vision::cvt2Hls(cv::Mat& im)
{
    cv::Mat im_hls;
    cv::GaussianBlur( im, im, cv::Size(5, 5), 2, 2 );
    cv::cvtColor(im,im_hls,CV_BGR2HLS);
    return im_hls; 
}

// Take a photo with spacebar and name it (The photo is saved to Photots).
void Vision::takePhoto(cv::Mat& im,std::string name)
{
    std::cout << "Im ready to take a photo" << std::endl;

    int key = cv::waitKey(5);
    if(key == 32)
    {
        std::string path = "Photos/" + name + ".jpg";

        // Take a photo of what you see.
        cv::imwrite( path, im );
        std::cout << "[PHOTO TAKEN]" << std::endl;
    }
}

// Finds the contour of marbles and draws them on the image (using Hough circles).
std::pair<int,bool> Vision::findContour(cv::Mat& im)
{
   // Plan:
   //   First convert the image to grayscale
   //   Find Hough circles (tune parameters)
   //   apply contours on original image
   //   Alles Gut!
   
    // grayscale the camera image.
    cv::Mat im_gray; 

    // The resulting pair
    std::pair<int,bool> result = {im.cols/2,false};

    // Convert to GrayScale
    cv::cvtColor(im,im_gray,CV_BGR2GRAY);
    
    // Find dem Hough Circles
    std::vector<cv::Vec3f> circles;
 	cv::HoughCircles(im_gray, circles, CV_HOUGH_GRADIENT, 1, im_gray.rows/8, 100, 20, 0, 0);
    
    // If there is marbles on the image.
    if (circles.size() > 0) { result.second = true; }
    else                    { return result; }

    int tmp_radius  = cvRound(circles[0][2]);
    int n_radius    = 0;

    // Draw the found Hough circles
    for( size_t i = 0; i < circles.size(); i++ )
    {
        cv::Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
        int radius = cvRound(circles[i][2]);
        
        if(tmp_radius < radius) { tmp_radius = radius; n_radius = i; }
        
        // circle center
        cv::circle( im, center, 3, cv::Scalar(0,255,0), -1, 8, 0 );
        // circle outline
        cv::circle( im, center, radius, cv::Scalar(0,255,0), 2, 8, 0 );
    }
    
    result.first = circles[n_radius][0];

    // Return
	return result;
}


// Checks if the entire image is black.
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











