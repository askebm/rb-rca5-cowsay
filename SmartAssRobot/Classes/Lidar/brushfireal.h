#ifndef BRUSHFIREAL_H
#define BRUSHFIREAL_H

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <gazebo/gazebo_client.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>

#include <array>
#include <vector>

using namespace std;
using namespace cv;

struct Point1
{
    int x;
    int y;
};

class BrushfireAl
{
private:
    Mat binary_image;
    void convertToBinary(Mat img);
    Mat grey_scale;
    void findNodes(); //just a debugging algo
    void addNodes(vector<int> x);
    void nodeVector(); //support algo.
    Mat colour_img;
    vector<Point1> important_nodes = {};
    int start_x;
    int start_y;
    int findWay();
    vector<vector<int>> matrix_real;
public:
    BrushfireAl();
    void brushfire();
};

#endif // BRUSHFIREAL_H

//(100,100) -room 1
//(200,200) room 2 - potential but room 1 only has acces through room 2,
//(400,175) room 3 - beed to go through here for 4 and 7.
//(600,100) room 4 - needs to go through room 3 to get there.
//(900,100) room 5 - part of the long hallway
//(1100,100) room 6 - last room at the top end
//(650,250) room 7 - under room 4.
//(100,600) room 8 - room that is technically a hallway
//(300,600) room 9 - quite big room
//(600,550) room 10 - the top room of the two at the bottom.
//(1050,600) room 11 - big room that is part of the hallway
//(550,700) room  12 - first of the two small rooms.
//(750,700) room 13 - beside room 12. needs to go through this to get to room 12.

