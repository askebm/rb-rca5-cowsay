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

struct Neighbour
{
        int equal;
        int higher;
};

class BrushfireAl
{
private:
    Mat binary_image;
    void convertToBinary(Mat img);
    Mat grey_scale;
    void findNodes(); //just a debugging algo
    void addNodes(vector<int> x);
    void nodeVector(); //support function - to test Dijstra's ability to find hardcodded nodes
    Mat colour_img;
    vector<Point1> important_nodes = {};
    int start_x;
    int start_y;
    int findWay();
    vector<vector<int>> matrix_real;
    bool checkGradient(int x, int y);
    void check_neighbours(vector<Neighbour> &neighbours, int row, int col, int which_type);
public:
    BrushfireAl();
    void brushfire();

};

#endif // BRUSHFIREAL_H
