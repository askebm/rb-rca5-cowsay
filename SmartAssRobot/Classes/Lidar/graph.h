#pragma once

#include<vector>
#include<memory>
#include<opencv2/opencv.hpp>
#include<exception>
#include "brushfire.h"
#include <limits>
#include <cstdlib>

#define DEBUG_ROADMAP 0

using namespace std;
using namespace cv;

struct cross;

class FullConnectedRoad : std::exception {

public:
    FullConnectedRoad() {}
    ~FullConnectedRoad() {}

};
class roadmap {

public:
    roadmap(const cv::Mat & brushfire, const int max_value) noexcept;
    void gerenate_GVD(const cv::Mat & brushfire, const int max_value);
    void optimize_GVD();
    void draw_roadmap(cv::Mat & dst) const noexcept;
    int gradient2(const cv::Mat & brushfire, const cv::Mat & src, const int row, const int col);
    void gradient(const cv::Mat & src, cv::Mat & angleDst, cv::Mat & magDst, std::vector<std::tuple<float, cv::Vec3b>> & angle, std::vector< std::tuple<float,cv::Vec3b>> & mag);
    ~roadmap();

private:
    struct pixel
    {
        int row;
        int col;
        pixel(const int row, const int col) noexcept : row(row), col(col) {}
        bool operator==(const pixel & p) const noexcept { return p.row == row && p.col == col ? true : false; }
        bool operator!=(const pixel & p) const noexcept { return p.row != row || p.col != col ? true : false; }
    };

    enum direction { LEFT, LEFT_UP, UP, RIGHT_UP, RIGHT, RIGHT_DOWN, DOWN, LEFT_DOWN};

struct road
{

    int pt1 = -1;
    int pt2 = -1;
    const direction dir;
    const int index;

    road(int pt1, int pt2, const direction dir, const int i) noexcept : pt1(pt1), pt2(pt2), dir(dir), index(i)  {}
    road(const direction dir, const int i) noexcept : dir(dir), index(i)  {}
    ~road() {}
    // Compare two roads, but not acounting to index
    bool operator==(const road r ) const
    {
        return (pt1 == r.pt1 || pt1 == r.pt2) && (pt2 == r.pt1 || pt2 == r.pt2) && (dir == r.dir || opposite_direction(dir) == r.dir ) ? true : false;
    }
    bool operator!=(const road r ) const
    {
        return (pt1 == r.pt1 || pt1 == r.pt2) && (pt2 == r.pt1 || pt2 == r.pt2) ? false : true;
    }
    void connect(int p)
    {
        if(pt1 == -1)
            pt1 = p;
        else if (pt2 == -1)
            pt2 = p;

        else
            throw FullConnectedRoad();
    }
    void remove_road()
    {

        pt1 = -1;
        pt2 = -1;
    }
};

struct intersection {

    const pixel point;
    const int index;

    std::vector<int> road_numbers;

    intersection(const pixel p, const int i) noexcept : point(p), index(i)  {}
    void add_road(const int road_number)
    {
        for(const int i : road_numbers )
        {
          if(i == road_number)
              return;
        }
        road_numbers.push_back(road_number);
    }
    void remove_road(const int road_number) {

        for(size_t i = 0; i < road_numbers.size(); i++ )
        {
            if(road_numbers[i] == road_number)
                road_numbers.erase(road_numbers.begin() + i );

        }
    }
};


    cv::Mat roadMap;
    cv::Mat roads;

    std::vector<intersection>  graph;
    std::vector<road> roadsGraph;

    void gerenate_GVD1(const cv::Mat & brushfire, std::vector<pixel> & nodes, const int max_value );
    void gerenate_GVD2(std::vector<pixel> & nodes);
    void gerenate_GVD2(const cv::Mat & brushfire, const cv::Mat & contourImage, std::vector<pixel> & nodes);
    void gerenate_GVD3(std::vector<pixel> & nodes);

    void mark_road(const pixel p);
    int new_road(const int start, const direction dir);
    void connect_to_road(const int intersection_number, const int road_number);
    void split_road(const int road_number, const int new_road);
    void add_road(const int intersection_number1, const int intersection_number2, const int road_number);
    void delete_road(const int road_number);
    void remove_duplicate_road(const int intersection_number, const int road_number);


    int create_intersection(const pixel point);
    void connect_to_intersection(const int new_intersection, const direction dir, const pixel p, bool use_dir = true);

    int gradient(const cv::Mat & src, const int row, const int col);
    int check_neighbors(const cv::Mat & map, const uchar value, const int row, const int col);
    int check_neighbors(const cv::Mat & map, const cv::Vec3b value, const int row, const int col);

    bool detect_indre_cornor(const cv::Mat & map, const int row, const int col, const cv::Vec3b mapColor);

    int find_moving_directions(const cv::Mat & map, std::vector<direction> & dir_list, const pixel p, const cv::Vec3b color) const noexcept;
    int reduce_directions(std::vector<direction> & dir_list, const direction dir) const noexcept;
    void create_road(cv::Mat & map, const direction dir, const pixel p, const pixel c_p, std::vector<pixel> & nodes, const int road) noexcept;
    void expand(cv::Mat & map, const pixel p, const direction dir);

    std::string print(const direction d) const noexcept;
    std::string print(const intersection i) const noexcept;
    std::string print(const road r) const noexcept;
    std::string print(const pixel p) const noexcept;

    direction degress2direction(const float angle) const noexcept;
    direction static opposite_direction(const direction dir) noexcept;

    void remove_nodes(std::vector<pixel> & nodes, const pixel p) const noexcept;

    void explore_directions(cv::Mat & map, const pixel p, const std::vector<direction> directions, std::vector<pixel> & nodes ) noexcept;
    int explore_move(const cv::Mat & map, const pixel p, const pixel c_p, const direction direction) const noexcept;
    void explore_draw(cv::Mat & map, const pixel p, const pixel c_p, const direction dir, const int move_left, std::vector<pixel> & nodes) noexcept;
    void explore_draw(cv::Mat & map, const pixel p, const pixel c_p, const direction dir, const pixel end) const noexcept;

    friend road;

    bool move(std::vector<cv::Point> & n1, const cv::Mat & brushfire, cv::Mat & map, cv::Point start, cv::Point end);
    void move_draw(const std::vector<cv::Point> & n1, std::vector<pixel> & n2, cv::Mat & map);


    int remove_duplicate_nodes(std::vector<pixel> & nodes);
};
