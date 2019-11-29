#include "graph.h"

roadmap::roadmap(const Mat & brushfir, const int max_value) noexcept
{
    roads = Mat::zeros(brushfir.rows, brushfir.cols, CV_32S);
    gerenate_GVD(brushfir, max_value);

}
void roadmap::remove_nodes(vector<pixel> & nodes, const pixel p) const noexcept
{
    for(size_t i = 0; i < nodes.size(); i++ )
    {
        if(nodes[i] == p)
            nodes.erase(nodes.begin() + i );
    }
}
void roadmap::gerenate_GVD(const Mat & brushfire, const int max_value)
{
    roadMap = brushfire.clone();
    Mat contourImage(roadMap.rows, roadMap.cols, CV_8UC1);
    contourImage.setTo(0);

    vector<pixel> nodes;

    gerenate_GVD1(brushfire, nodes, max_value);
    gerenate_GVD2(nodes);
    for(const pixel p : nodes )
    {
        contourImage.at<uchar>(p.row, p.col) = 255;
    }
    gerenate_GVD2(brushfire, contourImage, nodes);
    remove_duplicate_nodes(nodes);
    gerenate_GVD3(nodes);

}
void roadmap::draw_roadmap(Mat & return_image) const noexcept
{
   // return_image = Mat(roadMap.rows, roadMap.cols, CV_8UC3);
  //  return_image.setTo(255);
    for(const road r : roadsGraph )
    {
        if(r.pt1 != -1 && r.pt2 != -1)
        {
            const intersection i1 = graph[r.pt1];
            const intersection i2 = graph[r.pt2];
            return_image.at<Vec3b>(i1.point.row, i1.point.col) = Vec3b(0,255,0);
            return_image.at<Vec3b>(i2.point.row, i2.point.col) =  Vec3b(0,255,0);
            explore_draw(return_image, i1.point, i1.point, r.dir, i2.point);
        }
    }
}

void roadmap::gradient(const Mat & src, Mat & angleDst, Mat & magDst, vector< tuple<float, Vec3b>> & angle, vector< tuple<float, Vec3b>> & mag)
{

    angle.clear();
    mag.clear();

    magDst = src.clone();
    angleDst = src.clone();

    Mat gray;
    cvtColor(src, gray,  COLOR_BGR2GRAY);
    gray.convertTo(gray, CV_32F, 1/255.0);
    Mat dx, dy;
    Sobel(gray, dx, CV_32F, 1,0, 1 );
    Sobel(gray, dy, CV_32F,0,1, 1 );
    Mat angle2, mag2;

    cartToPolar(dx,dy,mag2,angle2,1);

    for(int row = 0 ; row < angle2.rows ; row++ )
    {
        for(int col = 0 ; col < angle2.cols; col++  )
        {
            if(src.at<Vec3b>(row,col) == Vec3b(0,0,0))
                continue;
            bool flag = true;
            for(const tuple<float, Vec3b> f1 : angle )
            {

                if(angle2.at<float>(row,col) == get<0>(f1)) {
                    flag = false;
                    angleDst.at<Vec3b>(row,col) = get<1>(f1);
                }
            }

            if(flag)
            {
                Vec3b color = Vec3b( rand() % 255, rand() % 255, rand() % 255);
                angle.push_back(tuple<float, Vec3b>(angle2.at<float>(row,col), color));
                angleDst.at<Vec3b>(row,col) = color;
            }

            bool flag2 = true;
            for(const tuple<float, Vec3b> f1 : mag )
            {

                if(angle2.at<float>(row,col) == get<0>(f1))
                {
                    flag2 = false;
                    magDst.at<Vec3b>(row,col) = get<1>(f1);
                }
            }

            if(flag2)
            {
                Vec3b color = Vec3b( rand() % 255, rand() % 255, rand() % 255    );
                mag.push_back(tuple<float, Vec3b>(mag2.at<float>(row,col), color));
                magDst.at<Vec3b>(row,col) = color;
            }
        }
    }

    vector< tuple<float, Vec3b>> angleNew;

    for(const tuple<float, Vec3b> infor : angle )
    {
        if(angleNew.size() != 0)
        {

            const float tmp = get<0>(infor);

            for(size_t i = 0; i < angleNew.size(); i++  )
            {
                if(get<0>(angleNew[i] ) > tmp )
                {
                    angleNew.insert(angleNew.begin() + i, infor  );
                    break;
                }

                if(i == angleNew.size() - 1 )
                {
                    angleNew.push_back(infor);
                    break;
                }
            }
        }
        else
            angleNew.push_back(infor);
    }
    angle.clear();
    angle = angleNew;
}
roadmap::~roadmap() {}


void roadmap::gerenate_GVD1(const Mat & brushfire, vector<pixel> & nodes, const int max_value) {

    const int rows = roadMap.rows;
    const int cols = roadMap.cols;
    bool flag;

    for(int row = 0; row < rows ; row++ )
    {
        for(int col = 0; col < cols; col++ )
        {
            pixel point{row,col};
            uchar i = brushfire.at<Vec3b>(row,col)[0];
            if(i == 0)
            {
                continue;
            }

            flag = detect_indre_cornor(brushfire, row, col, Vec3b(0,0,0));

            if(flag)
            {
                nodes.push_back(point);
                create_intersection(point);
                continue;
            }
            flag = detect_indre_cornor(brushfire, row, col, Vec3b(i-value_step,i - value_step,i - value_step));
            if(flag)
            {
                nodes.push_back(point);
                roadMap.at<Vec3b>(row,col) = Vec3b(0,255,0);
                continue;
            }

            int n = check_neighbors(brushfire,i, row, col);  // Same index as the current pixel
            int n2;

            if(i != starting_value)
                n2 = check_neighbors(brushfire,i - value_step, row,col);  // One lower then th current pixel
            else
                n2 = check_neighbors(brushfire,0,row,col);

            int n3 = check_neighbors(brushfire,i + value_step, row, col); // One high then the current pixel

            if(i == max_value && brushfire.at< Vec3b>(row, col)  == Vec3b((uchar) max_value, (uchar) max_value, (uchar) max_value ) )
            {
                roadMap.at< Vec3b>(row,col) = Vec3b(0,255,0);
                nodes.push_back(point);
            }
            else
            {
                if(n3 == 0 && n == 6 && n2 == 3)
                {
                    roadMap.at< Vec3b>(row,col) = Vec3b(0, 255, 0);
                    nodes.push_back(point);
                }
                else if(n3 == 0 && n == 5 && n2 == 4)
                {
                    roadMap.at< Vec3b>(row,col) = Vec3b(0, 255, 0);
                    nodes.push_back(point);
                }
                else if(n3 == 0 && n2 > 4 && n < 5)
                {
                    roadMap.at< Vec3b>(row,col) = Vec3b(0,255,0);
                    nodes.push_back(point);
                }
                else if(n3 == 0 && n2 > 3)
                {
                    if (n == 2 || n == 3)
                    {
                        roadMap.at<Vec3b>(row,col) = Vec3b(0,225,0);
                        nodes.push_back(point);
                    }
                }
                else if(n3 == 0 && n2 < 3 && n > 4)
                {
                    roadMap.at< Vec3b>(row,col) =Vec3b(0,255,0);
                    nodes.push_back(point);
                }
                else if(n3 <= 2 && n == 3 && n2 > 3)
                {
                    roadMap.at<Vec3b>(row,col) =Vec3b(0,225,0);
                    nodes.push_back(point);
                }
            }
        }
    }
}
void roadmap::gerenate_GVD2(vector<pixel> & nodes)
{
    for(const pixel p : nodes )
    {
        if(roadMap.at< Vec3b>(p.row, p.col) == Vec3b(0,0,255))
            continue;

        int n = check_neighbors(roadMap, Vec3b(255,0,0), p.row, p.col)
            + check_neighbors(roadMap, Vec3b(0,255,0), p.row, p.col)
            + check_neighbors(roadMap, Vec3b(255,255,0), p.row, p.col) ;

        int n2 = check_neighbors(roadMap, Vec3b(0,0,0), p.row, p.col);

        if(n == 2 && n2 < 4)
        {
            vector<direction> green, blue, light_blue, black;
            int r = find_moving_directions(roadMap, light_blue, p, Vec3b(255,255,0))
                + find_moving_directions(roadMap, green, p, Vec3b(0,255,0))
                + find_moving_directions(roadMap, blue, p, Vec3b(255,0,0));

            roadMap.at<Vec3b>(p.row, p.col) = Vec3b(255,255,0);
            explore_directions(roadMap, p, green, nodes);
            if( roadMap.at< Vec3b>(p.row, p.col) == Vec3b(255,255,0))
                roadMap.at< Vec3b>(p.row, p.col) = Vec3b(0,255,0);
        }
    }
}
void roadmap::gerenate_GVD2(const Mat & brushfire, const Mat & contourImage, vector<pixel> & nodes)
{
    Mat contourImage2 = contourImage.clone() ;
    vector< Vec4i> Hier;
    vector<vector<Point>> contour;

    threshold(contourImage2 , contourImage2, 255/2, 255, CV_THRESH_BINARY);
    findContours(contourImage2, contour, Hier, CV_RETR_LIST, CV_CHAIN_APPROX_NONE);

    int base = 0;
    size_t max = 0;

    for(size_t i = 0; i < contour.size(); i++ )
    {
        if(contour[i].size() > max )
        {
            max = contour[i].size();
            base = i;
        }
    }
    double distance_min = numeric_limits<double>::max();
    int x = 0;
    int i = 0;
    int old = contour.size()+1;
    while( ((size_t) old != contour.size() || x < 10) && contour.size() != 1  )
    {
        old = contour.size();

        vector<Point> n2;
        bool flag = false;
        for(const Point p1 : contour[i] )
        {
            for(const Point p2 : contour[base])
            {
                if(p2 != p1)
                {
                    double distance = (p2.x - p1.x)*(p2.x - p1.x) + (p2.y - p1.y)*(p2.y - p1.y);
                    if(distance < 500)
                    {
                        vector<    Point> n1;
                        if(brushfire.at<    Vec3b>(p1.y,p1.x)[0] < brushfire.at<Vec3b>(p2.y,p2.x)[0])
                            flag = move(n1, brushfire, roadMap, p1, p2);
                        else
                            flag = move(n1, brushfire, roadMap, p2, p1);

                        if(flag == true)
                        {
                            if(distance < distance_min)
                            {
                                distance_min = distance;
                                n2.clear();
                                n2 = n1;
                            }

                            n1.clear();
                        }
                    }
                }
            }
        }

        if(n2.size() != 0 )
        {
            move_draw(n2, nodes, roadMap);
            for(const Point p : contour[i] )
            {
                contour[base].push_back(p);
            }

            for(const Point p : n2 )
            {
                contourImage2.at<uchar>(p.y,p.x) = 255;
                contour[base].push_back(p);
            }

            contour.erase(contour.begin() + i );
            Hier.erase(Hier.begin() + i );

            if( i < base)
                base--;
        }
        else
            i++;

        n2.clear();
        distance_min = numeric_limits<double>::max();

        if(i == base)
            i++;

        if((size_t) i == contour.size() )
        {
            i = 0;
            x++;
        }

        if(i == base)
            i++;
    }
}
void roadmap::gerenate_GVD3(vector<pixel> & nodes)
{
    size_t size = nodes.size();
    size_t size_new = 0;
    while(size != size_new)
    {
        size = size_new;
        for(const pixel p : nodes )
        {
            if(roadMap.at<Vec3b>(p.row, p.col) == Vec3b(255,0,0))
            {
                vector<direction> dir_list;
                find_moving_directions(roadMap, dir_list,p, Vec3b(0,255,0));

                for(const direction dir : dir_list )
                    create_road(roadMap, dir, p, p,nodes, -1);

                remove_nodes(nodes, p);
            }
        }
        size_new = nodes.size();
    }
}

void roadmap::mark_road(const pixel p)
{
    if(roadMap.at<Vec3b>(p.row, p.col) != Vec3b(0,0, 255) && roadMap.at<Vec3b>(p.row, p.col) !=    Vec3b(255,0,0))
    {
        roadMap.at<Vec3b>(p.row, p.col) = Vec3b(0,0, 255);
        roads.at<int>(p.row, p.col) = (int)roadsGraph.size();
    }
}
int roadmap::new_road(const int start, const direction dir)
{
    roadsGraph.push_back(road(start, -1, dir, roadsGraph.size()));
    graph[start].add_road(roadsGraph.size() - 1);

    return roadsGraph.size() - 1;
}
void roadmap::connect_to_road(const int intersection_number, const int road_number)
{
    try
    {
        roadsGraph[road_number].connect(intersection_number);
        graph[intersection_number].add_road(road_number);
        remove_duplicate_road(intersection_number, road_number);
    }
    catch (FullConnectedRoad r)
    {
        cout << "tried to add intersection " << intersection_number << " at (" << graph[intersection_number].point.row << ", "  << graph[intersection_number].point.col << ") to road " << road_number << endl;
    }
}
void roadmap::delete_road(const int road_number)
{
    graph[roadsGraph[road_number].pt1].remove_road(road_number);
    graph[roadsGraph[road_number].pt2].remove_road(road_number);
    roadsGraph[road_number].remove_road();
}
void roadmap::remove_duplicate_road(const int intersection_number, const int road_number)
{
    for( const int r : graph[intersection_number].road_numbers )
    {
        if(roadsGraph[r] == roadsGraph[road_number] && r != road_number)
        {
            delete_road(road_number);
            break;
        }
    }
}

int roadmap::create_intersection(const pixel point)
{

    if(roadMap.at<    Vec3b>(point.row, point.col)  ==    Vec3b(255,0,0))
    {
        return roads.at<int>(point.row,point.col);
    }
    else
    {
        roadMap.at<    Vec3b>(point.row, point.col) =    Vec3b(255,0,0);
        graph.push_back(intersection(point, graph.size()));
        roads.at<int>(point.row, point.col) = graph.size() - 1;
        return graph.size()-1;
    }
}

void roadmap::connect_to_intersection(const int new_intersection, const direction dir, const pixel p, bool use_dir)
{

    pixel p_new(-1,-1) ;

    if(use_dir)
    {
        switch(dir)
        {
            case LEFT:
                p_new = pixel( p.row, p.col - 1);
                break;
            case LEFT_UP:
                p_new = pixel(p.row - 1, p.col - 1);
                break;
            case UP:
                p_new = pixel(p.row - 1, p.col);
                break;
            case RIGHT_UP:
                p_new = pixel(p.row - 1, p.col + 1);
                break;
            case RIGHT:
                p_new = pixel(p.row, p.col + 1);
                break;
            case RIGHT_DOWN:
                p_new = pixel(p.row + 1, p.col + 1);
                break;
            case DOWN:
                p_new = pixel(p.row + 1, p.col);
                break;
            case LEFT_DOWN:
                p_new = pixel(p.row + 1, p.col - 1);
                break;
        }
    }

    const int old_intersection = roads.at<int>(p_new.row, p_new.col);

    if(roadMap.at< Vec3b>(p_new.row, p_new.col) != Vec3b(255,0,0) )
        cout << "Something wrong" << endl;

    int road_new = new_road(new_intersection,dir);
    connect_to_road(old_intersection, road_new);
}
#define CHECK_APOVE src.at<float>(row - 1, col)
#define CHECK_LEFT_APOVE src.at<float>(row - 1, col-1)
#define CHECK_RIGHT_APOVE src.at<float>(row - 1, col+1)
#define CHECK_LEFT src.at<float>(row, col - 1)
#define CHECK_RIGHT src.at<float>(row, col +1)
#define CHECK_DOWN src.at<float>(row + 1, col)
#define CHECK_LEFT_DOWN src.at<float>(row + 1, col-1)
#define CHECK_RIGHT_DOWN src.at<float>(row + 1, col+1)
// BiCube
int roadmap::gradient2(const Mat & brushfire, const Mat & src, const int row, const int col)
{
    float apove_left = CHECK_LEFT_APOVE;
    float apove = CHECK_APOVE;
    float apove_right = CHECK_RIGHT_APOVE;

    float center_left = CHECK_LEFT;
    float center = src.at<float>(row,col);
    float center_right = CHECK_RIGHT;

    float down_left = CHECK_LEFT_DOWN;
    float down = CHECK_DOWN;
    float down_right = CHECK_RIGHT_DOWN;
    float sum = 0;

    if(apove_left != -1)
        sum += apove_left;

    if(apove != -1)
        sum += apove;

    if(apove_right != -1)
        sum += apove_right;

    if(center_left != -1)
        sum += center_left;

    if(center != -1)
        sum += center;

    if(center_right != -1)
        sum += center_right;

    if(down_left != -1)
        sum += down_left;

    if(down != -1)
        sum += down;

    if(down_right != -1)
        sum += down_right;

    return (int) ceil(sum) % 360;
}
int roadmap::check_neighbors(const Mat & map, const uchar value, const int row, const int col)
{
    return check_neighbors(map, Vec3b(value,value,value), row, col);
}
int roadmap::check_neighbors(const Mat & map, const Vec3b value, const int row, const int col)
{
    int count = 0;
    for(int r = row - 1; r < row + 2; r++ )
    {
        if(r > -1 && r < map.rows) {

            for(int c = col - 1; c < col + 2 ; c++ )
            {
                if(c > -1 && c < map.cols)
                {
                    if(map.at<    Vec3b>(r,c) == value)
                    {
                        count++;
                    }
                }
            }
        }
    }

    return count;

}

#define PIXEL_APOVE(row, col) map.at<Vec3b>(row - 1, col) == mapColor
#define PIXEL_LEFT(row, col)  map.at<Vec3b>(row, col - 1) == mapColor
#define PIXEL_RIGHT(row, col) map.at<Vec3b>(row, col + 1) == mapColor
#define PIXEL_DOWN(row, col)  map.at<Vec3b>(row + 1, col) == mapColor

#define PIXEL_APOVE_RIGHT(row, col) map.at<Vec3b>(row - 1, col + 1) == mapColor
#define PIXEL_APOVE_LEFT(row, col)  map.at<Vec3b>(row - 1, col - 1) == mapColor
#define PIXEL_DOWN_RIGHT(row, col)  map.at<Vec3b>(row + 1, col + 1) == mapColor
#define PIXEL_DOWN_LEFT(row, col)   map.at<Vec3b>(row + 1, col - 1) == mapColor

bool roadmap::detect_indre_cornor(const Mat & map, const int row, const int col, const Vec3b mapColor)
{

    if(PIXEL_APOVE(row, col)  && PIXEL_LEFT(row, col) )
    {
        roadMap.at< Vec3b>(row,col) = Vec3b(0,255,0);
        return true;
    }

    if(PIXEL_APOVE(row, col) && PIXEL_RIGHT(row, col) )
    {
        roadMap.at< Vec3b>(row,col) = Vec3b(0,255,0);
        return true;
    }

    if(PIXEL_DOWN(row, col) && PIXEL_LEFT(row, col) )
    {
        roadMap.at<Vec3b>(row,col) = Vec3b(0,255,0);
        return true;
    }

    if(PIXEL_DOWN(row,col) && PIXEL_RIGHT(row, col) )
    {
        roadMap.at<Vec3b>(row,col) = Vec3b(0,255,0);
        return true;
    }
    return false;
}
int roadmap::find_moving_directions(const Mat & map, vector<direction> & dir_list, const pixel p,    Vec3b color) const noexcept
{
    dir_list.clear();

    if(map.at<Vec3b>(p.row + 1, p.col) == color )
        dir_list.push_back(DOWN);
    // Green apove
    if(map.at< Vec3b>(p.row - 1, p.col) == color)
        dir_list.push_back(UP);
    // Green left
    if(map.at<Vec3b>(p.row, p.col - 1) == color)
        dir_list.push_back(LEFT);
    // Green right
    if(map.at<Vec3b>(p.row, p.col + 1) == color)
        dir_list.push_back(RIGHT);
    // Green apove right
    if(map.at<Vec3b>(p.row - 1, p.col + 1) == color)
        dir_list.push_back(RIGHT_UP);
    // Green down left
    if(map.at<Vec3b>(p.row + 1, p.col - 1) == color)
        dir_list.push_back(LEFT_DOWN);
    // Green apove left
    if(map.at<Vec3b>(p.row - 1, p.col - 1) == color)
        dir_list.push_back(LEFT_UP);
    // Green down right
    if(map.at<Vec3b>(p.row + 1, p.col + 1) == color)
        dir_list.push_back(RIGHT_DOWN);

    return dir_list.size();
}
int roadmap::reduce_directions(vector<direction> & dir_list, const direction dir) const noexcept
{

    for(size_t i = 0; i < dir_list.size(); i++ )
    {
        if(dir == dir_list[i])
            dir_list.erase(dir_list.begin() +i );
        else
        {

            if(( dir == LEFT_UP && dir_list[i] == RIGHT_DOWN) || (dir == RIGHT_DOWN && dir_list[i] == LEFT_UP) )
                dir_list.erase(dir_list.begin() + i );
            else if ((dir == RIGHT_UP && dir_list[i] == LEFT_DOWN) || (dir == LEFT_DOWN && dir_list[i] == RIGHT_UP)  )
                dir_list.erase(dir_list.begin() + i );
            else if ((dir == LEFT && dir_list[i] == RIGHT) || (dir == RIGHT && dir_list[i] == LEFT))
                dir_list.erase(dir_list.begin() + i);
            else if ((dir == UP && dir_list[i] == DOWN) || (dir == DOWN && dir_list[i] == UP)  )
                dir_list.erase(dir_list.begin() + i);
        }
    }
    return dir_list.size();
}
void roadmap::create_road( Mat & map, const direction dir, const pixel p, const pixel c_p, vector<pixel> & nodes, const int road) noexcept {

    int _road = road;

    if(road == -1)
    {
        int intersection = create_intersection(p);
        _road = new_road(intersection, dir);
    }

    vector<direction> blue, red, green;
    vector<direction> blue2, red2, green2;
    int n = find_moving_directions(map, blue, c_p, Vec3b(255,0,0)) +  find_moving_directions(map, green, c_p,    Vec3b(0,255,0)) + find_moving_directions(map, red, c_p,    Vec3b(0,0,255));
    blue2 = blue;
    green2 = green;
    red2 = red;
    n = reduce_directions(blue, opposite_direction(dir)) + reduce_directions(red, dir) + reduce_directions(green, dir);

    if(!(c_p.col > 0 && c_p.col < map.cols && c_p.row > 0 && c_p.row < map.rows ))
        return;

    if(p != c_p)
    {
        remove_nodes(nodes, c_p);
        mark_road(c_p);
    }

    if(n > 0 && c_p != p)
    {

        int new_intersection  = create_intersection(c_p);
        connect_to_road(new_intersection, road);
        if(blue2.size() >  1 )
        {
            for(const direction d : blue2 )
            {
                if(opposite_direction(dir) != d )
                    connect_to_intersection(new_intersection, d, c_p);
            }
            // Create road to interscept
        }

        if(green.size() > 0 )
        {
            for(const direction d : green )
            {
                create_road(map, d, c_p, c_p, nodes, -1);
            }
        }

    }
    else
    {
        if(map.at<    Vec3b>(c_p.row, c_p.col) ==    Vec3b(255,0,0) && p != c_p )
        {
            connect_to_road(roads.at<int>(c_p.row, c_p.col) , road );
            return;
        }
        pixel p_new(-1,-1) ;

        switch(dir) {
            case LEFT:
                p_new = pixel( c_p.row, c_p.col - 1);
                break;
            case LEFT_UP:
                p_new = pixel(c_p.row - 1, c_p.col - 1);
                break;
            case UP:
                p_new = pixel(c_p.row - 1, c_p.col);
                break;
            case RIGHT_UP:
                p_new = pixel(c_p.row - 1, c_p.col + 1);
                break;
            case RIGHT:
                p_new = pixel(c_p.row, c_p.col + 1);
                break;
            case RIGHT_DOWN:
                p_new = pixel(c_p.row + 1, c_p.col + 1);
                break;
            case DOWN:
                p_new = pixel(c_p.row + 1, c_p.col);
                break;
            case LEFT_DOWN:
                p_new = pixel(c_p.row + 1, c_p.col - 1);
                break;
        }
        create_road(map, dir, p, p_new, nodes, _road);
    }
}

roadmap::direction roadmap::degress2direction(const float angle) const noexcept
{

    if(angle == 0 || angle == 360)
        return LEFT;

    if(angle > 0 && angle < 90)
        return LEFT_UP;

    if(angle == 90)
        return UP;

    if(90 < angle && angle < 180)
        return RIGHT_UP;

    if(angle == 180)
        return RIGHT;

    if(180 < angle && angle < 270)
        return RIGHT_DOWN;

    if(angle == 270)
        return DOWN;

    if(270 < angle )
        return LEFT_DOWN;
}

roadmap::direction roadmap::opposite_direction(const direction dir) noexcept
{

    switch(dir) {
        case LEFT:
            return RIGHT;
        case LEFT_UP:
            return RIGHT_DOWN;
        case UP:
            return DOWN;
        case RIGHT_UP:
            return LEFT_DOWN;
        case RIGHT:
            return LEFT;
        case RIGHT_DOWN:
            return LEFT_UP;
        case DOWN:
            return UP;
        case LEFT_DOWN:
            return RIGHT_UP;
    }
}
void roadmap::explore_directions(Mat & map, const pixel p, const vector<direction> directions, vector<pixel> & nodes) noexcept
{
    for( const direction d : directions)
    {
        direction d2 = opposite_direction(d);

        int steps = explore_move(map, p, p, d2);
        if(steps != -1)
        {
            explore_draw(map,p,p,d2, steps, nodes);
        }
    }
}

int roadmap::explore_move(const Mat & map, const pixel p, const pixel c_p, const direction dir) const noexcept
{
    if(map.at<Vec3b>(c_p.row, c_p.col) == Vec3b(0,0,0))
        return 0;

    vector<direction> dir_list, dir_list2;
    int n = find_moving_directions(map, dir_list, c_p, Vec3b(0,255,0) ) + find_moving_directions(map, dir_list2, c_p, Vec3b(255,0,0) );


    if((n > 0 && p != c_p) )
        return 1;

    if(map.at<Vec3b>(c_p.row, c_p.col) == Vec3b(0,255,0) || map.at<Vec3b>(c_p.row, c_p.col) == Vec3b(255,0,0)  )
        return 1;

    pixel p_new(-1,-1);
    switch(dir) {
        case LEFT:
            p_new = pixel( c_p.row, c_p.col - 1);
            break;
        case LEFT_UP:
            p_new = pixel(c_p.row - 1, c_p.col - 1);
            break;
        case UP:
            p_new = pixel(c_p.row - 1, c_p.col);
            break;
        case RIGHT_UP:
            p_new = pixel(c_p.row - 1, c_p.col + 1);
            break;
        case RIGHT:
            p_new = pixel(c_p.row, c_p.col + 1);
            break;
        case RIGHT_DOWN:
            p_new = pixel(c_p.row + 1, c_p.col + 1);
            break;
        case DOWN:
            p_new = pixel(c_p.row + 1, c_p.col);
            break;
        case LEFT_DOWN:
            p_new = pixel(c_p.row + 1, c_p.col - 1);
            break;
    }
    int r =  explore_move(map, p, p_new, dir);

    if(r == 0)
        return 0;
    if(p == c_p)
        return r;

    return 1 + r;

}

void roadmap::explore_draw(Mat & map, const pixel p, const pixel c_p, const direction dir, const int moves_left, vector<pixel> & nodes) noexcept
{
    int move = moves_left;

    if(move < 1)
        return;

    if(move == 1 && p != c_p)
    {
        nodes.push_back(c_p);
        create_intersection(c_p);
    }
    else
    {
        map.at<    Vec3b>(c_p.row, c_p.col) =    Vec3b(0,255,0);
        nodes.push_back(c_p);
    }

    if(p != c_p)
        move--;


    pixel p_new(-1,-1);
    switch(dir) {
        case LEFT:
            p_new = pixel( c_p.row, c_p.col - 1);
            break;
        case LEFT_UP:
            p_new = pixel(c_p.row - 1, c_p.col - 1);
            break;
        case UP:
            p_new = pixel(c_p.row - 1, c_p.col);
            break;
        case RIGHT_UP:
            p_new = pixel(c_p.row - 1, c_p.col + 1);
            break;
        case RIGHT:
            p_new = pixel(c_p.row, c_p.col + 1);
            break;
        case RIGHT_DOWN:
            p_new = pixel(c_p.row + 1, c_p.col + 1);
            break;
        case DOWN:
            p_new = pixel(c_p.row + 1, c_p.col);
            break;
        case LEFT_DOWN:
            p_new = pixel(c_p.row + 1, c_p.col - 1);
            break;
    }
    explore_draw(map,p,p_new, dir, move, nodes);

}

void roadmap::explore_draw(Mat & map, const pixel p, const pixel c_p, const direction dir, const pixel end) const noexcept
{
    if(c_p == end)
        return;

    if (p != c_p) {
        map.at<    Vec3b>(c_p.row, c_p.col) =    Vec3b(0,255,0);
    }

    pixel p_new(-1,-1);
    switch(dir) {
        case LEFT:
            p_new = pixel( c_p.row, c_p.col - 1);
            break;
        case LEFT_UP:
            p_new = pixel(c_p.row - 1, c_p.col - 1);
            break;
        case UP:
            p_new = pixel(c_p.row - 1, c_p.col);
            break;
        case RIGHT_UP:
            p_new = pixel(c_p.row - 1, c_p.col + 1);
            break;
        case RIGHT:
            p_new = pixel(c_p.row, c_p.col + 1);
            break;
        case RIGHT_DOWN:
            p_new = pixel(c_p.row + 1, c_p.col + 1);
            break;
        case DOWN:
            p_new = pixel(c_p.row + 1, c_p.col);
            break;
        case LEFT_DOWN:
            p_new = pixel(c_p.row + 1, c_p.col - 1);
            break;
    }
    explore_draw(map,p,p_new, dir, end);
}
bool roadmap::move(vector<Point> & n1, const Mat & brushfire, Mat & map, Point start,Point end)
{
    Point current = start;
    while(current != end)
    {
        // Find direction
        int y = end.y - start.y;
        int x = end.x - start.x;

        direction dir;

        if( y < 0)
        {
           if(x < 0)
               dir = LEFT_UP;
           else
               dir = RIGHT_UP;

        }
        else if( y > 0)
        {
            if( x < 0)
                dir = LEFT_DOWN;
            else
                dir = RIGHT_DOWN;
        }
        else if(y == 0)
        {
            if(x < 0)
                dir = LEFT;
            else
                dir = RIGHT;

        }
        else if(x == 0)
        {
            if(y < 0)
                dir = UP;
            else
                dir = DOWN;
        }
        else
        {
            dir = LEFT;
        }
        Point p_new(0, 0);
        switch(dir)
        {
            case LEFT:
                p_new +=     Point(-1, 0);
                break;
            case LEFT_UP:
                p_new +=     Point( - 1,  - 1);
                break;
            case UP:
                p_new +=     Point(0, -1);
                break;
            case RIGHT_UP:
                p_new +=     Point( 1, -1);
                break;
            case RIGHT:
                p_new +=     Point(1,0);
                break;
            case RIGHT_DOWN:
                p_new +=     Point(1,1);
                break;
            case DOWN:
                p_new +=     Point(0,1);
                break;
            case LEFT_DOWN:
                p_new +=     Point(-1,1);
                break;
        }

        if(p_new == Point(0,0))
        {
            return false;
        }

        uchar current_brush = brushfire.at<Vec3b>(current.y, current.x)[0];

        current += p_new;

        uchar new_brush = brushfire.at<Vec3b>(current.y, current.x)[0];

        if(new_brush < current_brush) {
            return false;
        }

        switch(dir) {
            case LEFT:
                break;
            case LEFT_UP:
                if(brushfire.at<Vec3b>(current.y, current.x+1) == Vec3b(0,0,0) || brushfire.at<Vec3b>(current.y + 1, current.x) == Vec3b(0,0,0) )
                {
                    return false;
                }
                break;
            case UP:
                break;
            case RIGHT_UP:
                if(brushfire.at<Vec3b>(current.y, current.x-1) == Vec3b(0,0,0) || brushfire.at<Vec3b>(current.y + 1, current.x) == Vec3b(0,0,0) )
                {
                    return false;
                }
                break;
            case RIGHT:
                break;
            case RIGHT_DOWN:
                if(brushfire.at<Vec3b>(current.y, current.x-1) == Vec3b(0,0,0)  ||  brushfire.at<    Vec3b>(current.y - 1, current.x) ==  Vec3b(0,0,0) )
                {
                    return false;
                }
                break;
            case DOWN:
                break;
            case LEFT_DOWN:
                if(brushfire.at<Vec3b>(current.y, current.x+1) == Vec3b(0,0,0)  ||  brushfire.at<Vec3b>(current.y - 1, current.x) == Vec3b(0,0,0) ) {
                    return false;
                }
                break;
        }

        if(brushfire.at<Vec3b>(current.y, current.x) == Vec3b(0,0,0))
        {
            return false;
        }

        if((map.at<Vec3b>(current.y, current.x) == Vec3b(0,255,0) && map.at<Vec3b>(current.y, current.x) == Vec3b(255,0,0)) && current != end)
        {
            return false;
        }

        n1.push_back(current);
    }
    return true;
}


void roadmap::move_draw(const vector<Point> & n1, vector<pixel> & n2, Mat & map){
    for(const Point p : n1 )
    {
        if(map.at<Vec3b>(p.y,p.x) != Vec3b(0,255,0)  )
        {
            n2.push_back(pixel(p.y,p.x));
            map.at<Vec3b>(p.y, p.x) = Vec3b(0,255,0);
        }
    }
}


int roadmap::remove_duplicate_nodes(vector<pixel> & nodes)
{
    int count = 0;
    for(size_t i = 0; i < nodes.size(); i++ )
    {
        for(size_t j = 0; j < nodes.size(); j++ )
        {
            if(i != j)
            {
                if(nodes[i] == nodes[j])
                {
                    nodes.erase(nodes.begin() + j );
                    count++;
                }
            }
        }
    }
    return count;
}
