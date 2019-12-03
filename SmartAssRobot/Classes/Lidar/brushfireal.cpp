#include "brushfireal.h"

static boost::mutex mutex_b;

BrushfireAl::BrushfireAl()
{
    Mat image = imread("/home/annie/git_repo/rb-rca5-cowsay/models/bigworld/meshes/floor_plan.png", CV_LOAD_IMAGE_GRAYSCALE);
    mutex_b.lock();
    grey_scale = image.clone();
    colour_img =image.clone();
    cvtColor(colour_img, colour_img, CV_GRAY2RGB);
    mutex_b.unlock();

    convertToBinary(image); // converts the image to binary and scales up all images.

    start_x = binary_image.rows/2;
    start_y = binary_image.cols/2;
    brushfire();

    imwrite( "/home/annie/brushfire_1.png", grey_scale );
    mutex_b.lock();
    cv::namedWindow("Image_o");
    cv::imshow("Image_o", binary_image);
    cv::namedWindow("Image");
    cv::imshow("Image", grey_scale);
    cv::namedWindow("Image_colour");
    cv::imshow("Image_colour", colour_img);
    mutex_b.unlock();
}

void BrushfireAl::convertToBinary(Mat img)
{
    mutex_b.lock();
    resize(img, img, Size(), 10, 10, INTER_CUBIC); // upscale 10x
    resize(grey_scale, grey_scale, Size(), 10, 10, INTER_CUBIC); // upscale 10x
    resize(colour_img, colour_img, Size(), 10, 10, INTER_CUBIC); // upscale 10x
    threshold(img, binary_image, 127,255, THRESH_BINARY);
    mutex_b.unlock();

}
void BrushfireAl::brushfire()
{

    vector<int> pixel_array;
    int counter = 0;
    int scale_factor;
    int up;
    int down;
    int right;
    int left;
    scale_factor = binary_image.rows;

    for(int x = 0; x <= binary_image.cols; x++)
    {
        for(int y = 0; y <= binary_image.rows; y++)
        {
            if(binary_image.at<uchar>(Point(x,y)) == 0)
            {
                    grey_scale.at<uchar>(Point(x,y)) = 1;
            }
            else
                grey_scale.at<uchar>(Point(x,y)) = 0;

            pixel_array.push_back(grey_scale.at<uchar>(Point(x,y)));
            counter++;
        }
    }
    int division_factor_row = counter/binary_image.rows;
    int division_factor_col = counter/binary_image.cols;
    int i = 0;

    while(i < 200)
    {
        for(int x = 0; x <= pixel_array.size(); x++)
        {
            int check_sum = 256;
            int zero_check = 0;
            if((x + 1) % division_factor_row != 1)
            {
                right  = pixel_array[x + 1];
                if(check_sum > right)
                {
                    if(right == 0)
                    {
                        zero_check++;
                    }
                    else
                        check_sum = right;
                }
            }
            if(x % division_factor_col != 0)
            {
                left = pixel_array[x - 1];
                if(check_sum > left )
                {
                    if(left == 0)
                    {
                        zero_check++;
                    }
                    else
                        check_sum = left;
                }
            }
            if(x - scale_factor >= 0)
            {
                down = pixel_array[x - scale_factor];
                if(check_sum > down)
                {
                    if(down == 0)
                    {
                        zero_check++;
                    }
                    else
                        check_sum = down;
                }
            }
            if(x + scale_factor <= counter)
            {
                up = pixel_array[x + scale_factor];
                if(check_sum > up)
                {
                    if(up == 0)
                    {
                        zero_check++;
                    }
                    else
                        check_sum = up;
                }
            }
            if(pixel_array[x] != 1)
            {
                if(zero_check == 4)
                {
                    check_sum = 0;
                }
                else if(check_sum > 0)
                {
                     pixel_array[x] = check_sum + 1;
                }
            }
        }
        i++;
    }
    int counter1 = 0;
    vector<vector<int>> matrix(binary_image.cols, vector<int>(binary_image.rows));
    matrix_real = matrix;
    for(int x = 0; x <= binary_image.cols; x++)
    {
        for(int y = 0; y <= binary_image.rows; y++)
        {
            grey_scale.at<uchar>(Point(x,y)) = pixel_array[counter1];
            counter1++;
        }

    }
    addNodes(pixel_array);
}

//Adding the path the a blank map image. The road will be hown in green. Intersection are not added here.
void BrushfireAl::addNodes(vector<int> pixel_array)
{
    Vec3b colourPath = colour_img.at<Vec3b>(Point(100,100));
    colourPath[0] = 0;
    colourPath[1] = 0;
    colourPath[2] = 200;

    int counter = 0;
    for(int x = 0; x <= binary_image.cols; x++)
    {
        for(int y = 0; y <= binary_image.rows; y++)
        {
            if(pixel_array[counter] == 1)
            {
                colourPath[0] = 0;
                colourPath[1] = 0;
                colourPath[2] = 0;
                colour_img.at<Vec3b>(Point(x,y)) = colourPath;
            }
            else if(checkGradient(x,y))
            {
                colourPath[0] = 0;
                colourPath[1] = 255;
                colourPath[2] = 0;
                colour_img.at<Vec3b>(Point(x,y)) = colourPath;
            }
            else
            {
                colourPath[0] = 255;
                colourPath[1] = 255;
                colourPath[2] = 255;
                colour_img.at<Vec3b>(Point(x,y)) = colourPath;
            }
            counter++;
        }
    }
    nodeVector();

    Vec3b colourPoint = colour_img.at<Vec3b>(Point(100,100));
    colourPoint[0] = 0;
    colourPoint[1] = 0;
    colourPoint[2] = 255;
    for(int i = 0; i < important_nodes.size(); i++)
    {
        colour_img.at<Vec3b>(Point(important_nodes[i].x, important_nodes[i].y)) = colourPoint;
    }
    colourPoint[0] = 0;
    colourPoint[1] = 255;
    colourPoint[2] = 0;
    colour_img.at<Vec3b>(Point(start_y, start_x)) = colourPoint;

}
// Support function test Dijstra's ability to find hardcoddedd nodes. Makes a vector filed with nodes for the map.
// The addNode function adds them lastly after having generated the path
void BrushfireAl::nodeVector()
{
    Point1 add;
    add.x = 100;
    add.y = 100;
    important_nodes.push_back(add);
    add.x = 200;
    add.y = 200;
    important_nodes.push_back(add);
    add.x = 400;
    add.y = 175;
    important_nodes.push_back(add);
    add.x = 600;
    add.y = 100;
    important_nodes.push_back(add);
    add.x = 900;
    add.y = 100;
    important_nodes.push_back(add);
    add.x = 1100;
    add.y = 100;
    important_nodes.push_back(add);
    add.x = 650;
    add.y = 250;
    important_nodes.push_back(add);
    add.x = 100;
    add.y = 600;
    important_nodes.push_back(add);
    add.x = 300;
    add.y = 600;
    important_nodes.push_back(add);
    add.x = 600;
    add.y = 550;
    important_nodes.push_back(add);
    add.x = 1050;
    add.y = 600;
    important_nodes.push_back(add);
    add.x = 550;
    add.y = 700;
    important_nodes.push_back(add);
    add.x = 750;
    add.y = 700;
    important_nodes.push_back(add);
}
// Debugging function to find a position in the map to palce a hardcodded node.
void BrushfireAl::findNodes()
{
    Vec3b colour = colour_img.at<Vec3b>(Point(100,100));
    colour[0] = 255;
    colour[1] = 0;
    colour[2] = 0;
    for(int i = 0; i <= 20; i++)
    {
        for(int j = 0; j <= 20; j++)
        {
            colour_img.at<Vec3b>(Point(colour_img.cols/2 + i,colour_img.rows / 2 + i)) = colour;
        }
    }
}
//This functions uses check_neighbours to see how many pixels around it has a higher pixel value
// than it. If it has less than four neighbours higher or equal
bool BrushfireAl::checkGradient(int x, int y)
{
    vector<Neighbour> neighbours;
    check_neighbours(neighbours, y, x, 1);

    int equal = 0;
    int higher = 0;
    for(int i = 0; i < neighbours.size(); i++)
    {
        if(neighbours[i].equal == 1)
        {
            equal++;
        }
        else if(neighbours[i].higher == 1)
        {
            higher++;
        }
    }
    if(equal < 3 && higher == 2)
    {
        return true;
    }
    else if(higher == 1 && equal > 3)
    {
        return true;
    }
    else
        return false;
}

// checks all 8 neightbours. - the which types refers to whether it looks for white pixels or the pixel value of neighbours.
void BrushfireAl::check_neighbours(vector<Neighbour> & neighbours, int row, int col, int which_type)
{
    vector<Point1> neighbours1;
    if(which_type == 1)
    {
        for(int temp_r = row - 1; temp_r < row + 2; temp_r++)
        {
            // checks if the temperary row value doesnt go under 0 or exceeds the image's row dimensions
            if(temp_r >= 0 && temp_r < grey_scale.rows)
            {
                for(int temp_c = col - 1; temp_c < col + 2 ; temp_c++ )
                {
                    // checks if the temperary column value doesnt go under 0 or exceeds the image's column dimensions
                    if(temp_c >= 0 && temp_c < grey_scale.cols)
                    {
                        //checks if the current pixel is white, if so, save it.
                        if(grey_scale.at<uchar>(temp_r,temp_c) < grey_scale.at<uchar>(row,col) && grey_scale.at<uchar>(temp_r,temp_c) != 0 && grey_scale.at<uchar>(temp_r,temp_c) != 1)
                        {
                            Neighbour temp;
                            temp.higher = 1;
                            temp.equal = 0;
                            neighbours.push_back(temp);
                        }
                        else if(grey_scale.at<uchar>(temp_r,temp_c) == grey_scale.at<uchar>(row,col) && grey_scale.at<uchar>(temp_r,temp_c) != 0 && grey_scale.at<uchar>(temp_r,temp_c) != 1)
                        {
                            Neighbour temp;
                            temp.higher = 0;
                            temp.equal = 1;
                            neighbours.push_back(temp);
                        }
                    }
                }
            }
        }
    }

   else
    {
        for(int temp_r = row - 1; temp_r < row + 2; temp_r++)
        {
            // checks if the temperary row value doesnt go under 0 or exceeds the image's row dimensions
            if(temp_r >= 0 && temp_r < grey_scale.rows)
            {
                for(int temp_c = col - 1; temp_c < col + 2 ; temp_c++ )
                {
                    // checks if the temperary column value doesnt go under 0 or exceeds the image's column dimensions
                    if(temp_c >= 0 && temp_c < grey_scale.cols)
                    {
                        //checks if the current pixel is white, if so, save it.
                        if(grey_scale.at<uchar>(temp_r,temp_c) == uchar(255))
                        {
                            Point1 temp;
                            temp.x = temp_c;
                            temp.y = temp_r;
                            neighbours1.push_back(temp);
                        }
                    }
                }
            }
        }
    }
}
