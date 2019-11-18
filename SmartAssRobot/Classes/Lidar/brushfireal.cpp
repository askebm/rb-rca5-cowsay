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
            else if(pixel_array[counter] == 255)
            {
                colourPath[0] = 255;
                colourPath[1] = 0;
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

