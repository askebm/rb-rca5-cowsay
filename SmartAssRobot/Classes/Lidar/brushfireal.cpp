#include "brushfireal.h"

static boost::mutex mutex_b;

BrushfireAl::BrushfireAl()
{
    Mat image = imread("/home/annie/git_repo/rb-rca5-cowsay/models/bigworld/meshes/floor_plan.png", CV_LOAD_IMAGE_GRAYSCALE);
    mutex_b.lock();
    grey_scale = image.clone();
    mutex_b.unlock();

    convertToBinary(image);

    brushfire();

    mutex_b.lock();
    cv::namedWindow("Image_o");
    cv::imshow("Image_o", binary_image);
    cv::namedWindow("Image");
    cv::imshow("Image", grey_scale);
    mutex_b.unlock();

}

void BrushfireAl::convertToBinary(Mat img)
{
    mutex_b.lock();
    threshold(img, binary_image, 127,255, THRESH_BINARY);
    resize(binary_image, binary_image, Size(), 10, 10, INTER_CUBIC); // upscale 10x
    resize(grey_scale, grey_scale, Size(), 10, 10, INTER_CUBIC); // upscale 10x
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
            pixel_array.push_back(grey_scale.at<uchar>(Point(x,y)));
            counter++;
        }
    }

    int division_factor_row = counter/binary_image.rows;
    int division_factor_col = counter/binary_image.cols;
    int i = 0;

    while(i < 100)
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

}
