#include "brushfire.h"

// checks all 9 neightbours.
void check_neighbors(Mat & image,  vector<pixel_t> & neighbors, int row, int col)
{
    for(int r = row - 1; r < row + 2; r++ )
    {
        if(r > -1 && r < image.rows)
        {
            for(int c = col - 1; c < col + 2 ; c++ )
            {
                if(c > -1 && c < image.cols)
                {
                    if(image.at<Vec3b>(r,c) == Vec3b(255,255,255))
                    {
                        pixel_t tmp;
                        tmp.row = r;
                        tmp.col = c;
                        neighbors.push_back(tmp);
                    }
                }
            }
        }
    }
}

int generate_brushfire(Mat & source, Mat & dst)
{
    cout << "generating brushfire" <<  endl;

    vector<pixel_t> neighbors;

    int rows = source.rows;
    int cols = source.cols;
    for(int row = 0; row < rows ; row++ )
    {
        for(int col = 0; col < cols ; col++ )
        {
            if(source.at<Vec3b>(row,col) == Vec3b(0,0,0))
                    check_neighbors(source, neighbors, row, col);
        }
    }
    cout << "Checked the image for obstalce found " << neighbors.size() << " neighbors" <<  endl;

    uchar color = BRUSHFIRE_BEGIN;

    dst = source.clone();
    while(neighbors.size() != 0 )
    {

        vector<pixel_t> newneighbors;

        for(pixel_t pixel : neighbors )
        {

           if(dst.at<cv::Vec3b>(pixel.row, pixel.col) == cv::Vec3b(255,255,255) )
           {
              dst.at<cv::Vec3b>(pixel.row, pixel.col) = cv::Vec3b(color, color,color);
              check_neighbors(dst, newneighbors, pixel.row, pixel.col);
           }
        }
            color += BRUSHFIRE_STEP_SIZE;
            neighbors = newneighbors;

     cout << "There are " << newneighbors.size() << " new neighbors" <<  endl;
    }

    color -= BRUSHFIRE_STEP_SIZE;
    color -= BRUSHFIRE_STEP_SIZE;

    return (int)color;
}
