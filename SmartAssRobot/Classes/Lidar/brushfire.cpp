#include "brushfire.h"

// checks all 9 neightbours.
void check_neighbours(Mat & image,  vector<pixel> & neighbours, int row, int col)
{ 
    for(int temp_r = row - 1; temp_r < row + 2; temp_r++)
    {
        // checks if the temperary row value doesnt go under 0 or exceeds the image's row dimensions
        if(temp_r >= 0 && temp_r < image.rows)
        {
            for(int temp_c = col - 1; temp_c < col + 2 ; temp_c++ )
            {
                // checks if the temperary column value doesnt go under 0 or exceeds the image's column dimensions
                if(temp_c >= 0 && temp_c < image.cols)
                {
                    //checks if the current pixel is white, if so, save it.
                    if(image.at<Vec3b>(temp_r,temp_c) == Vec3b(255,255,255))
                    {
                        pixel temp;
                        temp.row = temp_r;
                        temp.col = temp_c;
                        neighbours.push_back(temp);
                    }
                }
            }
        }
    }
}

int generate_brushfire(Mat & source, Mat & return_image)
{
    cout << "generating brushfire" <<  endl;

    vector<pixel> neighbours;

    for(int rows = 0; rows < source.rows; rows++ )
    {
        for(int cols = 0; cols < source.cols; cols++ )
        {
            if(source.at<Vec3b>(rows,cols) == Vec3b(0,0,0))
                check_neighbours(source, neighbours, rows,cols);
        }
    }

    uchar colour = starting_value;

    return_image = source.clone();
    while(neighbours.size() != 0 )
    {

        std::vector<pixel> newneighbours;

        for(pixel pixel_v : neighbours )
        {

           if(return_image.at<Vec3b>(pixel_v.row, pixel_v.col) == Vec3b(255,255,255) )
           {
               return_image.at<Vec3b>(pixel_v.row, pixel_v.col) = Vec3b(colour, colour,colour);
               check_neighbours(return_image, newneighbours, pixel_v.row, pixel_v.col);
           }
        }

        colour += value_step;
        neighbours = newneighbours;

    }

    colour -= value_step;
    colour -= value_step;

    return (int)colour;
}
