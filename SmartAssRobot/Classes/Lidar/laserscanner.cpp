#include "laserscanner.h"

Laserscanner::Laserscanner()
{

}

void Laserscanner::update(ConstLaserScanStampedPtr &msg)
{
    angle_min = float(msg->scan().angle_min());
    angle_max = float(msg->scan().angle_max());
    angle_increment = float(msg->scan().angle_step());

    range_min = float(msg->scan().range_min());
    range_max = float(msg->scan().range_max());

    nranges = msg->scan().ranges_size();
    nintensities = msg->scan().intensities_size();

    //cout << "I was here!" << endl;
    vel = 1;
}
bool Laserscanner::hasUpdated(bool flag)
{

    while(!flag)
    {

    }
    return true;

}

vector<rays> Laserscanner::rayCasting(double x, double y, double direction, double betaa)
{

    double offset_angle = betaa;
    vector<rays> new_rays;
    struct rays new_r;
    for(int i = 0; i < 200; i++)
    {
        new_rays.push_back(new_r);
    }
    for(int i = 0; i < nranges; i++)
    {
        float angle = angle_min + i * angle_increment + offset_angle;
        double x1 = x + range_max *cos(angle);
        double y1 = y + range_max*sin(angle);  
        new_r.distance = drawLines(x,x1,y,y1);
        new_r.dir = angle;
        new_rays[i] = new_r;
    }
    return new_rays;
}

double Laserscanner::drawLines(double x0, double x1, double y0, double y1)
{
    double distance_r;
    vector<double> x;
    vector<double> y;
    double temp_y;
    double temp_x;
    double number_of_x_it = abs(x1-x0);
    double number_of_y_it = abs(y1-y0);
    if(abs(y1 -y0) < abs(x1 - x0))
    {
        if(x0 > x1)
        {
            double  i = 0;
            temp_x = x1;
            x.push_back(temp_x);
            while(number_of_x_it > i)
            {
                temp_x=+1;
                x.push_back(temp_x);
                i++;
            }
            i = 0;
            temp_y = y1;
            y.push_back(temp_y);
            while(i < number_of_x_it)
            {
                if(i < number_of_y_it)
                {
                    temp_y=+1;
                }
                y.push_back(temp_y);
                i++;
            }
        }
        else
        {
            double i = 0;
            temp_x = x0;
            x.push_back(temp_x);
            while(number_of_x_it > i)
            {
                temp_x=+1;
                x.push_back(temp_x);
                i++;
            }
            i = 0;
            temp_y = y0;
            y.push_back(temp_y);
            while(i < number_of_x_it)
            {
                if(i < number_of_y_it)
                {
                    temp_y=+1;
                }
                y.push_back(temp_y);
                i++;
            }

        }
    }
    else
    {
        if(y0 > y1)
        {
            double  i = 0;
            temp_x = x1;
            x.push_back(temp_x);
            while(number_of_y_it > i)
            {
                if(i < number_of_x_it)
                {
                    temp_x=+1;
                }
                x.push_back(temp_x);
                i++;
            }
            i = 0;
            temp_y = y1;
            y.push_back(temp_y);
            while(i < number_of_x_it)
            {
                temp_y=+1;
                y.push_back(temp_y);
                i++;
            }
        }
        else
        {
            double i = 0;
            temp_x = x0;
            x.push_back(temp_x);
            while(number_of_y_it > i)
            {
                //only change the x coordinate
                if(i < number_of_x_it)
                {
                    temp_x=+1;
                }
                x.push_back(temp_x);
                i++;
            }
            i = 0;
            temp_y = y0;
            y.push_back(temp_y);
            while(i < number_of_y_it)
            {
                temp_y=+1;
                y.push_back(temp_y);
                i++;
            }
        }
    }

    distance_r = calDistance(x,y);

    return distance_r;
}
double Laserscanner::calDistance(vector<double> x, vector<double> y)
{
//    int midt_x = map.rows/2;
//    int midt_y = map.cols/2;
//    cout << midt_x << " " << midt_y << endl;
    double distance;
    int tmp = 0;
    for(int i = 0; i < x.size(); i++)
    {
        if((int)map.at<uchar>(Point(x[i],y[i])) == 255)
        {
            tmp =+ 1;
        }
        else
            break;
    }


    distance = sqrt(pow(abs(x[0]-x[tmp]),2) + pow(abs(y[0]-y[tmp]),2));
    return distance;
}
