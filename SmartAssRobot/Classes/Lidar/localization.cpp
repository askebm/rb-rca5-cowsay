#include "localization.h"

Localization::Localization(){}

vector<vector<float>> Localization::lineExtraction(vector<float> p, vector<float> theta, int n )
{
    double line_model = 0;
    vector<float> temp_p;
    vector<float> temp_theta;
    int no_o_points = 2; // number of points - starts with 2.

    for(int i = 0; i <= n; i++)
    {
        temp_p[i] = p[i];
        temp_p[i+1] = p[i+1];
        temp_theta[i] = theta[i];
        temp_theta[i+1] = theta[i+1];

        leastSquare(temp_p,temp_theta, no_o_points); // computes the starting line

        while(line_model <= threshhold_d && line_model >= -threshhold_d && i <= n)
        {
                i++;
                temp_p[i] = p[i];
                temp_theta[i] = theta[i];
                line_model = p[i]*cos(theta[i]-alpha) - r; // check if the point is part of the line
                no_o_points++; // counts how many points make up one line
                leastSquare(temp_p,temp_theta, no_o_points); // recompute the line parameters
        }

        if(i < n) // if there are still more points in the origin vector, store line
        {
            temp_p.pop_back();
            temp_theta.pop_back();
            i = i - 1;
            for(int  k = 0; k < no_o_points; k++)
            {
                cout <<"hej" << endl;
            }
            no_o_points = 2;
            points.push_back(temp_p);
            points.push_back(temp_theta);
            temp_p.clear();
            temp_theta.clear();
        }
        else
        {
            points.push_back(temp_p);
            points.push_back(temp_theta);
            return points;
        }
    }
}

void Localization::rawData(float range, float angle, int counter)
{
    // creates a tempt vectors for x and y that will be cleared after use
    vector<float> p;
    vector<float> theta;

    // while still in the same lidar "frame" find x and y points and pushes them to their respective vcectors.
    while(counter <= 200)
    {
        p.push_back(abs(range));
        theta.push_back(abs(angle));
    }

    lineExtraction(p,theta, counter);
    //flushes the tempt vectors so ready for a new "frame".
    p.clear();
    theta.clear();
}
void Localization::leastSquare(vector<float> p, vector<float> theta, int n)
{
    double x1 = 0, x2 = 0, y1 = 0, y2 =0; // to calculate atan2

    for(int i = 0; i <= n; i++)
    {
        for(int j = 0; j <= n; j++)
        {
            x1 = x1 + p[i]*p[j]*cos(theta[i])*sin(theta[j]);
        }
    }
    x1 = -1/2 * x1;

    for(int i = 0; i <= n; i++)
    {
        for(int j = 0; j <= n; j++)
        {
            x2 = x2 + p[i]*p[j]*cos(theta[i]+theta[j]);
        }
    }
    x2 = -1/1 * x1;

    for(int i = 0; i <= n; i++)
    {
        y1 = y1 + p[i]*p[i]*sin(2*theta[i]);
    }

    for(int i = 0; i <= n; i++)
    {
        y2 = y2 + p[i]*p[i]*cos(2*theta[i]);
    }

    double x = x1/x2;
    double y = y1/y2;

    alpha = 1/2*atan2(y,x);

    for(int i = 0; i <= n; i++)
    {
        r = r + p[i]*cos(theta[i] - alpha);
    }
}

Localization::~Localization(){}

