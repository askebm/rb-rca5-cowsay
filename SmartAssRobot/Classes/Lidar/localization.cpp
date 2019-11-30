#include "localization.h"

double particle::getX()
{
    return x;
}
double particle::getY()
{
    return y;
}
double particle::getBeta()
{
    return beta;
}
double particle::getWeight()
{
    return weight;
}
double particle::getLastTime()
{
    return last_time;
}
void particle::updateState(double x_n, double y_n, double beta_n, double time)
{
    last_time = time;
    x = x_n;
    y = y_n;
    beta = beta_n;
}

Localization::Localization(int sample_size, Mat m)
{
    N = sample_size;
    map = m;
    init();
}

Localization::~Localization(){}

void Localization::init()
{
    first_flag = true;

    for(int i = 0; i < N; i++)
    {

    }
    Laserscanner s;
    prediction(s);

}
void Localization::prediction(Laserscanner s)
{
    boost::random::normal_distribution<double> error_vel(0, 0.2);
    boost::random::normal_distribution<double> error_angle(0, 0.2);
    bool updated = s.hasUpdated(updated);

    if(updated)
    {
        if(first_flag)
        {
            for(int i = 0; i < N; i++)
            {
                //cal smaple time
                current_time = time(&timer);
                double sample_time = 5;
                //get the partciles current position
                double current_x = samples[i].getX();
                double current_y = samples[i].getY();
                double current_beta = samples[i].getBeta();

                // new predicted values for all particles based on their previous position
                double new_beta = s.angle + error_angle(gen);
                double new_x = current_x + (s.vel + error_vel(gen))*sample_time*cos(current_beta);
                double new_y = current_y + (s.vel + error_vel(gen))*sample_time*sin(current_beta);

                // check if the particles generated coordinates are inside the map
                if(checkCoordinates(new_x, new_y))
                {
                   //update the predicted values and last time.
                   samples[i].updateState(new_x, new_y, new_beta, current_time);
                }
            }
        }
        else
        {
            for(int i = 0; i < N; i++)
            {
                //cal smaple time
                current_time = time(&timer);
                double sample_time = current_time - samples[i].getLastTime();
                //get the partciles current position
                double current_x = samples[i].getX();
                double current_y = samples[i].getY();
                double current_beta = samples[i].getBeta();

                // new predicted values for all particles based on their previous position
                double new_beta = s.angle + error_angle(gen);
                double new_x = current_x + (s.vel + error_vel(gen))*sample_time*cos(current_beta);
                double new_y = current_y + (s.vel + error_vel(gen))*sample_time*sin(current_beta);

                // check if the particles generated coordinates are inside the map
                if(checkCoordinates(new_x, new_y))
                {
                   //update the predicted values and last time.
                   samples[i].updateState(new_x, new_y, new_beta, current_time);
                }

            }
        }
    }

    updatePos();


}
vector<particle> Localization::resampling(vector<particle> M)
{
    vector<particle> new_m;
    boost::random::normal_distribution<> delta(0, N-1);
    double c = M[0].weight;
    int i = 0;
    double u;
    int sum;
    for(int j = 0; j < N; j++)
    {
        u = delta(gen) + j*pow(N,-1);
        while(u > c)
        {
            i =+ 1;
            c =+ M[i].weight;
        }
        if(i > N)
            i = 0;


        sum =+ c;


    }

    return new_m;
}

void Localization::updatePos()
{



    //resampling();
}

bool Localization::checkCoordinates(double x, double y)
{
    double rows = map.rows;
    double cols = map.cols;
    if(x > rows -1 || x < 0)
    {
        return false;
    }
    if(y > cols - 1 || y < 0)
    {
        return false;
    }
    else
        return true;

}
//float Localization::ray(Mat* map, Point2f p, float angle){
//    Point2f delta(cos(angle),sin(angle));
//    float scale = (72. / 25.4) * 2;
//    for(float r = 0; r < _range_max * scale; r+=2){
//        if(map->at<uchar>(p+r*delta) == 0)
//            return r / scale;
//    }
//    return _range_max;
//}
