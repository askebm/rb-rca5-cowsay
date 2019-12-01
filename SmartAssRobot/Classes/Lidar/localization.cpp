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

Localization::Localization(int sample_size, Laserscanner *ls)
{
    N = sample_size;
    init(ls);
}

Localization::~Localization(){}

void Localization::init(Laserscanner *ls)
{
    uniform_real_distribution<double> vel_variance(-2.5, 2.5);
    uniform_real_distribution<double> angle_variance(-2.89, 2.89);
    first_flag = true;

    particle p;

    for(int i = 0; i < N; i++)
    {
        //gives all starting particles a random starting point in a 5x5 cube
        double start_x = ls->robot_x + vel_variance(gen);
        double start_y = ls->robot_y + vel_variance(gen);
        double start_beta = ls->robot_angle + angle_variance(gen);
        current_time = time(&timer);
        p.last_time = current_time;
        p.x = start_x;
        p.y = start_y;
        p.beta = start_beta;
        if(checkCoordinates(start_x, start_y))
        {
           //update the predicted values and last time.
           samples.push_back(p);
        }
        else
            samples[i] = {};
    }
    prediction(ls);

}
void Localization::prediction(Laserscanner *sh)
{
    boost::random::normal_distribution<double> error_vel(0, 0.2);
    boost::random::normal_distribution<double> error_angle(0, 0.2);
    bool updated = true;//sh->hasUpdated(updated);

    cout << "i am here before the for loop in init" << endl;
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
                double new_beta = current_beta + (sh->angle_vel + error_angle(gen))*sample_time;
                double new_x = current_x + (sh->vel + error_vel(gen))*sample_time*cos(current_beta);
                double new_y = current_y + (sh->vel + error_vel(gen))*sample_time*sin(current_beta);

                // check if the particles generated coordinates are inside the map
                if(checkCoordinates(new_x, new_y))
                {
                   //update the predicted values and last time.
                   samples[i].updateState(new_x, new_y, new_beta, current_time);
                   cout << samples[i].x << " " << samples[i].getY() << " " << samples[i].beta << endl;
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
                double new_beta = current_beta + (sh->angle_vel + error_angle(gen))*sample_time;
                double new_x = current_x + (sh->vel + error_vel(gen))*sample_time*cos(current_beta);
                double new_y = current_y + (sh->vel + error_vel(gen))*sample_time*sin(current_beta);

                // check if the particles generated coordinates are inside the map
                if(checkCoordinates(new_x, new_y))
                {
                   //update the predicted values and last time.
                   samples[i].updateState(new_x, new_y, new_beta, current_time);
                }
            }
        }
        updated = false;
    }
    else
        prediction(sh);
   // updatePos();
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
    Mat map = imread("/home/annie/git_repo/rb-rca5-cowsay/models/bigworld/meshes/floor_plan.png", CV_LOAD_IMAGE_GRAYSCALE);
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

