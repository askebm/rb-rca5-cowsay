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
Localization::Localization(int sample_size, Laserscanner ls)
{
    N = sample_size;
    init(ls);
}
Localization::~Localization(){}

void Localization::init(Laserscanner ls)
{
    uniform_real_distribution<double> vel_variance(-2.5, 2.5);
    uniform_real_distribution<double> angle_variance(-2.89, 2.89);
    first_flag = true;

    particle p;
    for(int i = 0; i < N; i++)
    {
        r_x = ls.robot_x;
        r_y = ls.robot_y;
        //gives all starting particles a random starting point in a 5x5 cube
        double start_x = ls.robot_x + vel_variance(gen);
        double start_y = ls.robot_y + vel_variance(gen);
        double start_beta = ls.robot_angle + angle_variance(gen);
        current_time = time(&timer);
        p.last_time = current_time;
        p.x = start_x;
        p.y = start_y;
        p.beta = start_beta;
        p.weight = 1/200.0;
        p.likelihood = 0.0;
        if(checkCoordinates(start_x, start_y))
        {
            //update the predicted values and last time.
            samples.push_back(p);
        }
        else
            samples[i] = {};
    }
    //cout << "I made to through init" << endl;
    updatePos(ls);
}
void Localization::prediction(Laserscanner sh)
{
    boost::random::normal_distribution<double> error_vel(0, 0.2);
    boost::random::normal_distribution<double> error_angle(0, 0.2);
    //updated = sh.hasUpdated(updated);

    //cout << "I made it to prediction" << endl;
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
                double new_beta = current_beta + (sh.angle_vel + error_angle(gen))*sample_time;
                double new_x = current_x + (sh.vel + error_vel(gen))*sample_time*cos(current_beta);
                double new_y = current_y + (sh.vel + error_vel(gen))*sample_time*sin(current_beta);

                // check if the particles generated coordinates are inside the map
                if(checkCoordinates(new_x, new_y))
                {
                   //update the predicted values and last time.
                   samples[i].updateState(new_x, new_y, new_beta, current_time);
                  // cout << samples[i].x << " " << samples[i].getY() << " " << samples[i].beta << endl;
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
                double new_beta = current_beta + (sh.angle_vel + error_angle(gen))*sample_time;
                double new_x = current_x + (sh.vel + error_vel(gen))*sample_time*cos(current_beta);
                double new_y = current_y + (sh.vel + error_vel(gen))*sample_time*sin(current_beta);

                // check if the particles generated coordinates are inside the map
                if(checkCoordinates(new_x, new_y))
                {
                   //update the predicted values and last time.
                   samples[i].updateState(new_x, new_y, new_beta, current_time);
                  // cout << samples[i].x << " " << samples[i].getY() << " " << samples[i].beta << endl;
                }
            }
        }
    }
    else
        prediction(sh);

    updatePos(sh);
}
void Localization::updatePos(Laserscanner lsa)
{
    if(updated)
    {
        double sigma = 2.0;
        r_x = lsa.robot_x;
        r_y = lsa.robot_y;
        //get the rays for robot;
        vector<rays> robot_rays;
        rays ray;
        for(int i = 0; i < N; i++)
        {
            ray.distance = lsa.range[i];
            ray.dir = lsa.angle[i];
            robot_rays.push_back(ray);

        }

        vector<rays> temp;
        //get rays for samples
        for(int i = 0; i < N; i++)
        {
            temp = lsa.rayCasting(samples[i].getX(),samples[i].getY(), samples[i].getBeta());
            samples[i].ray = temp;
        }
        //calculate the likelihood for a particle is around the robot for all particles.

        double norm_const = 0.0;
        vector<double> temp_weight;
        const double extra = 1/(sigma*sqrt(2*pi));
        double sum = 0;
        for(int i = 0; i < N; i++) // samples size
        {
            double likelihood = 1.0;
            for(int j = 0; j < N; j++) //nranges size - FIX
            {
                likelihood = likelihood * extra *exp(-(pow(samples[i].ray[j].distance-robot_rays[i].distance,2))/(2*pow(sigma,2)));
            }
            //save the likelihood for the particle
            samples[i].likelihood = likelihood;
            //cal new weight based on the likelihood of the particles rays.
            temp_weight.push_back(samples[i].weight * samples[i].likelihood);


            // get the sum of all weights for normalizing.
            norm_const = norm_const + temp_weight[i];
        }
        //normalize new weight of particles
        for(int i = 0; i < N; i++)
        {
            samples[i].weight = temp_weight[i]/norm_const;
            sum= sum + samples[i].weight;
          //  cout << samples[i].weight << endl;

        }
        //cout << sum << endl;
        resampling();
    }
    else
        updatePos(lsa);
}
void Localization::resampling()
{
    vector<particle> new_m;
    double rand = 1.0/double(N);
    double c = samples[0].weight;
    uniform_real_distribution<double> delta(0, rand);
    double delta_i = delta(gen);
    int i = 0;
    for(int j = 0; j < N; j++)
    {
        double u = delta_i + j*rand;
        while (u > c)
        {
            i++;
            c = c + samples[i].weight;
        }
        particle new_particle = samples[i];
        new_particle.weight = 1.0/N;
        new_m.push_back(new_particle);
    }
    samples = new_m;
    new_m.clear();

    updateMap();
}

void Localization::updateMap()
{
    Mat new_draw = map.clone();

    Vec3b colour = new_draw.at<Vec3b>(100,100);
    colour[0] = 255;
    colour[1] = 0;
    colour[2] = 0;

    for(int i  = 0; i < N; i++)
    {
        int x = (samples[i].x + 42)/0.07;
        int y = ((-1)* samples[i].y + 20)/0.07;

        circle(new_draw, Point((int)x,(int)y), 3, Vec3b(255,0,0), 1);
    }
    double x = (r_x + 42)/0.07;
    double y = ((-1) * r_y + 28)/0.07;

    circle(new_draw, Point((int)x,(int)y), 3, Vec3b(0,0,255), 1);


    mutex_k.lock();
    cv::namedWindow("new_draw");
    cv::imshow("new_draw", new_draw);
    cv::imwrite("/home/annie/newMap.png", new_draw );
    mutex_k.unlock();

}
//needs fixing - as the middle of the picture is starting point 0,0 for robot.
bool Localization::checkCoordinates(double x, double y)
{
//    Mat map = imread("/home/annie/git_repo/rb-rca5-cowsay/models/bigworld/meshes/floor_plan.png", CV_LOAD_IMAGE_GRAYSCALE);
//    double rows = map.rows;
//    double cols = map.cols;

//    if(x > rows -1 || x < 0)
//    {
//        return false;
//    }
//    if(y > cols - 1 || y < 0)
//    {
//        return false;
//    }
//    else
        return true;

}

