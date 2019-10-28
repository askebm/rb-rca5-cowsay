#include <gazebo/gazebo_client.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <fl/Engine.h>
#include <fl/imex/FllImporter.h>

float shortest_distance;
float angle_to_shortest;

#include "Vision.h"
#include "fuzzy.h"
#include "debug.h"

static boost::mutex mutex;

std::vector<std::pair<float,float>> laserLines;

void statCallback(ConstWorldStatisticsPtr &_msg)
{
  (void)_msg;
  // Dump the message contents to stdout.
  //  std::cout << _msg->DebugString();
  //  std::cout << std::flush;
}

void poseCallback(ConstPosesStampedPtr &_msg)
{
  // Dump the message contents to stdout.
  //  std::cout << _msg->DebugString();

  for (int i = 0; i < _msg->pose_size(); i++) {
    if (_msg->pose(i).name() == "pioneer2dx") {

      /*std::cout << std::setprecision(2) << std::fixed << std::setw(6)
                << _msg->pose(i).position().x() << std::setw(6)
                << _msg->pose(i).position().y() << std::setw(6)
                << _msg->pose(i).position().z() << std::setw(6)
                << _msg->pose(i).orientation().w() << std::setw(6)
                << _msg->pose(i).orientation().x() << std::setw(6)
                << _msg->pose(i).orientation().y() << std::setw(6)
                << _msg->pose(i).orientation().z() << std::endl;*/
    }
  }
}

void cameraCallback(ConstImageStampedPtr &msg)
{

  std::size_t width = msg->image().width();
  std::size_t height = msg->image().height();
  const char *data = msg->image().data().c_str();
  cv::Mat im(int(height), int(width), CV_8UC3, const_cast<char *>(data));

  im = im.clone();
  cv::cvtColor(im, im, cv::COLOR_RGB2BGR);

  /// Find the goddamn marbles mate!
  std::cout << "1" << std::endl;
  Vision counturFinder;
    std::cout << "1" << std::endl;

  cv::namedWindow("test");
    std::cout << "1" << std::endl;

    cv::imshow("test",counturFinder.findContour( im ));
    std::cout << "1" << std::endl;


/*    if( testRedness( cv::Point(0,0) , findRedPixels(im) ) )
    {
        cv::namedWindow("test");
        cv::Mat hls_im;
        cv::cvtColor(im,hls_im,CV_BGR2HLS);
        cv::imshow("test",hls_im);
        cv::waitKey(0);
    }*/

    std::cout << "YO" << std::endl;

  mutex.lock();
  cv::imshow("camera", im);
  mutex.unlock();
}

void lidarCallback(ConstLaserScanStampedPtr &msg)
{

    //  std::cout << ">> " << msg->DebugString() << std::endl;
    float angle_min                 = float(msg->scan().angle_min());
    double angle_max                = msg->scan().angle_max();
    float angle_increment           = float(msg->scan().angle_step());

    float range_min                 = float(msg->scan().range_min());
    float range_max                 = float(msg->scan().range_max());

    int sec                         = msg->time().sec();
    int nsec                        = msg->time().nsec();

    int nranges                     = msg->scan().ranges_size();
    int nintensities                = msg->scan().intensities_size();

    assert(nranges == nintensities);

    int width                       = 400;
    int height                      = 400;
    float px_per_m                  = 200 / range_max;

    // Declare the angle/distance pair
    std::pair<float, float> angleDistPair;

    // Constant for when to react to surface
    int detectLimit                 = 1;

  cv::Mat im(height, width, CV_8UC3);
  im.setTo(0);

    laserLines.clear();

    for (int i = 0; i < nranges; i++)
  {
    float angle                     = angle_min + i * angle_increment;
    float range                     = std::min(float(msg->scan().ranges(i)), range_max);

    //    double intensity = msg->scan().intensities(i);
    cv::Point2f startpt(200.5f + range_min * px_per_m * std::cos(angle),
                        200.5f - range_min * px_per_m * std::sin(angle));
    cv::Point2f endpt(200.5f + range * px_per_m * std::cos(angle),
                      200.5f - range * px_per_m * std::sin(angle));

    simple_debugprint(" | range variable = ",range," | ","angle = ",angle," | ");

    // The line we use to detect collisions
      cv::line(im, startpt * 16, endpt * 16, cv::Scalar(255, 255, 255, 255), 1,
               cv::LINE_AA, 4);

      // Test for angles from 0 to pi/2 and range < limitDetect
      if(  (angle < angle_max/2 && angle > angle_min/2) && (range < detectLimit)  )
      {

          // Fill the pair with the data we want.
          angleDistPair.first = angle;
          angleDistPair.second = range;

          // Push the pair to the vector of pairs.
          mutex.lock();
          laserLines.push_back(angleDistPair);
          mutex.unlock();

          // draw Region of Interest lines.
          cv::line(im, startpt * 16, endpt * 16, cv::viz::Color::red(), 1,
                   cv::LINE_AA, 4);
      }
  }



     std::pair<float,float> scan_of_interest = find_shortest_laser(laserLines);
    // The laserLines isn't empty and the shortest distance and the angle is assigned to their respektable variables.
    if( wallFound( scan_of_interest ) )
    {
        // Determine which way to drive with fuzzy logic
        mutex.lock();
        shortest_distance = scan_of_interest.second;
        angle_to_shortest = scan_of_interest.first;
        mutex.unlock();
    }
    else
    {
        simple_debugprint("[WALL DETECT]: DRIVE FORWARD!");
        // In this case drive forward.
        shortest_distance = -100;
        angle_to_shortest = -100;
    }

    cv::circle(im, cv::Point(200, 200), 2, cv::Scalar(0, 0, 255));
    cv::putText(im, std::to_string(sec) + ":" + std::to_string(nsec),
              cv::Point(10, 20), cv::FONT_HERSHEY_PLAIN, 1.0,
              cv::Scalar(255, 0, 0));

  mutex.lock();
  cv::imshow("lidar", im);
  mutex.unlock();
}

int main(int _argc, char **_argv)
{
    // Load gazebo
    gazebo::client::setup(_argc, _argv);

    // Create our node for communication
    gazebo::transport::NodePtr node(new gazebo::transport::Node());
    node->Init();

    // Listen to Gazebo topics
    gazebo::transport::SubscriberPtr statSubscriber     = node->Subscribe("~/world_stats", statCallback);

    gazebo::transport::SubscriberPtr poseSubscriber     = node->Subscribe("~/pose/info", poseCallback);

    gazebo::transport::SubscriberPtr cameraSubscriber   = node->Subscribe("~/pioneer2dx/camera/link/camera/image", cameraCallback);

    gazebo::transport::SubscriberPtr lidarSubscriber    = node->Subscribe("~/pioneer2dx/hokuyo/link/laser/scan", lidarCallback);

    // Publish to the robot vel_cmd topic
    gazebo::transport::PublisherPtr movementPublisher   = node->Advertise<gazebo::msgs::Pose>("~/pioneer2dx/vel_cmd");

    // Publish a reset of the world
    gazebo::transport::PublisherPtr worldPublisher      = node->Advertise<gazebo::msgs::WorldControl>("~/world_control");

    gazebo::msgs::WorldControl controlMessage;
    controlMessage.mutable_reset()->set_all(true);
    worldPublisher->WaitForConnection();
    worldPublisher->Publish(controlMessage);

    const int key_esc     = 27;

    float speed           = 0.0;
    float dir             = 0.0;

    int key;

    // Loop
    while (true)
    {
        // Sleep
        gazebo::common::Time::MSleep(10);

        mutex.lock();
        key                 = cv::waitKey(1);
        mutex.unlock();

        // Use Fuzzy Logic to decide a direction and speed for the robot.
        mutex.lock();
        std::pair<float,float> fuzzy_directions     = rc_fuzzy(std::pair<float,float>{angle_to_shortest,shortest_distance});
        dir                                         = fuzzy_directions.first;
        speed                                       = fuzzy_directions.second;
        mutex.unlock();

        simple_debugprint("dir = ",dir," | ","speed = ",speed," | ");

        if (key == key_esc)
          break;

        // Generate a pose
        ignition::math::Pose3d pose(double(speed), 0, 0, 0, 0, double(dir));

        // Convert to a pose message
        gazebo::msgs::Pose msg;
        gazebo::msgs::Set(&msg, pose);
        movementPublisher->Publish(msg);
    }

    // Make sure to shut everything down.
    gazebo::client::shutdown();
}
