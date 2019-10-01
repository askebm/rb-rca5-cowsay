#include <gazebo/gazebo_client.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>

#include <opencv2/opencv.hpp>

#include <fl/Headers.h>

#include <iostream>

#define MAX_LIDAR_RANGE 2
#define ROBOT_WIDTH 0.15

static boost::mutex mutex;

static boost::mutex _lidar_mutex;
static double obstacle;
static double distance;

void lidarCallbackTransform (ConstLaserScanStampedPtr &msg)
{
  //  std::cout << ">> " << msg->DebugString() << std::endl;
  float angle_min = float(msg->scan().angle_min());
  float angle_max = float(msg->scan().angle_max());
  //  double angle_max = msg->scan().angle_max();
  float angle_increment = float(msg->scan().angle_step());

  float range_min = float(msg->scan().range_min());
  float range_max = float(msg->scan().range_max());

  int sec = msg->time().sec();
  int nsec = msg->time().nsec();

  int nranges = msg->scan().ranges_size();
  int nintensities = msg->scan().intensities_size();

	float cls_range = MAX_LIDAR_RANGE;
	double total_mass = 0;
	double weighted_x = 0;

  for (int i = 0; i < nranges; i++) {
    float angle = angle_min + i * angle_increment;
    float range = std::min(float(msg->scan().ranges(i)), range_max);
		float y = std::abs(range * std::cos(angle));
		float x = range * std::sin(angle);
		if( (-ROBOT_WIDTH <= x) && (x <= ROBOT_WIDTH))
		{
			if (y<cls_range) {
				cls_range = y;
			}
			auto y_i = MAX_LIDAR_RANGE - y;
			total_mass +=y_i;
			weighted_x +=y_i*x;
		}
	}
	
	// Normalise angle and range between 0 and 1
	_lidar_mutex.lock();
	obstacle =  ((total_mass==0)?0.5:0.5-(4*weighted_x/total_mass));
	distance = cls_range/MAX_LIDAR_RANGE;
	_lidar_mutex.unlock();

}

int main(int _argc, char **_argv) {
  // Load gazebo
  gazebo::client::setup(_argc, _argv);

  // Create our node for communication
  gazebo::transport::NodePtr node(new gazebo::transport::Node());
  node->Init();

  // Listen to Gazebo topics
  gazebo::transport::SubscriberPtr lidarSubscriber =
      node->Subscribe("~/pioneer2dx/hokuyo/link/laser/scan", lidarCallbackTransform);

  // Publish to the robot vel_cmd topic
  gazebo::transport::PublisherPtr movementPublisher =
      node->Advertise<gazebo::msgs::Pose>("~/pioneer2dx/vel_cmd");

  // Publish a reset of the world
  gazebo::transport::PublisherPtr worldPublisher =
      node->Advertise<gazebo::msgs::WorldControl>("~/world_control");
  gazebo::msgs::WorldControl controlMessage;
  controlMessage.mutable_reset()->set_all(true);
  worldPublisher->WaitForConnection();
  worldPublisher->Publish(controlMessage);

	// Setup fuzzy controller
	fl::Engine* fl_engine = fl::FllImporter().fromFile("ObstacleAvoidance.fll");
	std::string status;
	if (not fl_engine->isReady(&status))
			throw fl::Exception("[engine error] engine is not ready:n" + status, FL_AT);
	fl::InputVariable* fl_obstacle = fl_engine->getInputVariable("obstacle");
	fl::InputVariable* fl_distance = fl_engine->getInputVariable("distance");
	fl::OutputVariable* fl_steer = fl_engine->getOutputVariable("mSteer");
	fl::OutputVariable* fl_speed = fl_engine->getOutputVariable("velocity");

  float speed = 0.5;
  double dir = 0.0;

  // Loop
  while (true) {
    gazebo::common::Time::MSleep(10);

		_lidar_mutex.lock();
		std::cout << "Distance is: " << distance << std::endl;
		fl_distance->setValue(distance);
		std::cout << "Obstacle is: " << obstacle << std::endl;
		fl_obstacle->setValue(obstacle);
		_lidar_mutex.unlock();

		fl_engine->process();

		speed = fl_speed->getValue();
		std::cout << "Speed is: " << speed << std::endl;
		dir = (double(fl_steer->getValue())-0.5)*10;
		std::cout << "Dir is: " << dir << std::endl;
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
