#include <gazebo/gazebo_client.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>

#include <opencv2/opencv.hpp>

#include <fl/Headers.h>

#include <iostream>

static boost::mutex mutex;

static boost::mutex _lidar_mutex;
static double obstacle;
static double distance;
static float* _range_max;
static void (*lidarCallback)(ConstLaserScanStampedPtr&);

void lidarCallbackTransform (ConstLaserScanStampedPtr &msg)
{
  //  std::cout << ">> " << msg->DebugString() << std::endl;
  float angle_min = float(msg->scan().angle_min());
  float angle_max = float(msg->scan().angle_max());
  //  double angle_max = msg->scan().angle_max();
  float angle_increment = float(msg->scan().angle_step());

  float range_min = float(msg->scan().range_min());
  //float range_max = float(msg->scan().range_max());
	float range_max = 2.0;

  int sec = msg->time().sec();
  int nsec = msg->time().nsec();

  int nranges = msg->scan().ranges_size();
  int nintensities = msg->scan().intensities_size();

	float cls_angle;
	float cls_range = range_max;

  for (int i = 0; i < nranges; i++) {
    float angle = angle_min + i * angle_increment;
    float range = std::min(float(msg->scan().ranges(i)), range_max);
		if (range < cls_range) {
			cls_range = range;
			cls_angle = angle;
		}
	}
	
	// Normalise angle and range between 0 and 1
	_lidar_mutex.lock();
	obstacle =  1 - (double)(cls_angle - angle_min)/double(angle_max + cv::abs(angle_min));
	distance = double(cls_range/range_max);
	_lidar_mutex.unlock();

}

void lidarCallbackInit(ConstLaserScanStampedPtr &msg)
{
	auto nranges = (msg->scan().ranges_size());
	auto angle_min = (msg->scan().angle_min());
	auto angle_step = (msg->scan().angle_step());

	_range_max = new float[nranges];

	for (int i = 0; i < nranges; ++i) {
		auto angle = angle_min + i * angle_step;
		_range_max[i] = std::min(0.15 / std::cos(angle), double(2));
	}
	lidarCallback = lidarCallbackTransform;
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
