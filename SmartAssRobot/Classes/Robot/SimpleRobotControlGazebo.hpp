#ifndef SIMPLEROBOTCONTROLGAZEBO_H
#define SIMPLEROBOTCONTROLGAZEBO_H
#include "SimpleRobotControl.hpp"
#include "gazebo/transport/TransportTypes.hh"
#include "vector3d.pb.h"

namespace rba {
 
class SimpleRobotControlGazebo : private SimpleRobotControl
{
private:
	void publish(const double& speed, const double& steer){
    ignition::math::Pose3d pose(speed, 0, 0, 0, 0, steer);

    gazebo::msgs::Pose msg;
    gazebo::msgs::Set(&msg, pose);
    movementPublisher->Publish(msg);

	}
	void callback(ConstLaserScanStampedPtr& msg){
		lidarDataMutex.lock();
		//  std::cout << ">> " << msg->DebugString() << std::endl;
		lidarData.angle_min = float(msg->scan().angle_min());
		lidarData.angle_max = float(msg->scan().angle_max());
		//  double angle_max = msg->scan().angle_max();
		lidarData.angle_increment = float(msg->scan().angle_step());

		lidarData.range_min = float(msg->scan().range_min());
		lidarData.range_max = float(msg->scan().range_max());

		lidarData.sec = msg->time().sec();
		lidarData.nsec = msg->time().nsec();

		lidarData.nranges = msg->scan().ranges_size();
		lidarData.nintensities = msg->scan().intensities_size();
		if (lidarData.ranges == nullptr) {
			lidarData.ranges = new float[lidarData.nranges];
		}
		for (int i = 0; i < lidarData.nranges; ++i) {
			lidarData.ranges[i] = float(msg->scan().ranges(i));
		}

		lidarDataMutex.unlock();
	}
	void goalCallback(const boost::shared_ptr<const gazebo::msgs::Vector3d>& msg) {
		this->setGoal(msg->x(),msg->y(),msg->z());
	}
	gazebo::transport::PublisherPtr movementPublisher;
	gazebo::transport::SubscriberPtr lidarSubscriber,goalSubscriber;
	gazebo::transport::NodePtr node;

public:
	SimpleRobotControlGazebo(const gazebo::transport::NodePtr _node,
			const std::string& pioneerTopic="~/pioneer2dx/vel_cmd", 
			const std::string& lidarTopic="~/pioneer2dx/hokuyo/link/laser/scan",
			const std::string& nextGoalTopic="~/pioneer2dx/SimpleRobotControl/goal")
		: node(_node)
	{
		initEngine();
		movementPublisher = node->Advertise<gazebo::msgs::Pose>(pioneerTopic);
		lidarSubscriber = node->Subscribe(lidarTopic,&SimpleRobotControlGazebo::callback,this);
		goalSubscriber = node->Subscribe(nextGoalTopic, &SimpleRobotControlGazebo::goalCallback,this);
		initProcess();
	}

};

}
 
 #endif /* SIMPLEROBOTCONTROLGAZEBO_H */
