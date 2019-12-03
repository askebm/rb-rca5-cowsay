#ifndef SIMPLEROBOTCONTROL_H
#define SIMPLEROBOTCONTROL_H

#include <fl/rule/RuleBlock.h>
#include <gazebo/gazebo_client.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>

#include <initializer_list>
#include <string>
#include <mutex>
#include <atomic>
#include <thread>
#include <cmath>


#include <fl/Headers.h>

namespace rba{

class SimpleRobotControl
{
protected:
	fl::Engine* fuzzyEngine;
	void process(const int msec);
	void initProcess(const int msec=10, const bool& thread=true);
	virtual void publish(const double& speed, const double& steer)=0;
	std::thread processThread;
	std::atomic_bool isRunning{true};
	void initEngine();

	struct point {double x,y;};
	struct position {
		struct point p;
		double yaw;
		std::mutex mutex;
	};
	struct position currentPosition={{0,0},0};
	struct point goal={0,0};


	void setGoal(double x,double y, double z=1);

	struct lidarData {
		float angle_min, angle_max, angle_increment, range_min, range_max;
		int sec, nsec, nranges = 0, nintensities = 0;
		float* ranges = nullptr;
	} lidarData;
	std::mutex lidarDataMutex;
	static struct robotSpecs {
		static constexpr float width = 0.2;
		static constexpr float width_half = 0.1;
		static constexpr float maxLidarRange = 2;
	} robotSpecs;
	struct fuzzyVariablesStruct {
		fl::InputVariable* obstacle;
		fl::InputVariable* distance;
		fl::OutputVariable* steer;
	 	fl::OutputVariable* speed;
		fl::InputVariable* errorYaw;
		fl::RuleBlock* obstacleAvoidance;
		fl::RuleBlock* marbleCatcher;
	} fuzzyVariables;
	void cleanUp();
public:
	void setEngineFromFile(const std::string file);
	SimpleRobotControl();
	~SimpleRobotControl();
};

}
#endif /* SIMPLEROBOTCONTROL_H */
