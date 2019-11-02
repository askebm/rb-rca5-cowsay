#ifndef SIMPLEROBOTCONTROL_H
#define SIMPLEROBOTCONTROL_H

#include <gazebo/gazebo_client.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>

#include <string>
#include <mutex>
#include <atomic>
#include <thread>


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

	struct lidarData {
		float angle_min, angle_max, angle_increment, range_min, range_max;
		int sec, nsec, nranges, nintensities;
		float** ranges;
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
	} fuzzyVariables;
	void cleanUp();
public:
	void setEngineFromFile(const std::string file);
	SimpleRobotControl();
	~SimpleRobotControl();
};

}

#endif /* SIMPLEROBOTCONTROL_H */
