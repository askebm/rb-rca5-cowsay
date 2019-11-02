#include "SimpleRobotControl.hpp"
#include "fuzzyEngine.cpp"

using namespace rba;

SimpleRobotControl::SimpleRobotControl() {
}

SimpleRobotControl::~SimpleRobotControl() {
	isRunning.store(false);
}

void SimpleRobotControl::setEngineFromFile(std::string file) {
	fuzzyEngine = fl::FllImporter().fromFile(file);
	std::string status;
	if( !fuzzyEngine->isReady(&status))
		throw fl::Exception("[engine error] engine is not ready:n" + status, FL_AT);
	return void();
	fuzzyVariables.obstacle = fuzzyEngine->getInputVariable("obstacle");
	fuzzyVariables.distance = fuzzyEngine->getInputVariable("distance");
	fuzzyVariables.steer = fuzzyEngine->getOutputVariable("mSteer");
	fuzzyVariables.speed = fuzzyEngine->getOutputVariable("velocity");
}




/*! \brief Main Process
 *
 *  Main process for the simple robot controller.
 *
 *
 * \param msec Wait time between each loop
 * \param thread should process run a seperat thread
 */
void SimpleRobotControl::process(const int msec) {

	while( isRunning.load() ){

    gazebo::common::Time::MSleep(msec);
		lidarDataMutex.lock();
		auto localLidarData = lidarData;
		lidarDataMutex.unlock();
		float cls_range = robotSpecs.maxLidarRange;
		double total_mass = 0;
		double weighted_x = 0;

		for (int i = 0; i < localLidarData.nranges; i++) {
			float angle = localLidarData.angle_min + i * localLidarData.angle_increment;
			float range = std::min(localLidarData.ranges[0][i], localLidarData.range_max);
			float y = range * std::cos(angle);
			float x = range * std::sin(angle);
			if( (-robotSpecs.width_half <= x) && (x <= robotSpecs.width_half))
			{
				if (y<cls_range) {
					cls_range = y;
				}
				auto y_i = robotSpecs.maxLidarRange - y;
				total_mass +=y_i;
				weighted_x +=y_i*x;
			}
		}
		
		// Normalise angle and range between 0 and 1
		double obstacle =  ((total_mass==0)?0.5:0.5-(4*weighted_x/total_mass));
		double distance = cls_range/robotSpecs.maxLidarRange;

		fuzzyVariables.distance->setValue(distance);
		fuzzyVariables.obstacle->setValue(obstacle);
		fuzzyEngine->process();

		double speed = fuzzyVariables.speed->getValue();
		double steer = 10*( fuzzyVariables.steer->getValue() - 0.5 );

		this->publish(speed,steer);
	}
}

/*! \brief Start main process
 *
 *  Start main process, will by default start in another thread
 *
 * \param thread bool to choose wether or not process should be threaded
 */
void SimpleRobotControl::initProcess(const int msec, const bool& thread) {
	processThread = std::thread(&SimpleRobotControl::process, this, msec);
}


/*! \brief initialise fuzze engine
 *
 *  initialises fuzzy engine from auto generated file
 *
 * \return void()
 */
void SimpleRobotControl::initEngine() {
	fuzzyEngine = CreateEngine();
	fuzzyVariables.obstacle = fuzzyEngine->getInputVariable("obstacle");
	fuzzyVariables.distance = fuzzyEngine->getInputVariable("distance");
	fuzzyVariables.steer = fuzzyEngine->getOutputVariable("mSteer");
	fuzzyVariables.speed = fuzzyEngine->getOutputVariable("velocity");
}
