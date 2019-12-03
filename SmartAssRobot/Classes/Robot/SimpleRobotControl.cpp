#include "SimpleRobotControl.hpp"
#include "fuzzyEngine.hpp"
#include <initializer_list>

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
		auto localLidarData = this->lidarData;
		lidarDataMutex.unlock();
		float cls_range = robotSpecs.maxLidarRange;
		double total_mass = 0;
		double weighted_x = 0;

		// Get yaw error
		auto deltaX = goal.x - currentPosition.p.x;
		auto deltaY = goal.y - currentPosition.p.y;
		auto wantedYaw = std::atan2(deltaY,deltaX);
		auto errorYaw = wantedYaw - currentPosition.yaw;

		// Distance To goal
		auto distGoal = std::sqrt(deltaX*deltaX + deltaY*deltaY);

		// Precalc sin & cos
		/*
		auto COS = std::cos(-errorYaw);
		auto SIN = std::sin(-errorYaw);
		*/

		bool enableMarbleCatcher = true;

		for (int i = 0; i < localLidarData.nranges; i++) {
			float range = std::min(localLidarData.ranges[i], localLidarData.range_max);
			if( range < 2.5 ) {
				float angle = localLidarData.angle_min + i * localLidarData.angle_increment;
				float y = range * std::cos(angle);
				float x = range * std::sin(angle);
				// Imidiate avaoidance
				if( (-robotSpecs.width_half <= x) && (x <= robotSpecs.width_half) && (0 <= y) ) {
					if (y<cls_range) {
						cls_range = y;
					}
					auto y_i = robotSpecs.maxLidarRange - y;
					total_mass +=y_i;
					weighted_x +=y_i*x;
				}
				// Implied avoidance
				auto xAbs = std::abs(x);
				if ( (xAbs < 0.4) && (robotSpecs.maxLidarRange < xAbs) && (y< 1) && (0 <= y) ) {
					enableMarbleCatcher = (enableMarbleCatcher && false);
				}

				/*
				auto X = x*COS - y*SIN;
				auto Y = x*SIN + y*COS;
				if ( (-robotSpecs.width_half < X ) && ( x <= robotSpecs.width_half ) && ( 0 <= Y) ) {
					fuzzyVariables.obstacleAvoidance->setEnabled(true);
					fuzzyVariables.marbleCatcher->setEnabled(false);
				} else {
					fuzzyVariables.obstacleAvoidance->setEnabled(false);
					fuzzyVariables.marbleCatcher->setEnabled(true);
				}
				*/
			}
		}
		
		// Normalise angle and range between 0 and 1
		double obstacle =  ((total_mass==0)?0.5:0.5-(4*weighted_x/total_mass));
		double distance = cls_range/robotSpecs.maxLidarRange;

		bool enableObsAvoid = (distGoal < 1);
		fuzzyVariables.obstacleAvoidance->setEnabled( enableObsAvoid);
		fuzzyVariables.marbleCatcher->setEnabled( ( !enableObsAvoid || enableMarbleCatcher) );

		fuzzyVariables.errorYaw->setValue(errorYaw);
		fuzzyVariables.obstacle->setValue(obstacle);
		fuzzyVariables.distance->setValue(distance);
		fuzzyEngine->process();

		double speed = (distGoal < 0.2)?0:(fuzzyVariables.speed->getValue());
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
	fuzzyVariables.errorYaw = fuzzyEngine->getInputVariable("erroryaw");
	fuzzyVariables.obstacleAvoidance = fuzzyEngine->getRuleBlock("ObstacleAvoidance");
	fuzzyVariables.marbleCatcher = fuzzyEngine->getRuleBlock("MarbleCatcher");
}


/*! \brief Set next goal for the fuzzy controller
 *
 *
 * \param x x-coordinate
 * \param y y-coordinate
 * \param z Missused to tell iof coordinat is relative or absolut
 * 1 absolut
 * 0 relative
 * \return Return parameter description
 */
void SimpleRobotControl::setGoal(double x,double y, double z) {
	if ( z == 1 ) {
		goal.x=x;
		goal.y=y;
	} else if ( z == 0 ) {
		goal.x=currentPosition.p.x+x;
		goal.y=currentPosition.p.y+y;
	} else {
		throw std::runtime_error("z is neither 1 or 0");
	}
}
