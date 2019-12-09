#include <fl/Headers.h>
#include <Robot/fuzzyEngine.hpp>
#include <iostream>



int main(int argc, char *argv[])
{
	auto engine = rba::CreateEngine();

	auto obstacle = engine->getInputVariable("obstacle");
	auto distance = engine->getInputVariable("distance");
	auto steer = engine->getOutputVariable("mSteer");
	auto speed = engine->getOutputVariable("velocity");
	auto errorYaw = engine->getInputVariable("erroryaw");
	auto obstacleAvoidance = engine->getRuleBlock("ObstacleAvoidance");
	auto marbleCatcher = engine->getRuleBlock("MarbleCatcher");

	marbleCatcher->setEnabled(true);
	obstacleAvoidance->setEnabled(false);

//std::cout << "Obstacle\tDistance\tSpeed\tSteer"  << std::endl;
	std::cout << "erroryaw\tspeed\tsteer" << std::endl;
	for (double error = -3.15; error < 3.15; error += 0.01) {
			errorYaw->setValue(error);
			engine->process();
			double s = speed->getValue();
			double m = steer->getValue();
			std::cout << 
				error << "\t" << 
				s << "\t" <<
				m << std::endl;
	}
	return 0;
}
