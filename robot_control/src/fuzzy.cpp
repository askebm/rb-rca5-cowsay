#include <iostream>
#include <fstream>

#include "fl/Headers.h"

#include "fuzzy.h"

int fuzzy(int argc, char* argv[])
{

    fl::Engine* engine = fl::FllImporter().fromFile("ObstacleAvoidance.fll");

    std::string status;

    if (not engine->isReady(&status))
        throw fl::Exception("[engine error] engine is not ready:\n" + status, FL_AT);

    fl::InputVariable* obstacle = engine->getInputVariable("obstacle");

    fl::OutputVariable* steer = engine->getOutputVariable("mSteer");

    fl::InputVariable* distance = engine->getInputVariable("distance");

    for (int i = 0; i <= 50; ++i)
    {
        fl::scalar location = obstacle->getMinimum() + i * (obstacle->range() / 50);
        obstacle->setValue(location);
        engine->process();


        for(int j = 0; j <= 50; j++)
        {
            fl::scalar directionInt = distance->getMinimum() + j* ( distance->range() / 50 );
            distance->setValue(directionInt);
            engine->process();
            FL_LOG(" | obstacle.input 1 = " << fl::Op::str(location) << " | obstacle.input 2 = " << fl::Op::str(distance->getValue()) << " => " << "steer.output = " << fl::Op::str(steer->getValue()));


        }

    }
}