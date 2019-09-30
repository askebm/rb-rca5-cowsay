#include <iostream>
#include <fstream>
#include <vector>

#include "fl/Headers.h"

#include "fuzzy.h"


// Input order in vector ////////////////////////////////
/*
 + angle : The shortest sensor lines' angle. It is used for determining which way to turn.
 + distance : The distance, is the length of the line mentioned above. Here the length of this line determines how "hard" to turn ... short line = hard turn, long line = slow turn.
*/
std::vector<float> fuzzy(std::vector<float> inputs)
{

    fl::Engine* engine = fl::FllImporter().fromFile("ObstacleAvoidance.fll");

    std::string status;

    if (not engine->isReady(&status))
        throw fl::Exception("[engine error] engine is not ready:\n" + status, FL_AT);

    // Inputs for fuzzy
    fl::InputVariable* obstacle = engine->getInputVariable("obstacle");
    fl::InputVariable* distance = engine->getInputVariable("distance");
    fl::InputVariable* angle = engine->getInputVariable("angle");

    // Outputs for fuzzy
    fl::OutputVariable* steer = engine->getOutputVariable("mSteer");

    // Give inputs the wanted values.
    fl::scalar turn = inputs[0];
    angle->setValue(turn);
    fl::scalar dist = inputs[0];
    distance->setValue(dist);
    // Process inputs in engine
    engine->process();

/*    for (int i = 0; i <= 50; ++i)
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

    }*/
}