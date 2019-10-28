#include <iostream>
#include <fstream>
#include <vector>
#include <opencv2/core/types.hpp>

#include "fl/Headers.h"

#include "fuzzy.h"
#include "debug.h"


// Input order in vector ////////////////////////////////
/*
 + angle : The shortest sensor lines' angle. It is used for determining which way to turn.
 + distance : The distance, is the length of the line mentioned above. Here the length of this line determines how "hard" to turn ... short line = hard turn, long line = slow turn.
*/

std::pair<float, float> rc_fuzzy(std::pair<float,float> data)
{
    std::pair<float,float> fuzz_inputs;
    std::pair<float,float> fuzz_outputs;

    std::pair<float,float> result;

    simple_debugprint("before given to fuzzy: ","angle: ",data.first," shortestdistance: ",data.second);
    if( data.first == -100 && data.second == -100)
    {
        // Go straight forward.
        result.first    = 0;
        result.second   = 1;
        simple_debugprint("I CHOOSE TO DRIVE FORWARD!");
    }
    else
    {
        simple_debugprint("I CHOOSE TO FUZZY!");

        // Do fuzzy logic.
        fuzz_inputs.first       = data.first;
        fuzz_inputs.second      = data.second;

        const float SCALE       = 1;

        fuzz_outputs            = fuzzy(fuzz_inputs);
        result.first            = SCALE * (/*1 - */fuzz_outputs.first);
        result.second           = SCALE * fuzz_outputs.second;
    }
    return result;
}

std::pair<float,float> fuzzy(std::pair<float,float> angleAndDistance)
{
    std::string status;

    // start Fuzzy engine
    static fl::Engine* engine = fl::FllImporter().fromFile("ObstacleAvoidance.fll");

    if (not engine->isReady(&status))
        throw fl::Exception("[engine error] engine is not ready:\n" + status, FL_AT);

    std::pair<float,float> results;

    // Inputs for fuzzy
    static fl::InputVariable* distance      = engine->getInputVariable("distance");
    static fl::InputVariable* angle         = engine->getInputVariable("angle");

    // Outputs for fuzzy
    static fl::OutputVariable* steer        = engine->getOutputVariable("out_angle");
    static fl::OutputVariable* speed        = engine->getOutputVariable("speed");

    // Give inputs the wanted values.
    fl::scalar fl_angle                     = angleAndDistance.first;
    angle->setValue(fl_angle);

    fl::scalar fl_dist                      = angleAndDistance.second;
    distance->setValue(fl_dist);

    simple_debugprint(" | measured angle to obstacle = ",fl_angle," | measured shortest distance = ",fl_dist," | ");

    // Process inputs in engine
    engine->process();

    simple_debugprint(" | the angle I want to steer towards = ",steer->getValue()," | the speed I want to drive at = ",speed->getValue()," | ");

    results.first   = steer->getValue();
    results.second  = speed->getValue();

    return results;

}

std::pair<float,float> find_shortest_laser(std::vector<std::pair<float,float>> laserLines)
{

    // We search for the shortest measured distance, if laserLines is empty, then the robot should drive forward.

    float temp_dist = 100.0;
    std::pair<float,float> result;

    simple_debugprint("SIZE OF LASER LIST: ",laserLines.size());

    simple_debugprint("THIS IS THE LASER LIST: ");
    for(auto l : laserLines)
        simple_debugprint("  Angle: ",l.first," Distance: ",l.second);

    for(int i = 0; i < laserLines.size(); i++)
    {
        if(laserLines[i].second < temp_dist)
        {
            simple_debugprint("laserLines[i].second: ",laserLines[i].second);
            simple_debugprint("temp_dist: ",temp_dist);

            temp_dist = laserLines[i].second;
            result = laserLines[i];
            simple_debugprint("find_shortest_laser: A WALL HAS BEEN FOUND");
        }
    }

    // In case no wall is detected
    if(laserLines.empty())
    {
        simple_debugprint("find_shortest_laser: NO WALL IS FOUND");
        return std::pair<float,float>{-100,-100};
    }

    return result;

}

bool wallFound(std::pair<float,float> scan_of_interest)
{
    if(scan_of_interest.first == -100 && scan_of_interest.second == -100)
        return false;
    else
        return true;
}
