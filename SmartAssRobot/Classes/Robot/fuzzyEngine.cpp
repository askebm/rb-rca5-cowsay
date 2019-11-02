#ifndef FUZZYENGINE_CPP_IN_FWR0QY67
#define FUZZYENGINE_CPP_IN_FWR0QY67

#include <fl/Headers.h>


fl::Engine* CreateEngine() {
	
//Code automatically generated with fuzzylite 6.0.

using namespace fl;

Engine* engine = new Engine;
engine->setName("ObstacleAvoidance");
engine->setDescription("");

InputVariable* distance = new InputVariable;
distance->setName("distance");
distance->setDescription("");
distance->setEnabled(true);
distance->setRange(0.000, 1.000);
distance->setLockValueInRange(false);
distance->addTerm(new Ramp("close", 1.000, 0.000));
distance->addTerm(new Ramp("far", 0.000, 1.000));
engine->addInputVariable(distance);

InputVariable* obstacle = new InputVariable;
obstacle->setName("obstacle");
obstacle->setDescription("");
obstacle->setEnabled(true);
obstacle->setRange(0.000, 1.000);
obstacle->setLockValueInRange(false);
obstacle->addTerm(new Ramp("left", 1.000, 0.000));
obstacle->addTerm(new Ramp("right", 0.000, 1.000));
engine->addInputVariable(obstacle);

OutputVariable* mSteer = new OutputVariable;
mSteer->setName("mSteer");
mSteer->setDescription("");
mSteer->setEnabled(true);
mSteer->setRange(0.000, 1.000);
mSteer->setLockValueInRange(false);
mSteer->setAggregation(new Maximum);
mSteer->setDefuzzifier(new Centroid(100));
mSteer->setDefaultValue(fl::nan);
mSteer->setLockPreviousValue(false);
mSteer->addTerm(new Ramp("left", 1.000, 0.000));
mSteer->addTerm(new Ramp("right", 0.000, 1.000));
mSteer->addTerm(new Gaussian("straight", 0.500, 0.167));
engine->addOutputVariable(mSteer);

OutputVariable* velocity = new OutputVariable;
velocity->setName("velocity");
velocity->setDescription("");
velocity->setEnabled(true);
velocity->setRange(0.000, 1.000);
velocity->setLockValueInRange(false);
velocity->setAggregation(new Maximum);
velocity->setDefuzzifier(new Centroid(100));
velocity->setDefaultValue(fl::nan);
velocity->setLockPreviousValue(false);
velocity->addTerm(new Ramp("fast", 1.000, 0.000));
velocity->addTerm(new Ramp("slow", 0.000, 1.000));
engine->addOutputVariable(velocity);

RuleBlock* mamdani = new RuleBlock;
mamdani->setName("mamdani");
mamdani->setDescription("");
mamdani->setEnabled(true);
mamdani->setConjunction(new Minimum);
mamdani->setDisjunction(fl::null);
mamdani->setImplication(new AlgebraicProduct);
mamdani->setActivation(new General);
mamdani->addRule(Rule::parse("if obstacle is left then mSteer is right", engine));
mamdani->addRule(Rule::parse("if obstacle is right then mSteer is left", engine));
mamdani->addRule(Rule::parse("if distance is close then velocity is slow", engine));
mamdani->addRule(Rule::parse("if distance is far then velocity is fast", engine));
engine->addRuleBlock(mamdani);



return engine;

}

#endif /* end of include guard: FUZZYENGINE_CPP_IN_FWR0QY67 */
