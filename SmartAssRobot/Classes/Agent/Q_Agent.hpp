#ifndef Q_AGENT_H
#define Q_AGENT_H
#include <string>
#include <map>
#include <iostream>
#include <utility>

#include "Graph.hpp"

class QAgent
{
public:
	typedef unsigned int Action;
	typedef std::pair<Graph::VisitState,Graph::Vertex::VertID> State;
	typedef std::pair<State,Action> StateAction;
	typedef int Reward;
	typedef std::map<StateAction,Reward> Policy;
private:
	Reward defaultReward = 0;
	Policy policy;
	Graph& graph;
	double epsilon = 0.1; // greed factor
	double alpha = 0.5; // learning rate
	double lambda = 0.9; //discount factor

	State currentState;

	Action getNextAction(const State& s);
	Reward getReward(const State& s,const Action& a);
	Reward getMaxReward(const State& s, Action& a);
	State getNextState(const Action& a, const State& s);
	

	void episode();

	bool isTerminalState(const State& s);

public:
	void savePolicy(const std::string&& file);
	void loadPolicy(const std::string&& file);
	
	QAgent();
	virtual ~QAgent();
};

#endif /* Q_AGENT_H */
