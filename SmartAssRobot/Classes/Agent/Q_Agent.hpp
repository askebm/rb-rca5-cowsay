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
	typedef double Reward;
	typedef std::map<StateAction,Reward> Policy;
private:
	struct {
		Reward defaultReward = 0;
		Reward isVisited = 0;
		Reward unVisited = 0;
		Reward marbleValue = 20;
	} Rewards;

	Policy policy;
	Graph& graph;

	State currentState;

	Action getNextAction(const State& s);
	Reward getReward(const State& s,const Action& a);
	Reward getMaxReward(const State& s, Action& a);
	Reward Qmax(const State& s, Action& a);
	State getNextState(const Action& a, const State& s);
	


	bool isTerminalState(const State& s);

public:
	double epsilon = 0.1; // greed factor
	double alpha = 0.2; // learning rate
	double lambda = 0.9; //discount factor

	void savePolicy(const std::string&& file);
	void loadPolicy(const std::string&& file);
	void setState(const State& s);

	Reward episode(int time=200,bool print = false);

	QAgent(State s, Graph& g);
	virtual ~QAgent();
};

#endif /* Q_AGENT_H */
