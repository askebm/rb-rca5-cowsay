#include "Q_Agent.hpp"
#include <fstream>
#include <exception>
#include <limits>
#include <stdexcept>
#include <algorithm>
#include <cstdlib>

void QAgent::savePolicy(const std::string&& file) {
	std::ofstream fout(file,std::ofstream::out);
	if (!fout.is_open()) {
		throw std::runtime_error("File not open: " + file);
	}
	for (const auto& i : policy) {
		auto& SA = i.first;
		auto& QSA = i.second;
		auto& S = SA.first;
		auto& A = SA.second;
		for (int j = 0; j < S.first.size(); ++j) {
			fout << S.first[j];
		}
		fout << "," << S.second << "," << A  << "," << QSA << std::endl;
	}
	fout.close();
}

void QAgent::loadPolicy(const std::string&&  file) {
	std::ifstream fin(file,std::ifstream::in);
	if (!fin.is_open()) {
		throw std::runtime_error("File not open: " + file);
	}
	while (!fin.eof()) {
		std::string s;
		std::getline(fin,s);
		auto delim1 = s.find(",");
		if (delim1 == std::string::npos) {
			break;
		}
		auto delim2 = s.find(",",delim1+1);
		auto delim3 = s.find(",",delim2+1);

		Graph::VisitState vState(s.substr(0,delim1-1));
		Graph::Vertex::VertID id = stoul(s.substr(delim1+1,delim2-1));
		State state{vState,id};

		Action action = stoul(s.substr(delim2+1,delim3-1));

		Reward value = stoi(s.substr(delim3+1));

		StateAction SA{state,action};
		policy[SA] = value;
	}
	fin.close();
}

void QAgent::episode() {
	while (!isTerminalState(currentState)) {
		auto& s = currentState;
		auto a = getNextAction(s);
		auto r = getReward(s,a);
		auto sNew = getNextState(a,s);

		StateAction sa{s,a};

		policy[sa] += alpha * (r + lambda * getMaxReward(sNew,a) - policy[sa]);
	}
}


QAgent::Action QAgent::getNextAction(const State& s) {
	Action a;
	auto random = static_cast <double> (rand()) / static_cast <double> (RAND_MAX);
	// We do the random thing here
	if (random <= epsilon) {
		auto availAction = graph.getAvailableIDs(s.second);
		a = availAction[rand()%availAction.size()];
	}
	// We get the money here
	getMaxReward(s,a);
	return a;
}

QAgent::Reward QAgent::getReward(const State& s,const Action& a){
	StateAction sa{s,a};
	// Do we hace policy on the StateAction pair yet
	if (!policy.count(sa)) {
		policy[sa] = defaultReward;
	}
	return policy[sa];
}

QAgent::Reward QAgent::getMaxReward(const State& s, Action& a){
	auto& id = s.second;
	auto availAction = graph.getAvailableIDs(id);
	a = availAction[0];
	auto r = std::numeric_limits<Reward>::min();
	
	for (const auto& i : availAction) {
		if (r < getReward(s,a)) {
			a = i;
		}
	}
	return r;
}
QAgent::State QAgent::getNextState(const Action& a, const State& s){
	auto sNew = s;
	sNew.first[a] = 1;
	sNew.second = a;
	return sNew;
}

bool QAgent::isTerminalState(const State& s) {
	auto& vs = s.first;
	for (int i = 0; i < vs.size(); ++i) {
		if (!vs[i]) {
			return false;
		}
	}
	return true;
}
