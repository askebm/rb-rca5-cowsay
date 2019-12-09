#define _USE_MATH_DEFINES
#include <cmath>

#include <Agent/Graph.hpp>
#include <Agent/Q_Agent.hpp>
#include <string>
#include <cstdlib>


int main(int argc, char *argv[])
{
	Graph graph;

	static constexpr double length = 2000000;

	std::string _TEST_NAME = argv[1];
	auto _N_NODES = std::atoi(argv[2]);
	auto _N_TESTS = std::atoi(argv[3]);
	auto _LAMBDA =	double(std::atoi(argv[4]))/double(100);
	auto _EPSILON =	double(std::atoi(argv[5]))/double(100);
	auto _ALPHA =		double(std::atoi(argv[6]))/double(100);
	auto _FREQ = std::atoi(argv[7]);

	Graph::Vertex* nodes[_N_NODES];

	int startNode = 1;

	if (_TEST_NAME == "circle") {
		auto angle = static_cast<double>( M_PI) / static_cast<double>(_N_NODES);
		for (int i = 0; i < _N_NODES; ++i) {
			auto lCos = std::cos(angle*i);
			auto lSin = std::sin(angle*i);
			int x = double(lCos*length)+double(2*length);
			int y = double(lSin*length)+double(2*length);
			nodes[i] = graph.createVertex(x,y,1);
		}
		for (int i = 0; i < _N_NODES; ++i) {
			for (int k = 0; k < _N_NODES; ++k) {
				graph.addAdjancency({nodes[i],nodes[k]});
			}
		}
	} else if (_TEST_NAME == "patience") {
		if (!(_N_NODES%2)) {
			throw("Nodes shoudl be unequal");
		}
		int x = 100;
		int y = 10;
		int n = (_N_NODES-1) >> 1;

		nodes[0] = graph.createVertex(x,y, (_N_NODES+1)/2);

		for (int i = 1; i < n+1; ++i) {
			nodes[i] = graph.createVertex(x*i+x,y,0);
		}
		for (int i = n+1; i < _N_NODES; ++i) {
			nodes[i] = graph.createVertex(i*x+x,y,1);
		}

		for (int i = 0; i < _N_NODES-1; ++i) {
			graph.addAdjancency({nodes[i],nodes[i+1]});
		}
		startNode = (_N_NODES+1)/2;
	}

	

	auto vState = graph.getVisitState();
	vState[startNode-1] = 1;
	auto state = QAgent::State{vState,startNode-1};

	QAgent agent(state,graph);

	double time = graph.getCostByIDs(0,1) * (_N_NODES)/2 +1;

	agent.lambda = _LAMBDA;
	agent.epsilon = _EPSILON;
	agent.alpha = _ALPHA;

	for (int i = 0; i < _N_TESTS; ++i) {
		if ( !(i%_FREQ) ) {
			agent.epsilon =0;
			agent.alpha =0;
			std::cout << agent.episode(time) << std::endl;
			agent.epsilon = _EPSILON;
			agent.alpha = _ALPHA;
		} 
		agent.episode(time);
	}


	//agent.savePolicy("/tmp/policy.txt");
	return 0;
}
