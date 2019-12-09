#include <Agent/Graph.hpp>
#include <Agent/Q_Agent.hpp>
#include <string>
#include <cstdlib>

int main(int argc, char *argv[])
{
	Graph graph;
	
	static constexpr int N = 100;

	static constexpr double pi8 = 0.0314;
	static constexpr double length = 20000;

	Graph::Vertex* nodes[N];
	for (int i = 0; i < N; ++i) {
		auto angle = pi8*i;
		nodes[i] = graph.createVertex(std::cos(angle)*length+2*length,std::sin(angle)*length+2*length,1);
	}
	for (int i = 0; i < N; ++i) {
		for (int k = 0; k < N; ++k) {
			graph.addAdjancency({nodes[i],nodes[k]});
		}
	}

	int startNode = 1;


	auto vState = graph.getVisitState();
	vState[startNode-1] = 1;
	auto state = QAgent::State{vState,startNode-1};

	QAgent agent(state,graph);

	double time = graph.getCostByIDs(0,1) * N/2;

	agent.lambda = 0.8;
	agent.epsilon = 0.1;
	agent.alpha = 0.2;

	for (int i = 0; i < 10000000; ++i) {
		std::cout << agent.episode(time) << std::endl;
	}

	agent.epsilon = 0;
	agent.alpha = 0;
	//agent.episode(time,true);

	agent.savePolicy("/tmp/policy.txt");







	return 0;
}
