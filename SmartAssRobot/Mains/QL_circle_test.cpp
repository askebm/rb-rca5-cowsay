#include <Agent/Graph.hpp>
#include <Agent/Q_Agent.hpp>
#include <string>
#include <cstdlib>

int main(int argc, char *argv[])
{
	Graph graph;

	static constexpr double pi8 = 0.3927;
	static constexpr double length = 50;

	Graph::Vertex* nodes[8];
	for (int i = 0; i < 8; ++i) {
		auto angle = pi8*i;
		nodes[i] = graph.createVertex(std::cos(angle)*length+2*length,std::sin(angle)*length+2*length,1);
	}
	graph.addAdjancency({nodes[0],nodes[1],nodes[2],nodes[3],nodes[4],nodes[5],nodes[6],nodes[7]});

	int startNode = 1;

	auto vState = graph.getVisitState();
	vState[startNode-1] = 1;
	auto state = QAgent::State{vState,startNode-1};

	QAgent agent(state,graph);

	int time = graph.getCostByIDs(0,1) * 4 + 4;

	agent.lambda = 1;
	agent.epsilon = 0.2;
	agent.alpha = 0.3;

	for (int i = 0; i < 20000; ++i) {
		std::cout << agent.episode(time) << std::endl;
	}

	agent.epsilon = 0;
	agent.alpha = 0;
	//agent.episode(time,true);

	agent.savePolicy("/tmp/policy.txt");







	return 0;
}
