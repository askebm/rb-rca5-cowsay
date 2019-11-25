#include <Agent/Graph.hpp>
#include <Agent/Q_Agent.hpp>
#include <string>

int main(int argc, char *argv[])
{
	Graph graph;


	auto v1 = graph.createVertex(20,20,3);
	auto v2 = graph.createVertex(23,7,0);
	auto v3 = graph.createVertex(15,3,1);
	auto v4 = graph.createVertex(10,7,5);
	auto v5 = graph.createVertex(10,20,0);
	auto v6 = graph.createVertex(5,20,0);
	auto v7 = graph.createVertex(1,7,7);
	auto v8 = graph.createVertex(7,7,0);
	auto v9 = graph.createVertex(5,3,5);

	graph.addAdjancency({v1,v2});
	graph.addAdjancency({v2,v3});
	graph.addAdjancency({v3,v4});
	graph.addAdjancency({v4,v5});
	graph.addAdjancency({v5,v6});
	graph.addAdjancency({v6,v7,v8,v9});

	
	int startNode = 5;

	auto vState = graph.getVisitState();
	vState[startNode-1] = 1;
	auto state = QAgent::State{vState,startNode-1};

	QAgent agent(state,graph);
	std::cout << agent.episode() << std::endl;
	for (int i = 0; i < 20000; ++i) {
		agent.episode(50);
	}

	return 0;
}
