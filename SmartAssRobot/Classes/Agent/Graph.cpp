#include "Graph.hpp"
#include <cmath>

unsigned int Graph::Vertex::cnt=0;
bool operator==(const Graph::Vertex::Point& p1,const Graph::Vertex::Point& p2) {
	return ( (p1.x == p2.x) && (p1.y == p2.y) );
}

Graph::Vertex::Vertex(const Point p) : position(p) {
	id = cnt++;
}

Graph::~Graph(){}

void Graph::Vertex::addAdjacent(const Vertex *v){
	if (v != this) {
		double dist = std::sqrt(std::abs(
				(position.x - v->position.x) * (position.x - v->position.x) +
				(position.y - v->position.y) * (position.y - v->position.y)
				));
		adjacent.push_back({const_cast<Vertex*>(v),dist});
	}
}

void Graph::addAdjancency(std::initializer_list<Vertex*> v){
	for (const auto& i : v) {
		for (const auto& j : v) {
			i->addAdjacent(j);
		}
	}
}

Graph::Vertex* Graph::createVertex(int x,int y,int m){
	//Add check to see if point already exists
	for (auto& i : vertList) {
		if (i->position == Vertex::Point{x,y}) {
			return i;
		}
	}
	auto newVert = new Vertex(Vertex::Point{x,y});
	newVert->marbles = m;
	vertList.push_back(newVert);
	return newVert;
}

unsigned int Graph::getSize() const {
	return vertList.size();
}

Graph::VisitState Graph::getVisitState() const {
	VisitState result((size_t)getSize());
	for (const auto& i : vertList) {
		result[i->id] = i->visited;
	}
	return result;
}


std::vector<Graph::IDCost> Graph::getAvailableIDs(const Vertex::VertID& id) const{
	Graph::Vertex& cv = *vertList[id];
	std::vector<Graph::IDCost> result(cv.adjacent.size());
	int k = 0;
	for (const auto i : cv.adjacent) {
		result[k++] = IDCost{i.first->id,i.second};
	}
	return result;
}


double Graph::getCostByIDs(const Graph::Vertex::VertID& id1,const Graph::Vertex::VertID id2) {
	for (const auto& i : vertList[id1]->adjacent) {
		if (i.first->id == id2) {
			return i.second;
		}
	}
	return -1;
}

int Graph::getNumberOfMarblesByID(const Graph::Vertex::VertID &id) const {
	return vertList[id]->marbles;
}


