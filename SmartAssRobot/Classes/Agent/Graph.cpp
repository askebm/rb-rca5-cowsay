#include "Graph.hpp"

unsigned int Graph::Vertex::cnt=0;

Graph::Vertex::Vertex(const Point p)
	: position(p) {
	id = cnt++;
}

void Graph::Vertex::addAdjacent(const Vertex *v){
	if (v != this) {
		adjacent.push_back(const_cast<Vertex*>(v));
	}
}

void Graph::addAdjancency(std::initializer_list<Vertex*> v){
	for (const auto& i : v) {
		for (const auto& j : v) {
			i->addAdjacent(j);
		}
	}
}

Graph::Vertex* Graph::createVertex(int x,int y){
	//Add check to see if point already exists
	for (auto& i : vertList) {
		if (i->position == Vertex::Point{x,y}) {
			return i;
		}
	}
	auto newVert = new Vertex(Vertex::Point{x,y});
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


std::vector<unsigned int> Graph::getAvailableIDs(const Vertex::VertID& id) const{
	Graph::Vertex& cv = *vertList[id];
	std::vector<Graph::Vertex::VertID> result(cv.adjacent.size());
	int k = 0;
	for (const auto i : cv.adjacent) {
		result[k++] = i ->id;
	}
	return result;
}

