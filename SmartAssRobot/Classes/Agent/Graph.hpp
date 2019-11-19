#ifndef GRAPH_H
#define GRAPH_H
#include <vector>
#include <initializer_list>
#include <set>
#include <boost/dynamic_bitset.hpp>

class Graph
{
public:
	typedef boost::dynamic_bitset<> VisitState;
	class Vertex {
		public:
			typedef struct {int x,y;} Point;
			typedef unsigned int VertID;
		private:
			static unsigned int cnt;
		public:
			VertID id;
			std::vector<Vertex*> adjacent;
			void addAdjacent(const Vertex* v);

			Point position;
			bool visited{false};
			int marbles = 0;

			Vertex(const Point p);
	};
private:
	std::vector<Vertex*> vertList;

public:
	unsigned int getSize() const;
	Vertex* createVertex(int x,int y);
	void addAdjancency(std::initializer_list<Vertex*> v);
	// add prune
	VisitState getVisitState() const;
	
	Vertex* currentVert;
	std::vector<unsigned int> getAvailableIDs(const Vertex::VertID& id) const;

	Graph();
	virtual ~Graph();
};

bool operator==(const Graph::Vertex::Point& p1,const Graph::Vertex::Point& p2) {
	return ( (p1.x == p2.x) && (p1.y == p2.y) );
}

#endif /* GRAPH_H */
