#ifndef GRAPH_H
#define GRAPH_H
#include <vector>
#include <initializer_list>
#include <set>
#include <boost/dynamic_bitset.hpp>
#include <utility>

class Graph
{
public:
	typedef boost::dynamic_bitset<> VisitState;
	
	class Vertex {
		public:
			typedef struct {int x,y;} Point;
			typedef unsigned int VertID;
			typedef std::pair<Vertex*,double> Adjacent;
		private:
			static unsigned int cnt;
		public:
			VertID id;
			std::vector<Adjacent> adjacent;
			void addAdjacent(const Vertex* v);

			Point position;
			bool visited{false};
			int marbles = 0;

			Vertex(const Point p);
	};
	typedef std::pair<Vertex::VertID,double> IDCost;
private:
	std::vector<Vertex*> vertList;

public:
	unsigned int getSize() const;
	Vertex* createVertex(int x,int y,int m=0);
	void addAdjancency(std::initializer_list<Vertex*> v);
	// add prune
	VisitState getVisitState() const;
	
	Vertex* currentVert;
	std::vector<IDCost> getAvailableIDs(const Vertex::VertID& id) const;

	double getCostByIDs(const Vertex::VertID& id1,const Vertex::VertID id2);
	int getNumberOfMarblesByID(const Vertex::VertID& id) const;


	Graph(){}
	virtual ~Graph();
};


#endif /* GRAPH_H */
