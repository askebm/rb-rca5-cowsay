#ifndef GRAPH_H
#define GRAPH_H

#include<vector>
#include<memory>
#include<exception>
#include <limits>
#include <cstdlib>
#include<bits/stdc++.h>

#include "brushfire.h"

using namespace std;

//define infinite distance
# define INF 0x3f3f3f3f

// iPair ==>  Integer Pair
typedef pair<int, int> iPair;

// This class represents a directed graph using adjacency representation
class Graph
{
    int number_of_v;    // No. of vertices

    // to store vertex and weight pair for every edge
    list< pair<int, int> > *adj;

public:
    Graph(int number_of_v);  // Constructor

    // function to add an edge to graph
    void addEdge(int vertex_2, int vertex_1, int weight);

    // prints shortest path from source - any vertex you decide
    void shortestPath(int source);

    //
    void EfficientWay();

};

#endif // GRAPH_H
