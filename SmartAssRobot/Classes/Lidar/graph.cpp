#include "graph.h"

// Allocates memory for adjacency list
Graph::Graph(int number_of_v)
{
    this->number_of_v = number_of_v;
    adj = new list<iPair> [number_of_v];
}

void Graph::addEdge(int vertex_2, int vertex_1, int weight)
{
    adj[vertex_2].push_back(make_pair(vertex_1, weight));
    adj[vertex_1].push_back(make_pair(vertex_2, weight));
}

// Prints shortest paths from src to all other vertices
void Graph::shortestPath(int source)
{
    // Using the inbuild PQ in cpp
    priority_queue< iPair, vector <iPair> , greater<iPair> > pq;

    // Create a vector for distances and initialize all distances as infinite (INF)
    vector<int> dist(number_of_v, INF);

    // Insert source vertex in priority queue and initialize its distance as 0.
    pq.push(make_pair(0, source));
    dist[source] = 0;

    // Looping till priority queue becomes empty
    while (!pq.empty())
    {
        // The first vertex in pair is the minimum distance vertex, extract it from priority queue.
        // vertex label is stored in second of pair
        int vertex_1 = pq.top().second;
        pq.pop();

        // Get all adjacent vertices of a vertex
        list< pair<int, int> >::iterator i;
        for (i = adj[vertex_1].begin(); i != adj[vertex_1].end(); ++i)
        {
            // Get vertex label and weight of current adjacent of "vertex_1".
            int v = (*i).first;
            int weight = (*i).second;

            //  If there is shorted path to v through u.
            if (dist[v] > dist[vertex_1] + weight)
            {
                // Updating distance of v
                dist[v] = dist[vertex_1] + weight;
                pq.push(make_pair(dist[vertex_1], v));
            }
        }
    }

    // Print shortest distances
    for (int i = 0; i < number_of_v; ++i)
        printf("%d \t\t %d\n", i, dist[i]);
}

void Graph::EfficientWay()
{

}
