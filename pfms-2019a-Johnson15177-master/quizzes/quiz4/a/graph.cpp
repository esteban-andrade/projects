#include <set>
#include <queue>
#include "graph.h"

void Graph::addVertex(vertex v) {
  //Insert a vertex with no edges
    edges_t temp;
    temp.emplace(v, 0);
    weightedGraph_.emplace(v, temp);
}

bool Graph::hasVertex(vertex v) {
  //Check if the vertex v, exists in the graph
    if(weightedGraph_.find(v) != weightedGraph_.end())
    {
        return true;
    }
    else
    {
        return false;
    }

}

void Graph::addEdge(vertex u, vertex v, weight w) {
  //Assumes that u & v have already been added to the graph
  //We need to record the same edge twice as this is an undirected graph
  weightedGraph_.at(u).insert({v, w}); //Inserting an edge between u and v, with weight w
  weightedGraph_.at(v).insert({u, w}); //Inserting an edge between v and u, with weight w
}

std::vector<Graph::vertex> Graph::getVertices(void) {
  //Iterate through the weightedGraph_ and push back each vertex to the vertices vector
    graph_t::iterator it;
    std::vector<Graph::vertex> vertices;
    for( it = weightedGraph_.begin(); it != weightedGraph_.end(); it++)
    {
        vertices.push_back(it->first);
    }
    return vertices;
}

std::vector<Graph::vertex> Graph::bfs(vertex start) {
  //Perform a breadth first search on the entire graph
    std::vector<Graph::vertex> vertices;


    //std::map<vertex, vertex> parents;
    vertex current;
    std::set<vertex> visited;

    std::queue<vertex> q;

    visited.insert(start);

    q.push(start);

    while(!q.empty())
    {
        current = q.front();
        q.pop();

        for(graph_t::iterator it = weightedGraph_.begin(); it != weightedGraph_.end(); it++)
        {
            if(visited.find(it->first) == visited.end())
            {
                //parents[it] = current;
                vertices.push_back(it->first);
                visited.insert(it->first);

                q.push(it->first);
            }
        }
    }
    return vertices;
}
