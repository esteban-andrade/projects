#include <set>
#include <queue>
#include <iostream>
#include "graph.h"

using namespace std;

void Graph::addVertex(vertex v) {
    //Insert a vertex with no edges
    if (!hasVertex(v)) {
        edges_t temp;
        temp[v] = 0;
        weightedGraph_[v] = temp;
    }
}

bool Graph::hasVertex(vertex v) {
    //Check if the vertex v, exists in the graph
    bool temp = false;
    for (auto x : weightedGraph_) {
        for (auto y : weightedGraph_) {
            if (v == y.first) {
                temp = true;
            }
        }
    }

    return temp;
}

void Graph::addEdge(vertex u, vertex v, weight w) {
    //Assumes that u & v have already been added to the graph
    //We need to record the same edge twice as this is an undirected graph
    weightedGraph_.at(u).insert({v, w}); //Inserting an edge between u and v, with weight w
    weightedGraph_.at(v).insert({u, w}); //Inserting an edge between v and u, with weight w
}


std::vector<Graph::vertex> Graph::getVertices(void) {
    //Iterate through the weightedGraph_ and push back each vertex to the vertices vector
    vector<Graph::vertex> temp;
    for (auto x : weightedGraph_) {
        temp.push_back(x.first);
    }

    return temp;
}

std::vector<Graph::vertex> Graph::getNeighbours(vertex) {

}

std::vector<Graph::vertex> Graph::bfs(vertex start) {
    //Perform a breadth first search on the entire graph
    return std::vector<Graph::vertex>();
}

/*int breadth(AdjacencyList graph, int start, int end) {
    queue<int> s;
    set<int> discovered;
    map<int, int> parents;
    s.push(start);
    while (!s.empty()) {
        int v = s.front();
        s.pop();
        if (v == end) {
            cout << v << end;
        }
        for (auto w : graph.at(v)) {
            if (discovered.find() == discovered.end()) {
                discovered.insert(w);

            }
        }
    }
}*/
