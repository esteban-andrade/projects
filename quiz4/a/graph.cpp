#include <set>
#include <queue>
#include "graph.h"

void Graph::addVertex(vertex v) {
  //Insert a vertex with no edges
}

bool Graph::hasVertex(vertex v) {
  //Check if the vertex v, exists in the graph
  return false;
}

void Graph::addEdge(vertex u, vertex v, weight w) {
  //Assumes that u & v have already been added to the graph
  //We need to record the same edge twice as this is an undirected graph
  weightedGraph_.at(u).insert({v, w}); //Inserting an edge between u and v, with weight w
  weightedGraph_.at(v).insert({u, w}); //Inserting an edge between v and u, with weight w
}


std::vector<Graph::vertex> Graph::getVertices(void) {
  //Iterate through the weightedGraph_ and push back each vertex to the vertices vector
  return std::vector<Graph::vertex>();
}

std::vector<Graph::vertex> Graph::bfs(vertex start) {
  //Perform a breadth first search on the entire graph
  return std::vector<Graph::vertex>();
}
