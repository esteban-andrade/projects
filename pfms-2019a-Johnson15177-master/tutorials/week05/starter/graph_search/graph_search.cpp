#include <iostream>
#include <vector>
#include <stack>
#include <set>
#include <queue>
#include <map>
#include <algorithm>

using namespace std;

//! A container for the adjacency list
typedef vector<vector<int> > AdjacencyList;

//! Students should attempt to create a function that prints the adjacency list


int main () {
    // Build the graph
    AdjacencyList example_graph = {
        {1, 2},     // Node 0 is connected to nodes 1 and 2 (via edges)
        {0, 4},     // Node 1 is connected to nodes 0 and 4 (via edges)
        {0, 3, 4},  // Node 2 is connected to nodes 0, 3 and 4 (via edges)

        //! Complete for remaining nodes 3,4,5 and 6
    };

    return 0;
}
