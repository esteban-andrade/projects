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
   /* void (AdjacencyList graph, int start, int goal) {
    //procedure BFS (G, start_v)
    //2 let S be a queue
        queue<int> s;
        
    //3 S.enqueue(start_v)
        s.push(start
    }  */

int main () {
    // Build the graph
    AdjacencyList example_graph = {
        {1, 2},     // Node 0 is connected to nodes 1 and 2 (via edges)
        {0, 4},     // Node 1 is connected to nodes 0 and 4 (via edges)
        {0, 3, 4},  // Node 2 is connected to nodes 0, 3 and 4 (via edges)

        //! Complete for remaining nodes 3,4,5 and 6
        {2, 4, 5},  //Node 3 is connected to nodes 2, 4, and 5
        {1, 2, 3},  //Node 4 is connected to nodes 1, 2 and 3 
        {3, 6},     //Node 5 is connected to nodes 3 and 6
        {5}        //Node 6 is connected to node 5 
    };

    for (AdjacencyList::iterator inner_vec = example_graph.begin(); 
    inner_vec != example_graph.end(); inner_vec++) {
        cout << "Node " << inner_vec - example_graph.begin() << " is adjacent to [";
        for (vector<int>::iterator adjacent = inner_vec->begin(); 
        adjacent != inner_vec->end(); adjacent++) {
            cout << *adjacent << " ";
        }          
    cout << "]" << endl;
    }
    cout << endl;
    
    return 0;
}
