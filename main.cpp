/*
COMSC-210 | Lab 34 | Rylan Der
IDE Used: Visual Studio Code
*/

#include <vector>
#include <iostream>

using namespace std;

struct Edge {
    int src, dest, weight;
};

typedef pair<int, int> Pair;  // Creates an alias 'Pair' for the pair<int,int> data type
                              // so below we can use vector<Pair> rather than vector<pair<int, int>>
class Graph {
    public:
    // a vector of vectors of Pairs to represent an adjacency list
    vector<vector<Pair>> adjList;
    // constructor
    Graph(vector<Edge> const &edges, int size) {
        // resize the vector to hold SIZE elements of type vector<Edge>
        adjList.resize(size);
        // add edges to the directed graph
        for (auto &edge: edges) {
            int src = edge.src;
            int dest = edge.dest;
            int weight = edge.weight;
            adjList[src].push_back(make_pair(dest, weight));
            adjList[dest].push_back(make_pair(src, weight));
        }
    }

    void printGraph() {
        for (int i = 0; i < adjList.size(); i++) {
            for (Pair v: adjList[i])
                cout << "(" << i << ", " << v.first << ", " << v.second << ") ";
            cout << endl;
        }
    }
};

int main() {
    // list of edges based on the drawing
    vector<Edge> edges = {
        {0, 1, 12},
        {0, 2, 8},
        {0, 3, 21},
        {2, 3, 6},
        {2, 6, 2},
        {2, 5, 5},
        {2, 4, 4},
        {6, 5, 6},
        {5, 4, 9}
    };

    // number of vertices (0 through 6)
    int size = 7;

    // construct the graph
    Graph g(edges, size);

    // print it
    g.printGraph();

    return 0;
}