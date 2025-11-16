/*
COMSC-210 | Lab 34 | Rylan Der
IDE Used: Visual Studio Code
*/

#include <vector>
#include <queue>
#include <stack>
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
        cout << "Graph's adjacency list:" << endl;
        for (int i = 0; i < adjList.size(); i++) {
            cout << i << " --> ";
            for (Pair v: adjList[i])
                cout << "(" << i << ", " << v.first << ", " << v.second << ") ";
            cout << endl;
        }
    }

    // ------------------------------------------------------------
    // BFS : Breadth-First Search
    // ------------------------------------------------------------
    void BFS(int start) {
        vector<bool> visited(adjList.size(), false);
        queue<int> q;

        visited[start] = true;
        q.push(start);

        cout << "BFS starting from vertex " << start << ": ";

        while (!q.empty()) {
            int v = q.front();
            q.pop();

            cout << v << " ";  // visit

            // explore neighbors
            for (auto &p : adjList[v]) {
                int neighbor = p.first;
                if (!visited[neighbor]) {
                    visited[neighbor] = true;
                    q.push(neighbor);
                }
            }
        }

        cout << endl;
    }

    // ------------------------------------------------------------
    // DFS : Depth-First Search (explicit stack, NOT recursion)
    // ------------------------------------------------------------
    void DFS(int start) {
        vector<bool> visited(adjList.size(), false);
        stack<int> s;

        s.push(start);

        cout << "DFS starting from vertex " << start << ": ";

        while (!s.empty()) {
            int v = s.top();
            s.pop();

            if (!visited[v]) {
                visited[v] = true;
                cout << v << " ";  // visit

                // push neighbors onto stack
                // (We must push in reverse order if we want ascending order traversal)
                for (int i = adjList[v].size() - 1; i >= 0; i--) {
                    int neighbor = adjList[v][i].first;
                    if (!visited[neighbor]) {
                        s.push(neighbor);
                    }
                }
            }
        }

        cout << endl;
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

    //DFS
    g.DFS(0);
    g.BFS(0);

    return 0;
}