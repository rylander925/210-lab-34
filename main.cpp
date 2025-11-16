/*
COMSC-210 | Lab 34 | Rylan Der
IDE Used: Visual Studio Code 
*/

#include <vector>
#include <queue>
#include <stack>
#include <iostream>
#include <string>
#include <map>
#include <tuple>

using namespace std;

struct Edge {
    int src, dest, weight;
};

typedef pair<int, int> Pair;  // (neighbor, weight)

class Graph {
public:
    // a vector of vectors of Pairs to represent an adjacency list
    vector<vector<Pair>> adjList;

    // A simple container for traversal results
    struct TraversalResult {
        vector<int> order;          // visitation order (indices)
        vector<int> prev;           // previous node in traversal
        vector<int> distFromPrev;   // weight from prev -> node
        vector<int> distFromStart;  // cumulative distance from start
    };

    // constructor
    Graph(vector<Edge> const &edges, int size) {
        adjList.resize(size);
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
        for (int i = 0; i < (int)adjList.size(); i++) {
            cout << i << " --> ";
            for (Pair v: adjList[i])
                cout << "(" << i << ", " << v.first << ", " << v.second << ") ";
            cout << endl;
        }
    }

    // ------------------------------------------------------------
    // BFS : Breadth-First Search (returns TraversalResult)
    // Tracks cumulative distances along the BFS-tree path
    // ------------------------------------------------------------
    TraversalResult BFS(int start) {
        int n = adjList.size();
        vector<bool> visited(n, false);
        queue<int> q;
        TraversalResult res;
        res.prev.assign(n, -1);
        res.distFromPrev.assign(n, 0);
        res.distFromStart.assign(n, 0);

        visited[start] = true;
        q.push(start);
        res.order.push_back(start);
        res.prev[start] = -1;
        res.distFromPrev[start] = 0;
        res.distFromStart[start] = 0;

        while (!q.empty()) {
            int v = q.front(); q.pop();

            // explore neighbors
            for (auto &p : adjList[v]) {
                int neighbor = p.first;
                int w = p.second;
                if (!visited[neighbor]) {
                    visited[neighbor] = true;
                    q.push(neighbor);

                    // record traversal info for neighbor
                    res.order.push_back(neighbor);
                    res.prev[neighbor] = v;
                    res.distFromPrev[neighbor] = w;
                    res.distFromStart[neighbor] = res.distFromStart[v] + w;
                }
            }
        }

        // prune result vectors to visited nodes only (optional but convenient)
        return res;
    }

    // ------------------------------------------------------------
    // DFS : Depth-First Search (explicit stack) returning TraversalResult
    // We track the cumulative distance along the actual DFS visit path.
    // ------------------------------------------------------------
    TraversalResult DFS(int start) {
        int n = adjList.size();
        vector<bool> visited(n, false);
        TraversalResult res;
        res.prev.assign(n, -1);
        res.distFromPrev.assign(n, 0);
        res.distFromStart.assign(n, 0);

        // stack holds tuples: (node, parent, weight_from_parent, dist_from_start_to_node)
        // For start: parent = -1, weight = 0, dist = 0
        typedef tuple<int,int,int,int> StackElem;
        stack<StackElem> s;
        s.push(make_tuple(start, -1, 0, 0));

        while (!s.empty()) {
            auto [v, parent, weightFromParent, distToV] = s.top();
            s.pop();

            if (visited[v]) continue;

            // mark visited and record
            visited[v] = true;
            res.order.push_back(v);
            res.prev[v] = parent;
            res.distFromPrev[v] = weightFromParent;
            res.distFromStart[v] = distToV;

            // Push neighbors. To get the intuitive DFS order similar to recursion
            // (so the first neighbor in adjList is explored first), we push them
            // onto the stack in reverse order.
            for (int i = (int)adjList[v].size() - 1; i >= 0; --i) {
                int neighbor = adjList[v][i].first;
                int w = adjList[v][i].second;
                if (!visited[neighbor]) {
                    // dist to neighbor along this DFS path = distToV + w
                    s.push(make_tuple(neighbor, v, w, distToV + w));
                }
            }
        }

        return res;
    }
};

// ------------------------------
// Town class that wraps Graph
// ------------------------------
class Town {
public:
    Graph graph;
    vector<string> idxToName;       // index -> building name
    map<string,int> nameToIdx;      // building name -> index

    Town(const vector<string>& buildingNames, const vector<Edge>& edges)
        : graph(edges, (int)buildingNames.size()), idxToName(buildingNames)
    {
        for (int i = 0; i < (int)buildingNames.size(); ++i) {
            nameToIdx[buildingNames[i]] = i;
        }
    }

    void printTownMap() {
        cout << "City Map (Buildings and Roads):" << endl;
        cout << "================================" << endl;
        for (int i = 0; i < (int)graph.adjList.size(); ++i) {
            cout << idxToName[i] << " (ID " << i << ") connects to:" << endl;
            for (auto &p : graph.adjList[i]) {
                int neighbor = p.first;
                int w = p.second;
                cout << "  → " << idxToName[neighbor] << " (Road length: " << w << " units)" << endl;
            }
            cout << endl;
        }
    }

    // BFS-based detective witness outreach (layer-by-layer)
    void investigateCrimeScene(const string &startBuilding) {
        auto it = nameToIdx.find(startBuilding);
        if (it == nameToIdx.end()) {
            cout << "Unknown start building: " << startBuilding << endl;
            return;
        }
        int start = it->second;
        auto res = graph.BFS(start);

        cout << "Witness Outreach (BFS) from " << idxToName[start]
             << " (Crime Scene):" << endl;
        cout << "========================================" << endl;
        cout << "Purpose: Question potential witnesses by proximity (layer-by-layer)\n" << endl;

        // We'll show the start first
        cout << "Checking Building: " << idxToName[start] << " (ID " << start << ")" << endl;
        cout << "  → Distance from previous: N/A" << endl;
        cout << "  → Total distance from start: 0" << endl << endl;

        // For every visited node in BFS order after the start, print details
        for (int i = 1; i < (int)res.order.size(); ++i) {
            int node = res.order[i];
            int prev = res.prev[node];
            int dprev = res.distFromPrev[node];
            int dstart = res.distFromStart[node];

            cout << "Checking Building: " << idxToName[node] << " (ID " << node << ")" << endl;
            cout << "  → Reached from: " << (prev >= 0 ? idxToName[prev] : string("N/A"))
                 << " (edge length: " << dprev << " units)" << endl;
            cout << "  → Total distance from crime scene: " << dstart << " units" << endl;

            // Optionally list immediate neighbors to question next (those not yet visited may be enqueued later)
            cout << "  → Nearby roads from this building:" << endl;
            for (auto &p : graph.adjList[node]) {
                int nb = p.first;
                int w = p.second;
                cout << "      - " << idxToName[nb] << " (length: " << w << ")" << endl;
            }
            cout << endl;
        }

        cout << "BFS complete: total buildings considered = " << res.order.size() << endl << endl;
    }

    // DFS-based escape route trace (deep pursuit)
    void traceEscapeRoute(const string &startBuilding) {
        auto it = nameToIdx.find(startBuilding);
        if (it == nameToIdx.end()) {
            cout << "Unknown start building: " << startBuilding << endl;
            return;
        }
        int start = it->second;
        auto res = graph.DFS(start);

        cout << "Escape Route Trace (DFS) from " << idxToName[start]
             << " (Crime Scene):" << endl;
        cout << "========================================" << endl;
        cout << "Purpose: Trace deep pursuit routes that a fleeing suspect might take\n" << endl;

        for (int i = 0; i < (int)res.order.size(); ++i) {
            int node = res.order[i];
            int prev = res.prev[node];
            int dprev = res.distFromPrev[node];
            int dstart = res.distFromStart[node];

            cout << "Investigating Building: " << idxToName[node] << " (ID " << node << ")" << endl;
            if (prev >= 0) {
                cout << "  → Came from: " << idxToName[prev]
                     << " (edge length: " << dprev << " units)" << endl;
            } else {
                cout << "  → Origin (crime scene)" << endl;
            }
            cout << "  → Cumulative distance along this DFS path: " << dstart << " units" << endl;

            // If not last element, show likely next step(s) based on the adjacency
            cout << "  → Possible next steps (roads out):" << endl;
            for (auto &p : graph.adjList[node]) {
                int nb = p.first;
                int w = p.second;
                cout << "      - " << idxToName[nb] << " (length: " << w << ")" << endl;
            }
            cout << endl;
        }

        cout << "DFS complete: total buildings inspected in route trace = " << res.order.size() << endl << endl;
    }
};


int main() {
    // Building names in index order 0..12
    vector<string> buildingNames = {
        "City Hall",                // 0
        "Fire Station",             // 1
        "Central Plaza",            // 2
        "Library",                  // 3
        "City Bank",                // 4
        "General Hospital",         // 5
        "Utility Control Center",   // 6
        "Shopping District",        // 7
        "Warehouse Depot",          // 8
        "Train Station",            // 9
        "Power Substation",         // 10
        "Apartment Complex",        // 11
        "Industrial Plant"          // 12
    };

    // Edges for the modified graph (weights = road lengths)
    vector<Edge> edges = {
        {0, 2, 15},
        {0, 3, 7},
        {2, 3, 10},
        {2, 4, 3},
        {3, 5, 12},
        {4, 5, 8},
        {7, 0, 6},
        {7, 8, 9},
        {8, 2, 11},
        {8, 9, 4},
        {9, 10, 5},
        {10, 4, 14},
        {11, 3, 13},
        {11, 12, 2},
        {12, 5, 16},
        // newly added roads for isolated nodes per Option 3:
        {1, 3, 6},   // Fire Station <-> Library (close)
        {6, 10, 4}   // Utility Control Center <-> Power Substation
    };

    int size = (int)buildingNames.size(); // 13

    // Create town
    Town myTown(buildingNames, edges);

    // Print the town map (detailed)
    myTown.printTownMap();

    // Investigate (BFS) from the crime scene located at "City Hall" (0)
    myTown.investigateCrimeScene("City Hall");

    // Trace escape route (DFS) from the crime scene located at "City Hall" (0)
    myTown.traceEscapeRoute("City Hall");

    return 0;
}
