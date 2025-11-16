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

/**
 * Represents a weighted undirected connection between two vertices.
 * 'src' and 'dest' are vertex indices, and 'weight' is the edge length.
 */
struct Edge {
    int src, dest, weight;
};

typedef pair<int, int> Pair;  // (neighbor, weight)

/**
 * Graph class implementing an adjacency-list representation
 * of an undirected weighted graph.
 */
class Graph {
public:
    vector<vector<Pair>> adjList;  // adjacency list for each vertex

    /**
     * Stores the results of a BFS or DFS traversal, including:
     * - order: the visitation order
     * - prev: previous node on traversal path
     * - distFromPrev: edge weight from prev → node
     * - distFromStart: cumulative distance from the start vertex
     */
    struct TraversalResult {
        vector<int> order;
        vector<int> prev;
        vector<int> distFromPrev;
        vector<int> distFromStart;
    };

    /**
     * Constructs a graph from a set of edges and number of vertices.
     *
     * @param edges  Vector of edges defining the graph.
     * @param size   Number of vertices in the graph.
     */
    Graph(vector<Edge> const &edges, int size) {
        adjList.resize(size);
        for (auto &edge : edges) {
            adjList[edge.src].push_back(make_pair(edge.dest, edge.weight));
            adjList[edge.dest].push_back(make_pair(edge.src, edge.weight));
        }
    }

    /**
     * Prints the adjacency list of the graph.
     */
    void printGraph() {
        cout << "Graph's adjacency list:" << endl;
        for (int i = 0; i < (int)adjList.size(); i++) {
            cout << i << " --> ";
            for (Pair v : adjList[i])
                cout << "(" << i << ", " << v.first << ", " << v.second << ") ";
            cout << endl;
        }
    }

    /**
     * Performs Breadth-First Search (BFS) starting from 'start'.
     * Tracks visitation order and cumulative distances.
     *
     * @param start  Index of the starting vertex.
     * @return TraversalResult containing BFS order and distance info.
     */
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

        while (!q.empty()) {
            int v = q.front();
            q.pop();

            for (auto &p : adjList[v]) {
                int neighbor = p.first;
                int w = p.second;

                if (!visited[neighbor]) {
                    visited[neighbor] = true;
                    q.push(neighbor);

                    res.order.push_back(neighbor);
                    res.prev[neighbor] = v;
                    res.distFromPrev[neighbor] = w;
                    res.distFromStart[neighbor] = res.distFromStart[v] + w;
                }
            }
        }
        return res;
    }

    /**
     * Performs Depth-First Search (DFS) using an explicit stack.
     * Tracks visitation order and cumulative DFS-path distances.
     *
     * @param start  Index of the starting vertex.
     * @return TraversalResult containing DFS order and distance info.
     */
    TraversalResult DFS(int start) {
        int n = adjList.size();
        vector<bool> visited(n, false);

        TraversalResult res;
        res.prev.assign(n, -1);
        res.distFromPrev.assign(n, 0);
        res.distFromStart.assign(n, 0);

        typedef tuple<int, int, int, int> StackElem;
        stack<StackElem> s;

        s.push(make_tuple(start, -1, 0, 0));

        while (!s.empty()) {
            auto [v, parent, weightFromParent, distToV] = s.top();
            s.pop();

            if (visited[v]) continue;

            visited[v] = true;
            res.order.push_back(v);
            res.prev[v] = parent;
            res.distFromPrev[v] = weightFromParent;
            res.distFromStart[v] = distToV;

            for (int i = adjList[v].size() - 1; i >= 0; --i) {
                int neighbor = adjList[v][i].first;
                int w = adjList[v][i].second;

                if (!visited[neighbor]) {
                    s.push(make_tuple(neighbor, v, w, distToV + w));
                }
            }
        }
        return res;
    }
};

/**
 * Town class: wraps a Graph and provides building names and
 * BFS/DFS-based narrative investigation tools.
 */
class Town {
public:
    Graph graph;
    vector<string> idxToName;
    map<string, int> nameToIdx;

    /**
     * Constructs a Town with named buildings and road connections.
     *
     * @param buildingNames  Names for each building index.
     * @param edges          Roads connecting buildings.
     */
    Town(const vector<string>& buildingNames, const vector<Edge>& edges)
        : graph(edges, (int)buildingNames.size()), idxToName(buildingNames)
    {
        for (int i = 0; i < (int)buildingNames.size(); ++i)
            nameToIdx[buildingNames[i]] = i;
    }

    /**
     * Prints all buildings and the roads connecting them.
     */
    void printTownMap() {
        cout << "City Map (Buildings and Roads):" << endl;
        cout << "================================" << endl;
        for (int i = 0; i < (int)graph.adjList.size(); ++i) {
            cout << idxToName[i] << " (ID " << i << ") connects to:" << endl;
            for (auto &p : graph.adjList[i]) {
                cout << "  → " << idxToName[p.first]
                     << " (Road length: " << p.second << " units)" << endl;
            }
            cout << endl;
        }
    }

    /**
     * Runs a BFS investigation from a crime-scene building.
     * Prints detailed witness-checking information.
     *
     * @param startBuilding  Name of the building where BFS begins.
     */
    void investigateCrimeScene(const string &startBuilding) {
        auto it = nameToIdx.find(startBuilding);
        if (it == nameToIdx.end()) {
            cout << "Unknown start building: " << startBuilding << endl;
            return;
        }

        int start = it->second;
        auto res = graph.BFS(start);

        cout << "Witness Outreach (BFS) from " << idxToName[start] << " (Crime Scene):" << endl;
        cout << "========================================" << endl;
        cout << "Purpose: Question potential witnesses by proximity (layer-by-layer)\n" << endl;

        cout << "Checking Building: " << idxToName[start] << " (ID " << start << ")" << endl;
        cout << "  → Distance from previous: N/A" << endl;
        cout << "  → Total distance from start: 0" << endl << endl;

        for (int i = 1; i < (int)res.order.size(); ++i) {
            int node = res.order[i];
            int prev = res.prev[node];
            int dprev = res.distFromPrev[node];
            int dstart = res.distFromStart[node];

            cout << "Checking Building: " << idxToName[node] << " (ID " << node << ")" << endl;
            cout << "  → Reached from: " << (prev >= 0 ? idxToName[prev] : string("N/A"))
                 << " (edge length: " << dprev << " units)" << endl;
            cout << "  → Total distance from crime scene: " << dstart << " units" << endl;

            cout << "  → Nearby roads from this building:" << endl;
            for (auto &p : graph.adjList[node]) {
                cout << "      - " << idxToName[p.first]
                     << " (length: " << p.second << ")" << endl;
            }
            cout << endl;
        }

        cout << "BFS complete: total buildings considered = "
             << res.order.size() << endl << endl;
    }

    /**
     * Runs a DFS-based building trace to follow deep escape paths.
     *
     * @param startBuilding  Name of the building where DFS begins.
     */
    void traceEscapeRoute(const string &startBuilding) {
        auto it = nameToIdx.find(startBuilding);
        if (it == nameToIdx.end()) {
            cout << "Unknown start building: " << startBuilding << endl;
            return;
        }

        int start = it->second;
        auto res = graph.DFS(start);

        cout << "Escape Route Trace (DFS) from " << idxToName[start] << " (Crime Scene):" << endl;
        cout << "========================================" << endl;
        cout << "Purpose: Trace deep pursuit routes that a fleeing suspect might take\n" << endl;

        for (int i = 0; i < (int)res.order.size(); ++i) {
            int node = res.order[i];
            int prev = res.prev[node];
            int dprev = res.distFromPrev[node];
            int dstart = res.distFromStart[node];

            cout << "Investigating Building: " << idxToName[node] << " (ID " << node << ")" << endl;

            if (prev >= 0)
                cout << "  → Came from: " << idxToName[prev]
                     << " (edge length: " << dprev << " units)" << endl;
            else
                cout << "  → Origin (crime scene)" << endl;

            cout << "  → Cumulative distance along this DFS path: "
                 << dstart << " units" << endl;

            cout << "  → Possible next steps (roads out):" << endl;
            for (auto &p : graph.adjList[node]) {
                cout << "      - " << idxToName[p.first]
                     << " (length: " << p.second << ")" << endl;
            }
            cout << endl;
        }

        cout << "DFS complete: total buildings inspected in route trace = "
             << res.order.size() << endl << endl;
    }
};

/**
 * Entry point: creates the town, prints its map,
 * and runs both BFS and DFS story-based analyses.
 */
int main() {
    vector<string> buildingNames = {
        "City Hall", "Fire Station", "Central Plaza", "Library",
        "City Bank", "General Hospital", "Utility Control Center",
        "Shopping District", "Warehouse Depot", "Train Station",
        "Power Substation", "Apartment Complex", "Industrial Plant"
    };

    vector<Edge> edges = {
        {0, 2, 15},{0, 3, 7},{2, 3, 10},{2, 4, 3},{3, 5, 12},
        {4, 5, 8},{7, 0, 6},{7, 8, 9},{8, 2, 11},{8, 9, 4},
        {9, 10, 5},{10, 4, 14},{11, 3, 13},{11, 12, 2},{12, 5, 16},
        {1, 3, 6},{6, 10, 4}
    };

    Town myTown(buildingNames, edges);

    myTown.printTownMap();
    myTown.investigateCrimeScene("City Hall");
    myTown.traceEscapeRoute("City Hall");

    return 0;
}
