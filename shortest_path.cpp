/**
 * Shortest Path Algorithm Implementation
 * Includes: Bellman-Ford Algorithm & Dijkstra's Algorithm
 * 
 * Compile: g++ -std=c++17 -o shortest_path shortest_path.cpp
 * Run: ./shortest_path
 */

#include <iostream>
#include <vector>
#include <queue>
#include <climits>
#include <iomanip>
#include <algorithm>

using namespace std;

// Define infinity constant to represent unreachable nodes
const int INF = INT_MAX;

/**
 * Edge Structure
 * Stores the source, destination, and weight of an edge
 */
struct Edge {
    int from;    // Source vertex
    int to;      // Destination vertex
    int weight;  // Edge weight
    
    Edge(int f, int t, int w) : from(f), to(t), weight(w) {}
};

/**
 * Graph Class
 * Supports creation of directed and undirected graphs
 * with shortest path computation
 */
class Graph {
private:
    int numVertices;                         // Number of vertices
    vector<Edge> edges;                      // Edge list (for Bellman-Ford)
    vector<vector<pair<int, int>>> adjList;  // Adjacency list (for Dijkstra)
    bool isDirected;                         // Whether the graph is directed

public:
    /**
     * Constructor
     * @param n Number of vertices (numbered from 0 to n-1)
     * @param directed Whether the graph is directed (default: true)
     */
    Graph(int n, bool directed = true) : numVertices(n), isDirected(directed) {
        adjList.resize(n);
    }

    /**
     * Add an edge to the graph
     * @param from Source vertex
     * @param to Destination vertex
     * @param weight Edge weight
     */
    void addEdge(int from, int to, int weight) {
        // Add to edge list
        edges.emplace_back(from, to, weight);
        // Add to adjacency list
        adjList[from].emplace_back(to, weight);
        
        // For undirected graphs, add the reverse edge
        if (!isDirected) {
            edges.emplace_back(to, from, weight);
            adjList[to].emplace_back(from, weight);
        }
    }

    /**
     * Get the number of vertices
     */
    int getNumVertices() const {
        return numVertices;
    }

    /**
     * Get the edge list
     */
    const vector<Edge>& getEdges() const {
        return edges;
    }

    /**
     * Get the adjacency list
     */
    const vector<vector<pair<int, int>>>& getAdjList() const {
        return adjList;
    }

    /**
     * Print the graph structure
     */
    void printGraph() const {
        cout << "\n===== Graph Structure =====\n";
        cout << "Number of vertices: " << numVertices << "\n";
        cout << "Graph type: " << (isDirected ? "Directed" : "Undirected") << "\n";
        cout << "\nAdjacency List:\n";
        
        for (int i = 0; i < numVertices; i++) {
            cout << "Vertex " << i << " -> ";
            if (adjList[i].empty()) {
                cout << "(no outgoing edges)";
            } else {
                for (size_t j = 0; j < adjList[i].size(); j++) {
                    cout << adjList[i][j].first << "(w:" << adjList[i][j].second << ")";
                    if (j < adjList[i].size() - 1) cout << ", ";
                }
            }
            cout << "\n";
        }
        cout << "===========================\n";
    }
};

/**
 * Helper function to print shortest path results
 * @param source Source vertex
 * @param dist Distance array
 * @param parent Parent array for path reconstruction
 * @param algorithmName Name of the algorithm
 */
void printResult(int source, const vector<int>& dist, const vector<int>& parent, 
                 const string& algorithmName) {
    int n = dist.size();
    
    cout << "\n===== " << algorithmName << " Results =====\n";
    cout << "Source: Vertex " << source << "\n\n";
    cout << left << setw(10) << "Dest" << setw(14) << "Distance" << "Path\n";
    cout << string(55, '-') << "\n";
    
    for (int i = 0; i < n; i++) {
        cout << left << setw(10) << i;
        
        if (dist[i] == INF) {
            cout << setw(14) << "UNREACHABLE" << "-\n";
        } else {
            cout << setw(14) << dist[i];
            
            // Reconstruct path by backtracking
            vector<int> path;
            int current = i;
            while (current != -1) {
                path.push_back(current);
                current = parent[current];
            }
            
            // Reverse to get correct order
            reverse(path.begin(), path.end());
            
            // Print the path
            for (size_t j = 0; j < path.size(); j++) {
                cout << path[j];
                if (j < path.size() - 1) cout << " -> ";
            }
            cout << "\n";
        }
    }
    cout << string(55, '=') << "\n";
}

/**
 * Bellman-Ford Algorithm
 * 
 * Core Concept:
 * 1. Perform V-1 relaxation iterations over all edges
 * 2. The V-th iteration checks for negative cycles
 * 
 * Time Complexity: O(VE)
 * Space Complexity: O(V)
 * 
 * @param graph Graph object
 * @param source Source vertex
 * @return true if successful, false if negative cycle exists
 */
bool bellmanFord(const Graph& graph, int source) {
    int n = graph.getNumVertices();
    const vector<Edge>& edges = graph.getEdges();
    
    // Initialize distance and parent arrays
    vector<int> dist(n, INF);
    vector<int> parent(n, -1);
    
    // Distance from source to itself is 0
    dist[source] = 0;
    
    cout << "\n[Bellman-Ford] Starting execution...\n";
    cout << "Performing " << n - 1 << " relaxation iterations\n";
    
    // Perform V-1 relaxation iterations
    // Principle: Shortest path contains at most V-1 edges
    for (int i = 1; i <= n - 1; i++) {
        bool updated = false;
        
        // Relax all edges
        for (const Edge& e : edges) {
            // Relaxation: if source is reachable and path through this edge is shorter
            if (dist[e.from] != INF && dist[e.from] + e.weight < dist[e.to]) {
                dist[e.to] = dist[e.from] + e.weight;
                parent[e.to] = e.from;
                updated = true;
            }
        }
        
        // Optimization: early termination if no updates in this iteration
        if (!updated) {
            cout << "No updates after iteration " << i << ", terminating early\n";
            break;
        }
    }
    
    // V-th relaxation: check for negative cycles
    // Principle: if relaxation is still possible, a negative cycle exists
    cout << "Checking for negative cycles...\n";
    for (const Edge& e : edges) {
        if (dist[e.from] != INF && dist[e.from] + e.weight < dist[e.to]) {
            cout << "WARNING: Negative cycle detected! Cannot compute shortest paths.\n";
            return false;
        }
    }
    
    cout << "OK: No negative cycle found, computation complete\n";
    
    // Print results
    printResult(source, dist, parent, "Bellman-Ford Algorithm");
    
    return true;
}

/**
 * Dijkstra's Algorithm
 * 
 * Core Concept:
 * 1. Use a priority queue (min-heap) to select the unprocessed vertex with minimum distance
 * 2. Relax all neighbors of the selected vertex
 * 3. Repeat until all vertices are processed
 * 
 * Time Complexity: O((V+E) log V) with priority queue
 * Space Complexity: O(V)
 * 
 * NOTE: This algorithm does NOT work with negative edge weights!
 * 
 * @param graph Graph object
 * @param source Source vertex
 */
void dijkstra(const Graph& graph, int source) {
    int n = graph.getNumVertices();
    const vector<vector<pair<int, int>>>& adjList = graph.getAdjList();
    
    // Initialize distance and parent arrays
    vector<int> dist(n, INF);
    vector<int> parent(n, -1);
    vector<bool> visited(n, false);  // Track processed vertices
    
    // Distance from source to itself is 0
    dist[source] = 0;
    
    // Priority queue: (distance, vertex)
    // Using greater<> to create a min-heap (smaller distance = higher priority)
    priority_queue<pair<int, int>, vector<pair<int, int>>, greater<pair<int, int>>> pq;
    pq.push({0, source});
    
    cout << "\n[Dijkstra] Starting execution...\n";
    
    int processedCount = 0;
    
    while (!pq.empty()) {
        // Extract vertex with minimum distance
        auto [d, u] = pq.top();
        pq.pop();
        
        // Skip if already processed (avoid duplicate processing)
        if (visited[u]) continue;
        
        // Mark as processed
        visited[u] = true;
        processedCount++;
        
        cout << "Processing vertex " << u << " (current distance: " << d << ")\n";
        
        // Relax all neighbors
        for (const auto& [v, weight] : adjList[u]) {
            // If path through u is shorter
            if (!visited[v] && dist[u] + weight < dist[v]) {
                dist[v] = dist[u] + weight;
                parent[v] = u;
                // Add updated distance to priority queue
                pq.push({dist[v], v});
            }
        }
    }
    
    cout << "OK: Completed, processed " << processedCount << " vertices\n";
    
    // Print results
    printResult(source, dist, parent, "Dijkstra's Algorithm");
}

/**
 * Create a sample graph for testing (no negative weights)
 * 
 * Graph Structure (Directed, 7 edges):
 * 
 *        1
 *       /|\
 *     4/ | \2
 *     /  |  \
 *    0   5   2
 *     \     /
 *     2\   /1
 *       \ /
 *        4---3---3
 *            1
 */
Graph createSampleGraph() {
    Graph g(5, true);  // 5 vertices, directed graph
    
    g.addEdge(0, 1, 4);   // 0 -> 1, weight 4
    g.addEdge(0, 4, 2);   // 0 -> 4, weight 2
    g.addEdge(1, 2, 2);   // 1 -> 2, weight 2
    g.addEdge(1, 4, 5);   // 1 -> 4, weight 5
    g.addEdge(4, 2, 1);   // 4 -> 2, weight 1
    g.addEdge(2, 3, 1);   // 2 -> 3, weight 1
    g.addEdge(4, 3, 3);   // 4 -> 3, weight 3
    
    return g;
}

/**
 * Create a sample graph with negative edge weights
 * (for testing Bellman-Ford's advantage over Dijkstra)
 */
Graph createNegativeWeightGraph() {
    Graph g(4, true);  // 4 vertices, directed graph
    
    g.addEdge(0, 1, 4);    // 0 -> 1, weight 4
    g.addEdge(0, 2, 5);    // 0 -> 2, weight 5
    g.addEdge(1, 2, -3);   // 1 -> 2, weight -3 (NEGATIVE!)
    g.addEdge(2, 3, 4);    // 2 -> 3, weight 4
    
    return g;
}

/**
 * Interactive graph input from user
 */
Graph inputGraph() {
    int n, m;
    bool directed;
    
    cout << "\n===== Create New Graph =====\n";
    cout << "Enter number of vertices: ";
    cin >> n;
    
    cout << "Enter number of edges: ";
    cin >> m;
    
    cout << "Is this a directed graph? (1: Yes, 0: No): ";
    cin >> directed;
    
    Graph g(n, directed);
    
    cout << "\nEnter each edge (format: source destination weight):\n";
    cout << "(Vertex numbers start from 0, weights can be negative)\n";
    for (int i = 0; i < m; i++) {
        int from, to, weight;
        cout << "Edge " << i + 1 << ": ";
        cin >> from >> to >> weight;
        g.addEdge(from, to, weight);
    }
    
    return g;
}

/**
 * Main Program
 */
int main() {
    cout << "+----------------------------------------------------------+\n";
    cout << "|          Shortest Path Algorithm Implementation          |\n";
    cout << "|        Bellman-Ford & Dijkstra's Algorithm               |\n";
    cout << "+----------------------------------------------------------+\n";
    
    Graph* currentGraph = nullptr;
    
    while (true) {
        cout << "\n========== Main Menu ==========\n";
        cout << "1. Load sample graph (no negative weights)\n";
        cout << "2. Load sample graph with negative weights\n";
        cout << "3. Input custom graph\n";
        cout << "4. Display current graph structure\n";
        cout << "5. Run Bellman-Ford Algorithm\n";
        cout << "6. Run Dijkstra's Algorithm\n";
        cout << "7. Run both algorithms (compare results)\n";
        cout << "0. Exit\n";
        cout << "================================\n";
        cout << "Enter your choice: ";
        
        int choice;
        cin >> choice;
        
        switch (choice) {
            case 0:
                cout << "\nThank you for using this program. Goodbye!\n";
                delete currentGraph;
                return 0;
                
            case 1:
                delete currentGraph;
                currentGraph = new Graph(createSampleGraph());
                cout << "\nOK: Sample graph loaded (no negative weights)\n";
                currentGraph->printGraph();
                break;
                
            case 2:
                delete currentGraph;
                currentGraph = new Graph(createNegativeWeightGraph());
                cout << "\nOK: Sample graph with negative weights loaded\n";
                cout << "WARNING: This graph contains negative edges. Dijkstra may give incorrect results!\n";
                currentGraph->printGraph();
                break;
                
            case 3:
                delete currentGraph;
                currentGraph = new Graph(inputGraph());
                cout << "\nOK: Custom graph created\n";
                currentGraph->printGraph();
                break;
                
            case 4:
                if (currentGraph == nullptr) {
                    cout << "\nERROR: Please create or load a graph first!\n";
                } else {
                    currentGraph->printGraph();
                }
                break;
                
            case 5:
                if (currentGraph == nullptr) {
                    cout << "\nERROR: Please create or load a graph first!\n";
                } else {
                    int source;
                    cout << "Enter source vertex (0 ~ " << currentGraph->getNumVertices() - 1 << "): ";
                    cin >> source;
                    
                    if (source < 0 || source >= currentGraph->getNumVertices()) {
                        cout << "ERROR: Invalid vertex number!\n";
                    } else {
                        bellmanFord(*currentGraph, source);
                    }
                }
                break;
                
            case 6:
                if (currentGraph == nullptr) {
                    cout << "\nERROR: Please create or load a graph first!\n";
                } else {
                    int source;
                    cout << "Enter source vertex (0 ~ " << currentGraph->getNumVertices() - 1 << "): ";
                    cin >> source;
                    
                    if (source < 0 || source >= currentGraph->getNumVertices()) {
                        cout << "ERROR: Invalid vertex number!\n";
                    } else {
                        cout << "\nWARNING: Dijkstra's algorithm does NOT work with negative edge weights!\n";
                        dijkstra(*currentGraph, source);
                    }
                }
                break;
                
            case 7:
                if (currentGraph == nullptr) {
                    cout << "\nERROR: Please create or load a graph first!\n";
                } else {
                    int source;
                    cout << "Enter source vertex (0 ~ " << currentGraph->getNumVertices() - 1 << "): ";
                    cin >> source;
                    
                    if (source < 0 || source >= currentGraph->getNumVertices()) {
                        cout << "ERROR: Invalid vertex number!\n";
                    } else {
                        cout << "\n+--------------------------------------+\n";
                        cout << "|      Algorithm Comparison Results    |\n";
                        cout << "+--------------------------------------+\n";
                        cout << "NOTE: If graph contains negative edges, Dijkstra may give incorrect results\n";
                        
                        // Run both algorithms
                        bellmanFord(*currentGraph, source);
                        dijkstra(*currentGraph, source);
                    }
                }
                break;
                
            default:
                cout << "\nERROR: Invalid option. Please try again!\n";
        }
    }
    
    return 0;
}