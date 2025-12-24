# Shortest Path Algorithm Implementation

A C++ implementation of two classic shortest path algorithms: **Bellman-Ford** and **Dijkstra's Algorithm**.

## Features

- Support for both **directed** and **undirected** graphs
- Interactive command-line interface
- Path reconstruction with full route display
- Negative cycle detection (Bellman-Ford)
- Sample graphs included for testing
- Custom graph input support

## Algorithm Comparison

| Feature | Bellman-Ford | Dijkstra |
|---------|--------------|----------|
| **Time Complexity** | O(VE) | O((V+E) log V) |
| **Space Complexity** | O(V) | O(V) |
| **Negative Weights** | ✅ Supported | ❌ Not Supported |
| **Negative Cycle Detection** | ✅ Yes | ❌ No |
| **Best Use Case** | Graphs with negative edges | Non-negative weight graphs |

## Requirements

- C++17 or later
- g++ compiler (or any C++17 compatible compiler)

## Compilation

```bash
g++ -std=c++17 -o shortest_path shortest_path.cpp
```

## Usage

```bash
./shortest_path
```

### Menu Options

```
1. Load sample graph (no negative weights)
2. Load sample graph with negative weights
3. Input custom graph
4. Display current graph structure
5. Run Bellman-Ford Algorithm
6. Run Dijkstra's Algorithm
7. Run both algorithms (compare results)
0. Exit
```

## Sample Graphs

### Graph 1: No Negative Weights (5 vertices, 7 edges)

```
        1
       /|\
     4/ | \2
     /  |  \
    0   5   2
     \     /
     2\   /1
       \ /
        4---3
           \
            3
```

**Edges:**
- 0 → 1 (weight: 4)
- 0 → 4 (weight: 2)
- 1 → 2 (weight: 2)
- 1 → 4 (weight: 5)
- 4 → 2 (weight: 1)
- 2 → 3 (weight: 1)
- 4 → 3 (weight: 3)

### Graph 2: With Negative Weights (4 vertices, 4 edges)

**Edges:**
- 0 → 1 (weight: 4)
- 0 → 2 (weight: 5)
- 1 → 2 (weight: **-3**) ← Negative edge
- 2 → 3 (weight: 4)

## Example Output

```
===== Bellman-Ford Algorithm Results =====
Source: Vertex 0

Dest      Distance      Path
-------------------------------------------------------
0         0             0
1         4             0 -> 1
2         3             0 -> 4 -> 2
3         4             0 -> 4 -> 2 -> 3
4         2             0 -> 4
=======================================================
```

## Custom Graph Input

When selecting option 3, you can input your own graph:

1. Enter the number of vertices
2. Enter the number of edges
3. Specify if the graph is directed (1) or undirected (0)
4. Enter each edge in the format: `source destination weight`

**Example:**
```
Enter number of vertices: 4
Enter number of edges: 5
Is this a directed graph? (1: Yes, 0: No): 1

Enter each edge (format: source destination weight):
Edge 1: 0 1 10
Edge 2: 0 2 5
Edge 3: 1 2 2
Edge 4: 1 3 1
Edge 5: 2 3 4
```

## Algorithm Details

### Bellman-Ford Algorithm

**Principle:** Relaxes all edges V-1 times, where V is the number of vertices.

```
for i = 1 to V-1:
    for each edge (u, v) with weight w:
        if dist[u] + w < dist[v]:
            dist[v] = dist[u] + w
            parent[v] = u
```

**Negative Cycle Detection:** If any edge can still be relaxed after V-1 iterations, a negative cycle exists.

**Optimization:** Early termination if no updates occur in an iteration.

### Dijkstra's Algorithm

**Principle:** Greedily selects the unvisited vertex with the smallest distance and relaxes its neighbors.

```
while priority_queue is not empty:
    u = vertex with minimum distance
    mark u as visited
    for each neighbor v of u:
        if dist[u] + weight(u,v) < dist[v]:
            dist[v] = dist[u] + weight(u,v)
            parent[v] = u
```

**Data Structure:** Uses a min-heap (priority queue) for efficient vertex selection.

**Limitation:** Does not work correctly with negative edge weights.

## Project Structure

```
.
├── shortest_path.cpp    # Main source file
└── README.md            # This file
```

## Classes and Functions

| Component | Description |
|-----------|-------------|
| `struct Edge` | Represents an edge with source, destination, and weight |
| `class Graph` | Graph representation with edge list and adjacency list |
| `bellmanFord()` | Bellman-Ford algorithm implementation |
| `dijkstra()` | Dijkstra's algorithm implementation |
| `printResult()` | Helper function to display shortest path results |
| `createSampleGraph()` | Creates a sample graph without negative weights |
| `createNegativeWeightGraph()` | Creates a sample graph with negative weights |
| `inputGraph()` | Interactive graph input from user |

## License

This project is open source and available for educational purposes.

## Author

Created for educational demonstration of shortest path algorithms.