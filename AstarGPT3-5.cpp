#include <iostream>
#include <vector>
#include <unordered_map>
#include <queue>
#include <limits>
#include <algorithm>

using namespace std;

struct Node {
    char id;
    int heuristic;  // Fixed heuristic value for the node
    vector<pair<Node*, int>> neighbors; // pair: (neighbor, edge_cost)

    Node(char _id, int _heuristic) : id(_id), heuristic(_heuristic) {}
};

class AStar {
public:
    unordered_map<char, Node*> nodes;
    unordered_map<char, char> cameFrom;

    void addEdge(char from, char to, int cost, int h = 0, int h2 = 0) {
        if (!nodes.count(from)) nodes[from] = new Node{from, h};  // Set heuristic to 0 by default
        if (!nodes.count(to)) nodes[to] = new Node{to, h2};

        nodes[from]->neighbors.emplace_back(nodes[to], cost);
        nodes[to]->neighbors.emplace_back(nodes[from], cost);
    }

    int astar(char start, char goal) {
        priority_queue<pair<int, char>, vector<pair<int, char>>, greater<pair<int, char>>> openSet;
        unordered_map<char, int> gScore;

        for (auto& entry : nodes) {
            gScore[entry.first] = numeric_limits<int>::max();
        }

        gScore[start] = 0;
        openSet.emplace(heuristic(start, goal), start);

        while (!openSet.empty()) {
            char current = openSet.top().second;
            openSet.pop();

            if (current == goal) {
                // Path found, reconstruct and print it
                // cout << "Path: ";
                reconstructPath(goal);
                return gScore[goal];
            }

            for (auto& neighbor : nodes[current]->neighbors) {
                int tentative_gScore = gScore[current] + neighbor.second;

                if (tentative_gScore < gScore[neighbor.first->id]) {
                    gScore[neighbor.first->id] = tentative_gScore;
                    cameFrom[neighbor.first->id] = current;

                    openSet.emplace(tentative_gScore + heuristic(neighbor.first->id, goal), neighbor.first->id);
                }
            }
        }

        // If the loop completes without finding a path
        // cout << "No path found!" << endl;
        return -1;
    }

    void reconstructPath(char current) {
        if (cameFrom.find(current) != cameFrom.end()) {
            reconstructPath(cameFrom[current]);
        }
        // cout << current << " ";
    }

    int heuristic(char current, char goal) {
        // For simplicity, you can use a fixed heuristic value for every node
        return nodes[current]->heuristic;
    }
};

int main() {
    AStar graph;

    // Add edges to the graph
    graph.addEdge('A', 'B', 2, 9, 3);
    graph.addEdge('A', 'C', 10, 9, 5);
    graph.addEdge('A', 'D', 3, 9, 6);
    graph.addEdge('B', 'E', 8, 3, 8);
    graph.addEdge('C', 'G', 2, 5, 2);
    graph.addEdge('D', 'F', 4, 6, 4);
    graph.addEdge('D', 'C', 2);
    graph.addEdge('G', 'H', 2, 2, 0);
    graph.addEdge('F', 'G', 5);
    graph.addEdge('E', 'F', 5);
    graph.addEdge('E', 'H', 10);

    char start = 'A';
    char goal = 'H';

    int cost = 0;

    cost += graph.astar(start, goal);
    std::cout << "Total Cost: " << cost << endl;

    return 0;
}
