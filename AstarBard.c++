// bard-jan-24-gemini-pro

#include <iostream>
#include <queue>
#include <unordered_map>
#include <vector>

using namespace std;

struct Node {
    char name;
    int heuristic;
    vector<pair<Node*, int>> connections;

    Node(char name, int heuristic) : name(name), heuristic(heuristic) {}

    void connect(Node* node, int cost) {
        connections.push_back({node, cost});
    }
};

// Function to find the cheapest path using A*
vector<Node*> aStar(Node* start, Node* goal) {
    priority_queue<pair<int, Node*>, vector<pair<int, Node*>>, greater<>> openSet;
    unordered_map<Node*, int> gScore;
    unordered_map<Node*, Node*> cameFrom;

    openSet.push({0, start});
    gScore[start] = 0;

    while (!openSet.empty()) {
        Node* current = openSet.top().second;
        openSet.pop();

        if (current == goal) {
            // Reconstruct the path
            vector<Node*> path;
            Node* node = goal;
            while (node != start) {
                path.push_back(node);
                node = cameFrom[node];
            }
            path.push_back(start);
            std::cout << "Cost: " << gScore[current] << "\n";
            return path;
        }

        for (auto& neighbor : current->connections) {
            int tentativeGScore = gScore[current] + neighbor.second;
            if (!gScore.count(neighbor.first) || tentativeGScore < gScore[neighbor.first]) {
                cameFrom[neighbor.first] = current;
                gScore[neighbor.first] = tentativeGScore;
                int fScore = tentativeGScore + neighbor.first->heuristic;
                openSet.push({fScore, neighbor.first});
            }
        }
    }

    return {}; // No path found
}

int main() {
    // Your graph definition here
    // ...
        Node A('A', 9);
    Node B('B', 3);
    Node C('C', 5);
    Node D('D', 6);
    Node E('E', 8);
    Node F('F', 4);
    Node G('G', 2);
    Node H('H', 0);

    // Different paths with their destination and cost
    A.connect(&B, 2);
    A.connect(&D, 3);
    A.connect(&C, 10);

    B.connect(&A, 2);
    B.connect(&E, 8);

    C.connect(&A, 10);
    C.connect(&G, 2);
    C.connect(&D, 2);

    D.connect(&A, 3);
    D.connect(&C, 2);
    D.connect(&F, 4);

    E.connect(&B, 8);
    E.connect(&F, 5);
    E.connect(&H, 10);

    F.connect(&E, 5);
    F.connect(&G, 5);
    F.connect(&D, 4);

    G.connect(&C, 2);
    G.connect(&F, 5);
    G.connect(&H, 1);

    H.connect(&E, 10);
    H.connect(&G, 1);

    Node* start = &A;
    Node* goal = &H;

    vector<Node*> path = aStar(start, goal);

    if (path.empty()) {
        cout << "No path found." << endl;
    } else {
        cout << "Cheapest path: ";
        for (Node* node : path) {
            cout << node->name << " ";
        }
        cout << endl;
    }

    return 0;
}