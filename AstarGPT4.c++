// GPT4-TURBO

#include <iostream>
#include <vector>
#include <queue>
#include <map>
#include <unordered_set>

class Node {
public:
    char name;
    int heuristic;
    std::vector<std::pair<Node*, int>> connections; // Pair of adjacent node and cost

    Node(char n, int h) : name(n), heuristic(h) {}

    void connect(Node* node, int cost) {
        connections.push_back(std::make_pair(node, cost));
    }
};

class Path {
public:
    int cost;
    std::vector<Node*> nodes;

    Path() : cost(0) {}

    Path(const Path& p) : cost(p.cost), nodes(p.nodes) {}

    void addNode(Node* node, int cost) {
        nodes.push_back(node);
        this->cost += cost;
    }
};

struct PathCostComparator {
    bool operator()(const Path& a, const Path& b) const {
        return (a.cost + a.nodes.back()->heuristic) > (b.cost + b.nodes.back()->heuristic);
    }
};

Path AStarSearch(Node* start, Node* goal) {
    std::priority_queue<Path, std::vector<Path>, PathCostComparator> frontier;
    std::unordered_set<char> explored;
    Path initial;
    initial.addNode(start, 0);
    frontier.push(initial);

    while (!frontier.empty()) {
        Path current = frontier.top();
        frontier.pop();
        Node* currentNode = current.nodes.back();

        if (currentNode == goal) {
            return current;
        }

        if (explored.find(currentNode->name) != explored.end()) {
            continue;
        }

        explored.insert(currentNode->name);

        for (auto& edge : currentNode->connections) {
            Node* nextNode = edge.first;
            int cost = edge.second;
            if (explored.find(nextNode->name) == explored.end()) {
                Path newPath(current);
                newPath.addNode(nextNode, cost);
                frontier.push(newPath);
            }
        }
    }

    return Path(); // Return empty path if goal is not reachable
}

int main() {
    // Nodes, with their names and heuristic
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

    // Perform A* search from node A to node H
    Path path = AStarSearch(&A, &H);

    if (path.nodes.empty()) {
        std::cout << "No path found!" << std::endl;
    } else {
        std::cout << "Path found: ";
        for (Node* node : path.nodes) {
            std::cout << node->name << " ";
        }
        std::cout << "\nCost: " << path.cost << std::endl;
    }

    return 0;
}