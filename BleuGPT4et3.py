import nltk
from nltk.translate.bleu_score import sentence_bleu, SmoothingFunction

# Supprimez cette ligne si vous n'avez pas encore téléchargé les ressources NLTK nécessaires.
nltk.download('punkt')

# Phrases de référence et candidate
reference = """#include <iostream>
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
}"""
candidate = """#include <iostream>
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
}"""

# Tokenize les phrases en listes de mots
reference_tokens = nltk.word_tokenize(reference.lower())
candidate_tokens = nltk.word_tokenize(candidate.lower())

# Paramètre n pour les trigrammes
n = 1

# Calcul du score BLEU avec fonction de lissage et trigrammes
smoothie = SmoothingFunction().method4
bleu_score = sentence_bleu([reference_tokens], candidate_tokens, smoothing_function=smoothie, weights=(1/n,)*n)

#print(f"Phrases de référence tokenisées : {reference_tokens}")
#print(f"Phrases candidates tokenisées : {candidate_tokens}")
print(f"Score BLEU (trigrammes) : {bleu_score}")