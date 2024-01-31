import nltk
from nltk.translate.bleu_score import sentence_bleu, SmoothingFunction

# Supprimez cette ligne si vous n'avez pas encore téléchargé les ressources NLTK nécessaires.
nltk.download('punkt')

# Phrases de référence et candidate
reference = """#include <iostream>
#include <vector>
#include <list>

#define INF 9999

struct Node;

struct Link {

    Node* next;
    int cost;

    Link(Node* n, int c) {

        next = n;
        cost = c;

    }

};

struct Node {

    char id;
    int heuristic;
    int total_cost;
    int total_heuristic;
    std::vector<Link> links;
    Node* prev;

    Node(char i, int h) {

        id = i;
        heuristic = h;
        total_cost = INF;
        total_heuristic = INF;

    }

    void connect(Node* n, int c) {

        links.push_back(Link(n, c));

    }

};

void insert_asc(std::list<Node*> *list, Node* elt) {

    int id = 0;
    std::list<Node*> copy = *list;

    while(!copy.empty()) {

        Node* current = copy.front();
        copy.pop_front();

        if(elt->total_cost < current->total_cost) {

            break;

        }

        id++;

    }

    auto it = list->begin();

    for(int i = 0; i < id; i++) {
        it++;
    }

    list->insert(it, elt);

}

bool in(std::vector<Node*> *vec, Node* elt) {

    for(unsigned long int i = 0; i < vec->size(); i++) {
        if((*vec)[i] == elt) return true;
    }

    return false;

}

void print_path(Node* graph, Node* end) {

    std::cout << "Path cost: " << end->total_cost << "\n";
    Node* current = end;

    while(current != graph) {

        std::cout << current->id << " <- ";
        current = current->prev;

    }

    std::cout << current->id << "\n";

}

void explore(Node* graph, Node* end) {

    std::list<Node*> path;
    std::list<Node*> frontier;
    std::vector<Node*> explored;

    graph->total_cost = 0;
    graph->total_heuristic = graph->heuristic;
    frontier.push_back(graph);

    while(!frontier.empty()) {

        Node* current = frontier.front();
        frontier.pop_front();
        explored.push_back(current);

        path.push_back(current);

        if(end == current) {
            return print_path(graph, current);
        }

        for(auto link : current->links) {

            int cost = current->total_cost + link.cost;
            if(cost < link.next->total_cost) {
                link.next->prev = current;
                link.next->total_cost = cost;
                link.next->total_heuristic = cost + link.next->heuristic;
            }
            if(!in(&explored, link.next)) insert_asc(&frontier, link.next);

        }

    }

}

int main() {

    Node A('A', 9);
    Node B('B', 3);
    Node C('C', 5);
    Node D('D', 6);
    Node E('E', 8);
    Node F('F', 4);
    Node G('G', 2);
    Node H('H', 0);

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

    Node* graph = &A;

    explore(graph, &H);

    return 0;

}"""
candidate = """#include <iostream>
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