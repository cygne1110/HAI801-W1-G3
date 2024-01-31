// compile with g++ -Wall -O3 -o AstarArthur2 AstarArthur2.c++

#include <iostream>
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

}