import nltk
from nltk.translate.bleu_score import sentence_bleu, SmoothingFunction

# Supprimez cette ligne si vous n'avez pas encore téléchargé les ressources NLTK nécessaires.
nltk.download('punkt')

# Phrases de référence et candidate
reference = """#include <deque>
#include <memory>
#include <iostream>

class Node;

typedef std::shared_ptr<Node> NodeRef;

struct NodePath
{
    int cost;
    NodeRef A, B;
};

class Node
{
    private : 
    public : 


        char id;
        int heuristic;
        std::deque<NodePath> paths;

        static void addPath(NodeRef A, NodeRef B, int cost)
        {
            NodePath path;
            path.cost = cost;
            path.A = A;
            path.B = B;
            A->paths.push_back(path);
            B->paths.push_back(path);
        }

        NodeRef follow(NodePath path)
        {
            if(path.A.get() != this && path.B.get() != this)
            {
                std::cout << "yooooo\n";
                int *i = nullptr;
                std::cout << *i;
            }

            return path.A.get() == this ? path.B : path.A;
        };

        Node(int id, int heuristic) : id(id), heuristic(heuristic){};
};

struct AstarNode
{
    int hg;
    int g;
    std::deque<NodeRef> Paths;
};

void computeAstar(std::deque<NodeRef> &nodes)
{
    NodeRef currentNode = nodes[0];
    NodeRef target = nodes[nodes.size()-1];

    std::deque<AstarNode> explored;

    AstarNode newNode;
    newNode.Paths.push_back(currentNode);
    newNode.g = 0;
    explored.push_back(newNode);
    AstarNode* bestPath = &explored.front();

    while(currentNode != target)
    {
        std::deque<NodePath> &p = bestPath->Paths.back()->paths;

        for(size_t i = 0; i < p.size()-1; i++)
        {
            AstarNode newNode = *bestPath;
            newNode.Paths.push_back(currentNode->follow(p[i]));
            newNode.g += p[i].cost;
            newNode.hg = newNode.g + newNode.Paths.back()->heuristic;
            explored.push_back(newNode);
        }

        if(p.size())
        {
            bestPath->Paths.push_back(currentNode->follow(p.back()));
            bestPath->g += p.back().cost;
            bestPath->hg = bestPath->g + bestPath->Paths.back()->heuristic;
        }

        for(auto &i : explored)
            if(i.hg <= bestPath->hg)
                bestPath = &i;

        currentNode = bestPath->Paths.back();
    }

    std::cout << "finished\n";
    std::cout << bestPath->Paths.size() << "\n";
    std::cout << bestPath->hg << "\n";

    for(auto i : bestPath->Paths)
    {
        std::cout << i->id << " ==> ";
    }
    std::cout << "finish\n";
}


int main()
{
    NodeRef A(new Node('A', 9));
    NodeRef B(new Node('B', 3));
    NodeRef C(new Node('C', 5));
    NodeRef D(new Node('D', 6));
    NodeRef E(new Node('E', 8));
    NodeRef F(new Node('F', 4));
    NodeRef G(new Node('G', 2));
    NodeRef H(new Node('H', 0));

    Node::addPath(A, B, 2);
    Node::addPath(A, C, 10);
    Node::addPath(A, D, 3);
    Node::addPath(B, E, 8);
    Node::addPath(C, G, 2);
    Node::addPath(D, F, 4);
    Node::addPath(D, C, 2);
    Node::addPath(G, H, 1);
    Node::addPath(F, G, 5);
    Node::addPath(E, F, 5);
    Node::addPath(E, H, 10);
    Node::addPath(G, H, 1);

    std::deque<NodeRef> nodes{A, B, C, D, E, F, G, H};

    computeAstar(nodes);
}"""
candidate = """#include <iostream> 
#include <vector> 
#include <list>

#define INF 9999

struct Node; struct Link {

    Node* next; int cost; Link(Node* n, int c) {

        next = n; cost = c;

    }

};

struct Node {

    char id; int heuristic; int total_cost; int total_heuristic; std::vector<Link> links; Node* prev;

    Node(char i, int h) {

        id = i; heuristic = h; total_cost = INF; total_heuristic = INF;

    }

    void connect(Node* n, int c) {

        links.push_back(Link(n, c));

    }

};

void insert_asc(std::list<Node*> *list, Node* elt) {

    int id = 0; std::list<Node*> copy = *list;

    while(!copy.empty()) {

        Node* current = copy.front(); copy.pop_front();

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


# Tokenize les phrases en listes de mots
reference_tokens = nltk.word_tokenize(reference.lower())
candidate_tokens = nltk.word_tokenize(candidate.lower())

# Paramètre n pour les trigrammes
n = 5

# Calcul du score BLEU avec fonction de lissage et trigrammes
smoothie = SmoothingFunction().method4
bleu_score = sentence_bleu([reference_tokens], candidate_tokens, smoothing_function=smoothie, weights=(1/n,)*n)

#print(f"Phrases de référence tokenisées : {reference_tokens}")
#print(f"Phrases candidates tokenisées : {candidate_tokens}")
print(f"Score BLEU (trigrammes) : {bleu_score}")