// compile with g++ -Wall -O3 -o AstarArthur1 AstarArthur1.c++

#include <deque>
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
}

