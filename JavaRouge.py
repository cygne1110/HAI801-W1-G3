from rouge import Rouge

#Compare Java et ArthurC++1

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
candidate = """import java.util.Queue;
import java.util.PriorityQueue;
import java.util.Stack;
import java.util.Comparator;
import java.util.LinkedList;
import java.util.Map;
import java.util.HashMap;

public class Astar {
    
    private Queue<Coord2D> fetchVoisins(Coord2D node) {
        Queue<Coord2D> res = new LinkedList<Coord2D>();
        if(node.x - 1 > 0) {
            res.add(new Coord2D(node.x - 1, node.y));
        }
        if(node.x + 1 < SIZE_X) {
            res.add(new Coord2D(node.x + 1, node.y));
        }
        if(node.y - 1 > 0) {
            res.add(new Coord2D(node.x, node.y - 1));
        }
        if(node.y + 1 < SIZE_Y) {
            res.add(new Coord2D(node.x, node.y + 1));
        }
        return res;
    }

    private boolean containsLess(Queue<Coord2D> queue, Coord2D elt) {
        if(contains(queue, elt)) return false;
        Queue<Coord2D> copy = new LinkedList<Coord2D>(queue);
        while(!copy.isEmpty()) {
            Coord2D e = copy.remove();
            if(elt.equals(e) && e.cout < elt.cout) return true;
        }
        return false;
    }

    private boolean contains(Queue<Coord2D> queue, Coord2D elt) {
        Queue<Coord2D> copy = new LinkedList<Coord2D>(queue);
        while(!copy.isEmpty()) {
            Coord2D e = copy.remove();
            if(elt.equals(e)) return true;
        }
        return false;
    }

    private Queue<Coord2D> reconstructPath(Map<Coord2D, Coord2D> cameFrom, Coord2D dest) {
        Stack<Coord2D> tmp = new Stack<Coord2D>();
        Coord2D curr = dest;
        tmp.push(curr);
        while(cameFrom.containsKey(curr)) {
            curr = cameFrom.get(curr);
            tmp.push(curr);
        }
        Queue<Coord2D> path = new LinkedList<Coord2D>();
        while(!tmp.isEmpty()) {
            path.add(tmp.pop());
        }
        path.remove();
        return path;
    }

    public Queue<Coord2D> pathFind(Coord2D dest) {
        Comparator<Coord2D> comparator = new Coord2DComparator();
        PriorityQueue<Coord2D> openList = new PriorityQueue<Coord2D>(comparator);
        Queue<Coord2D> closedList = new LinkedList<Coord2D>();
        Map<Coord2D, Coord2D> cameFrom = new HashMap<Coord2D, Coord2D>();
        Coord2D start = new Coord2D(x, y, 0, 0);
        assert(grid[dest.x][dest.y].traversable());

        openList.add(start);
        while(!openList.isEmpty()) {
            Coord2D tmp = openList.remove();
            closedList.add(tmp);
            if(tmp.equals(dest)) {
                return reconstructPath(cameFrom, tmp);
            }
            Queue<Coord2D> voisins = fetchVoisins(tmp);
            for(Coord2D v: voisins) {
                if (contains(closedList, v) || containsLess(openList, v)) {
                    continue;
                }
                v.cout = tmp.cout + 1;
                v.manhattanDistance(v, dest);
                v.heuristique += v.cout;
                cameFrom.put(v, tmp);
                openList.add(v);
            }
        }
        System.out.println("pathFind did not find a path");
        return null;

    }

}"""

# Initialiser l'objet Rouge
rouge = Rouge()

# Calculer les métriques ROUGE
scores = rouge.get_scores(candidate, reference)

# Afficher les résultats
print("Scores ROUGE :")
print(f"ROUGE-1 Precision: {scores[0]['rouge-1']['p']}")
print(f"ROUGE-1 Recall: {scores[0]['rouge-1']['r']}")
print(f"ROUGE-1 F1 Score: {scores[0]['rouge-1']['f']}")

print(f"ROUGE-2 Precision: {scores[0]['rouge-2']['p']}")
print(f"ROUGE-2 Recall: {scores[0]['rouge-2']['r']}")
print(f"ROUGE-2 F1 Score: {scores[0]['rouge-2']['f']}")

print(f"ROUGE-L Precision: {scores[0]['rouge-l']['p']}")
print(f"ROUGE-L Recall: {scores[0]['rouge-l']['r']}")
print(f"ROUGE-L F1 Score: {scores[0]['rouge-l']['f']}")
