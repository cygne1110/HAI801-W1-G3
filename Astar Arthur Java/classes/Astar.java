// Doesn't compile, just here to have code

import java.util.Queue;
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

}