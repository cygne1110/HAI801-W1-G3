package java.classes;

import java.util.Comparator;
public class Coord2DComparator implements Comparator<Coord2D> {

    public int compare(Coord2D n1, Coord2D n2) {
        return Integer.compare(n1.heuristique, n2.heuristique);
    }

}