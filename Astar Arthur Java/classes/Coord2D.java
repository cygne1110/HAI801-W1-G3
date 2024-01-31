package java.classes;

public class Coord2D {
    public int x;
    public int y;

    public int heuristique;
    public int cout;

    Coord2D(int a, int b) {x = a; y = b;}

    Coord2D(int a, int b, int h, int c) {x = a; y = b; heuristique = h; cout = c;}

    public void manhattanDistance(Coord2D dep, Coord2D dest) {
        heuristique = Math.abs(dep.x - dest.x + dep.y - dest.y);
    }

    public boolean equals(Coord2D other) {
        return (x == other.x && y == other.y);
    }

}