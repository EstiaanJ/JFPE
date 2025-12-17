package xyz.ejvr.physics;

public record Boundary(double minX, double minY, double maxX, double maxY) {

    public Boundary {
        if (minX >= maxX || minY >= maxY) {
            throw new IllegalArgumentException("Boundary min values must be less than max values");
        }
    }

    public boolean contains(VectorDouble point) {
        return point.x() >= minX && point.x() <= maxX && point.y() >= minY && point.y() <= maxY;
    }
}
