package xyz.ejvr.physics;

public record Aabb(double minX, double minY, double maxX, double maxY) {

    public Aabb {
        if (minX > maxX || minY > maxY) {
            throw new IllegalArgumentException("AABB minimums must not exceed maximums");
        }
    }

    public boolean intersects(Aabb other) {
        return maxX >= other.minX && minX <= other.maxX && maxY >= other.minY && minY <= other.maxY;
    }

    public Aabb translated(VectorDouble delta) {
        return new Aabb(minX + delta.x(), minY + delta.y(), maxX + delta.x(), maxY + delta.y());
    }
}
