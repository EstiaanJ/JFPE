package xyz.ejvr.physics;

public record Collision(
        int firstIndex,
        int secondIndex,
        VectorDouble normal,
        double penetration,
        VectorDouble contactPoint
) {

    public Collision {
        if (firstIndex < 0 || secondIndex < 0 || firstIndex == secondIndex) {
            throw new IllegalArgumentException("Collision indices must refer to distinct bodies");
        }
        if (normal.radiusSquared() == 0) {
            throw new IllegalArgumentException("Collision normal cannot be zero");
        }
    }
}
