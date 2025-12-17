package xyz.ejvr.physics;

import java.util.Objects;

public record LineSegment(VectorDouble start, VectorDouble end) implements Shape {

    public LineSegment {
        Objects.requireNonNull(start, "LineSegment start cannot be null");
        Objects.requireNonNull(end, "LineSegment end cannot be null");
        if (start.equals(end)) {
            throw new IllegalArgumentException("LineSegment start and end cannot be the same point");
        }
    }

    public VectorDouble direction() {
        return end.sub(start);
    }

    public double length() {
        return direction().radius();
    }

    public VectorDouble midpoint() {
        return new VectorDouble((start.x() + end.x()) / 2, (start.y() + end.y()) / 2);
    }

    @Override
    public double area() {
        return 0;
    }

    @Override
    public double boundingRadius() {
        return Math.max(start.radius(), end.radius());
    }
}
