package xyz.ejvr.physics;

public record AxisAlignedRectangle(double halfWidth, double halfHeight) implements Shape {

    public AxisAlignedRectangle {
        if (halfWidth <= 0 || halfHeight <= 0) {
            throw new IllegalArgumentException("Rectangle half extents must be positive");
        }
    }

    public double width() {
        return halfWidth * 2;
    }

    public double height() {
        return halfHeight * 2;
    }

    @Override
    public double area() {
        return width() * height();
    }

    @Override
    public double boundingRadius() {
        return Math.hypot(halfWidth, halfHeight);
    }

    @Override
    public double momentOfInertia(double mass) {
        double width = width();
        double height = height();
        return (mass / 12.0) * (width * width + height * height);
    }
}
