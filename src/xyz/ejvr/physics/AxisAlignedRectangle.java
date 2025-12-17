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
}
