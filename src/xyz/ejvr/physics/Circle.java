package xyz.ejvr.physics;

public record Circle(double radius) implements Shape {

    public Circle {
        if (radius <= 0) {
            throw new IllegalArgumentException("Circle radius must be positive");
        }
    }

    @Override
    public double area() {
        return Math.PI * radius * radius;
    }

    @Override
    public double boundingRadius() {
        return radius;
    }
}
