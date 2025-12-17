package xyz.ejvr.physics;

import java.util.Objects;

public record Body(
        Shape shape,
        VectorDouble position,
        VectorDouble velocity,
        VectorDouble acceleration,
        double orientation,
        double angularVelocity,
        double angularAcceleration,
        double mass,
        double restitution,
        double drag,
        boolean immovable
) {

    public Body {
        Objects.requireNonNull(shape, "Body shape cannot be null");
        Objects.requireNonNull(position, "Body position cannot be null");
        Objects.requireNonNull(velocity, "Body velocity cannot be null");
        Objects.requireNonNull(acceleration, "Body acceleration cannot be null");

        if (mass <= 0 && !immovable) {
            throw new IllegalArgumentException("Mass must be positive for movable bodies");
        }
        if (restitution < 0 || restitution > 1) {
            throw new IllegalArgumentException("Restitution must be between 0 and 1");
        }
        if (drag < 0) {
            throw new IllegalArgumentException("Drag must be non-negative");
        }
    }

    public double inverseMass() {
        return immovable ? 0.0 : 1.0 / mass;
    }

    public double inverseInertia() {
        if (immovable) {
            return 0.0;
        }
        double momentOfInertia = switch (shape) {
            case Circle circle -> 0.5 * mass * circle.radius() * circle.radius();
            case AxisAlignedRectangle rectangle -> mass * (rectangle.width() * rectangle.width() + rectangle.height() * rectangle.height()) / 12.0;
            case RotatedRectangle rectangle -> mass * (rectangle.width() * rectangle.width() + rectangle.height() * rectangle.height()) / 12.0;
            case LineSegment line -> mass * line.length() * line.length() / 12.0;
        };
        return momentOfInertia == 0 ? 0.0 : 1.0 / momentOfInertia;
    }

    public Body integrate(double deltaTime) {
        if (immovable) {
            return this;
        }
        VectorDouble newVelocity = velocity.add(acceleration.scale(deltaTime));
        double newAngularVelocity = angularVelocity + angularAcceleration * deltaTime;

        double dampingFactor = Math.max(0.0, 1.0 - drag * deltaTime);
        newVelocity = newVelocity.scale(dampingFactor);
        newAngularVelocity *= dampingFactor;

        VectorDouble newPosition = position.add(newVelocity.scale(deltaTime));
        double newOrientation = orientation + newAngularVelocity * deltaTime;

        return withKinematics(newPosition, newVelocity, newOrientation, newAngularVelocity);
    }

    public Body withKinematics(VectorDouble newPosition, VectorDouble newVelocity, double newOrientation, double newAngularVelocity) {
        return new Body(
                shape,
                newPosition,
                newVelocity,
                acceleration,
                newOrientation,
                newAngularVelocity,
                angularAcceleration,
                mass,
                restitution,
                drag,
                immovable
        );
    }

    public Body withVelocity(VectorDouble newVelocity) {
        return new Body(
                shape,
                position,
                newVelocity,
                acceleration,
                orientation,
                angularVelocity,
                angularAcceleration,
                mass,
                restitution,
                drag,
                immovable
        );
    }

    public Body withPosition(VectorDouble newPosition) {
        return new Body(
                shape,
                newPosition,
                velocity,
                acceleration,
                orientation,
                angularVelocity,
                angularAcceleration,
                mass,
                restitution,
                drag,
                immovable
        );
    }

    public Body withLinearKinematics(VectorDouble newPosition, VectorDouble newVelocity) {
        return new Body(
                shape,
                newPosition,
                newVelocity,
                acceleration,
                orientation,
                angularVelocity,
                angularAcceleration,
                mass,
                restitution,
                drag,
                immovable
        );
    }

    public Body withAngularVelocity(double newAngularVelocity) {
        return new Body(
                shape,
                position,
                velocity,
                acceleration,
                orientation,
                newAngularVelocity,
                angularAcceleration,
                mass,
                restitution,
                drag,
                immovable
        );
    }

    public Aabb aabb() {
        return switch (shape) {
            case Circle circle -> circleAabb(circle);
            case AxisAlignedRectangle rectangle -> orientedRectangleAabb(rectangle.halfWidth(), rectangle.halfHeight());
            case RotatedRectangle rectangle -> orientedRectangleAabb(rectangle.halfWidth(), rectangle.halfHeight());
            case LineSegment line -> lineAabb(line);
        };
    }

    private Aabb circleAabb(Circle circle) {
        double radius = circle.radius();
        return new Aabb(position.x() - radius, position.y() - radius, position.x() + radius, position.y() + radius);
    }

    private Aabb orientedRectangleAabb(double halfWidth, double halfHeight) {
        VectorDouble[] vertices = rectangleVertices(halfWidth, halfHeight);
        double minX = vertices[0].x();
        double maxX = vertices[0].x();
        double minY = vertices[0].y();
        double maxY = vertices[0].y();

        for (int i = 1; i < vertices.length; i++) {
            minX = Math.min(minX, vertices[i].x());
            maxX = Math.max(maxX, vertices[i].x());
            minY = Math.min(minY, vertices[i].y());
            maxY = Math.max(maxY, vertices[i].y());
        }

        return new Aabb(minX, minY, maxX, maxY);
    }

    private Aabb lineAabb(LineSegment line) {
        VectorDouble rotatedStart = rotatePoint(line.start());
        VectorDouble rotatedEnd = rotatePoint(line.end());
        double minX = Math.min(rotatedStart.x(), rotatedEnd.x());
        double maxX = Math.max(rotatedStart.x(), rotatedEnd.x());
        double minY = Math.min(rotatedStart.y(), rotatedEnd.y());
        double maxY = Math.max(rotatedStart.y(), rotatedEnd.y());
        return new Aabb(minX + position.x(), minY + position.y(), maxX + position.x(), maxY + position.y());
    }

    public VectorDouble rotatePoint(VectorDouble point) {
        return VectorDouble.rotatePoint(0, 0, orientation, point);
    }

    public VectorDouble[] rectangleVertices(double halfWidth, double halfHeight) {
        VectorDouble[] vertices = new VectorDouble[4];
        vertices[0] = rotatePoint(new VectorDouble(-halfWidth, -halfHeight)).add(position);
        vertices[1] = rotatePoint(new VectorDouble(halfWidth, -halfHeight)).add(position);
        vertices[2] = rotatePoint(new VectorDouble(halfWidth, halfHeight)).add(position);
        vertices[3] = rotatePoint(new VectorDouble(-halfWidth, halfHeight)).add(position);
        return vertices;
    }
}
