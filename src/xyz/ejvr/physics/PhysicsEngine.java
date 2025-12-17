package xyz.ejvr.physics;

import java.util.List;

public final class PhysicsEngine {

    private PhysicsEngine() {
    }

    public static World step(World world, double deltaTime) {
        List<Body> integratedBodies = world.bodies()
                .stream()
                .map(body -> body.integrate(deltaTime))
                .toList();

        List<Body> boundedBodies = integratedBodies.stream()
                .map(body -> applyBoundaries(body, world.boundaries()))
                .toList();

        List<Collision> collisions = CollisionDetection.detectAll(boundedBodies);
        List<Body> resolvedBodies = CollisionResolution.resolve(boundedBodies, collisions);

        return new World(resolvedBodies, world.boundaries());
    }

    private static Body applyBoundaries(Body body, List<Boundary> boundaries) {
        return boundaries.stream()
                .reduce(body, PhysicsEngine::resolveBoundaryCollision, (first, second) -> second);
    }

    private static Body resolveBoundaryCollision(Body body, Boundary boundary) {
        if (body.immovable()) {
            return body;
        }
        return switch (body.shape()) {
            case Circle circle -> resolveCircleBoundary(body, boundary, circle);
            case AxisAlignedRectangle rectangle -> resolveRectangleBoundary(body, boundary);
            case RotatedRectangle rectangle -> resolveRectangleBoundary(body, boundary);
            case LineSegment line -> resolveLineBoundary(body, boundary, line);
        };
    }

    private static Body resolveCircleBoundary(Body body, Boundary boundary, Circle circle) {
        double x = body.position().x();
        double y = body.position().y();
        double vx = body.velocity().x();
        double vy = body.velocity().y();
        double restitution = body.restitution();
        double radius = circle.radius();

        if (x - radius < boundary.minX()) {
            x = boundary.minX() + radius;
            vx = Math.abs(vx) * restitution;
        } else if (x + radius > boundary.maxX()) {
            x = boundary.maxX() - radius;
            vx = -Math.abs(vx) * restitution;
        }

        if (y - radius < boundary.minY()) {
            y = boundary.minY() + radius;
            vy = Math.abs(vy) * restitution;
        } else if (y + radius > boundary.maxY()) {
            y = boundary.maxY() - radius;
            vy = -Math.abs(vy) * restitution;
        }

        return body.withLinearKinematics(new VectorDouble(x, y), new VectorDouble(vx, vy));
    }

    private static Body resolveRectangleBoundary(Body body, Boundary boundary) {
        double x = body.position().x();
        double y = body.position().y();
        double vx = body.velocity().x();
        double vy = body.velocity().y();
        double restitution = body.restitution();

        Aabb aabb = body.aabb();
        if (aabb.minX() < boundary.minX()) {
            double correction = boundary.minX() - aabb.minX();
            x += correction;
            vx = Math.abs(vx) * restitution;
        } else if (aabb.maxX() > boundary.maxX()) {
            double correction = aabb.maxX() - boundary.maxX();
            x -= correction;
            vx = -Math.abs(vx) * restitution;
        }

        if (aabb.minY() < boundary.minY()) {
            double correction = boundary.minY() - aabb.minY();
            y += correction;
            vy = Math.abs(vy) * restitution;
        } else if (aabb.maxY() > boundary.maxY()) {
            double correction = aabb.maxY() - boundary.maxY();
            y -= correction;
            vy = -Math.abs(vy) * restitution;
        }

        return body.withLinearKinematics(new VectorDouble(x, y), new VectorDouble(vx, vy));
    }

    private static Body resolveLineBoundary(Body body, Boundary boundary, LineSegment line) {
        VectorDouble start = line.start().add(body.position());
        VectorDouble end = line.end().add(body.position());

        double minLineX = Math.min(start.x(), end.x());
        double maxLineX = Math.max(start.x(), end.x());
        double minLineY = Math.min(start.y(), end.y());
        double maxLineY = Math.max(start.y(), end.y());

        double x = body.position().x();
        double y = body.position().y();
        double vx = body.velocity().x();
        double vy = body.velocity().y();
        double restitution = body.restitution();

        if (minLineX < boundary.minX()) {
            double correction = boundary.minX() - minLineX;
            x += correction;
            vx = Math.abs(vx) * restitution;
        } else if (maxLineX > boundary.maxX()) {
            double correction = maxLineX - boundary.maxX();
            x -= correction;
            vx = -Math.abs(vx) * restitution;
        }

        if (minLineY < boundary.minY()) {
            double correction = boundary.minY() - minLineY;
            y += correction;
            vy = Math.abs(vy) * restitution;
        } else if (maxLineY > boundary.maxY()) {
            double correction = maxLineY - boundary.maxY();
            y -= correction;
            vy = -Math.abs(vy) * restitution;
        }

        return body.withLinearKinematics(new VectorDouble(x, y), new VectorDouble(vx, vy));
    }
}
