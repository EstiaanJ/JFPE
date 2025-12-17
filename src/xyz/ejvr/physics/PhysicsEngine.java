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
        Aabb bounds = Aabb.fromBody(body);

        double x = body.position().x();
        double y = body.position().y();
        double vx = body.velocity().x();
        double vy = body.velocity().y();
        double restitution = body.restitution();

        if (bounds.minX() < boundary.minX()) {
            double correction = boundary.minX() - bounds.minX();
            x += correction;
            vx = Math.abs(vx) * restitution;
        } else if (bounds.maxX() > boundary.maxX()) {
            double correction = bounds.maxX() - boundary.maxX();
            x -= correction;
            vx = -Math.abs(vx) * restitution;
        }

        if (bounds.minY() < boundary.minY()) {
            double correction = boundary.minY() - bounds.minY();
            y += correction;
            vy = Math.abs(vy) * restitution;
        } else if (bounds.maxY() > boundary.maxY()) {
            double correction = bounds.maxY() - boundary.maxY();
            y -= correction;
            vy = -Math.abs(vy) * restitution;
        }

        VectorDouble newPosition = new VectorDouble(x, y);
        VectorDouble newVelocity = new VectorDouble(vx, vy);
        return body.withKinematics(newPosition, newVelocity, body.orientation(), body.angularVelocity());
    }
}
