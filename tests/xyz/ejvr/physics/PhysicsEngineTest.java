package xyz.ejvr.physics;

import org.junit.jupiter.api.Test;

import java.util.List;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

class PhysicsEngineTest {

    @Test
    void advancesKinematics() {
        Body body = new Body(
                new Circle(1.0),
                new VectorDouble(0, 0),
                new VectorDouble(2, 0),
                new VectorDouble(0, 0),
                1.0,
                1.0,
                false
        );

        World world = new World(List.of(body), List.of());
        World result = PhysicsEngine.step(world, 0.5);

        VectorDouble newPosition = result.bodies().getFirst().position();
        assertEquals(new VectorDouble(1.0, 0.0), newPosition);
    }

    @Test
    void reflectsAgainstBoundary() {
        Body body = new Body(
                new Circle(1.0),
                new VectorDouble(1.2, 1.0),
                new VectorDouble(-3.0, 0),
                new VectorDouble(0, 0),
                1.0,
                1.0,
                false
        );

        Boundary boundary = new Boundary(0, 0, 10, 10);
        World world = new World(List.of(body), List.of(boundary));
        World result = PhysicsEngine.step(world, 0.5);

        Body updatedBody = result.bodies().getFirst();
        assertEquals(boundary.minX() + ((Circle) body.shape()).radius(), updatedBody.position().x());
        assertTrue(updatedBody.velocity().x() > 0);
    }

    @Test
    void resolvesCircleCircleCollision() {
        Body first = new Body(
                new Circle(1.0),
                new VectorDouble(0, 0),
                new VectorDouble(1.0, 0),
                new VectorDouble(0, 0),
                1.0,
                1.0,
                false
        );
        Body second = new Body(
                new Circle(1.0),
                new VectorDouble(1.5, 0),
                new VectorDouble(-1.0, 0),
                new VectorDouble(0, 0),
                1.0,
                1.0,
                false
        );

        World world = new World(List.of(first, second), List.of());
        World result = PhysicsEngine.step(world, 0.5);

        Body resolvedFirst = result.bodies().getFirst();
        Body resolvedSecond = result.bodies().get(1);

        assertEquals(-1.0, resolvedFirst.velocity().x(), 1e-9);
        assertEquals(1.0, resolvedSecond.velocity().x(), 1e-9);
    }
}
