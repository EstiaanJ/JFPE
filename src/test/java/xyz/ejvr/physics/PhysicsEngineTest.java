package xyz.ejvr.physics;

import org.junit.jupiter.api.Test;

import java.util.List;

import static org.junit.jupiter.api.Assertions.assertEquals;

final class PhysicsEngineTest {

    private static final VectorDouble ZERO = new VectorDouble(0, 0);

    @Test
    void bouncesCircleOffBoundary() {
        Boundary boundary = new Boundary(0, 0, 10, 10);
        Body movingCircle = new Body(new Circle(1), new VectorDouble(9, 5), new VectorDouble(2, 0), ZERO, 0.0, 0.0, 0.0, 1, 0.5, 0.0, false);
        World initial = new World(List.of(movingCircle), List.of(boundary));

        World stepped = PhysicsEngine.step(initial, 1.0);
        Body result = stepped.bodies().getFirst();

        assertEquals(9.0, result.position().x(), 1e-9);
        assertEquals(5.0, result.position().y(), 1e-9);
        assertEquals(-1.0, result.velocity().x(), 1e-9);
        assertEquals(0.0, result.velocity().y(), 1e-9);
    }

    @Test
    void resolvesHeadOnCollisionBetweenEqualCircles() {
        Body first = new Body(new Circle(1), new VectorDouble(0, 0), new VectorDouble(1, 0), ZERO, 0.0, 0.0, 0.0, 1, 1, 0.0, false);
        Body second = new Body(new Circle(1), new VectorDouble(1.5, 0), new VectorDouble(-1, 0), ZERO, 0.0, 0.0, 0.0, 1, 1, 0.0, false);
        World world = new World(List.of(first, second), List.of());

        World result = PhysicsEngine.step(world, 0.0);
        Body resolvedFirst = result.bodies().getFirst();
        Body resolvedSecond = result.bodies().get(1);

        assertEquals(-1.0, resolvedFirst.velocity().x(), 1e-9);
        assertEquals(1.0, resolvedSecond.velocity().x(), 1e-9);
        assertEquals(-0.25, resolvedFirst.position().x(), 1e-9);
        assertEquals(1.75, resolvedSecond.position().x(), 1e-9);
    }

    @Test
    void appliesDragAndAngularMotionDuringIntegration() {
        Body spinning = new Body(new Circle(1), new VectorDouble(0, 0), new VectorDouble(10, 0), ZERO, 0.0, 2.0, 0.0, 2.0, 1.0, 0.5, false);
        World world = new World(List.of(spinning), List.of());

        World result = PhysicsEngine.step(world, 1.0);
        Body integrated = result.bodies().getFirst();

        assertEquals(5.0, integrated.velocity().x(), 1e-9);
        assertEquals(5.0, integrated.position().x(), 1e-9);
        assertEquals(1.0, integrated.angularVelocity(), 1e-9);
        assertEquals(1.0, integrated.orientation(), 1e-9);
    }
}
