package xyz.ejvr.physics;

import org.junit.jupiter.api.Test;

import java.util.Optional;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

final class CollisionDetectionTest {

    private static final VectorDouble ZERO = new VectorDouble(0, 0);

    @Test
    void detectsCircleCircleOverlap() {
        Body first = new Body(new Circle(1), new VectorDouble(0, 0), ZERO, ZERO, 1, 1, false);
        Body second = new Body(new Circle(1), new VectorDouble(1.5, 0), ZERO, ZERO, 1, 1, false);

        Optional<Collision> collision = CollisionDetection.detect(0, 1, first, second);

        assertTrue(collision.isPresent(), "Collision should be detected for overlapping circles");
        Collision result = collision.orElseThrow();

        assertEquals(0.5, result.penetration(), 1e-9);
        assertEquals(1.0, result.normal().x(), 1e-9);
        assertEquals(0.0, result.normal().y(), 1e-9);
        assertEquals(0.75, result.contactPoint().x(), 1e-9);
        assertEquals(0.0, result.contactPoint().y(), 1e-9);
    }

    @Test
    void detectsRectangleRectangleAlongMinorOverlap() {
        AxisAlignedRectangle rectangle = new AxisAlignedRectangle(1, 1);
        Body first = new Body(rectangle, new VectorDouble(0, 0), ZERO, ZERO, 1, 1, false);
        Body second = new Body(rectangle, new VectorDouble(1.5, 0.25), ZERO, ZERO, 1, 1, false);

        Optional<Collision> collision = CollisionDetection.detect(0, 1, first, second);

        assertTrue(collision.isPresent(), "Collision should be detected for intersecting rectangles");
        Collision result = collision.orElseThrow();

        assertEquals(0.5, result.penetration(), 1e-9);
        assertEquals(1.0, result.normal().x(), 1e-9);
        assertEquals(0.0, result.normal().y(), 1e-9);
        assertEquals(1.0, result.contactPoint().x(), 1e-9);
        assertEquals(0.25, result.contactPoint().y(), 1e-9);
    }

    @Test
    void detectsLineCircleContact() {
        Body lineBody = new Body(
                new LineSegment(new VectorDouble(-1, 0), new VectorDouble(1, 0)),
                new VectorDouble(0, 0),
                ZERO,
                ZERO,
                1,
                1,
                false
        );
        Body circleBody = new Body(new Circle(1), new VectorDouble(0, 0.5), ZERO, ZERO, 1, 1, false);

        Optional<Collision> collision = CollisionDetection.detect(0, 1, lineBody, circleBody);

        assertTrue(collision.isPresent(), "Collision should be detected when circle touches line segment");
        Collision result = collision.orElseThrow();

        assertEquals(0.5, result.penetration(), 1e-9);
        assertEquals(0.0, result.normal().x(), 1e-9);
        assertEquals(1.0, result.normal().y(), 1e-9);
        assertEquals(0.0, result.contactPoint().x(), 1e-9);
        assertEquals(0.0, result.contactPoint().y(), 1e-9);
    }
}
