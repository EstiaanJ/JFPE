package xyz.ejvr.physics;

import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

class VectorDoubleTest {

    @Test
    void diffX() {
        VectorDouble vec = new VectorDouble(4.5, 2.0);
        assertEquals(2.5, vec.diffX(new VectorDouble(2.0, 1.0)));
    }

    @Test
    void diffY() {
        VectorDouble vec = new VectorDouble(4.5, 2.0);
        assertEquals(1.0, vec.diffY(new VectorDouble(1.0, 1.0)));
    }

    @Test
    void radiusSquared() {
        assertEquals(25.0, new VectorDouble(3, 4).radiusSquared());
    }

    @Test
    void radius() {
        assertEquals(5.0, new VectorDouble(3, 4).radius());
    }

    @Test
    void angle() {
        assertEquals(Math.PI / 2, new VectorDouble(0, 2).angle());
    }

    @Test
    void xFloat() {
        assertEquals(2.5f, new VectorDouble(2.5, 4).xFloat());
    }

    @Test
    void yFloat() {
        assertEquals(4.5f, new VectorDouble(2, 4.5).yFloat());
    }

    @Test
    void rotateAboutOrigin() {
        VectorDouble rotated = new VectorDouble(1, 0).rotateAboutOrigin(Math.PI / 2);
        assertEquals(0.0, rotated.x(), 1e-10);
        assertEquals(1.0, rotated.y(), 1e-10);
    }

    @Test
    void distanceBetween() {
        assertEquals(5.0, new VectorDouble(0, 0).distanceBetween(new VectorDouble(3, 4)));
    }

    @Test
    void distanceSquared() {
        assertEquals(25.0, new VectorDouble(0, 0).distanceSquared(new VectorDouble(3, 4)));
    }

    @Test
    void dotProduct() {
        assertEquals(11.0, new VectorDouble(1, 3).dotProduct(new VectorDouble(2, 3)));
    }

    @Test
    void crossProduct2D() {
        assertEquals(-1.0, new VectorDouble(1, 0).crossProduct2D(new VectorDouble(0, 1)));
    }

    @Test
    void add() {
        VectorDouble sum = new VectorDouble(1, 2).add(new VectorDouble(3, 5));
        assertEquals(new VectorDouble(4, 7), sum);
    }

    @Test
    void negate() {
        assertEquals(new VectorDouble(-1, -2), new VectorDouble(1, 2).negate());
    }

    @Test
    void sub() {
        assertEquals(new VectorDouble(-2, -3), new VectorDouble(1, 2).sub(new VectorDouble(3, 5)));
    }

    @Test
    void tangent() {
        assertEquals(new VectorDouble(-2, 1), new VectorDouble(1, 2).tangent());
    }

    @Test
    void scale() {
        assertEquals(new VectorDouble(2, 4), new VectorDouble(1, 2).scale(2));
    }

    @Test
    void normalize() {
        VectorDouble normalized = new VectorDouble(3, 4).normalize();
        assertEquals(1.0, normalized.radius(), 1e-10);
    }

    @Test
    void clamp() {
        VectorDouble clamped = new VectorDouble(5, -2).clamp(new VectorDouble(0, 0), new VectorDouble(3, 3));
        assertEquals(new VectorDouble(3, 0), clamped);
    }

    @Test
    void testToString() {
        String representation = new VectorDouble(3, 4).toString();
        assertTrue(representation.contains("r:"));
        assertTrue(representation.contains("x:"));
        assertTrue(representation.contains("y:"));
    }

    @Test
    void rotatePoint() {
        VectorDouble rotated = VectorDouble.rotatePoint(1, 1, Math.PI / 2, new VectorDouble(2, 1));
        assertEquals(new VectorDouble(1, 2), new VectorDouble(Math.round(rotated.x()), Math.round(rotated.y())));
    }

    @Test
    void x() {
        assertEquals(2.5, new VectorDouble(2.5, 3).x());
    }

    @Test
    void y() {
        assertEquals(3.5, new VectorDouble(2, 3.5).y());
    }
}
