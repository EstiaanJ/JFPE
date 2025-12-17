package xyz.ejvr.physics;

public record VectorDouble(double x, double y) {

    public VectorDouble {
        if (Double.isNaN(x) || Double.isNaN(y)) {
            throw new IllegalArgumentException("VectorDouble cannot contain NaN components");
        }
    }

    public VectorDouble(VectorDouble vec) {
        this(vec.x, vec.y);
    }

    public double diffX(VectorDouble p) {
        return x - p.x;
    }

    public double diffY(VectorDouble p) {
        return y - p.y;
    }

    public double radiusSquared() {
        return (x * x) + (y * y);
    }

    public double radius() {
        return Math.sqrt(radiusSquared());
    }

    public double angle() {
        return Math.atan2(y, x);
    }

    public float xFloat() {
        return (float) x;
    }

    public float yFloat() {
        return (float) y;
    }

    public VectorDouble rotateAboutOrigin(double angle) {
        return rotatePoint(0, 0, angle, this);
    }

    public double distanceBetween(VectorDouble point) {
        return Math.sqrt(distanceSquared(point));
    }

    public double distanceSquared(VectorDouble point) {
        double xFinal = this.x - point.x;
        double yFinal = this.y - point.y;
        return (xFinal * xFinal) + (yFinal * yFinal);
    }

    public double dotProduct(VectorDouble vec) {
        return x * vec.x + y * vec.y;
    }

    public double crossProduct2D(VectorDouble vec) {
        return (y * vec.x) - (x * vec.y);
    }

    public VectorDouble add(VectorDouble vec) {
        return new VectorDouble(x + vec.x, y + vec.y);
    }

    public VectorDouble negate() {
        return new VectorDouble(-x, -y);
    }

    public VectorDouble sub(VectorDouble vec) {
        return this.add(vec.negate());
    }

    public VectorDouble tangent() {
        return new VectorDouble(-y, x);
    }

    public VectorDouble scale(double scalar) {
        return new VectorDouble(x * scalar, y * scalar);
    }

    public VectorDouble normalize() {
        double magnitude = radius();
        if (magnitude == 0) {
            return this;
        }
        return scale(1 / magnitude);
    }

    public VectorDouble clamp(VectorDouble min, VectorDouble max) {
        double clampedX = Math.max(min.x, Math.min(max.x, x));
        double clampedY = Math.max(min.y, Math.min(max.y, y));
        return new VectorDouble(clampedX, clampedY);
    }

    @Override
    public String toString() {
        return "r: %s x: %s y: %s".formatted(radius(), x, y);
    }

    /*
    Adapted from:
    twe4ked, Nils Pipenbrinck
    Stack Overflow 2022
    https://stackoverflow.com/questions/2259476/rotating-a-point-about-another-point-2d
     */
    public static VectorDouble rotatePoint(double cx, double cy, double angle, VectorDouble p) {
        double translatedX = p.x - cx;
        double translatedY = p.y - cy;

        double sine = Math.sin(angle);
        double cosine = Math.cos(angle);

        double rotatedX = translatedX * cosine - translatedY * sine;
        double rotatedY = translatedX * sine + translatedY * cosine;

        return new VectorDouble(rotatedX + cx, rotatedY + cy);
    }
}
