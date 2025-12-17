package xyz.ejvr.physics;

import java.util.List;

public record Aabb(double minX, double minY, double maxX, double maxY) {

    public Aabb {
        if (Double.isNaN(minX) || Double.isNaN(minY) || Double.isNaN(maxX) || Double.isNaN(maxY)) {
            throw new IllegalArgumentException("AABB values cannot be NaN");
        }
        if (minX > maxX || minY > maxY) {
            throw new IllegalArgumentException("AABB min values must not exceed max values");
        }
    }

    public boolean overlaps(Aabb other) {
        return maxX >= other.minX && other.maxX >= minX && maxY >= other.minY && other.maxY >= minY;
    }

    public static Aabb fromBody(Body body) {
        return switch (body.shape()) {
            case Circle circle -> fromCircle(circle, body.position());
            case AxisAlignedRectangle rectangle -> fromRectangle(rectangle, body.position(), body.orientation());
            case LineSegment line -> fromLine(line, body.position(), body.orientation());
        };
    }

    private static Aabb fromCircle(Circle circle, VectorDouble center) {
        double radius = circle.radius();
        return new Aabb(center.x() - radius, center.y() - radius, center.x() + radius, center.y() + radius);
    }

    private static Aabb fromRectangle(AxisAlignedRectangle rectangle, VectorDouble center, double orientation) {
        VectorDouble xAxis = orientationAxis(orientation);
        VectorDouble yAxis = xAxis.tangent();

        VectorDouble cornerOffset = xAxis.scale(rectangle.halfWidth()).add(yAxis.scale(rectangle.halfHeight()));
        VectorDouble oppositeOffset = xAxis.scale(-rectangle.halfWidth()).add(yAxis.scale(-rectangle.halfHeight()));
        VectorDouble otherOffset = xAxis.scale(rectangle.halfWidth()).add(yAxis.scale(-rectangle.halfHeight()));
        VectorDouble finalOffset = xAxis.scale(-rectangle.halfWidth()).add(yAxis.scale(rectangle.halfHeight()));

        return fromPoints(List.of(
                center.add(cornerOffset),
                center.add(oppositeOffset),
                center.add(otherOffset),
                center.add(finalOffset)
        ));
    }

    private static Aabb fromLine(LineSegment line, VectorDouble position, double orientation) {
        VectorDouble start = rotate(line.start(), orientation).add(position);
        VectorDouble end = rotate(line.end(), orientation).add(position);
        return fromPoints(List.of(start, end));
    }

    private static Aabb fromPoints(List<VectorDouble> points) {
        double minX = points.stream().mapToDouble(VectorDouble::x).min().orElse(0.0);
        double minY = points.stream().mapToDouble(VectorDouble::y).min().orElse(0.0);
        double maxX = points.stream().mapToDouble(VectorDouble::x).max().orElse(0.0);
        double maxY = points.stream().mapToDouble(VectorDouble::y).max().orElse(0.0);
        return new Aabb(minX, minY, maxX, maxY);
    }

    private static VectorDouble orientationAxis(double orientation) {
        double cosine = Math.cos(orientation);
        double sine = Math.sin(orientation);
        return new VectorDouble(cosine, sine);
    }

    private static VectorDouble rotate(VectorDouble vector, double angle) {
        return vector.rotateAboutOrigin(angle);
    }
}
