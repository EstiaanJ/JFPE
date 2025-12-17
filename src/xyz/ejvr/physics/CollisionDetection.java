package xyz.ejvr.physics;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.stream.IntStream;

public final class CollisionDetection {

    private CollisionDetection() {
    }

    public static List<Collision> detectAll(List<Body> bodies) {
        int bodyCount = bodies.size();
        List<Collision> collisions = new ArrayList<>();
        IntStream.range(0, bodyCount).forEach(firstIndex ->
                IntStream.range(firstIndex + 1, bodyCount)
                        .mapToObj(secondIndex -> detect(firstIndex, secondIndex, bodies.get(firstIndex), bodies.get(secondIndex)))
                        .filter(Optional::isPresent)
                        .map(Optional::get)
                        .forEach(collisions::add)
        );
        return collisions;
    }

    public static Optional<Collision> detect(int firstIndex, int secondIndex, Body first, Body second) {
        Shape firstShape = first.shape();
        Shape secondShape = second.shape();

        if (firstShape instanceof Circle firstCircle && secondShape instanceof Circle secondCircle) {
            return detectCircleCircle(firstIndex, secondIndex, first, second, firstCircle, secondCircle);
        }
        if (firstShape instanceof AxisAlignedRectangle firstRect && secondShape instanceof AxisAlignedRectangle secondRect) {
            return detectRectangleRectangle(firstIndex, secondIndex, first, second, firstRect, secondRect);
        }
        if (firstShape instanceof Circle circle && secondShape instanceof AxisAlignedRectangle rectangle) {
            return detectCircleRectangle(firstIndex, secondIndex, first, second, circle, rectangle);
        }
        if (firstShape instanceof AxisAlignedRectangle rectangle && secondShape instanceof Circle circle) {
            return detectCircleRectangle(secondIndex, firstIndex, second, first, circle, rectangle)
                    .map(collision -> flipNormal(collision, firstIndex, secondIndex));
        }
        if (firstShape instanceof LineSegment line && secondShape instanceof Circle circle) {
            return detectLineCircle(firstIndex, secondIndex, first, second, line, circle);
        }
        if (firstShape instanceof Circle circle && secondShape instanceof LineSegment line) {
            return detectLineCircle(secondIndex, firstIndex, second, first, line, circle)
                    .map(collision -> flipNormal(collision, firstIndex, secondIndex));
        }
        if (firstShape instanceof LineSegment line && secondShape instanceof AxisAlignedRectangle rectangle) {
            return detectLineRectangle(firstIndex, secondIndex, first, second, line, rectangle);
        }
        if (firstShape instanceof AxisAlignedRectangle rectangle && secondShape instanceof LineSegment line) {
            return detectLineRectangle(secondIndex, firstIndex, second, first, line, rectangle)
                    .map(collision -> flipNormal(collision, firstIndex, secondIndex));
        }

        return Optional.empty();
    }

    private static Optional<Collision> detectCircleCircle(int firstIndex, int secondIndex, Body first, Body second, Circle firstCircle, Circle secondCircle) {
        VectorDouble delta = second.position().sub(first.position());
        double distanceSquared = delta.radiusSquared();
        double radiusSum = firstCircle.radius() + secondCircle.radius();
        double radiusSumSquared = radiusSum * radiusSum;

        if (distanceSquared >= radiusSumSquared) {
            return Optional.empty();
        }

        double distance = Math.sqrt(distanceSquared);
        VectorDouble normal = distance == 0 ? new VectorDouble(1, 0) : delta.scale(1 / distance);
        double penetration = radiusSum - distance;
        VectorDouble contactPoint = first.position().add(normal.scale(firstCircle.radius() - penetration / 2));

        return Optional.of(new Collision(firstIndex, secondIndex, normal, penetration, contactPoint));
    }

    private static Optional<Collision> detectRectangleRectangle(
            int firstIndex,
            int secondIndex,
            Body first,
            Body second,
            AxisAlignedRectangle firstRect,
            AxisAlignedRectangle secondRect
    ) {
        VectorDouble delta = second.position().sub(first.position());
        double overlapX = firstRect.halfWidth() + secondRect.halfWidth() - Math.abs(delta.x());
        double overlapY = firstRect.halfHeight() + secondRect.halfHeight() - Math.abs(delta.y());

        if (overlapX <= 0 || overlapY <= 0) {
            return Optional.empty();
        }

        if (overlapX < overlapY) {
            double normalX = Math.signum(delta.x());
            VectorDouble normal = new VectorDouble(normalX == 0 ? 1 : normalX, 0);
            VectorDouble contactPoint = new VectorDouble(
                    first.position().x() + firstRect.halfWidth() * normal.x(),
                    first.position().y() + delta.y()
            );
            return Optional.of(new Collision(firstIndex, secondIndex, normal, overlapX, contactPoint));
        }

        double normalY = Math.signum(delta.y());
        VectorDouble normal = new VectorDouble(0, normalY == 0 ? 1 : normalY);
        VectorDouble contactPoint = new VectorDouble(
                first.position().x() + delta.x(),
                first.position().y() + firstRect.halfHeight() * normal.y()
        );
        return Optional.of(new Collision(firstIndex, secondIndex, normal, overlapY, contactPoint));
    }

    private static Optional<Collision> detectCircleRectangle(
            int circleIndex,
            int rectangleIndex,
            Body circleBody,
            Body rectangleBody,
            Circle circle,
            AxisAlignedRectangle rectangle
    ) {
        VectorDouble circleCenter = circleBody.position();
        VectorDouble rectangleCenter = rectangleBody.position();
        VectorDouble difference = circleCenter.sub(rectangleCenter);

        VectorDouble closestPoint = difference.clamp(
                new VectorDouble(-rectangle.halfWidth(), -rectangle.halfHeight()),
                new VectorDouble(rectangle.halfWidth(), rectangle.halfHeight())
        ).add(rectangleCenter);

        VectorDouble delta = circleCenter.sub(closestPoint);
        double distanceSquared = delta.radiusSquared();
        if (distanceSquared > circle.radius() * circle.radius()) {
            return Optional.empty();
        }

        double distance = Math.sqrt(distanceSquared);
        VectorDouble normal = distance == 0 ? new VectorDouble(1, 0) : delta.scale(1 / distance);
        double penetration = circle.radius() - distance;
        return Optional.of(new Collision(circleIndex, rectangleIndex, normal, penetration, closestPoint));
    }

    private static Optional<Collision> detectLineCircle(
            int lineIndex,
            int circleIndex,
            Body lineBody,
            Body circleBody,
            LineSegment line,
            Circle circle
    ) {
        VectorDouble start = line.start().add(lineBody.position());
        VectorDouble end = line.end().add(lineBody.position());
        VectorDouble closest = closestPointOnSegment(start, end, circleBody.position());
        VectorDouble delta = circleBody.position().sub(closest);
        double distanceSquared = delta.radiusSquared();
        double radiusSquared = circle.radius() * circle.radius();
        if (distanceSquared > radiusSquared) {
            return Optional.empty();
        }

        double distance = Math.sqrt(distanceSquared);
        VectorDouble normal = distance == 0 ? line.direction().tangent().normalize() : delta.scale(1 / distance);
        double penetration = circle.radius() - distance;
        return Optional.of(new Collision(lineIndex, circleIndex, normal, penetration, closest));
    }

    private static Optional<Collision> detectLineRectangle(
            int lineIndex,
            int rectangleIndex,
            Body lineBody,
            Body rectangleBody,
            LineSegment line,
            AxisAlignedRectangle rectangle
    ) {
        VectorDouble start = line.start().add(lineBody.position());
        VectorDouble end = line.end().add(lineBody.position());

        VectorDouble rectCenter = rectangleBody.position();
        double minX = rectCenter.x() - rectangle.halfWidth();
        double maxX = rectCenter.x() + rectangle.halfWidth();
        double minY = rectCenter.y() - rectangle.halfHeight();
        double maxY = rectCenter.y() + rectangle.halfHeight();

        if (isPointInsideRectangle(start, minX, maxX, minY, maxY)) {
            VectorDouble normal = outwardNormal(rectCenter, start);
            double penetration = Math.min(maxX - start.x(), Math.min(start.x() - minX, Math.min(maxY - start.y(), start.y() - minY)));
            return Optional.of(new Collision(lineIndex, rectangleIndex, normal, penetration, start));
        }
        if (isPointInsideRectangle(end, minX, maxX, minY, maxY)) {
            VectorDouble normal = outwardNormal(rectCenter, end);
            double penetration = Math.min(maxX - end.x(), Math.min(end.x() - minX, Math.min(maxY - end.y(), end.y() - minY)));
            return Optional.of(new Collision(lineIndex, rectangleIndex, normal, penetration, end));
        }

        List<VectorDouble> corners = List.of(
                new VectorDouble(minX, minY),
                new VectorDouble(maxX, minY),
                new VectorDouble(maxX, maxY),
                new VectorDouble(minX, maxY)
        );
        List<LineSegment> edges = List.of(
                new LineSegment(corners.get(0), corners.get(1)),
                new LineSegment(corners.get(1), corners.get(2)),
                new LineSegment(corners.get(2), corners.get(3)),
                new LineSegment(corners.get(3), corners.get(0))
        );

        Optional<VectorDouble> intersection = edges.stream()
                .map(edge -> segmentIntersection(start, end, edge.start(), edge.end()))
                .filter(Optional::isPresent)
                .map(Optional::get)
                .findFirst();

        if (intersection.isEmpty()) {
            return Optional.empty();
        }

        VectorDouble contact = intersection.get();
        VectorDouble normal = outwardNormal(rectCenter, contact);
        double penetration = 0.0;
        return Optional.of(new Collision(lineIndex, rectangleIndex, normal, penetration, contact));
    }

    private static VectorDouble outwardNormal(VectorDouble rectangleCenter, VectorDouble contactPoint) {
        VectorDouble delta = contactPoint.sub(rectangleCenter);
        if (delta.x() == 0 && delta.y() == 0) {
            return new VectorDouble(1, 0);
        }
        if (Math.abs(delta.x()) > Math.abs(delta.y())) {
            double sign = Math.signum(delta.x());
            return new VectorDouble(sign == 0 ? 1 : sign, 0);
        }
        double sign = Math.signum(delta.y());
        return new VectorDouble(0, sign == 0 ? 1 : sign);
    }

    private static Optional<VectorDouble> segmentIntersection(VectorDouble aStart, VectorDouble aEnd, VectorDouble bStart, VectorDouble bEnd) {
        VectorDouble r = aEnd.sub(aStart);
        VectorDouble s = bEnd.sub(bStart);

        double denominator = r.crossProduct2D(s);
        double numerator = bStart.sub(aStart).crossProduct2D(r);

        if (denominator == 0 && numerator == 0) {
            return Optional.empty();
        }
        if (denominator == 0) {
            return Optional.empty();
        }

        double t = bStart.sub(aStart).crossProduct2D(s) / denominator;
        double u = bStart.sub(aStart).crossProduct2D(r) / denominator;

        if (t >= 0 && t <= 1 && u >= 0 && u <= 1) {
            return Optional.of(aStart.add(r.scale(t)));
        }
        return Optional.empty();
    }

    private static boolean isPointInsideRectangle(VectorDouble point, double minX, double maxX, double minY, double maxY) {
        return point.x() >= minX && point.x() <= maxX && point.y() >= minY && point.y() <= maxY;
    }

    private static VectorDouble closestPointOnSegment(VectorDouble start, VectorDouble end, VectorDouble point) {
        VectorDouble segment = end.sub(start);
        double lengthSquared = segment.radiusSquared();
        if (lengthSquared == 0) {
            return start;
        }
        double t = point.sub(start).dotProduct(segment) / lengthSquared;
        double clampedT = Math.max(0, Math.min(1, t));
        return start.add(segment.scale(clampedT));
    }

    private static Collision flipNormal(Collision collision, int firstIndex, int secondIndex) {
        return new Collision(firstIndex, secondIndex, collision.normal().negate(), collision.penetration(), collision.contactPoint());
    }
}
