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
        List<Aabb> bounds = bodies.stream().map(Aabb::fromBody).toList();
        IntStream.range(0, bodyCount).forEach(firstIndex ->
                IntStream.range(firstIndex + 1, bodyCount)
                        .filter(secondIndex -> bounds.get(firstIndex).overlaps(bounds.get(secondIndex)))
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
        OrientedRectangle firstOriented = new OrientedRectangle(first.position(), firstRect.halfWidth(), firstRect.halfHeight(), first.orientation());
        OrientedRectangle secondOriented = new OrientedRectangle(second.position(), secondRect.halfWidth(), secondRect.halfHeight(), second.orientation());

        List<VectorDouble> axes = List.of(firstOriented.axisX(), firstOriented.axisY(), secondOriented.axisX(), secondOriented.axisY());

        double minimumOverlap = Double.POSITIVE_INFINITY;
        VectorDouble smallestAxis = null;

        for (VectorDouble axis : axes) {
            VectorDouble normalizedAxis = axis.normalize();
            double distance = Math.abs(normalizedAxis.dotProduct(secondOriented.center().sub(firstOriented.center())));
            double projection = projectionRadius(firstOriented, normalizedAxis) + projectionRadius(secondOriented, normalizedAxis);
            double overlap = projection - distance;

            if (overlap <= 0) {
                return Optional.empty();
            }

            if (overlap < minimumOverlap) {
                minimumOverlap = overlap;
                smallestAxis = normalizedAxis;
            }
        }

        if (smallestAxis == null) {
            return Optional.empty();
        }

        VectorDouble direction = second.position().sub(first.position());
        if (smallestAxis.dotProduct(direction) < 0) {
            smallestAxis = smallestAxis.negate();
        }

        VectorDouble contactOnFirst = supportPoint(firstOriented, smallestAxis);
        VectorDouble contactOnSecond = supportPoint(secondOriented, smallestAxis.negate());
        VectorDouble contactPoint = contactOnFirst.add(contactOnSecond).scale(0.5);

        return Optional.of(new Collision(firstIndex, secondIndex, smallestAxis, minimumOverlap, contactPoint));
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

        VectorDouble relative = circleCenter.sub(rectangleCenter);
        VectorDouble localCircle = rotate(relative, -rectangleBody.orientation());

        VectorDouble localClosest = localCircle.clamp(
                new VectorDouble(-rectangle.halfWidth(), -rectangle.halfHeight()),
                new VectorDouble(rectangle.halfWidth(), rectangle.halfHeight())
        );

        VectorDouble localDelta = localCircle.sub(localClosest);
        double distanceSquared = localDelta.radiusSquared();
        if (distanceSquared > circle.radius() * circle.radius()) {
            return Optional.empty();
        }

        double distance = Math.sqrt(distanceSquared);
        VectorDouble localNormal = distance == 0 ? new VectorDouble(1, 0) : localDelta.scale(1 / distance);
        VectorDouble worldNormal = rotate(localNormal, rectangleBody.orientation());
        VectorDouble contactPoint = rotate(localClosest, rectangleBody.orientation()).add(rectangleCenter);
        double penetration = circle.radius() - distance;
        return Optional.of(new Collision(circleIndex, rectangleIndex, worldNormal, penetration, contactPoint));
    }

    private static Optional<Collision> detectLineCircle(
            int lineIndex,
            int circleIndex,
            Body lineBody,
            Body circleBody,
            LineSegment line,
            Circle circle
    ) {
        VectorDouble start = rotate(line.start(), lineBody.orientation()).add(lineBody.position());
        VectorDouble end = rotate(line.end(), lineBody.orientation()).add(lineBody.position());
        VectorDouble closest = closestPointOnSegment(start, end, circleBody.position());
        VectorDouble delta = circleBody.position().sub(closest);
        double distanceSquared = delta.radiusSquared();
        double radiusSquared = circle.radius() * circle.radius();
        if (distanceSquared > radiusSquared) {
            return Optional.empty();
        }

        double distance = Math.sqrt(distanceSquared);
        VectorDouble segmentDirection = end.sub(start);
        VectorDouble normal = distance == 0 ? segmentDirection.tangent().normalize() : delta.scale(1 / distance);
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
        VectorDouble worldStart = rotate(line.start(), lineBody.orientation()).add(lineBody.position());
        VectorDouble worldEnd = rotate(line.end(), lineBody.orientation()).add(lineBody.position());

        double orientation = rectangleBody.orientation();
        VectorDouble rectCenter = rectangleBody.position();

        VectorDouble localStart = rotate(worldStart.sub(rectCenter), -orientation);
        VectorDouble localEnd = rotate(worldEnd.sub(rectCenter), -orientation);

        Optional<Collision> localCollision = detectLineRectangleLocal(lineIndex, rectangleIndex, localStart, localEnd, rectangle);
        if (localCollision.isEmpty()) {
            return Optional.empty();
        }

        Collision collision = localCollision.get();
        VectorDouble worldNormal = rotate(collision.normal(), orientation);
        VectorDouble worldContact = rotate(collision.contactPoint(), orientation).add(rectCenter);
        return Optional.of(new Collision(lineIndex, rectangleIndex, worldNormal, collision.penetration(), worldContact));
    }

    private static Optional<Collision> detectLineRectangleLocal(
            int lineIndex,
            int rectangleIndex,
            VectorDouble start,
            VectorDouble end,
            AxisAlignedRectangle rectangle
    ) {
        double minX = -rectangle.halfWidth();
        double maxX = rectangle.halfWidth();
        double minY = -rectangle.halfHeight();
        double maxY = rectangle.halfHeight();

        if (isPointInsideRectangle(start, minX, maxX, minY, maxY)) {
            VectorDouble normal = outwardNormal(new VectorDouble(0, 0), start);
            double penetration = Math.min(maxX - start.x(), Math.min(start.x() - minX, Math.min(maxY - start.y(), start.y() - minY)));
            return Optional.of(new Collision(lineIndex, rectangleIndex, normal, penetration, start));
        }
        if (isPointInsideRectangle(end, minX, maxX, minY, maxY)) {
            VectorDouble normal = outwardNormal(new VectorDouble(0, 0), end);
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
        VectorDouble normal = outwardNormal(new VectorDouble(0, 0), contact);
        return Optional.of(new Collision(lineIndex, rectangleIndex, normal, 0.0, contact));
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

    private static double projectionRadius(OrientedRectangle rectangle, VectorDouble axis) {
        VectorDouble normalizedAxis = axis.normalize();
        double projectionX = Math.abs(normalizedAxis.dotProduct(rectangle.axisX())) * rectangle.halfWidth();
        double projectionY = Math.abs(normalizedAxis.dotProduct(rectangle.axisY())) * rectangle.halfHeight();
        return projectionX + projectionY;
    }

    private static VectorDouble supportPoint(OrientedRectangle rectangle, VectorDouble direction) {
        double signX = Math.signum(direction.dotProduct(rectangle.axisX()));
        double signY = Math.signum(direction.dotProduct(rectangle.axisY()));
        VectorDouble offset = rectangle.axisX().scale(rectangle.halfWidth() * signX)
                .add(rectangle.axisY().scale(rectangle.halfHeight() * signY));
        return rectangle.center().add(offset);
    }

    private static VectorDouble rotate(VectorDouble vector, double angle) {
        return vector.rotateAboutOrigin(angle);
    }

    private static VectorDouble orientationAxis(double orientation) {
        return new VectorDouble(Math.cos(orientation), Math.sin(orientation));
    }

    private record OrientedRectangle(
            VectorDouble center,
            double halfWidth,
            double halfHeight,
            double orientation,
            VectorDouble axisX,
            VectorDouble axisY
    ) {

        private OrientedRectangle(VectorDouble center, double halfWidth, double halfHeight, double orientation) {
            this(center, halfWidth, halfHeight, orientation, orientationAxis(orientation), orientationAxis(orientation).tangent());
        }
    }

    private static Collision flipNormal(Collision collision, int firstIndex, int secondIndex) {
        return new Collision(firstIndex, secondIndex, collision.normal().negate(), collision.penetration(), collision.contactPoint());
    }
}
