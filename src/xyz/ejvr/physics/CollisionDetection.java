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
        List<Aabb> bounds = bodies.stream().map(Body::aabb).toList();
        List<Collision> collisions = new ArrayList<>();
        IntStream.range(0, bodyCount).forEach(firstIndex ->
                IntStream.range(firstIndex + 1, bodyCount)
                        .filter(secondIndex -> bounds.get(firstIndex).intersects(bounds.get(secondIndex)))
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
        if (isRectangle(firstShape) && isRectangle(secondShape)) {
            return detectOrientedRectangles(firstIndex, secondIndex, first, second, rectangleDimensions(firstShape), rectangleDimensions(secondShape));
        }
        if (firstShape instanceof Circle circle && isRectangle(secondShape)) {
            return detectCircleRectangle(firstIndex, secondIndex, first, second, circle, rectangleDimensions(secondShape));
        }
        if (isRectangle(firstShape) && secondShape instanceof Circle circle) {
            return detectCircleRectangle(secondIndex, firstIndex, second, first, circle, rectangleDimensions(firstShape))
                    .map(collision -> flipNormal(collision, firstIndex, secondIndex));
        }
        if (firstShape instanceof LineSegment line && secondShape instanceof Circle circle) {
            return detectLineCircle(firstIndex, secondIndex, first, second, line, circle);
        }
        if (firstShape instanceof Circle circle && secondShape instanceof LineSegment line) {
            return detectLineCircle(secondIndex, firstIndex, second, first, line, circle)
                    .map(collision -> flipNormal(collision, firstIndex, secondIndex));
        }
        if (firstShape instanceof LineSegment line && isRectangle(secondShape)) {
            return detectLineRectangle(firstIndex, secondIndex, first, second, line, rectangleDimensions(secondShape));
        }
        if (isRectangle(firstShape) && secondShape instanceof LineSegment line) {
            return detectLineRectangle(secondIndex, firstIndex, second, first, line, rectangleDimensions(firstShape))
                    .map(collision -> flipNormal(collision, firstIndex, secondIndex));
        }

        return Optional.empty();
    }

    private static boolean isRectangle(Shape shape) {
        return shape instanceof AxisAlignedRectangle || shape instanceof RotatedRectangle;
    }

    private static RectangleDimensions rectangleDimensions(Shape shape) {
        return switch (shape) {
            case AxisAlignedRectangle rectangle -> new RectangleDimensions(rectangle.halfWidth(), rectangle.halfHeight());
            case RotatedRectangle rectangle -> new RectangleDimensions(rectangle.halfWidth(), rectangle.halfHeight());
            default -> throw new IllegalArgumentException("Unsupported rectangle shape: " + shape.getClass());
        };
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

    private static Optional<Collision> detectOrientedRectangles(
            int firstIndex,
            int secondIndex,
            Body first,
            Body second,
            RectangleDimensions firstRect,
            RectangleDimensions secondRect
    ) {
        VectorDouble[] firstVertices = first.rectangleVertices(firstRect.halfWidth(), firstRect.halfHeight());
        VectorDouble[] secondVertices = second.rectangleVertices(secondRect.halfWidth(), secondRect.halfHeight());

        List<VectorDouble> axes = List.of(
                rectangleAxis(first.orientation()),
                rectanglePerpendicularAxis(first.orientation()),
                rectangleAxis(second.orientation()),
                rectanglePerpendicularAxis(second.orientation())
        );

        double minimumOverlap = Double.POSITIVE_INFINITY;
        VectorDouble collisionAxis = null;

        for (VectorDouble axis : axes) {
            Projection firstProjection = projectOnto(firstVertices, axis);
            Projection secondProjection = projectOnto(secondVertices, axis);
            double overlap = firstProjection.overlap(secondProjection);

            if (overlap <= 0) {
                return Optional.empty();
            }

            if (overlap < minimumOverlap) {
                minimumOverlap = overlap;
                collisionAxis = axis;
            }
        }

        if (collisionAxis == null) {
            return Optional.empty();
        }

        VectorDouble normal = collisionAxis.normalize();
        VectorDouble delta = second.position().sub(first.position());
        if (delta.dotProduct(normal) < 0) {
            normal = normal.negate();
        }

        VectorDouble firstSupport = supportPoint(firstVertices, normal);
        VectorDouble secondSupport = supportPoint(secondVertices, normal.negate());
        VectorDouble contactPoint = firstSupport.add(secondSupport).scale(0.5);

        return Optional.of(new Collision(firstIndex, secondIndex, normal, minimumOverlap, contactPoint));
    }

    private static Optional<Collision> detectCircleRectangle(
            int circleIndex,
            int rectangleIndex,
            Body circleBody,
            Body rectangleBody,
            Circle circle,
            RectangleDimensions rectangle
    ) {
        VectorDouble circleCenter = circleBody.position();
        VectorDouble rectangleCenter = rectangleBody.position();

        VectorDouble relative = circleCenter.sub(rectangleCenter);
        VectorDouble localCircle = rotate(relative, -rectangleBody.orientation());

        double clampedX = Math.max(-rectangle.halfWidth(), Math.min(rectangle.halfWidth(), localCircle.x()));
        double clampedY = Math.max(-rectangle.halfHeight(), Math.min(rectangle.halfHeight(), localCircle.y()));

        VectorDouble closestLocal = new VectorDouble(clampedX, clampedY);
        VectorDouble closestWorld = rotate(closestLocal, rectangleBody.orientation()).add(rectangleCenter);

        VectorDouble delta = circleCenter.sub(closestWorld);
        double distanceSquared = delta.radiusSquared();
        double radiusSquared = circle.radius() * circle.radius();
        if (distanceSquared > radiusSquared) {
            return Optional.empty();
        }

        VectorDouble normal;
        if (distanceSquared == 0) {
            VectorDouble absLocal = new VectorDouble(Math.abs(localCircle.x()), Math.abs(localCircle.y()));
            VectorDouble localNormal;
            if (absLocal.x() > absLocal.y()) {
                localNormal = new VectorDouble(Math.signum(localCircle.x()), 0);
            } else {
                localNormal = new VectorDouble(0, Math.signum(localCircle.y()));
            }
            if (localNormal.radiusSquared() == 0) {
                localNormal = new VectorDouble(1, 0);
            }
            normal = rotate(localNormal, rectangleBody.orientation());
        } else {
            double distance = Math.sqrt(distanceSquared);
            normal = delta.scale(1 / distance);
        }

        double penetration = circle.radius() - Math.sqrt(distanceSquared);
        return Optional.of(new Collision(circleIndex, rectangleIndex, normal, penetration, closestWorld));
    }

    private static Optional<Collision> detectLineCircle(
            int lineIndex,
            int circleIndex,
            Body lineBody,
            Body circleBody,
            LineSegment line,
            Circle circle
    ) {
        VectorDouble start = lineBody.rotatePoint(line.start()).add(lineBody.position());
        VectorDouble end = lineBody.rotatePoint(line.end()).add(lineBody.position());
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
            RectangleDimensions rectangle
    ) {
        VectorDouble start = lineBody.rotatePoint(line.start()).add(lineBody.position());
        VectorDouble end = lineBody.rotatePoint(line.end()).add(lineBody.position());

        VectorDouble[] rectangleVertices = rectangleBody.rectangleVertices(rectangle.halfWidth(), rectangle.halfHeight());
        List<LineSegment> edges = rectangleEdges(rectangleVertices);

        if (isPointInsideOrientedRectangle(start, rectangleBody, rectangle)) {
            VectorDouble normal = outwardNormal(rectangleBody, rectangle, start);
            double penetration = penetrationDepth(rectangleBody, rectangle, start);
            return Optional.of(new Collision(lineIndex, rectangleIndex, normal, penetration, start));
        }
        if (isPointInsideOrientedRectangle(end, rectangleBody, rectangle)) {
            VectorDouble normal = outwardNormal(rectangleBody, rectangle, end);
            double penetration = penetrationDepth(rectangleBody, rectangle, end);
            return Optional.of(new Collision(lineIndex, rectangleIndex, normal, penetration, end));
        }

        Optional<VectorDouble> intersection = edges.stream()
                .map(edge -> segmentIntersection(start, end, edge.start(), edge.end()))
                .filter(Optional::isPresent)
                .map(Optional::get)
                .findFirst();

        if (intersection.isEmpty()) {
            return Optional.empty();
        }

        VectorDouble contact = intersection.get();
        VectorDouble normal = outwardNormal(rectangleBody, rectangle, contact);
        double penetration = 0.0;
        return Optional.of(new Collision(lineIndex, rectangleIndex, normal, penetration, contact));
    }

    private static VectorDouble outwardNormal(Body rectangleBody, RectangleDimensions rectangle, VectorDouble contactPoint) {
        VectorDouble localPoint = rotate(contactPoint.sub(rectangleBody.position()), -rectangleBody.orientation());
        double absX = Math.abs(localPoint.x());
        double absY = Math.abs(localPoint.y());
        VectorDouble localNormal;
        if (absX > absY) {
            double sign = Math.signum(localPoint.x());
            localNormal = new VectorDouble(sign == 0 ? 1 : sign, 0);
        } else {
            double sign = Math.signum(localPoint.y());
            localNormal = new VectorDouble(0, sign == 0 ? 1 : sign);
        }
        return rotate(localNormal, rectangleBody.orientation());
    }

    private static double penetrationDepth(Body rectangleBody, RectangleDimensions rectangle, VectorDouble point) {
        VectorDouble localPoint = rotate(point.sub(rectangleBody.position()), -rectangleBody.orientation());
        double remainingX = rectangle.halfWidth() - Math.abs(localPoint.x());
        double remainingY = rectangle.halfHeight() - Math.abs(localPoint.y());
        return Math.min(remainingX, remainingY);
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

    private static boolean isPointInsideOrientedRectangle(VectorDouble point, Body body, RectangleDimensions rectangle) {
        VectorDouble local = rotate(point.sub(body.position()), -body.orientation());
        return Math.abs(local.x()) <= rectangle.halfWidth() && Math.abs(local.y()) <= rectangle.halfHeight();
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

    private static VectorDouble rotate(VectorDouble vector, double angle) {
        return VectorDouble.rotatePoint(0, 0, angle, vector);
    }

    private static VectorDouble rectangleAxis(double orientation) {
        return new VectorDouble(Math.cos(orientation), Math.sin(orientation));
    }

    private static VectorDouble rectanglePerpendicularAxis(double orientation) {
        return new VectorDouble(-Math.sin(orientation), Math.cos(orientation));
    }

    private static Projection projectOnto(VectorDouble[] vertices, VectorDouble axis) {
        double min = vertices[0].dotProduct(axis);
        double max = min;
        for (int i = 1; i < vertices.length; i++) {
            double projection = vertices[i].dotProduct(axis);
            min = Math.min(min, projection);
            max = Math.max(max, projection);
        }
        return new Projection(min, max);
    }

    private static VectorDouble supportPoint(VectorDouble[] vertices, VectorDouble direction) {
        VectorDouble furthest = vertices[0];
        double maxProjection = furthest.dotProduct(direction);
        for (int i = 1; i < vertices.length; i++) {
            double projection = vertices[i].dotProduct(direction);
            if (projection > maxProjection) {
                maxProjection = projection;
                furthest = vertices[i];
            }
        }
        return furthest;
    }

    private static List<LineSegment> rectangleEdges(VectorDouble[] vertices) {
        return List.of(
                new LineSegment(vertices[0], vertices[1]),
                new LineSegment(vertices[1], vertices[2]),
                new LineSegment(vertices[2], vertices[3]),
                new LineSegment(vertices[3], vertices[0])
        );
    }

    private record Projection(double min, double max) {
        double overlap(Projection other) {
            double overlap = Math.min(max, other.max) - Math.max(min, other.min);
            return overlap;
        }
    }

    private record RectangleDimensions(double halfWidth, double halfHeight) {
    }

    private static Collision flipNormal(Collision collision, int firstIndex, int secondIndex) {
        return new Collision(firstIndex, secondIndex, collision.normal().negate(), collision.penetration(), collision.contactPoint());
    }
}
