package xyz.ejvr.physics;

import java.util.Comparator;
import java.util.List;
import java.util.Map;
import java.util.function.Function;
import java.util.stream.Collectors;
import java.util.stream.IntStream;

public final class CollisionResolution {

    private CollisionResolution() {
    }

    public static List<Body> resolve(List<Body> bodies, List<Collision> collisions) {
        Map<Integer, Body> resolvedBodies = bodiesStream(bodies);

        collisions.forEach(collision -> applyImpulse(collision, resolvedBodies));

        return resolvedBodies.entrySet().stream()
                .sorted(Comparator.comparingInt(Map.Entry::getKey))
                .map(Map.Entry::getValue)
                .toList();
    }

    private static Map<Integer, Body> bodiesStream(List<Body> bodies) {
        return IntStream.range(0, bodies.size())
                .boxed()
                .collect(Collectors.toMap(Function.identity(), bodies::get));
    }

    private static void applyImpulse(Collision collision, Map<Integer, Body> bodies) {
        Body first = bodies.get(collision.firstIndex());
        Body second = bodies.get(collision.secondIndex());

        double inverseMassA = first.inverseMass();
        double inverseMassB = second.inverseMass();
        double inverseInertiaA = first.inverseInertia();
        double inverseInertiaB = second.inverseInertia();

        double inverseMassSum = inverseMassA + inverseMassB;
        if (inverseMassSum == 0) {
            return;
        }

        VectorDouble ra = collision.contactPoint().sub(first.position());
        VectorDouble rb = collision.contactPoint().sub(second.position());

        VectorDouble firstAngularVelocity = ra.tangent().scale(first.angularVelocity());
        VectorDouble secondAngularVelocity = rb.tangent().scale(second.angularVelocity());

        VectorDouble relativeVelocity = second.velocity().add(secondAngularVelocity).sub(first.velocity().add(firstAngularVelocity));
        double velocityAlongNormal = relativeVelocity.dotProduct(collision.normal());

        if (velocityAlongNormal > 0) {
            return;
        }

        double restitution = Math.min(first.restitution(), second.restitution());
        double raCrossN = ra.crossProduct2D(collision.normal());
        double rbCrossN = rb.crossProduct2D(collision.normal());

        double angularContributionA = raCrossN * raCrossN * inverseInertiaA;
        double angularContributionB = rbCrossN * rbCrossN * inverseInertiaB;

        double impulseScalar = -(1 + restitution) * velocityAlongNormal;
        double impulseDenominator = inverseMassSum + angularContributionA + angularContributionB;
        if (impulseDenominator == 0) {
            return;
        }
        impulseScalar /= impulseDenominator;

        VectorDouble impulse = collision.normal().scale(impulseScalar);

        VectorDouble firstVelocity = first.velocity().sub(impulse.scale(inverseMassA));
        VectorDouble secondVelocity = second.velocity().add(impulse.scale(inverseMassB));

        double firstAngularVelocity = first.angularVelocity() - raCrossN * impulseScalar * inverseInertiaA;
        double secondAngularVelocity = second.angularVelocity() + rbCrossN * impulseScalar * inverseInertiaB;

        Body updatedFirst = first.withKinematics(first.position(), firstVelocity, first.orientation(), firstAngularVelocity);
        Body updatedSecond = second.withKinematics(second.position(), secondVelocity, second.orientation(), secondAngularVelocity);

        VectorDouble correction = collision.normal().scale(collision.penetration() / inverseMassSum);
        updatedFirst = updatedFirst.withPosition(updatedFirst.position().sub(correction.scale(inverseMassA)));
        updatedSecond = updatedSecond.withPosition(updatedSecond.position().add(correction.scale(inverseMassB)));

        bodies.put(collision.firstIndex(), updatedFirst);
        bodies.put(collision.secondIndex(), updatedSecond);
    }
}
