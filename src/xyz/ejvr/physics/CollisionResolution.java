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

        double inverseMassSum = inverseMassA + inverseMassB;
        if (inverseMassSum == 0) {
            return;
        }

        VectorDouble relativeVelocity = second.velocity().sub(first.velocity());
        double velocityAlongNormal = relativeVelocity.dotProduct(collision.normal());

        if (velocityAlongNormal > 0) {
            return;
        }

        double restitution = Math.min(first.restitution(), second.restitution());
        double impulseScalar = -(1 + restitution) * velocityAlongNormal;
        impulseScalar /= inverseMassSum;

        VectorDouble impulse = collision.normal().scale(impulseScalar);

        VectorDouble firstVelocity = first.velocity().sub(impulse.scale(inverseMassA));
        VectorDouble secondVelocity = second.velocity().add(impulse.scale(inverseMassB));

        Body updatedFirst = first.withVelocity(firstVelocity);
        Body updatedSecond = second.withVelocity(secondVelocity);

        VectorDouble correction = collision.normal().scale(collision.penetration() / inverseMassSum);
        updatedFirst = updatedFirst.withPosition(updatedFirst.position().sub(correction.scale(inverseMassA)));
        updatedSecond = updatedSecond.withPosition(updatedSecond.position().add(correction.scale(inverseMassB)));

        bodies.put(collision.firstIndex(), updatedFirst);
        bodies.put(collision.secondIndex(), updatedSecond);
    }
}
