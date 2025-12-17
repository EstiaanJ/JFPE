package xyz.ejvr.physics;

import java.util.Objects;

public record Body(
        Shape shape,
        VectorDouble position,
        VectorDouble velocity,
        VectorDouble acceleration,
        double mass,
        double restitution,
        boolean immovable
) {

    public Body {
        Objects.requireNonNull(shape, "Body shape cannot be null");
        Objects.requireNonNull(position, "Body position cannot be null");
        Objects.requireNonNull(velocity, "Body velocity cannot be null");
        Objects.requireNonNull(acceleration, "Body acceleration cannot be null");

        if (mass <= 0 && !immovable) {
            throw new IllegalArgumentException("Mass must be positive for movable bodies");
        }
        if (restitution < 0 || restitution > 1) {
            throw new IllegalArgumentException("Restitution must be between 0 and 1");
        }
    }

    public double inverseMass() {
        return immovable ? 0.0 : 1.0 / mass;
    }

    public Body integrate(double deltaTime) {
        VectorDouble newVelocity = velocity.add(acceleration.scale(deltaTime));
        VectorDouble newPosition = position.add(newVelocity.scale(deltaTime));
        return withKinematics(newPosition, newVelocity);
    }

    public Body withKinematics(VectorDouble newPosition, VectorDouble newVelocity) {
        return new Body(shape, newPosition, newVelocity, acceleration, mass, restitution, immovable);
    }

    public Body withVelocity(VectorDouble newVelocity) {
        return new Body(shape, position, newVelocity, acceleration, mass, restitution, immovable);
    }

    public Body withPosition(VectorDouble newPosition) {
        return new Body(shape, newPosition, velocity, acceleration, mass, restitution, immovable);
    }
}
