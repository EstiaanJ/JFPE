package xyz.ejvr.physics;

import java.util.Objects;

public record Body(
        Shape shape,
        VectorDouble position,
        VectorDouble velocity,
        VectorDouble acceleration,
        double orientation,
        double angularVelocity,
        double angularAcceleration,
        double mass,
        double restitution,
        double drag,
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
        if (drag < 0) {
            throw new IllegalArgumentException("Drag cannot be negative");
        }
    }

    public double inverseMass() {
        return immovable ? 0.0 : 1.0 / mass;
    }

    public double inverseInertia() {
        if (immovable) {
            return 0.0;
        }
        double inertia = shape.momentOfInertia(mass);
        return inertia == 0.0 ? 0.0 : 1.0 / inertia;
    }

    public Body integrate(double deltaTime) {
        VectorDouble accelerationDelta = acceleration.scale(deltaTime);
        VectorDouble newVelocity = applyLinearDrag(velocity.add(accelerationDelta), deltaTime);
        VectorDouble newPosition = position.add(newVelocity.scale(deltaTime));

        double newAngularVelocity = applyAngularDrag(angularVelocity + angularAcceleration * deltaTime, deltaTime);
        double newOrientation = orientation + newAngularVelocity * deltaTime;

        return withKinematics(newPosition, newVelocity, newOrientation, newAngularVelocity);
    }

    public Body withKinematics(VectorDouble newPosition, VectorDouble newVelocity) {
        return withKinematics(newPosition, newVelocity, orientation, angularVelocity);
    }

    public Body withKinematics(VectorDouble newPosition, VectorDouble newVelocity, double newOrientation, double newAngularVelocity) {
        return new Body(
                shape,
                newPosition,
                newVelocity,
                acceleration,
                newOrientation,
                newAngularVelocity,
                angularAcceleration,
                mass,
                restitution,
                drag,
                immovable
        );
    }

    public Body withVelocity(VectorDouble newVelocity) {
        return new Body(shape, position, newVelocity, acceleration, orientation, angularVelocity, angularAcceleration, mass, restitution, drag, immovable);
    }

    public Body withPosition(VectorDouble newPosition) {
        return new Body(shape, newPosition, velocity, acceleration, orientation, angularVelocity, angularAcceleration, mass, restitution, drag, immovable);
    }

    public Body withOrientation(double newOrientation) {
        return new Body(shape, position, velocity, acceleration, newOrientation, angularVelocity, angularAcceleration, mass, restitution, drag, immovable);
    }

    public Body withAngularVelocity(double newAngularVelocity) {
        return new Body(shape, position, velocity, acceleration, orientation, newAngularVelocity, angularAcceleration, mass, restitution, drag, immovable);
    }

    private VectorDouble applyLinearDrag(VectorDouble candidateVelocity, double deltaTime) {
        double dragFactor = Math.max(0.0, 1.0 - drag * deltaTime);
        return candidateVelocity.scale(dragFactor);
    }

    private double applyAngularDrag(double candidateAngularVelocity, double deltaTime) {
        double dragFactor = Math.max(0.0, 1.0 - drag * deltaTime);
        return candidateAngularVelocity * dragFactor;
    }
}
