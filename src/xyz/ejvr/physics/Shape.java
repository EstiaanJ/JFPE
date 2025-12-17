package xyz.ejvr.physics;

public sealed interface Shape permits Circle, AxisAlignedRectangle, LineSegment {

    double area();

    double boundingRadius();

    double momentOfInertia(double mass);
}
