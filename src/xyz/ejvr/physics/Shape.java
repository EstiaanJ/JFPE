package xyz.ejvr.physics;

public sealed interface Shape permits Circle, AxisAlignedRectangle, RotatedRectangle, LineSegment {

    double area();

    double boundingRadius();
}
