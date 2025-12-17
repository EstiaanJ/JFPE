# 2D Physics Engine Architecture (Top-Down)

## Goals
- Deterministic, functional-style updates using immutable records.
- Pure Newtonian motion with simple impulse-based collision response.
- Support for circles, axis-aligned rectangles, rotated rectangles, line segments, and world boundaries.
- Broad-phase filtering with axis-aligned bounding boxes ahead of narrow-phase SAT/circle tests.

## Core Types
- `VectorDouble`: Immutable 2D vector helper for geometry and kinematics.
- `Shape` sealed interface: Implemented by `Circle`, `AxisAlignedRectangle`, and `LineSegment`.
- `Body`: Composition of `Shape` with kinematic state (position, velocity, acceleration), angular state (orientation, angular velocity/acceleration), mass properties, restitution, per-body drag, and immovable flag.
- `Boundary`: Axis-aligned world limits used for clamping and reflecting kinematics.
- `World`: Immutable aggregate of bodies and boundaries passed into simulation steps.

## Simulation Flow
1. **Integration**: Advance each `Body` with constant acceleration to compute new velocity and position (Euler integration) while applying drag to linear and angular velocities.
2. **Boundary Resolution**: Clamp shapes against each `Boundary` using their axis-aligned bounding boxes, reflecting velocity components using the body's restitution.
3. **Broad Phase**: Cull non-intersecting body pairs via AABB overlap before narrow-phase checks.
4. **Collision Detection** (narrow phase):
   - Circle ⟷ Circle
   - Oriented Rectangle ⟷ Oriented Rectangle (SAT)
   - Circle ⟷ Oriented Rectangle (local-space clamp)
   - Line Segment ⟷ Circle
   - Line Segment ⟷ Oriented Rectangle
5. **Collision Resolution**: Apply impulse-based response using combined restitution, inverse masses, inverse inertias, plus positional correction along the contact normal.
5. **Output**: Return a new `World` instance with updated `Body` states; inputs remain unchanged.

## Extensibility Notes
- Broad-phase acceleration structures can be added later by inserting a pre-filter before the narrow-phase detection in `PhysicsEngine.step`.
- Additional shapes (polygons) can extend `Shape` and plug into `CollisionDetection` without altering `Body` or `World`.
- Alternate integrators (semi-implicit Euler, RK) can replace or wrap the current integration logic while keeping the resolution pipeline intact.

## Testing Strategy
- Deterministic unit tests on individual math helpers (`VectorDouble`).
- Collision detection unit tests that assert normals, penetration depths, and kinematic responses.
- Scenario-based engine tests covering boundary bounces and inter-body impulses.
