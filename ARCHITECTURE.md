# 2D Physics Engine Architecture (Top-Down)

## Goals
- Deterministic, functional-style updates using immutable records.
- Pure Newtonian motion with simple impulse-based collision response.
- Support for circles, axis-aligned rectangles, line segments, and world boundaries.
- No broad-phase optimizations yet (brute-force narrow phase only).

## Core Types
- `VectorDouble`: Immutable 2D vector helper for geometry and kinematics.
- `Shape` sealed interface: Implemented by `Circle`, `AxisAlignedRectangle`, and `LineSegment`.
- `Body`: Composition of `Shape` with kinematic state (position, velocity, acceleration), mass, restitution, and immovable flag.
- `Boundary`: Axis-aligned world limits used for clamping and reflecting kinematics.
- `World`: Immutable aggregate of bodies and boundaries passed into simulation steps.

## Simulation Flow
1. **Integration**: Advance each `Body` with constant acceleration to compute new velocity and position (Euler integration).
2. **Boundary Resolution**: Clamp shapes against each `Boundary`, reflecting velocity components using the body's restitution.
3. **Collision Detection** (narrow phase only):
   - Circle ⟷ Circle
   - Rectangle ⟷ Rectangle (axis-aligned AABBs)
   - Circle ⟷ Rectangle
   - Line Segment ⟷ Circle
   - Line Segment ⟷ Rectangle
4. **Collision Resolution**: Apply impulse-based response using combined restitution and inverse masses, plus positional correction along the contact normal.
5. **Output**: Return a new `World` instance with updated `Body` states; inputs remain unchanged.

## Extensibility Notes
- Broad-phase acceleration structures can be added later by inserting a pre-filter before the narrow-phase detection in `PhysicsEngine.step`.
- Additional shapes (polygons) can extend `Shape` and plug into `CollisionDetection` without altering `Body` or `World`.
- Alternate integrators (semi-implicit Euler, RK) can replace or wrap the current integration logic while keeping the resolution pipeline intact.

## Testing Strategy
- Deterministic unit tests on individual math helpers (`VectorDouble`).
- Collision detection unit tests that assert normals, penetration depths, and kinematic responses.
- Scenario-based engine tests covering boundary bounces and inter-body impulses.
