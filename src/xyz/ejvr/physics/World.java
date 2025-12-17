package xyz.ejvr.physics;

import java.util.List;
import java.util.Objects;

public record World(List<Body> bodies, List<Boundary> boundaries) {

    public World {
        Objects.requireNonNull(bodies, "World bodies cannot be null");
        Objects.requireNonNull(boundaries, "World boundaries cannot be null");
        bodies = List.copyOf(bodies);
        boundaries = List.copyOf(boundaries);
    }
}
