package xyz.ejvr.physics;

import processing.core.PApplet;

import java.util.List;
import java.util.Random;
import java.util.stream.IntStream;

public class ProcessingDemo extends PApplet {

    private static final int WORLD_WIDTH = 900;
    private static final int WORLD_HEIGHT = 600;
    private static final int BODY_COUNT = 12;
    private static final double MIN_RADIUS = 12;
    private static final double MAX_RADIUS = 32;
    private static final double MAX_SPEED = 180.0;

    private World world;
    private double lastStepSeconds;

    public static void main(String[] args) {
        PApplet.main(ProcessingDemo.class);
    }

    @Override
    public void settings() {
        size(WORLD_WIDTH, WORLD_HEIGHT);
    }

    @Override
    public void setup() {
        frameRate(60);
        world = new World(createBodies(), List.of(new Boundary(0, 0, WORLD_WIDTH, WORLD_HEIGHT)));
        lastStepSeconds = seconds();
    }

    @Override
    public void draw() {
        double now = seconds();
        double delta = now - lastStepSeconds;
        lastStepSeconds = now;

        world = PhysicsEngine.step(world, delta);

        background(18, 18, 18);
        world.bodies().forEach(body -> drawBody((Circle) body.shape(), body.position()));
    }

    private void drawBody(Circle circle, VectorDouble position) {
        float diameter = (float) (circle.radius() * 2);
        float x = (float) position.x();
        float y = (float) position.y();

        noStroke();
        fill(64, 156, 255, 200);
        ellipse(x, y, diameter, diameter);
    }

    private List<Body> createBodies() {
        Random random = new Random();
        return IntStream.range(0, BODY_COUNT)
                .mapToObj(index -> createBody(random))
                .toList();
    }

    private Body createBody(Random random) {
        double radius = randomInRange(random, MIN_RADIUS, MAX_RADIUS);
        Circle shape = new Circle(radius);

        double x = randomInRange(random, radius, WORLD_WIDTH - radius);
        double y = randomInRange(random, radius, WORLD_HEIGHT - radius);

        double speed = randomInRange(random, MAX_SPEED * 0.25, MAX_SPEED);
        double angle = random.nextDouble() * Math.PI * 2;
        VectorDouble velocity = new VectorDouble(Math.cos(angle) * speed, Math.sin(angle) * speed);

        return new Body(
                shape,
                new VectorDouble(x, y),
                velocity,
                new VectorDouble(0, 0),
                radius,
                0.95,
                false
        );
    }

    private double randomInRange(Random random, double min, double max) {
        return min + (max - min) * random.nextDouble();
    }

    private double seconds() {
        return millis() / 1000.0;
    }
}
