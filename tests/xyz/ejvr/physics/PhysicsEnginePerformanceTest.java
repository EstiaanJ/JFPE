package xyz.ejvr.physics;

import org.junit.jupiter.api.Test;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.DoubleSummaryStatistics;
import java.util.List;
import java.util.Random;
import java.util.stream.DoubleStream;
import java.util.stream.IntStream;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

class PhysicsEnginePerformanceTest {

    private static final VectorDouble ZERO = new VectorDouble(0, 0);
    private static final Random RANDOM = new Random(42);

    @Test
    void measuresFramesPerSecondForThousandBodies() {
        Boundary boundary = new Boundary(0, 0, 800, 800);
        List<Body> seedBodies = IntStream.range(0, 1000)
                .mapToObj(index -> randomMovingCircle(boundary))
                .toList();

        List<Double> fpsSamples = new ArrayList<>();
        IntStream.range(0, 15).forEach(iteration -> {
            World world = new World(seedBodies, List.of(boundary));
            fpsSamples.add(measureFramesPerSecond(world, 1.0 / 120.0));
        });

        double mean = fpsSamples.stream().mapToDouble(Double::doubleValue).average().orElse(0.0);
        double median = median(fpsSamples);
        DoubleSummaryStatistics stats = fpsSamples.stream().mapToDouble(Double::doubleValue).summaryStatistics();
        double range = stats.getMax() - stats.getMin();

        System.out.printf("1000-body FPS over five runs -> mean: %.2f, median: %.2f, range: %.2f%n", mean, median, range);

        assertTrue(mean > 0, "Mean FPS should be positive");
        assertTrue(median > 0, "Median FPS should be positive");
        assertFalse(Double.isNaN(range), "FPS range should be a number");
    }

    @Test
    void measuresCollisionDetectionAndResolutionTimeAcrossShapes() {
        List<Shape> shapes = List.of(
                new Circle(1),
                new AxisAlignedRectangle(1, 1),
                new LineSegment(new VectorDouble(-1, 0), new VectorDouble(1, 0))
        );
        List<Double> speeds = List.of(20.0, 50.0, 80.0, 110.0, 140.0);

        List<Double> timingsMillis = new ArrayList<>();
        shapes.forEach(first -> shapes.forEach(second ->
                speeds.forEach(speed -> timingsMillis.add(
                        measureCollisionPipeline(first, second, speed, 0.1)
                ))
        ));

        double averageMillis = timingsMillis.stream().mapToDouble(Double::doubleValue).average().orElse(0.0);
        double medianMillis = median(timingsMillis);
        DoubleSummaryStatistics stats = timingsMillis.stream().mapToDouble(Double::doubleValue).summaryStatistics();

        System.out.printf(
                "Collision detection + resolution timings (ms) -> mean: %.3f, median: %.3f, min: %.3f, max: %.3f%n",
                averageMillis,
                medianMillis,
                stats.getMin(),
                stats.getMax()
        );

        assertTrue(averageMillis > 0, "Average collision processing time should be recorded");
        assertTrue(medianMillis > 0, "Median collision processing time should be recorded");
    }

    private static double measureFramesPerSecond(World world, double deltaTime) {
        long startNanos = System.nanoTime();
        long deadline = startNanos + 1_000_000_000L;
        long frames = 0;
        World current = world;

        while (System.nanoTime() < deadline) {
            current = PhysicsEngine.step(current, deltaTime);
            frames++;
        }

        double elapsedSeconds = (System.nanoTime() - startNanos) / 1_000_000_000.0;
        return frames / elapsedSeconds;
    }

    private static double measureCollisionPipeline(Shape firstShape, Shape secondShape, double speed, double durationSeconds) {
        Body first = new Body(firstShape, new VectorDouble(-5, 0), new VectorDouble(speed, 0), ZERO, 0.0, 0.0, 0.0, 1, 1, 0.0, false);
        Body second = new Body(secondShape, new VectorDouble(5, 0), new VectorDouble(-speed, 0), ZERO, 0.0, 0.0, 0.0, 1, 1, 0.0, false);

        List<Body> bodies = List.of(first, second);
        double elapsed = 0.0;
        double deltaTime = 1.0 / 120.0;
        long accumulatedNanos = 0;

        while (elapsed < durationSeconds) {
            List<Body> integrated = bodies.stream()
                    .map(body -> body.integrate(deltaTime))
                    .toList();

            long startNanos = System.nanoTime();
            List<Collision> collisions = CollisionDetection.detectAll(integrated);
            bodies = CollisionResolution.resolve(integrated, collisions);
            accumulatedNanos += System.nanoTime() - startNanos;

            elapsed += deltaTime;
        }

        return accumulatedNanos / 1_000_000.0;
    }

    private static Body randomMovingCircle(Boundary boundary) {
        double radius = 3;
        double x = radius + RANDOM.nextDouble(boundary.maxX() - 2 * radius);
        double y = radius + RANDOM.nextDouble(boundary.maxY() - 2 * radius);
        double angle = RANDOM.nextDouble() * Math.PI * 2;
        double speed = 30 + RANDOM.nextDouble() * 70;
        VectorDouble velocity = new VectorDouble(Math.cos(angle) * speed, Math.sin(angle) * speed);
        return new Body(new Circle(radius), new VectorDouble(x, y), velocity, ZERO, 0.0, 0.0, 0.0, 1, 0.8, 0.02, false);
    }

    private static double median(List<Double> values) {
        return values.stream()
                .sorted(Comparator.naturalOrder())
                .skip((values.size() - 1L) / 2)
                .limit(2 - values.size() % 2)
                .flatMapToDouble(DoubleStream::of)
                .average()
                .orElse(0.0);
    }
}
