package com.controllerface.quickhull3d;

import com.github.quickhull3d.Point3d;
import org.joml.Vector3d;
import org.openjdk.jmh.annotations.*;
import org.openjdk.jmh.infra.Blackhole;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStreamReader;
import java.util.ArrayList;

public class ModelCompare {

    @State(Scope.Benchmark)
    public static class ModelData {

        Point3d[] original_model_points;
        Vector3d[] new_model_points;

        @Setup(Level.Trial)
        public void setup()
        {
            original_model_points = load_original_points();
            new_model_points = load_new_points();
        }

        public Point3d[] getOriginalPoints() {
            return original_model_points;
        }

        public Vector3d[] getNewPoints() {
            return new_model_points;
        }
    }

    private static Point3d[] load_original_points()
    {
        var buffer = new ArrayList<Point3d>();
        try (var model_stream = ModelCompare.class.getResourceAsStream("/model_file");
             var is = new InputStreamReader(model_stream);
             var reader = new BufferedReader(is))
        {
            reader.lines()
                    .forEach(line ->
                    {
                        var tokens = line.split(",");
                        assert tokens.length == 3;
                        double x = Double.parseDouble(tokens[0]);
                        double y = Double.parseDouble(tokens[1]);
                        double z = Double.parseDouble(tokens[2]);
                        var point = new Point3d(x, y, z);
                        buffer.add(point);
                    });
        } catch (IOException e) {
            throw new RuntimeException(e);
        }
        return buffer.toArray(new Point3d[0]);
    }

    private static Vector3d[] load_new_points()
    {
        var buffer = new ArrayList<Vector3d>();
        try (var model_stream = ModelCompare.class.getResourceAsStream("/model_file");
             var is = new InputStreamReader(model_stream);
             var reader = new BufferedReader(is))
        {
            reader.lines()
                    .forEach(line ->
                    {
                        var tokens = line.split(",");
                        assert tokens.length == 3;
                        double x = Double.parseDouble(tokens[0]);
                        double y = Double.parseDouble(tokens[1]);
                        double z = Double.parseDouble(tokens[2]);
                        var point = new Vector3d(x, y, z);
                        buffer.add(point);
                    });
        } catch (IOException e) {
            throw new RuntimeException(e);
        }
        return buffer.toArray(new Vector3d[0]);
    }

    @Benchmark
    public void testModelNew(ModelData modelData, Blackhole blackhole) {
        QuickHull3D convexHull = new QuickHull3D(modelData.getNewPoints());
        Vector3d[] vertices = convexHull.getDoubleVertices();
        blackhole.consume(vertices);
    }

    @Benchmark
    public void testModelOriginal(ModelData modelData, Blackhole blackhole) {
        com.github.quickhull3d.QuickHull3D convexHull = new com.github.quickhull3d.QuickHull3D(modelData.getOriginalPoints());
        Point3d[] vertices = convexHull.getVertices();
        blackhole.consume(vertices);
    }
}