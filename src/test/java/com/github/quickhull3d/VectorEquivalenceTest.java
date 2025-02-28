package com.github.quickhull3d;

import org.joml.Vector3d;
import org.junit.Test;

import java.util.Random;

public class VectorEquivalenceTest
{
    Random rand; // random number generator
    double diff_threshold = .000000000000001;

    @Test
    public void runTest()
    {
        rand = new Random();
        rand.setSeed(0x1234);

        double x = rand.nextDouble();
        double y = rand.nextDouble();
        double z = rand.nextDouble();

        Vector3d joml = new Vector3d(x, y, z);
        com.github.quickhull3d.Vector3d orig = new com.github.quickhull3d.Vector3d(x, y, z);

        assert joml.x == orig.x : "X difference: joml=" + joml.x + " orig: " + orig.x;
        assert joml.y == orig.y : "Y difference: joml=" + joml.y + " orig: " + orig.y;
        assert joml.z == orig.z : "Z difference: joml=" + joml.z + " orig: " + orig.z;

        double x2 = rand.nextDouble();
        double y2 = rand.nextDouble();
        double z2 = rand.nextDouble();

        Vector3d joml_2 = new Vector3d(x2, y2, z2);
        com.github.quickhull3d.Vector3d orig_2 = new com.github.quickhull3d.Vector3d(x2, y2, z2);

        joml.add(joml_2);
        orig.add(orig_2);

        assert joml.x == orig.x : "X difference: joml=" + joml.x + " orig: " + orig.x;
        assert joml.y == orig.y : "Y difference: joml=" + joml.y + " orig: " + orig.y;
        assert joml.z == orig.z : "Z difference: joml=" + joml.z + " orig: " + orig.z;

        double x3 = rand.nextDouble();
        double y3 = rand.nextDouble();
        double z3 = rand.nextDouble();

        Vector3d joml_3 = new Vector3d(x3, y3, z3);
        com.github.quickhull3d.Vector3d orig_3 = new com.github.quickhull3d.Vector3d(x3, y3, z3);

        joml.sub(joml_3);
        orig.sub(orig_3);

        assert joml.x == orig.x : "X difference: joml=" + joml.x + " orig: " + orig.x;
        assert joml.y == orig.y : "Y difference: joml=" + joml.y + " orig: " + orig.y;
        assert joml.z == orig.z : "Z difference: joml=" + joml.z + " orig: " + orig.z;

        double random_scale = rand.nextDouble();

        joml.mul(random_scale);
        orig.scale(random_scale);

        assert joml.x == orig.x : "X difference: joml=" + joml.x + " orig: " + orig.x;
        assert joml.y == orig.y : "Y difference: joml=" + joml.y + " orig: " + orig.y;
        assert joml.z == orig.z : "Z difference: joml=" + joml.z + " orig: " + orig.z;

        double random_scale2 = rand.nextDouble();

        joml.mul(random_scale2, joml_2);
        orig.scale(random_scale2, orig_2);

        assert joml_2.x == orig_2.x : "X difference: joml=" + joml_2.x + " orig: " + orig_2.x;
        assert joml_2.y == orig_2.y : "Y difference: joml=" + joml_2.y + " orig: " + orig_2.y;
        assert joml_2.z == orig_2.z : "Z difference: joml=" + joml_2.z + " orig: " + orig_2.z;

        assert orig_3.norm() == norm(joml_3) : "unequal 2-norms";

        double joml_dist = joml_2.distance(joml);
        double orig_dist = orig_2.distance(orig);

        assert joml_dist == orig_dist : "distance difference";

        double joml_dist_sq = joml_2.distanceSquared(joml);
        double orig_dist_sq = orig_2.distanceSquared(orig);

        assert joml_dist_sq == orig_dist_sq : "squared distance difference";

        double joml_dot = joml_2.dot(joml);
        double orig_dot = orig_2.dot(orig);

        assert joml_dot == orig_dot : "dot product difference";

        joml_2.cross(joml, joml_3);

        orig_2.cross(orig, orig_3);

        //orig_3.cross(orig_2, orig);

        assert joml_3.x == orig_3.x : "X difference: joml=" + joml_3.x + " orig: " + orig_3.x;
        assert joml_3.y == orig_3.y : "Y difference: joml=" + joml_3.y + " orig: " + orig_3.y;
        assert joml_3.z == orig_3.z : "Z difference: joml=" + joml_3.z + " orig: " + orig_3.z;

        joml_2.normalize();
        orig_2.normalize();

        double x_diff = joml_2.x - orig_2.x;
        double y_diff = joml_2.y - orig_2.y;
        double z_diff = joml_2.z - orig_2.z;

        assert x_diff < diff_threshold : "X difference: joml=" + x_diff;
        assert y_diff < diff_threshold : "Y difference: joml=" + y_diff;
        assert z_diff < diff_threshold : "Z difference: joml=" + z_diff;
    }

    public double norm(Vector3d vector3d) {
        return Math.sqrt(vector3d.x * vector3d.x + vector3d.y * vector3d.y + vector3d.z * vector3d.z);
    }
}
