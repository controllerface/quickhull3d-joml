package com.github.quickhull3d;

/*
 * #%L
 * A Robust 3D Convex Hull Algorithm in Java
 * %%
 * Copyright (C) 2004 - 2014 John E. Lloyd
 * %%
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 * #L%
 */

import java.util.Random;

/**
 * A three-element vector. This class is actually a reduced version of the
 * Vector3d class contained in the author's matlib package (which was partly
 * inspired by javax.vecmath). Only a mininal number of methods which are
 * relevant to convex hull generation are supplied here.
 * 
 * @author John E. Lloyd, Fall 2004
 */
public class Vector3d {

    /**
     * Precision of a double.
     */
    static private final double DOUBLE_PREC = 2.2204460492503131e-16;

    /**
     * First element
     */
    public double x;

    /**
     * Second element
     */
    public double y;

    /**
     * Third element
     */
    public double z;

    /**
     * Creates a 3-vector and initializes its elements to 0.
     */
    public Vector3d() {
    }

    /**
     * Creates a 3-vector with the supplied element values.
     * 
     * @param x
     *            first element
     * @param y
     *            second element
     * @param z
     *            third element
     */
    public Vector3d(double x, double y, double z) {
        set(x, y, z);
    }

    /**
     * Sets the values of this vector to those of v1.
     * 
     * @param v1
     *            vector whose values are copied
     */
    public void set(Vector3d v1) {
        x = v1.x;
        y = v1.y;
        z = v1.z;
    }

    /**
     * Adds this vector to v1 and places the result in this vector.
     * 
     * @param v1
     *            right-hand vector
     */
    public void add(Vector3d v1) {
        x += v1.x;
        y += v1.y;
        z += v1.z;
    }

    /**
     * Subtracts vector v1 from v2 and places the result in this vector.
     * 
     * @param v1
     *            left-hand vector
     * @param v2
     *            right-hand vector
     */
    public void sub(Vector3d v1, Vector3d v2) {
        x = v1.x - v2.x;
        y = v1.y - v2.y;
        z = v1.z - v2.z;
    }

    /**
     * Subtracts v1 from this vector and places the result in this vector.
     * 
     * @param v1
     *            right-hand vector
     */
    public void sub(Vector3d v1) {
        x -= v1.x;
        y -= v1.y;
        z -= v1.z;
    }

    /**
     * Scales the elements of this vector by <code>s</code>.
     * 
     * @param s
     *            scaling factor
     */
    public void scale(double s) {
        x = s * x;
        y = s * y;
        z = s * z;
    }

    /**
     * Scales the elements of this vector <code>s</code> and places the results
     * in v1.
     * 
     * @param s
     *            scaling factor
     * @param v1
     *            destination
     */
    public void scale(double s, Vector3d v1) {
        v1.x = this.x * s;
        v1.y = this.y * s;
        v1.z = this.z * s;
    }

    /**
     * Returns the 2 norm of this vector. This is the square root of the sum of
     * the squares of the elements.
     * 
     * @return vector 2 norm
     */
    public double norm() {
        return Math.sqrt(x * x + y * y + z * z);
    }

    /**
     * Returns the square of the 2 norm of this vector. This is the sum of the
     * squares of the elements.
     * 
     * @return square of the 2 norm
     */
    public double normSquared() {
        return x * x + y * y + z * z;
    }

    /**
     * Returns the Euclidean distance between this vector and vector v.
     * 
     * @return distance between this vector and v
     */
    public double distance(Vector3d v) {
        double dx = x - v.x;
        double dy = y - v.y;
        double dz = z - v.z;

        return Math.sqrt(dx * dx + dy * dy + dz * dz);
    }

    /**
     * Returns the squared of the Euclidean distance between this vector and
     * vector v.
     * 
     * @return squared distance between this vector and v
     */
    public double distanceSquared(Vector3d v) {
        double dx = x - v.x;
        double dy = y - v.y;
        double dz = z - v.z;

        return dx * dx + dy * dy + dz * dz;
    }

    /**
     * Returns the dot product of this vector and v1.
     * 
     * @param v1
     *            right-hand vector
     * @return dot product
     */
    public double dot(Vector3d v1) {
        return x * v1.x + y * v1.y + z * v1.z;
    }

    /**
     * Normalizes this vector in place.
     */
    public void normalize() {
        double lenSqr = x * x + y * y + z * z;
        double err = lenSqr - 1;
        if (err > (2 * DOUBLE_PREC) || err < -(2 * DOUBLE_PREC)) {
            double len = Math.sqrt(lenSqr);
            x /= len;
            y /= len;
            z /= len;
        }
    }

    /**
     * Sets the elements of this vector to zero.
     */
    public void setZero() {
        x = 0;
        y = 0;
        z = 0;
    }

    /**
     * Sets the elements of this vector to the prescribed values.
     * 
     * @param x
     *            value for first element
     * @param y
     *            value for second element
     * @param z
     *            value for third element
     */
    public void set(double x, double y, double z) {
        this.x = x;
        this.y = y;
        this.z = z;
    }

    /**
     * Computes the cross product of this vector and v1 and places the result in v2.
     * 
     * @param v
     *            right-hand vector
     * @param v2
     *            destination
     */
    public void cross(Vector3d v, Vector3d v2) {
        double rx = org.joml.Math.fma(this.y, v.z, -this.z * v.y);
        double ry = org.joml.Math.fma(this.z, v.x, -this.x * v.z);
        double rz = org.joml.Math.fma(this.x, v.y, -this.y * v.x);
        v2.x = rx;
        v2.y = ry;
        v2.z = rz;
    }

    /**
     * Sets the elements of this vector to uniformly distributed random values
     * in a specified range, using a supplied random number generator.
     * 
     * @param lower
     *            lower random value (inclusive)
     * @param upper
     *            upper random value (exclusive)
     * @param generator
     *            random number generator
     */
    protected void setRandom(double lower, double upper, Random generator) {
        double range = upper - lower;

        x = generator.nextDouble() * range + lower;
        y = generator.nextDouble() * range + lower;
        z = generator.nextDouble() * range + lower;
    }

    /**
     * Returns a string representation of this vector, consisting of the x, y,
     * and z coordinates.
     * 
     * @return string representation
     */
    public String toString() {
        return x + " " + y + " " + z;
    }
}
