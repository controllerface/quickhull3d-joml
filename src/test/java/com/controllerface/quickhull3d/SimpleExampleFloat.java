package com.controllerface.quickhull3d;

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

import org.joml.Vector3f;

/**
 * Simple example usage of QuickHull3D. Run as the command
 * 
 * <pre>
 *   java com.github.quickhull3d.SimpleExample
 * </pre>
 */
public class SimpleExampleFloat
{

    /**
     * Run for a simple demonstration of QuickHull3D.
     */
    public static void main(String[] args) {
        // x y z coordinates of 6 points
        Vector3f[] points = new Vector3f[]{
            new Vector3f(0.0f, 0.0f, 0.0f),
            new Vector3f(1.0f, 0.5f, 0.0f),
            new Vector3f(2.0f, 0.0f, 0.0f),
            new Vector3f(0.5f, 0.5f, 0.5f),
            new Vector3f(0.0f, 0.0f, 2.0f),
            new Vector3f(0.1f, 0.2f, 0.3f),
            new Vector3f(0.0f, 2.0f, 0.0f),
        };

        QuickHull3D hull = new QuickHull3D();
        hull.build(points);

        System.out.println("Vertices:");
        Vector3f[] vertices = hull.getFloatVertices();
        for (int i = 0; i < vertices.length; i++) {
            Vector3f pnt = vertices[i];
            System.out.println(pnt.x + " " + pnt.y + " " + pnt.z);
        }

        System.out.println("Faces:");
        int[][] faceIndices = hull.getFaces();
        for (int i = 0; i < vertices.length; i++) {
            for (int k = 0; k < faceIndices[i].length; k++) {
                System.out.print(faceIndices[i][k] + " ");
            }
            System.out.println("");
        }
    }
}
