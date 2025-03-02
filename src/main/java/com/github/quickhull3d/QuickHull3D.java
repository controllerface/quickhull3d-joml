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

import java.io.PrintStream;
import java.util.*;

import org.joml.Vector3d;
import org.joml.Vector3f;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

/**
 * Computes the convex hull of a set of three-dimensional points.
 * <p>
 * The algorithm is a three-dimensional implementation of Quickhull, as
 * described in Barber, Dobkin, and Huhdanpaa, <a
 * href=http://citeseer.ist.psu.edu/barber96quickhull.html> ``The Quickhull
 * Algorithm for Convex Hulls''</a> (ACM Transactions on Mathematical Software,
 * Vol. 22, No. 4, December 1996), and has a complexity of O(n log(n)) with
 * respect to the number of points. A well-known C implementation of Quickhull
 * that works for arbitrary dimensions is provided by <a
 * href=http://www.qhull.org>qhull</a>.
 * <p>
 * A hull is constructed by providing a set of points to either a constructor or
 * a build() method overload. After the hull is built, its
 * vertices and faces can be retrieved using {@link #getDoubleVertices() getVertices}
 * and {@link #getFaces() getFaces}. A typical usage might look like this:
 *
 * <pre>
 * // x y z coordinates of 6 points
 * Point3d[] points = new Point3d[]{
 *     new Point3d(0.0, 0.0, 0.0),
 *     new Point3d(1.0, 0.5, 0.0),
 *     new Point3d(2.0, 0.0, 0.0),
 *     new Point3d(0.5, 0.5, 0.5),
 *     new Point3d(0.0, 0.0, 2.0),
 *     new Point3d(0.1, 0.2, 0.3),
 *     new Point3d(0.0, 2.0, 0.0),
 * };
 *
 * QuickHull3D hull = new QuickHull3D();
 * hull.build(points);
 *
 * System.out.println(&quot;Vertices:&quot;);
 * Point3d[] vertices = hull.getVertices();
 * for (int i = 0; i &lt; vertices.length; i++) {
 *     Point3d pnt = vertices[i];
 *     System.out.println(pnt.x + &quot; &quot; + pnt.y + &quot; &quot; + pnt.z);
 * }
 *
 * System.out.println(&quot;Faces:&quot;);
 * int[][] faceIndices = hull.getFaces();
 * for (int i = 0; i &lt; faceIndices.length; i++) {
 *     for (int k = 0; k &lt; faceIndices[i].length; k++) {
 *         System.out.print(faceIndices[i][k] + &quot; &quot;);
 *     }
 *     System.out.println(&quot;&quot;);
 * }
 * </pre>
 *
 * <h3><a name=distTol>Robustness</h3> Because this algorithm uses floating
 * point arithmetic, it is potentially vulnerable to errors arising from
 * numerical imprecision. We address this problem in the same way as <a
 * href=http://www.qhull.org>qhull</a>, by merging faces whose edges are not
 * clearly convex. A face is convex if its edges are convex, and an edge is
 * convex if the centroid of each adjacent plane is clearly <i>below</i> the
 * plane of the other face. The centroid is considered below a plane if its
 * distance to the plane is less than the negative of a
 * {@link #getDistanceTolerance() distance tolerance}. This tolerance represents
 * the smallest distance that can be reliably computed within the available
 * numeric precision.
 * <p>
 * Numerical problems are more likely to arise in situations where data points
 * lie on or within the faces or edges of the convex hull. We have tested
 * QuickHull3D for such situations by computing the convex hull of a random
 * point set, then adding additional randomly chosen points which lie very close
 * to the hull vertices and edges, and computing the convex hull again. The hull
 * is deemed correct if {@link #check check} returns <code>true</code>. These
 * tests have been successful for a large number of trials, and so we are
 * confident that QuickHull3D is reasonably robust.
 * <h3>Merged Faces</h3> The merging of faces means that the faces returned by
 * QuickHull3D may be convex polygons instead of triangles. If triangles are
 * desired, the application may {@link #triangulate triangulate} the faces, but
 * it should be noted that this may result in triangles which are very small or
 * thin and hence difficult to perform reliable convexity tests on. In other
 * words, triangulating a merged face is likely to restore the numerical
 * problems which the merging process removed. Hence, is it possible that, after
 * triangulation, {@link #check check} will fail (the same behavior is observed
 * with triangulated output from <a href=http://www.qhull.org>qhull</a>).
 * <h3>Degenerate Input</h3>It is assumed that the input points are
 * non-degenerate in that they are not coincident, collinear, or coplanar, and
 * thus the convex hull has a non-zero volume. If the input points are detected
 * to be degenerate within the {@link #getDistanceTolerance() distance
 * tolerance}, an IllegalArgumentException will be thrown.
 *
 * @author John E. Lloyd, Fall 2004
 */
public class QuickHull3D
{
    /**
     * Logger to log to.
     */
    private static final Logger LOG = LoggerFactory.getLogger(QuickHull3D.class);

    /**
     * Specifies that (on output) vertex indices for a face should be listed in
     * clockwise order.
     */
    private static final int CLOCKWISE = 0x1;

    /**
     * Specifies that (on output) the vertex indices for a face should be
     * numbered starting from 1.
     */
    private static final int INDEXED_FROM_ONE = 0x2;

    /**
     * Specifies that (on output) the vertex indices for a face should be
     * numbered with respect to the original input points.
     */
    private static final int POINT_RELATIVE = 0x4;

    /**
     * Specifies that the distance tolerance should be computed automatically
     * from the input point data.
     */
    private static final double AUTOMATIC_TOLERANCE = -1;

    /**
     * Precision of a double.
     */
    private static final double DOUBLE_PREC = 2.2204460492503131e-16;

    /**
     * Precision of a float.
     */
    private static final float FLOAT_PREC = 1.1920929e-7f;

    private static final int NEGATIVE_INDEX = -1;

    // estimated size of the point set
    private double charLength;

    private Vertex[] pointBuffer = new Vertex[0];

    private int[] vertexPointIndices = new int[0];

    private final Face[] discardedFaces = new Face[3];

    private final Vertex[] maxVertices = new Vertex[3];

    private final Vertex[] minVertices = new Vertex[3];

    private final List<Face> faces = new ArrayList<>();

    private final List<HalfEdge> horizon = new ArrayList<>();

    private final FaceList newFaces = new FaceList();

    private final VertexList unclaimed = new VertexList();

    private final VertexList claimed = new VertexList();

    private int vertexCount;

    private int pointCount;

    private double explicitTolerance = AUTOMATIC_TOLERANCE;

    private double tolerance;

    private enum AdjacencyMergeType
    {
        NON_CONVEX,
        NON_CONVEX_WRT_LARGER_FACE,
    }

    /**
     * Creates an empty convex hull object.
     */
    public QuickHull3D()
    {
    }

    /**
     * Creates a convex hull object and initializes it to the convex hull of a
     * set of points whose coordinates are given by an array of doubles.
     *
     * @param coords x, y, and z coordinates of each input point. The length of
     *               this array will be three times the number of input points.
     * @throws IllegalArgumentException the number of input points is less than four, or the points
     *                                  appear to be coincident, collinear, or coplanar.
     */
    public QuickHull3D(double[] coords) throws IllegalArgumentException
    {
        build(coords, coords.length / 3);
    }

    /**
     * Creates a convex hull object and initializes it to the convex hull of a
     * set of points.
     *
     * @param points input points.
     * @throws IllegalArgumentException the number of input points is less than four, or the points
     *                                  appear to be coincident, collinear, or coplanar.
     */
    public QuickHull3D(Vector3d[] points) throws IllegalArgumentException
    {
        build(points, points.length);
    }

    /**
     * Creates a convex hull object and initializes it to the convex hull of a
     * set of points whose coordinates are given by an array of floats.
     *
     * @param coords x, y, and z coordinates of each input point. The length of
     *               this array will be three times the number of input points.
     * @throws IllegalArgumentException the number of input points is less than four, or the points
     *                                  appear to be coincident, collinear, or coplanar.
     */
    public QuickHull3D(float[] coords) throws IllegalArgumentException
    {
        build(coords, coords.length / 3);
    }

    /**
     * Creates a convex hull object and initializes it to the convex hull of a
     * set of points.
     *
     * @param points input points.
     * @throws IllegalArgumentException the number of input points is less than four, or the points
     *                                  appear to be coincident, collinear, or coplanar.
     */
    public QuickHull3D(Vector3f[] points) throws IllegalArgumentException
    {
        build(points, points.length);
    }

    /**
     * Constructs the convex hull of a set of points whose coordinates are given
     * by an array of doubles.
     *
     * @param coords x, y, and z coordinates of each input point. The length of
     *               this array will be three times the number of input points.
     * @throws IllegalArgumentException the number of input points is less than four, or the points
     *                                  appear to be coincident, collinear, or coplanar.
     */
    public void build(double[] coords) throws IllegalArgumentException
    {
        build(coords, coords.length / 3);
    }

    public void build(float[] coords) throws IllegalArgumentException
    {
        build(coords, coords.length / 3);
    }

    public void build(Vector3d[] points) throws IllegalArgumentException
    {
        build(points, points.length);
    }

    public void build(Vector3f[] points) throws IllegalArgumentException
    {
        build(points, points.length);
    }

    /**
     * Checks the correctness of the hull using the distance tolerance returned
     * by {@link QuickHull3D#getDistanceTolerance getDistanceTolerance}; see
     * {@link QuickHull3D#check(PrintStream, double) check(PrintStream,double)}
     * for details.
     *
     * @param ps print stream for diagnostic messages; may be set to
     *           <code>null</code> if no messages are desired.
     * @return true if the hull is valid
     * @see QuickHull3D#check(PrintStream, double)
     */
    public boolean check(PrintStream ps)
    {
        return check(ps, getDistanceTolerance());
    }

    /**
     * Returns the distance tolerance that was used for the most recently
     * computed hull. The distance tolerance is used to determine when faces are
     * unambiguously convex with respect to each other, and when points are
     * unambiguously above or below a face plane, in the presence of <a
     * href=#distTol>numerical imprecision</a>. Normally, this tolerance is
     * computed automatically for each set of input points, but it can be set
     * explicitly by the application.
     *
     * @return distance tolerance
     */
    public double getDistanceTolerance()
    {
        return tolerance;
    }

    /**
     * Triangulates any non-triangular hull faces. In some cases, due to
     * precision issues, the resulting triangles may be very thin or small, and
     * hence appear to be non-convex (this same limitation is present in <a
     * href=http://www.qhull.org>qhull</a>).
     */
    public void triangulate()
    {
        double minArea = 1000 * charLength * DOUBLE_PREC;
        newFaces.clear();
        for (Face face : faces)
        {
            if (face.mark == Face.VISIBLE)
            {
                face.triangulate(newFaces, minArea);
            }
        }
        for (Face face = newFaces.first(); face != null; face = face.next)
        {
            faces.add(face);
        }
    }

    /**
     * Returns the vertex points in this hull as an array of {@linkplain Vector3d} objects.
     *
     * @return array of vertex points
     * @see QuickHull3D#getFaces()
     */
    public Vector3d[] getDoubleVertices()
    {
        Vector3d[] vertices = new Vector3d[vertexCount];
        for (int i = 0; i < vertexCount; i++)
        {
            vertices[i] = pointBuffer[vertexPointIndices[i]].point;
        }
        return vertices;
    }

    /**
     * Returns the vertex points in this hull as an array of {@linkplain Vector3f} objects.
     *
     * @return array of vertex points
     * @see QuickHull3D#getFaces()
     */
    public Vector3f[] getFloatVertices()
    {
        Vector3f[] vertices = new Vector3f[vertexCount];
        for (int i = 0; i < vertexCount; i++)
        {
            vertices[i] = new Vector3f(pointBuffer[vertexPointIndices[i]].point);
        }
        return vertices;
    }

    /**
     * Returns an array specifying the index of each hull vertex with respect to
     * the original input points.
     *
     * @return vertex indices with respect to the original points
     */
    public int[] getVertexPointIndices()
    {
        int[] indices = new int[vertexCount];
        System.arraycopy(vertexPointIndices, 0, indices, 0, vertexCount);
        return indices;
    }

    /**
     * Returns the faces associated with this hull.
     * <p>
     * Each face is represented by an integer array which gives the indices of
     * the vertices. These indices are numbered relative to the hull vertices,
     * are zero-based, and are arranged counter-clockwise. More control over the
     * index format can be obtained using {@link #getFaces(int)
     * getFaces(indexFlags)}.
     *
     * @return array of integer arrays, giving the vertex indices for each face.
     * @see QuickHull3D#getDoubleVertices()
     * @see QuickHull3D#getFaces(int)
     */
    public int[][] getFaces()
    {
        return getFaces(0);
    }

    /**
     * Returns the faces associated with this hull.
     * <p>
     * Each face is represented by an integer array which gives the indices of
     * the vertices. By default, these indices are numbered with respect to the
     * hull vertices (as opposed to the input points), are zero-based, and are
     * arranged counter-clockwise. However, this can be changed by setting
     * {@link #POINT_RELATIVE POINT_RELATIVE}, {@link #INDEXED_FROM_ONE
     * INDEXED_FROM_ONE}, or {@link #CLOCKWISE CLOCKWISE} in the indexFlags
     * parameter.
     *
     * @param indexFlags specifies index characteristics (0 results in the default)
     * @return array of integer arrays, giving the vertex indices for each face.
     * @see QuickHull3D#getDoubleVertices()
     */
    public int[][] getFaces(int indexFlags)
    {
        int[][] allFaces = new int[faces.size()][];
        int faceIndex = 0;
        for (Face face : faces)
        {
            allFaces[faceIndex] = new int[face.vertexCount()];
            getFaceIndices(allFaces[faceIndex], face, indexFlags);
            faceIndex++;
        }
        return allFaces;
    }

    private void initBuffers(int pointCount)
    {
        if (pointBuffer.length < pointCount)
        {
            Vertex[] pointBuffer = new Vertex[pointCount];
            vertexPointIndices = new int[pointCount];
            System.arraycopy(this.pointBuffer, 0, pointBuffer, 0, this.pointBuffer.length);
            for (int i = this.pointBuffer.length; i < pointCount; i++)
            {
                pointBuffer[i] = new Vertex();
            }
            this.pointBuffer = pointBuffer;
        }
        faces.clear();
        claimed.clear();
        this.pointCount = pointCount;
    }

    private void buildHull()
    {
        int pointCount = 0;
        Vertex eyeVertex;

        computeMaxAndMin();
        createInitialSimplex();
        while ((eyeVertex = nextPointToAdd()) != null)
        {
            addPointToHull(eyeVertex);
            pointCount++;
            LOG.debug("iteration {} done", pointCount);
        }
        reindexFacesAndVertices();
        LOG.debug("hull done");
    }

    private void build(double[] coords, int pointCount) throws IllegalArgumentException
    {
        if (pointCount < 4)
        {
            throw new IllegalArgumentException("Less than four input points specified");
        }
        if (coords.length / 3 < pointCount)
        {
            throw new IllegalArgumentException("Coordinate array too small for specified number of points");
        }
        initBuffers(pointCount);
        setPoints(coords, pointCount);
        buildHull();
    }

    private void build(float[] coords, int pointCount) throws IllegalArgumentException
    {
        if (pointCount < 4)
        {
            throw new IllegalArgumentException("Less than four input points specified");
        }
        if (coords.length / 3 < pointCount)
        {
            throw new IllegalArgumentException("Coordinate array too small for specified number of points");
        }
        explicitTolerance = FLOAT_PREC;
        initBuffers(pointCount);
        double[] double_buffer = new double[coords.length];
        for (int i = 0; i < coords.length; i++)
        {
            double_buffer[i] = coords[i];
        }
        setPoints(double_buffer, pointCount);
        buildHull();
    }

    private void build(Vector3d[] points, int pointCount) throws IllegalArgumentException
    {
        if (pointCount < 4)
        {
            throw new IllegalArgumentException("Less than four input points specified");
        }
        if (points.length < pointCount)
        {
            throw new IllegalArgumentException("Point array too small for specified number of points");
        }
        initBuffers(pointCount);
        setPoints(points, pointCount);
        buildHull();
    }

    private void build(Vector3f[] points, int pointCount) throws IllegalArgumentException
    {
        if (pointCount < 4)
        {
            throw new IllegalArgumentException("Less than four input points specified");
        }
        if (points.length < pointCount)
        {
            throw new IllegalArgumentException("Point array too small for specified number of points");
        }
        initBuffers(pointCount);
        Vector3d[] double_buffer = new Vector3d[points.length];
        for (int i = 0; i < points.length; i++)
        {
            double_buffer[i] = new Vector3d(points[i]);
        }
        setPoints(double_buffer, pointCount);
        buildHull();
    }

    private void setPoints(double[] coords, int pointCount)
    {
        for (int i = 0; i < pointCount; i++)
        {
            Vertex vertex = pointBuffer[i];
            vertex.point.set(coords[i * 3], coords[i * 3 + 1], coords[i * 3 + 2]);
            vertex.index = i;
        }
    }

    private void setPoints(Vector3d[] pnts, int pointCount)
    {
        for (int i = 0; i < pointCount; i++)
        {
            Vertex vertex = pointBuffer[i];
            vertex.point.set(pnts[i]);
            vertex.index = i;
        }
    }

    private void addPointToFace(Vertex vertex, Face face)
    {
        vertex.face = face;

        if (face.outside == null)
        {
            claimed.add(vertex);
        }
        else
        {
            claimed.insertBefore(vertex, face.outside);
        }
        face.outside = vertex;
    }

    private void removePointFromFace(Vertex vertex, Face face)
    {
        if (vertex == face.outside)
        {
            if (vertex.next != null && vertex.next.face == face)
            {
                face.outside = vertex.next;
            }
            else
            {
                face.outside = null;
            }
        }
        claimed.delete(vertex);
    }

    private Vertex removeAllPointsFromFace(Face face)
    {
        if (face.outside != null)
        {
            Vertex end = face.outside;
            while (end.next != null && end.next.face == face)
            {
                end = end.next;
            }
            claimed.delete(face.outside, end);
            end.next = null;
            return face.outside;
        }
        else
        {
            return null;
        }
    }

    private void computeMaxAndMin()
    {
        Vector3d max = new Vector3d();
        Vector3d min = new Vector3d();

        for (int i = 0; i < 3; i++)
        {
            maxVertices[i] = minVertices[i] = pointBuffer[0];
        }
        max.set(pointBuffer[0].point);
        min.set(pointBuffer[0].point);

        for (int i = 1; i < pointCount; i++)
        {
            Vector3d pnt = pointBuffer[i].point;
            if (pnt.x > max.x)
            {
                max.x = pnt.x;
                maxVertices[0] = pointBuffer[i];
            }
            else if (pnt.x < min.x)
            {
                min.x = pnt.x;
                minVertices[0] = pointBuffer[i];
            }
            if (pnt.y > max.y)
            {
                max.y = pnt.y;
                maxVertices[1] = pointBuffer[i];
            }
            else if (pnt.y < min.y)
            {
                min.y = pnt.y;
                minVertices[1] = pointBuffer[i];
            }
            if (pnt.z > max.z)
            {
                max.z = pnt.z;
                maxVertices[2] = pointBuffer[i];
            }
            else if (pnt.z < min.z)
            {
                min.z = pnt.z;
                minVertices[2] = pointBuffer[i];
            }
        }

        // this epsilon formula comes from QuickHull, and I'm
        // not about to quibble.
        charLength = Math.max(max.x - min.x, max.y - min.y);
        charLength = Math.max(max.z - min.z, charLength);
        if (explicitTolerance == AUTOMATIC_TOLERANCE)
        {
            tolerance =
                3 * DOUBLE_PREC * (Math.max(Math.abs(max.x), Math.abs(min.x)) + Math.max(Math.abs(max.y), Math.abs(min.y)) + Math.max(Math.abs(max.z), Math.abs(min.z)));
        }
        else
        {
            tolerance = explicitTolerance;
        }
    }

    /**
     * Creates the initial simplex from which the hull will be built.
     */
    private void createInitialSimplex() throws IllegalArgumentException
    {
        double maxDiff = 0;
        int maxIndex = 0;

        for (int i = 0; i < 3; i++)
        {
            double diff = getByIndex(maxVertices[i].point, i) - getByIndex(minVertices[i].point, i);
            if (diff > maxDiff)
            {
                maxDiff = diff;
                maxIndex = i;
            }
        }

        if (maxDiff <= tolerance)
        {
            throw new IllegalArgumentException("Input points appear to be coincident");
        }
        Vertex[] vertexBuffer = new Vertex[4];
        // set first two vertices to be those with the greatest
        // one dimensional separation

        vertexBuffer[0] = maxVertices[maxIndex];
        vertexBuffer[1] = minVertices[maxIndex];

        // set third vertex to be the vertex farthest from
        // the line between vertex0 and vertex1
        Vector3d differenceA = new Vector3d();
        Vector3d differenceB = new Vector3d();
        Vector3d normal = new Vector3d();
        Vector3d crossProduct = new Vector3d();

        double maxSqLen = 0;
        vertexBuffer[1].point.sub(vertexBuffer[0].point, differenceA);
        differenceA.normalize();
        for (int i = 0; i < pointCount; i++)
        {
            pointBuffer[i].point.sub(vertexBuffer[0].point, differenceB);
            differenceA.cross(differenceB, crossProduct);

            double sqrLen = normSquared(crossProduct);
            if (sqrLen > maxSqLen && pointBuffer[i] != vertexBuffer[0] && // paranoid
                pointBuffer[i] != vertexBuffer[1])
            {
                maxSqLen = sqrLen;
                vertexBuffer[2] = pointBuffer[i];
                normal.set(crossProduct);
            }
        }
        if (Math.sqrt(maxSqLen) <= 100 * tolerance)
        {
            throw new IllegalArgumentException("Input points appear to be collinear");
        }
        normal.normalize();

        double maxDistance = 0;
        double dotProduct = vertexBuffer[2].point.dot(normal);
        for (int i = 0; i < pointCount; i++)
        {
            double distance = Math.abs(pointBuffer[i].point.dot(normal) - dotProduct);
            if (distance > maxDistance && pointBuffer[i] != vertexBuffer[0] && // paranoid
                pointBuffer[i] != vertexBuffer[1] && pointBuffer[i] != vertexBuffer[2])
            {
                maxDistance = distance;
                vertexBuffer[3] = pointBuffer[i];
            }
        }
        if (Math.abs(maxDistance) <= 100 * tolerance)
        {
            throw new IllegalArgumentException("Input points appear to be coplanar");
        }

        LOG.debug("initial vertices:");
        LOG.debug("{}: {}", vertexBuffer[0].index, vertexBuffer[0].point);
        LOG.debug("{}: {}", vertexBuffer[1].index, vertexBuffer[1].point);
        LOG.debug("{}: {}", vertexBuffer[2].index, vertexBuffer[2].point);
        LOG.debug("{}: {}", vertexBuffer[3].index, vertexBuffer[3].point);

        Face[] tris = new Face[4];

        if (vertexBuffer[3].point.dot(normal) - dotProduct < 0)
        {
            tris[0] = Face.createTriangle(vertexBuffer[0], vertexBuffer[1], vertexBuffer[2]);
            tris[1] = Face.createTriangle(vertexBuffer[3], vertexBuffer[1], vertexBuffer[0]);
            tris[2] = Face.createTriangle(vertexBuffer[3], vertexBuffer[2], vertexBuffer[1]);
            tris[3] = Face.createTriangle(vertexBuffer[3], vertexBuffer[0], vertexBuffer[2]);

            for (int i = 0; i < 3; i++)
            {
                int k = (i + 1) % 3;
                tris[i + 1].getEdge(1).setOpposite(tris[k + 1].getEdge(0));
                tris[i + 1].getEdge(2).setOpposite(tris[0].getEdge(k));
            }
        }
        else
        {
            tris[0] = Face.createTriangle(vertexBuffer[0], vertexBuffer[2], vertexBuffer[1]);
            tris[1] = Face.createTriangle(vertexBuffer[3], vertexBuffer[0], vertexBuffer[1]);
            tris[2] = Face.createTriangle(vertexBuffer[3], vertexBuffer[1], vertexBuffer[2]);
            tris[3] = Face.createTriangle(vertexBuffer[3], vertexBuffer[2], vertexBuffer[0]);

            for (int i = 0; i < 3; i++)
            {
                int k = (i + 1) % 3;
                tris[i + 1].getEdge(0).setOpposite(tris[k + 1].getEdge(1));
                tris[i + 1].getEdge(2).setOpposite(tris[0].getEdge((3 - i) % 3));
            }
        }

        faces.addAll(Arrays.asList(tris).subList(0, 4));

        for (int i = 0; i < pointCount; i++)
        {
            Vertex v = pointBuffer[i];

            if (v == vertexBuffer[0] || v == vertexBuffer[1] || v == vertexBuffer[2] || v == vertexBuffer[3])
            {
                continue;
            }

            maxDistance = tolerance;
            Face maxFace = null;
            for (int k = 0; k < 4; k++)
            {
                double dist = tris[k].distanceToPlane(v.point);
                if (dist > maxDistance)
                {
                    maxFace = tris[k];
                    maxDistance = dist;
                }
            }
            if (maxFace != null)
            {
                addPointToFace(v, maxFace);
            }
        }
    }

    private void getFaceIndices(int[] indices, Face face, int flags)
    {
        boolean ccw = (flags & CLOCKWISE) == 0;
        boolean indexedFromOne = (flags & INDEXED_FROM_ONE) != 0;
        boolean pointRelative = (flags & POINT_RELATIVE) != 0;

        HalfEdge hedge = face.firstEdge;
        int k = 0;
        do
        {
            int idx = hedge.head().index;
            if (pointRelative)
            {
                idx = vertexPointIndices[idx];
            }
            if (indexedFromOne)
            {
                idx++;
            }
            indices[k++] = idx;
            hedge = (ccw
                ? hedge.next
                : hedge.prev);
        } while (hedge != face.firstEdge);
    }

    private void resolveUnclaimedPoints(FaceList newFaces)
    {
        Vertex nextVertex = unclaimed.first();
        for (Vertex vertex = nextVertex; vertex != null; vertex = nextVertex)
        {
            nextVertex = vertex.next;

            double maxDist = tolerance;
            Face maxFace = null;
            for (Face newFace = newFaces.first(); newFace != null; newFace = newFace.next)
            {
                if (newFace.mark == Face.VISIBLE)
                {
                    double dist = newFace.distanceToPlane(vertex.point);
                    if (dist > maxDist)
                    {
                        maxDist = dist;
                        maxFace = newFace;
                    }
                    if (maxDist > 1000 * tolerance)
                    {
                        break;
                    }
                }
            }
            if (maxFace != null)
            {
                addPointToFace(vertex, maxFace);
                LOG.debug("{} CLAIMED BY {}", NEGATIVE_INDEX, maxFace.getVertexString());
            }
            else
            {
                if (vertex.index == NEGATIVE_INDEX)
                {
                    LOG.debug("{} DISCARDED", nextVertex);
                }
            }
        }
    }

    private void deleteFacePoints(Face face, Face absorbingFace)
    {
        Vertex faceVertices = removeAllPointsFromFace(face);
        if (faceVertices != null)
        {
            if (absorbingFace == null)
            {
                unclaimed.addAll(faceVertices);
            }
            else
            {
                Vertex nextVertex = faceVertices;
                for (Vertex vertex = nextVertex; vertex != null; vertex = nextVertex)
                {
                    nextVertex = vertex.next;
                    double dist = absorbingFace.distanceToPlane(vertex.point);
                    if (dist > tolerance)
                    {
                        addPointToFace(vertex, absorbingFace);
                    }
                    else
                    {
                        unclaimed.add(vertex);
                    }
                }
            }
        }
    }

    private double opposingFaceDistance(HalfEdge halfEdge)
    {
        return halfEdge.face.distanceToPlane(halfEdge.opposite.face.getCentroid());
    }

    private boolean doAdjacentMerge(Face face, AdjacencyMergeType mergeType)
    {
        HalfEdge hedge = face.firstEdge;

        boolean convex = true;
        do
        {
            Face oppositeFace = hedge.oppositeFace();
            boolean merge = false;

            switch (mergeType)
            {
                case NON_CONVEX:
                    // merge faces if they are definitively non-convex
                    if (opposingFaceDistance(hedge) > -tolerance || opposingFaceDistance(hedge.opposite) > -tolerance)
                    {
                        merge = true;
                    }
                    break;

                case NON_CONVEX_WRT_LARGER_FACE:
                    // merge faces if they are parallel or non-convex wrt to the larger face;
                    // otherwise, just mark the face non-convex for the second pass.
                    assert oppositeFace != null;
                    if (face.area > oppositeFace.area)
                    {
                        if (opposingFaceDistance(hedge) > -tolerance)
                        {
                            merge = true;
                        }
                        else if (opposingFaceDistance(hedge.opposite) > -tolerance)
                        {
                            convex = false;
                        }
                    }
                    else
                    {
                        if (opposingFaceDistance(hedge.opposite) > -tolerance)
                        {
                            merge = true;
                        }
                        else if (opposingFaceDistance(hedge) > -tolerance)
                        {
                            convex = false;
                        }
                    }
                    break;
            }

            if (merge)
            {
                assert oppositeFace != null;
                LOG.debug("  merging {}  and  {}", face.getVertexString(), oppositeFace.getVertexString());
                int numDiscarded = face.mergeAdjacentFace(hedge, discardedFaces);
                for (int i = 0; i < numDiscarded; i++)
                {
                    deleteFacePoints(discardedFaces[i], face);
                }
                LOG.debug("  result: {}", face.getVertexString());
                return true;
            }
            hedge = hedge.next;
        } while (hedge != face.firstEdge);

        if (!convex)
        {
            face.mark = Face.NON_CONVEX;
        }
        return false;
    }

    private void calculateHorizon(Vector3d eyePnt, HalfEdge edge0, Face face, List<HalfEdge> horizon)
    {
        deleteFacePoints(face, null);
        face.mark = Face.DELETED;
        LOG.debug("  visiting face {}", face.getVertexString());
        HalfEdge edge;
        if (edge0 == null)
        {
            edge0 = face.getEdge(0);
            edge = edge0;
        }
        else
        {
            edge = edge0.getNext();
        }
        do
        {
            Face oppFace = edge.oppositeFace();
            assert oppFace != null;

            if (oppFace.mark == Face.VISIBLE)
            {
                if (oppFace.distanceToPlane(eyePnt) > tolerance)
                {
                    calculateHorizon(eyePnt, edge.getOpposite(), oppFace, horizon);
                }
                else
                {
                    horizon.add(edge);
                    LOG.debug("  adding horizon edge {}", edge.getVertexString());
                }
            }
            edge = edge.getNext();
        } while (edge != edge0);
    }

    private HalfEdge addAdjoiningFace(Vertex eyeVtx, HalfEdge he)
    {
        Face face = Face.createTriangle(eyeVtx, he.tail(), he.head());
        faces.add(face);
        face.getEdge(-1).setOpposite(he.getOpposite());
        return face.getEdge(0);
    }

    private void addNewFaces(FaceList newFaces, Vertex eyeVtx, List<HalfEdge> horizon)
    {
        newFaces.clear();

        HalfEdge hedgeSidePrev = null;
        HalfEdge hedgeSideBegin = null;

        for (HalfEdge object : horizon)
        {
            HalfEdge hedgeSide = addAdjoiningFace(eyeVtx, object);
            LOG.debug("new face: {}", hedgeSide.face.getVertexString());
            if (hedgeSidePrev != null)
            {
                hedgeSide.next.setOpposite(hedgeSidePrev);
            }
            else
            {
                hedgeSideBegin = hedgeSide;
            }
            newFaces.add(hedgeSide.getFace());
            hedgeSidePrev = hedgeSide;
        }
        assert hedgeSideBegin != null;
        hedgeSideBegin.next.setOpposite(hedgeSidePrev);
    }

    private Vertex nextPointToAdd()
    {
        if (!claimed.isEmpty())
        {
            Vertex eyeVtx = null;
            Face eyeFace = claimed.first().face;
            double maxDist = 0;
            for (Vertex vertex = eyeFace.outside; vertex != null && vertex.face == eyeFace; vertex = vertex.next)
            {
                double dist = eyeFace.distanceToPlane(vertex.point);
                if (dist > maxDist)
                {
                    maxDist = dist;
                    eyeVtx = vertex;
                }
            }
            return eyeVtx;
        }
        else
        {
            return null;
        }
    }

    private void addPointToHull(Vertex eyeVtx)
    {
        horizon.clear();
        unclaimed.clear();

        LOG.debug("Adding point: {}", eyeVtx.index);
        LOG.debug(" which is {} above face {}", eyeVtx.face.distanceToPlane(eyeVtx.point), eyeVtx.face.getVertexString());

        removePointFromFace(eyeVtx, eyeVtx.face);
        calculateHorizon(eyeVtx.point, null, eyeVtx.face, horizon);
        newFaces.clear();
        addNewFaces(newFaces, eyeVtx, horizon);

        // first merge pass ... merge faces which are non-convex
        // as determined by the larger face
        for (Face face = newFaces.first(); face != null; face = face.next)
        {
            if (face.mark == Face.VISIBLE)
            {
                while (doAdjacentMerge(face, AdjacencyMergeType.NON_CONVEX_WRT_LARGER_FACE))
                    ;
            }
        }
        // second merge pass ... merge faces which are non-convex
        // wrt either face
        for (Face face = newFaces.first(); face != null; face = face.next)
        {
            if (face.mark == Face.NON_CONVEX)
            {
                face.mark = Face.VISIBLE;
                while (doAdjacentMerge(face, AdjacencyMergeType.NON_CONVEX))
                    ;
            }
        }
        resolveUnclaimedPoints(newFaces);
    }

    private void markFaceVertices(Face face)
    {
        HalfEdge firstEdge = face.getFirstEdge();
        HalfEdge currentEdge = firstEdge;
        do
        {
            currentEdge.head().index = 0;
            currentEdge = currentEdge.next;
        } while (currentEdge != firstEdge);
    }

    private void reindexFacesAndVertices()
    {
        for (int i = 0; i < pointCount; i++)
        {
            pointBuffer[i].index = -1;
        }
        // remove inactive faces and mark active vertices
        for (Iterator<Face> it = faces.iterator(); it.hasNext(); )
        {
            Face face = it.next();
            if (face.mark != Face.VISIBLE)
            {
                it.remove();
            }
            else
            {
                markFaceVertices(face);
            }
        }
        // reindex vertices
        vertexCount = 0;
        for (int i = 0; i < pointCount; i++)
        {
            Vertex vertex = pointBuffer[i];
            if (vertex.index == 0)
            {
                vertexPointIndices[vertexCount] = i;
                vertex.index = vertexCount++;
            }
        }
    }

    private boolean checkFaceConvexity(Face face, double tol, PrintStream ps)
    {
        double dist;
        HalfEdge he = face.firstEdge;
        do
        {
            face.checkConsistency();
            // make sure edge is convex
            dist = opposingFaceDistance(he);
            if (dist > tol)
            {
                if (ps != null)
                {
                    ps.println("Edge " + he.getVertexString() + " non-convex by " + dist);
                }
                return false;
            }
            dist = opposingFaceDistance(he.opposite);
            if (dist > tol)
            {
                if (ps != null)
                {
                    ps.println("Opposite edge " + he.opposite.getVertexString() + " non-convex by " + dist);
                }
                return false;
            }
            if (he.next.oppositeFace() == he.oppositeFace())
            {
                if (ps != null)
                {
                    ps.println("Redundant vertex " + he.head().index + " in face " + face.getVertexString());
                }
                return false;
            }
            he = he.next;
        } while (he != face.firstEdge);
        return true;
    }

    private boolean checkFaces(PrintStream ps, double tol)
    {
        // check edge convexity
        boolean convex = true;
        for (Face face : faces)
        {
            if (face.mark == Face.VISIBLE && !checkFaceConvexity(face, tol, ps))
            {
                convex = false;
            }
        }
        return convex;
    }

    /**
     * Checks the correctness of the hull. This is done by making sure that no
     * faces are non-convex and that no points are outside any face. These tests
     * are performed using the distance tolerance <i>tol</i>. Faces are
     * considered non-convex if any edge is non-convex, and an edge is
     * non-convex if the centroid of either adjoining face is more than
     * <i>tol</i> above the plane of the other face. Similarly, a point is
     * considered outside a face if its distance to that face's plane is more
     * than 10 times <i>tol</i>.
     * <p>
     * If the hull has been {@link #triangulate triangulated}, then this routine
     * may fail if some of the resulting triangles are very small or thin.
     *
     * @param ps  print stream for diagnostic messages; may be set to
     *            <code>null</code> if no messages are desired.
     * @param tol distance tolerance
     * @return true if the hull is valid
     * @see QuickHull3D#check(PrintStream)
     */
    private boolean check(PrintStream ps, double tol)
    {
        // check to make sure all edges are fully connected
        // and that the edges are convex
        double dist;
        double pointTol = 10 * tol;

        if (!checkFaces(ps, tolerance))
        {
            return false;
        }

        // check point inclusion

        for (int i = 0; i < pointCount; i++)
        {
            Vector3d pnt = pointBuffer[i].point;
            for (Face face : faces)
            {
                if (face.mark == Face.VISIBLE)
                {
                    dist = face.distanceToPlane(pnt);
                    if (dist > pointTol)
                    {
                        if (ps != null)
                        {
                            ps.println("Point " + i + " " + dist + " above face " + face.getVertexString());
                        }
                        return false;
                    }
                }
            }
        }
        return true;
    }

    private double normSquared(Vector3d vector3d)
    {
        return Math.fma(vector3d.x, vector3d.x, Math.fma(vector3d.y, vector3d.y, vector3d.z * vector3d.z));
    }

    private double getByIndex(Vector3d point3d, int i)
    {
        return switch (i)
        {
            case 0 -> point3d.x;
            case 1 -> point3d.y;
            case 2 -> point3d.z;
            default -> throw new ArrayIndexOutOfBoundsException(i);
        };
    }
}
