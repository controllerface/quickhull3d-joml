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

import org.joml.Vector3d;

/**
 * Basic triangular face used to form the hull.
 * <p>
 * The information stored for each face consists of a planar normal, a planar
 * offset, and a doubly-linked list of three <a href=HalfEdge>HalfEdges</a>
 * which surround the face in a counter-clockwise direction.
 *
 * @author John E. Lloyd, Fall 2004
 */
final class Face
{
    public static final int DELETED = 3;

    public static final int NON_CONVEX = 2;

    public static final int VISIBLE = 1;

    private final Vector3d centroid = new Vector3d();

    private final Vector3d normal = new Vector3d();

    int mark;

    int vertexCount;

    double area;

    double planeOffset;

    HalfEdge firstEdge;

    Face next;

    Vertex outside;

    public Face()
    {
        mark = VISIBLE;
    }

    public static Face createTriangle(Vertex vertex0, Vertex vertex1, Vertex vertex2)
    {
        return createTriangle(vertex0, vertex1, vertex2, 0);
    }

    /**
     * Constructs a triangle Face from vertices v0, v1, and v2.
     *
     * @param vertex0 first vertex
     * @param vertex1 second vertex
     * @param vertex2 third vertex
     */
    public static Face createTriangle(Vertex vertex0, Vertex vertex1, Vertex vertex2, double minArea)
    {
        Face face = new Face();
        HalfEdge edge0 = new HalfEdge(vertex0, face);
        HalfEdge edge1 = new HalfEdge(vertex1, face);
        HalfEdge edge2 = new HalfEdge(vertex2, face);

        edge0.prev = edge2;
        edge0.next = edge1;
        edge1.prev = edge0;
        edge1.next = edge2;
        edge2.prev = edge1;
        edge2.next = edge0;

        face.firstEdge = edge0;

        // compute the normal and offset
        face.computeNormalAndCentroid(minArea);
        return face;
    }

    public void computeCentroid(Vector3d centroid)
    {
        centroid.set(0);
        HalfEdge currentEdge = firstEdge;
        do
        {
            centroid.add(currentEdge.head().point);
            currentEdge = currentEdge.next;
        } while (currentEdge != firstEdge);
        centroid.mul(1 / (double) vertexCount);
    }

    public void computeNormal(Vector3d normal)
    {
        HalfEdge edge1 = firstEdge.next;
        HalfEdge edge2 = edge1.next;

        Vector3d point0 = firstEdge.head().point;
        Vector3d point2 = edge1.head().point;

        double deltaX = point2.x - point0.x;
        double deltaY = point2.y - point0.y;
        double deltaZ = point2.z - point0.z;

        normal.set(0);

        vertexCount = 2;

        while (edge2 != firstEdge)
        {
            double d1x = deltaX;
            double d1y = deltaY;
            double d1z = deltaZ;

            point2 = edge2.head().point;
            deltaX = point2.x - point0.x;
            deltaY = point2.y - point0.y;
            deltaZ = point2.z - point0.z;

            normal.x += d1y * deltaZ - d1z * deltaY;
            normal.y += d1z * deltaX - d1x * deltaZ;
            normal.z += d1x * deltaY - d1y * deltaX;

            edge2 = edge2.next;
            vertexCount++;
        }
        area = norm(normal);
        normal.mul(1 / area);
    }

    public void computeNormal(Vector3d normal, double minArea)
    {
        computeNormal(normal);

        if (area < minArea)
        {
            // make the normal more robust by removing
            // components parallel to the longest edge

            HalfEdge hedgeMax = null;
            double lenSqrMax = 0;
            HalfEdge hedge = firstEdge;
            do
            {
                double lenSqr = hedge.lengthSquared();
                if (lenSqr > lenSqrMax)
                {
                    hedgeMax = hedge;
                    lenSqrMax = lenSqr;
                }
                hedge = hedge.next;
            } while (hedge != firstEdge);

            assert hedgeMax != null;
            assert hedgeMax.head() != null;
            assert hedgeMax.tail() != null;

            Vector3d p2 = hedgeMax.head().point;
            Vector3d p1 = hedgeMax.tail().point;
            double lenMax = Math.sqrt(lenSqrMax);
            double ux = (p2.x - p1.x) / lenMax;
            double uy = (p2.y - p1.y) / lenMax;
            double uz = (p2.z - p1.z) / lenMax;
            double dot = normal.x * ux + normal.y * uy + normal.z * uz;
            normal.x -= dot * ux;
            normal.y -= dot * uy;
            normal.z -= dot * uz;

            normal.normalize();
        }
    }

    /**
     * Computes the distance from a point p to the plane of this face.
     *
     * @param p the point
     * @return distance from the point to the plane
     */
    public double distanceToPlane(Vector3d p)
    {
        return normal.x * p.x + normal.y * p.y + normal.z * p.z - planeOffset;
    }

    public Vector3d getCentroid()
    {
        return centroid;
    }

    /**
     * Gets the i-th half-edge associated with the face.
     *
     * @param i the half-edge index, in the range 0-2.
     * @return the half-edge
     */
    public HalfEdge getEdge(int i)
    {
        HalfEdge he = firstEdge;
        while (i > 0)
        {
            he = he.next;
            i--;
        }
        while (i < 0)
        {
            he = he.prev;
            i++;
        }
        return he;
    }

    public HalfEdge getFirstEdge()
    {
        return firstEdge;
    }

    public String getVertexString()
    {
        StringBuilder buffer = null;
        HalfEdge currentEdge = firstEdge;
        do
        {
            if (buffer == null)
            {
                buffer = new StringBuilder("" + currentEdge.head().index);
            }
            else
            {
                buffer.append(" ").append(currentEdge.head().index);
            }
            currentEdge = currentEdge.next;
        } while (currentEdge != firstEdge);
        return buffer.toString();
    }

    public int mergeAdjacentFace(HalfEdge adjacentEdge, Face[] discarded)
    {
        Face oppositeFace = adjacentEdge.oppositeFace();
        assert oppositeFace != null;

        int discardCount = 0;

        discarded[discardCount++] = oppositeFace;
        oppositeFace.mark = DELETED;

        HalfEdge oppositeEdge = adjacentEdge.getOpposite();

        HalfEdge prevAdjacentEdge = adjacentEdge.prev;
        HalfEdge nextAdjacentEdge = adjacentEdge.next;
        HalfEdge prevOppositeEdge = oppositeEdge.prev;
        HalfEdge nextOppositeEdge = oppositeEdge.next;

        while (prevAdjacentEdge.oppositeFace() == oppositeFace)
        {
            prevAdjacentEdge = prevAdjacentEdge.prev;
            nextOppositeEdge = nextOppositeEdge.next;
        }

        while (nextAdjacentEdge.oppositeFace() == oppositeFace)
        {
            prevOppositeEdge = prevOppositeEdge.prev;
            nextAdjacentEdge = nextAdjacentEdge.next;
        }

        HalfEdge currentEdge;

        for (currentEdge = nextOppositeEdge; currentEdge != prevOppositeEdge.next; currentEdge = currentEdge.next)
        {
            currentEdge.face = this;
        }

        if (adjacentEdge == firstEdge)
        {
            firstEdge = nextAdjacentEdge;
        }

        // handle the half edges at the head
        Face discardedFace;

        discardedFace = connectHalfEdges(prevOppositeEdge, nextAdjacentEdge);
        if (discardedFace != null)
        {
            discarded[discardCount++] = discardedFace;
        }

        // handle the half edges at the tail
        discardedFace = connectHalfEdges(prevAdjacentEdge, nextOppositeEdge);
        if (discardedFace != null)
        {
            discarded[discardCount++] = discardedFace;
        }

        computeNormalAndCentroid();
        checkConsistency();

        return discardCount;
    }

    public int vertexCount()
    {
        return vertexCount;
    }

    public void triangulate(FaceList newFaces, double minArea)
    {
        HalfEdge currentEdge;

        if (vertexCount() < 4)
        {
            return;
        }

        Vertex firstVertex = firstEdge.head();

        currentEdge = firstEdge.next;
        HalfEdge prevOpposingEdge = currentEdge.opposite;
        Face currentFace = null;

        for (currentEdge = currentEdge.next; currentEdge != firstEdge.prev; currentEdge = currentEdge.next)
        {
            Face face = createTriangle(firstVertex, currentEdge.prev.head(), currentEdge.head(), minArea);
            face.firstEdge.next.setOpposite(prevOpposingEdge);
            face.firstEdge.prev.setOpposite(currentEdge.opposite);
            prevOpposingEdge = face.firstEdge;
            newFaces.add(face);
            if (currentFace == null)
            {
                currentFace = face;
            }
        }
        currentEdge = new HalfEdge(firstEdge.prev.prev.head(), this);
        currentEdge.setOpposite(prevOpposingEdge);

        currentEdge.prev = firstEdge;
        currentEdge.prev.next = currentEdge;

        currentEdge.next = firstEdge.prev;
        currentEdge.next.prev = currentEdge;

        computeNormalAndCentroid(minArea);
        checkConsistency();

        for (Face face = currentFace; face != null; face = face.next)
        {
            face.checkConsistency();
        }
    }

    private void computeNormalAndCentroid()
    {
        computeNormal(normal);
        computeCentroid(centroid);
        planeOffset = normal.dot(centroid);
        int vertices = 0;
        HalfEdge he = firstEdge;
        do
        {
            vertices++;
            he = he.next;
        } while (he != firstEdge);

        if (vertices != vertexCount)
        {
            throw new HullGenerationException("face " + getVertexString() + " numVerts=" + vertexCount + " should be " + vertices);
        }
    }

    private void computeNormalAndCentroid(double minArea)
    {
        computeNormal(normal, minArea);
        computeCentroid(centroid);
        planeOffset = normal.dot(centroid);
    }

    private Face connectHalfEdges(HalfEdge prevEdge, HalfEdge edge)
    {
        Face discardedFace = null;

        if (prevEdge.oppositeFace() == edge.oppositeFace()) // then there is a redundant edge that we can get rid of
        {
            Face oppositeFace = edge.oppositeFace();
            HalfEdge oppositeEdge;

            if (prevEdge == firstEdge)
            {
                firstEdge = edge;
            }

            assert oppositeFace != null;
            if (oppositeFace.vertexCount() == 3)
            { // then we can get rid of the
                // opposite face altogether
                oppositeEdge = edge.getOpposite().prev.getOpposite();

                oppositeFace.mark = DELETED;
                discardedFace = oppositeFace;
            }
            else
            {
                oppositeEdge = edge.getOpposite().next;

                if (oppositeFace.firstEdge == oppositeEdge.prev)
                {
                    oppositeFace.firstEdge = oppositeEdge;
                }
                oppositeEdge.prev = oppositeEdge.prev.prev;
                oppositeEdge.prev.next = oppositeEdge;
            }
            edge.prev = prevEdge.prev;
            edge.prev.next = edge;

            edge.opposite = oppositeEdge;
            oppositeEdge.opposite = edge;

            // oppFace was modified, so need to recompute
            oppositeFace.computeNormalAndCentroid();
        }
        else
        {
            prevEdge.next = edge;
            edge.prev = prevEdge;
        }
        return discardedFace;
    }

    void checkConsistency()
    {
        // do a sanity check on the face
        HalfEdge currentEdge = firstEdge;
        double maxd = 0;
        int vertices = 0;

        if (vertexCount < 3)
        {
            throw new HullGenerationException("degenerate face: " + getVertexString());
        }
        do
        {
            HalfEdge hedgeOpp = currentEdge.getOpposite();
            if (hedgeOpp == null)
            {
                throw new HullGenerationException("face " + getVertexString() + ": " + "unreflected half edge " + currentEdge.getVertexString());
            }
            else if (hedgeOpp.getOpposite() != currentEdge)
            {
                throw new HullGenerationException("face " + getVertexString() + ": " + "opposite half edge " + hedgeOpp.getVertexString() + " has opposite "
                    + hedgeOpp.getOpposite().getVertexString());
            }
            if (hedgeOpp.head() != currentEdge.tail() || currentEdge.head() != hedgeOpp.tail())
            {
                throw new HullGenerationException("face " + getVertexString() + ": " + "half edge " + currentEdge.getVertexString() + " reflected by " + hedgeOpp.getVertexString());
            }
            Face oppFace = hedgeOpp.face;
            if (oppFace == null)
            {
                throw new HullGenerationException("face " + getVertexString() + ": " + "no face on half edge " + hedgeOpp.getVertexString());
            }
            else if (oppFace.mark == DELETED)
            {
                throw new HullGenerationException("face " + getVertexString() + ": " + "opposite face " + oppFace.getVertexString() + " not on hull");
            }
            double d = Math.abs(distanceToPlane(currentEdge.head().point));
            if (d > maxd)
            {
                maxd = d;
            }
            vertices++;
            currentEdge = currentEdge.next;
        } while (currentEdge != firstEdge);

        if (vertices != vertexCount)
        {
            throw new HullGenerationException("face " + getVertexString() + " numVerts=" + vertexCount + " should be " + vertices);
        }
    }

    public double norm(Vector3d vector3d)
    {
        return Math.sqrt(vector3d.x * vector3d.x + vector3d.y * vector3d.y + vector3d.z * vector3d.z);
    }
}
