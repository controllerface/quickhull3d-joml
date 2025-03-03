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
    private final Vector3d centroid = new Vector3d();
    private final Vector3d normal = new Vector3d();

    int vertexCount;
    double area;
    double planeOffset;

    State state;
    HalfEdge firstEdge;
    Face next;
    Vertex outside;

    public enum State
    {
        VISIBLE,
        NON_CONVEX,
        DELETED,
    }

    public Face()
    {
        state = State.VISIBLE;
    }

    public static Face createTriangle(Vertex vertex0, Vertex vertex1, Vertex vertex2)
    {
        return createTriangle(vertex0, vertex1, vertex2, 0);
    }

    /**
     * Constructs a triangle Face from three vertices.
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
        HalfEdge referenceEdge = firstEdge.next;
        HalfEdge currentEdge = referenceEdge.next;

        Vector3d referencePoint = firstEdge.head().point;
        Vector3d currentPoint = referenceEdge.head().point;

        double currentEdgeX = currentPoint.x - referencePoint.x;
        double currentEdgeY = currentPoint.y - referencePoint.y;
        double currentEdgeZ = currentPoint.z - referencePoint.z;

        normal.set(0);

        vertexCount = 2;

        while (currentEdge != firstEdge)
        {
            double previousEdgeX = currentEdgeX;
            double previousEdgeY = currentEdgeY;
            double previousEdgeZ = currentEdgeZ;

            currentPoint = currentEdge.head().point;
            currentEdgeX = currentPoint.x - referencePoint.x;
            currentEdgeY = currentPoint.y - referencePoint.y;
            currentEdgeZ = currentPoint.z - referencePoint.z;

            normal.x += previousEdgeY * currentEdgeZ - previousEdgeZ * currentEdgeY;
            normal.y += previousEdgeZ * currentEdgeX - previousEdgeX * currentEdgeZ;
            normal.z += previousEdgeX * currentEdgeY - previousEdgeY * currentEdgeX;

            currentEdge = currentEdge.next;
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

            HalfEdge longestEdge = null;
            double longestEdgeLenSq = 0;
            HalfEdge currentEdge = firstEdge;
            do
            {
                double currentEdgeLenSq = currentEdge.lengthSquared();
                if (currentEdgeLenSq > longestEdgeLenSq)
                {
                    longestEdge = currentEdge;
                    longestEdgeLenSq = currentEdgeLenSq;
                }
                currentEdge = currentEdge.next;
            } while (currentEdge != firstEdge);

            assert longestEdge != null;
            assert longestEdge.head() != null;
            assert longestEdge.tail() != null;

            Vector3d headPoint = longestEdge.head().point;
            Vector3d tailPoint = longestEdge.tail().point;

            double longestEdgeLen = currentEdge.length();
            double unitX = (headPoint.x - tailPoint.x) / longestEdgeLen;
            double unitY = (headPoint.y - tailPoint.y) / longestEdgeLen;
            double unitZ = (headPoint.z - tailPoint.z) / longestEdgeLen;
            double parallelComponent = normal.dot(unitX, unitY, unitZ);

            normal.sub(parallelComponent * unitX, parallelComponent * unitY, parallelComponent * unitZ);
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
     * Gets the i-th half-edge associated with the face, with iteration order defined as recursively
     * referencing the {@linkplain HalfEdge#next} property of the current edge being iterated. Negative
     * indices work backward, recursively referencing {@linkplain HalfEdge#prev} instead. Index `0` will
     * return the result of {@linkplain Face#getFirstEdge()}.
     * This method is only well-defined for faces that represent a triangle, with indices being in the
     * range [-2, 2]. Index arguments outside this range may result in undefined behavior.
     *
     * @param i the half-edge index, in the range [-2, 2].
     * @return the half-edge
     */
    public HalfEdge getEdge(int i)
    {
        assert i >= -2 && i <= 2 : "edge index" + i + " is out of range [-2, 2]";

        HalfEdge currentEdge = firstEdge;
        while (i > 0)
        {
            currentEdge = currentEdge.next;
            i--;
        }
        while (i < 0)
        {
            currentEdge = currentEdge.prev;
            i++;
        }
        return currentEdge;
    }

    public HalfEdge getFirstEdge()
    {
        return firstEdge;
    }

    public int mergeAdjacentFace(HalfEdge adjacentEdge, Face[] discarded)
    {
        Face oppositeFace = adjacentEdge.oppositeFace();
        assert oppositeFace != null;

        int discardCount = 0;

        discarded[discardCount++] = oppositeFace;
        oppositeFace.state = State.DELETED;

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
        HalfEdge nextEdge = firstEdge;
        do
        {
            vertices++;
            nextEdge = nextEdge.next;
        } while (nextEdge != firstEdge);

        if (vertices != vertexCount)
        {
            throw new HullGenerationException("face " + this + " vertexCount=" + vertexCount + " should be " + vertices);
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
            if (oppositeFace.vertexCount() == 3) // then we can get rid of the opposite face altogether
            {
                oppositeEdge = edge.getOpposite().prev.getOpposite();
                oppositeFace.state = State.DELETED;
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

    public void checkConsistency()
    {
        // do a sanity check on the face
        HalfEdge currentEdge = firstEdge;
        double maxDistance = 0;
        int vertices = 0;

        if (vertexCount < 3)
        {
            throw new HullGenerationException("degenerate face: " + this);
        }
        do
        {
            HalfEdge oppositeEdge = currentEdge.getOpposite();
            if (oppositeEdge == null)
            {
                throw new HullGenerationException("face " + this + ": " + "unreflected half edge " + currentEdge.getVertexString());
            }
            else if (oppositeEdge.getOpposite() != currentEdge)
            {
                throw new HullGenerationException("face " + this + ": " + "opposite half edge " + oppositeEdge.getVertexString() + " has opposite "
                    + oppositeEdge.getOpposite().getVertexString());
            }
            if (oppositeEdge.head() != currentEdge.tail() || currentEdge.head() != oppositeEdge.tail())
            {
                throw new HullGenerationException("face " + this + ": " + "half edge " + currentEdge.getVertexString() + " reflected by " + oppositeEdge.getVertexString());
            }
            Face oppositeFace = oppositeEdge.face;
            if (oppositeFace == null)
            {
                throw new HullGenerationException("face " + this + ": " + "no face on half edge " + oppositeEdge.getVertexString());
            }
            else if (oppositeFace.state == State.DELETED)
            {
                throw new HullGenerationException("face " + this + ": " + "opposite face " + oppositeFace + " not on hull");
            }
            double d = Math.abs(distanceToPlane(currentEdge.head().point));
            if (d > maxDistance)
            {
                maxDistance = d;
            }
            vertices++;
            currentEdge = currentEdge.next;
        } while (currentEdge != firstEdge);

        if (vertices != vertexCount)
        {
            throw new HullGenerationException("face " + this + " vertexCount=" + vertexCount + " should be " + vertices);
        }
    }

    private double norm(Vector3d vector3d)
    {
        return Math.sqrt(vector3d.x * vector3d.x + vector3d.y * vector3d.y + vector3d.z * vector3d.z);
    }

    @Override
    public String toString()
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
}
