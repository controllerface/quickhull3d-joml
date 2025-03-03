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

/**
 * Maintains a double-linked list of vertices for use by QuickHull3D
 */
final class VertexList
{
    private Vertex head;
    private Vertex tail;

    /**
     * Clears this list.
     */
    public void clear()
    {
        head = tail = null;
    }

    /**
     * Adds a vertex to the end of this list.
     */
    public void add(Vertex vertex)
    {
        if (head == null)
        {
            head = vertex;
        }
        else
        {
            tail.next = vertex;
        }
        vertex.prev = tail;
        vertex.next = null;
        tail = vertex;
    }

    /**
     * Adds a chain of vertices to the end of this list.
     */
    public void addAll(Vertex vertex)
    {
        if (head == null)
        {
            head = vertex;
        }
        else
        {
            tail.next = vertex;
        }
        vertex.prev = tail;
        while (vertex.next != null)
        {
            vertex = vertex.next;
        }
        tail = vertex;
    }

    /**
     * Deletes a vertex from this list.
     */
    public void delete(Vertex vertex)
    {
        if (vertex.prev == null)
        {
            head = vertex.next;
        }
        else
        {
            vertex.prev.next = vertex.next;
        }
        if (vertex.next == null)
        {
            tail = vertex.prev;
        }
        else
        {
            vertex.next.prev = vertex.prev;
        }
    }

    /**
     * Deletes a chain of vertices from this list.
     */
    public void delete(Vertex vertex1, Vertex vertex2)
    {
        if (vertex1.prev == null)
        {
            head = vertex2.next;
        }
        else
        {
            vertex1.prev.next = vertex2.next;
        }
        if (vertex2.next == null)
        {
            tail = vertex1.prev;
        }
        else
        {
            vertex2.next.prev = vertex1.prev;
        }
    }

    /**
     * Inserts a vertex into this list before another specified vertex.
     *
     * @param toInsert The new vertex to be inserted into the list.
     * @param insertBefore The existing vertex before which the new vertex will be inserted.
     */
    public void insertBefore(Vertex toInsert, Vertex insertBefore)
    {
        toInsert.prev = insertBefore.prev;
        if (insertBefore.prev == null)
        {
            head = toInsert;
        }
        else
        {
            insertBefore.prev.next = toInsert;
        }
        toInsert.next = insertBefore;
        insertBefore.prev = toInsert;
    }

    /**
     * Returns the first element in this list.
     */
    public Vertex first()
    {
        return head;
    }

    /**
     * Returns true if this list is empty.
     */
    public boolean isEmpty()
    {
        return head == null;
    }
}
