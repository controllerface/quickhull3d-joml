com.github.quickhull3d - A Robust 3D Convex Hull Algorithm in Java
===========

This is a 3D implementation of QuickHull for Java, based on the original paper by Barber, Dobkin, and Huhdanpaa and the C implementation known as qhull. The algorithm has O(n log(n)) complexity, works with double precision numbers, is fairly robust with respect to degenerate situations, and allows the merging of co-planar faces.

A tutorial on the QuickHull algorithm by Dirk Gregorius (Valve Software) was given at the 2014 Game Developers Conference in San Francisco.

There are some other 3D convex hull implementations available in netland, but I didn't find any that satisfied all the above criteria, so I created my own. The principal class is QuickHull3D, which is contained within the package com.github.quickhull3d. It is actually a reimplementation of an earlier piece of work, ConvexHull3D, which was based on an insertion algorithm and had a complexity of O(n^2).

Mark Newbold has used this package to create a very picturesque applet that creates and displays Waterman polyhedra 

See the maven project site here: [quickhull3d](http://quickhull3d.github.io/quickhull3d/)

---

### Basic usage

The `QuickHull3D` class is very simple to use, you just have to provide an array of points as input, and then retrieve another array of points as output. Generally you have two options, either passing the source points in a constructor, or by calling one the `build()` methods with the source points. Here's an example of typical usage. 

```java
// get your points from somewhere, like a loaded model
Vector3f[] model_verts = get_points();

// when passing points to constructor the `build()` method is called automatically
var hull = new QuickHull3D(model_verts);

// in most cases, you will want the hull to be triangulated for rendering or collision checks
hull.triangulate();

// get the resulting vertices and indices
Vector3f[] hull_verts = hull.getFloatVertices();
int[][] hull_indices = hull.getFaces();
```

For convenience, there's several constructors that can be used for raw `float` or `double` data, as well as the JOML types `Vector3f` and `Vector3d`:

```java
// raw float[] data
public QuickHull3D(float[] coords)

// raw double[] data
public QuickHull3D(double[] coords)
  
// JOML Vector3f data
public QuickHull3D(Vector3f[] coords)

// JOML Vector3d data
public QuickHull3D(Vector3d[] coords)
```

You can also create an empty `QuickHull3D` object, and call one of the `build()` overloads later, if you aren't sure which type of data you're going to have until you're about to use it:

```java
// raw float[] data
public void build(double[] coords)
  
// raw double[] data
public void build(float[] coords)
  
// JOML Vector3f data
public void build(Vector3d[] points)
  
// JOML Vector3d data
public void build(Vector3f[] points)
```

### Changes made in this fork

- The internal classes `Point3d` and `Vector3d` were removed and replaced with the `Vector3d` implementation found in the [JOML](https://github.com/JOML-CI/JOML) library
  - Methods that were implemented on the original vector class were made into helper functions
- Build system was changed from Maven to Gradle
- JDK version in was bumped up to `23`
- Removed logging dependencies and all logging calls
- Several `assert`s were added to ensure program correctness for debugging purposes
- Cases where mutually exclusive `int` values were used for state, `enum` classes were used instead
  - this communicates intent more clearly and removes ambiguity about whether a value is a bitfield or not
- Several terse variable names have been changed
  - For example, `vtx` -> `vertex`, `nump` -> `pointCount`, etc.
- In cases where doing so was trivial, `Math.fma()` was used for a potential performance improvement
- Support was added for `float[]` data types in addition to the existing `double[`] support
  - Float based versions of the existing unit tests were made to assert correct operation with `float` based data
- Test that relied on the c implementation of `qhull` was removed
  - Several methods were only implemented to support this feature and were removed as well
- Directories that did not contain source code or tests were removed (`/site`, `/checkstyle`, and `/eclipse`)
- Many common IDE code quality suggestions were applied, mainly to bring the code forward to more modern Java idioms, for example:
  - Various typos were fixed 
  - Where applicable, non-indexed (for-each) loops were used instead of indexed for loops
  - Where applicable, if/else blocks or other patterns were replaced with switch expressions. 
  - Instances of `StringBuffer` were replaced with `StringBuilder`
  - Unused functions and fields were removed, as were blocks commented out code
  - Several `protected` fields and methods were made `private` or package-private 
    - There did not seem to be any use case for sub-classing
    - Classes were also made final to communicate this intent explicitly
- My personal brace preferences were applied to the project
  - _( sorry, not sorry :-) )_ 

### Benchmarks

To be clear, my motivation for making this fork was not to improve the speed of the existing algorithm, I simply wanted a good convex hull generator that worked with the JOML library vector types. That said, I did want to make sure performance was not worse. There does appear to be a very modest throughput improvement, at least on the two test systems I have, likely due to the use of `Math.fma()` calls both within the JOML classes, but also in the places I was able to use them in the implementation itself. 

Benchmarks with he "Original" suffix use the original implementation, and any marked "New" use this forked one. I added a benchmark for testing `float` based vectors, which don't exist in the original code, but I was curious how much time the float/double conversion process would cost, since I personally plan to utilize that feature. As expected, there is some overhead associated with the conversion. There might be some room to improve it, but generally speaking I don't think it's a high priority.

Here's the JMH results from my main development system.  

| Benchmark                       | Mode  | Cnt | Score      | Error    | Units |
|---------------------------------|-------|-----|------------|----------|-------|
| ModelCompare.testModelNew       | thrpt | 5   | 3333.331   | 48.301   | ops/s |
| ModelCompare.testModelNewFloat  | thrpt | 5   | 3078.964   | 60.039   | ops/s |
| ModelCompare.testModelOriginal  | thrpt | 5   | 3225.163   | 50.996   | ops/s |
| SimpleCompare.testBasicNew      | thrpt | 5   | 191579.241 | 2526.303 | ops/s |
| SimpleCompare.testBasicOriginal | thrpt | 5   | 163885.735 | 2849.159 | ops/s |

The tests are in [src/jmh/java](src/jmh/java). The "simple" test is derived from the pre-existing uint test from the original repo. The "model" test uses raw model data from a test model that I have used in hobby project, it contains 7765 vertices, and has just shy of 10,000 triangles, so it's a fairly modestly sized model, but sufficiently large enough to represent something at least a little more real-world than the unit test. 