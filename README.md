com.github.quickhull3d - A Robust 3D Convex Hull Algorithm in Java
===========

This is a 3D implementation of QuickHull for Java, based on the original paper by Barber, Dobkin, and Huhdanpaa and the C implementation known as qhull. The algorithm has O(n log(n)) complexity, works with double precision numbers, is fairly robust with respect to degenerate situations, and allows the merging of co-planar faces.

A tutorial on the QuickHull algorithm by Dirk Gregorius (Valve Software) was given at the 2014 Game Developers Conference in San Francisco.

There are some other 3D convex hull implementations available in netland, but I didn't find any that satisfied all the above criteria, so I created my own. The principal class is QuickHull3D, which is contained within the package com.github.quickhull3d. It is actually a reimplementaion of an earlier piece of work, ConvexHull3D, which was based on an insertion algorithm and had a complexity of O(n^2).

Mark Newbold has used this package to create a very picturesque applet that creates and displays Waterman polyhedra 

See the maven project site here: [quickhull3d](http://quickhull3d.github.io/quickhull3d/)

---

### Changes made in this fork

- The internal classes `Point3d` and `Vector3d` were removed and replaced with the `Vector3d` implementation found in the [JOML](https://github.com/JOML-CI/JOML) library. This required some minor changes to the ordering of certain method calls within the logic, primarily `add()`, `sub()`, and `mul()` (called "scale" in the original code) were refactored to maintain correct operation
  - Some methods that were implemented on the original vector class were made into helper functions
- Support was added for `float[]` and `Vector3f` data types in addition to the existing double[] and `Vector3d` support
  - Float based versions of the existing unit tests were made that assert correct operation with `float` based data
- Test that relied on the c implementation of `qhull` was removed
  - Several methods were only implemented to support this feature and were removed as well
- Directories that did not contain source code or tests were removed (`/site`, `/checkstyle`, and `/eclipse`)
- Using IntelliJ, most common code change suggestions were applied, mainly to bring the code forward to more modern Java idioms, for example:
  - Where applicable, non-indexed (for-each) loops were used instead of indexed for loops
  - Instances of `StringBuffer` were replaced with `StringBuilder`
  - Unused functions and fields were removed, as well as commented out code
  - Several `protected` fields and methods were made `private` where they did not need to be exposed
  - Several logging calls were parameterized instead of using concatenation
    - Unnecessary calls to `LOG.isDebugEnabled()` were removed as a result
- My personal code-style settings were applied to the project
  - _( I'm sorry :-) )_ 
