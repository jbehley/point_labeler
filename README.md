# point cloud annotator (PCA)

 Tool for labeling of single point clouds or a stream of point clouds, which we used over the years to label
 many, many point clouds. From this experience, we hope to provide all the tools and features you need to 
 label even complicated objects effortlessly.

## Features:
 - Support for kitti point clouds.
 - Human-readable label description files in xml allow to define label names, ids, and colors.
 - Modern OpenGL shaders and view frustrum culling for rendering of even millions of points.
 - Tools for labeling of individual points, areas, and filling segments.
 - Filtering of labels makes it easy to label even complicated structures with ease.
 - Mulitple viewport showing fixed or variable views of the scene.

## Requirements
 We are building on matured technology, available on many platforms. The PCA should work on Linux, Mac, and Windows.
 
 - modern c++
 - boost, Qt5, cmake
 - opengl, etc.
 - glow.
 
## TODO
 
- shader-based visualization
- project/unproject for querying of hitted points...
- Loading & saving with python scripts for simple extensibility? 