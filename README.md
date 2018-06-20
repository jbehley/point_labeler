# point cloud annotator (PCA)

 Tool for labeling of single point clouds or a stream of point clouds. 
 
 Given the poses of a KITTI point cloud dataset, we load tiles of overlapping point clouds. Thus, multiple point clouds 
 are labeled at once in a certain area. 


## Features
 - Support for KITTI Vision Benchmark Point Clouds.
 - Human-readable label description files in xml allow to define label names, ids, and colors.
 - Modern OpenGL shaders for rendering of even millions of points.
 - Tools for labeling of individual points, areas, and filling segments.
 - Filtering of labels makes it easy to label even complicated structures with ease.
 
## Requirements

 - modern c++
 - boost, Qt5, cmake
 - opengl, glew
 - glow
 
## Build

 TODO: setup.sh
 
## Folder structure

When loading a dataset, the data must be organized as follows:

<pre>
point cloud folder
├── velodyne/             -- directory containing ".bin" files with Velodyne point clouds.   
├── labels/   [optional]  -- label directory, will be generated if not present.  
├── image_2/  [optional]  -- directory containing ".png" files from the color   camera.  
├── calib.txt [optional]  -- calibration of velodyne vs. camera. needed for projection of point cloud into camera.  
└── poses.txt             -- file containing the poses of every scan.
</pre>
 

## TODO

 See TODO.md.