## Version 1.0 ##

[x] Visibility/Filtering of labels.
  -- use vertex shader with transform feedback to update visibility  
  
[x] Label only visible points.  
[x] Save labels of unloaded point clouds on forward/backward automatically.  
[x] Polygon labeling: non-convex simple polygons.  
  -- ear cutting and use updateLabels shader for polygon labeling.
  -- use texture for representation of triangulated polygon.  
  
[ ] Segment-based labeling: use grid-based segmentation to find segments, which can be selected for labeling.  
[ ] Filter labels by category. Have category "Last used" that includes all lately used labels.  
[ ] Use color from images to colorize points.  
[ ] Draw oultine (2d) of brush for better control of labeling with brush.  
[ ] moving/static labeling (bit in labels?)  
[x] save on quit.  
[x] min/max range of points in laserscan.  
[x] remove ground by grid-based  segmentation of single scan.  
  -- use depth buffer to find minimum height, after loading points into buffers?  
[x] Use tiles instead of timestamps.  
[x] load scans concurrently and have indicator for loading.) [spinner over viewpoint?]  
[ ] center camera on loaded scans after loading.  
[ ] read scans incrementally. interupt reading if tile changes instead of waiting for finishing reading.  
  -- also have a proper progress indicator.  


## Future work

[ ] Only load points inside the tile into the GlBuffer. (Transform Feedback)
[ ] do projections only once (avoid the matrixmultiply?)


## Future work
1. integrate pcp (?)