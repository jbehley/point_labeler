
QT5_WRAP_UI(UI_HDRS MainFrame.ui)
  
set(VIZ_SHADER_SRC ${CMAKE_BINARY_DIR}/visualization_shaders.cpp)

compile_shaders(${VIZ_SHADER_SRC}
  shaders/color.glsl
  shaders/empty.vert
  shaders/empty.frag
  shaders/quad.geom
  shaders/passthrough.frag
  shaders/draw_pose.geom
  shaders/fill_tile_points.geom
  shaders/fill_tile_points.vert
  shaders/draw_points.vert
  shaders/draw_polygon.vert
  shaders/update_visibility.vert
  shaders/update_labels.vert
  shaders/gen_heightmap.vert
  shaders/gen_heightmap.frag
  shaders/draw_frustum.geom
  shaders/draw_heightmap.geom
  shaders/draw_heightmap.vert
  shaders/average_heightmap.frag
  shaders/draw_plane.geom
  shaders/draw_bbox.geom
  shaders/draw_bbox.vert
  shaders/draw_bbox_id.geom
  shaders/draw_bbox_id.frag
  shaders/selected_labels.vert
  shaders/selected_labels.geom)

add_executable(labeler 
  ${UI_HDRS}
  ${VIZ_SHADER_SRC}
  
  data/label_utils.cpp
  data/kitti_utils.cpp
  rv/string_utils.cpp
  rv/Stopwatch.cpp
  data/Math.cpp
  data/misc.cpp
  labeler.cpp
  widget/LabelButton.h
  widget/Mainframe.h
  widget/Viewport.h
  widget/LabelButton.cpp
  widget/Mainframe.cpp
  widget/Viewport.cpp
  widget/CADCamera.h
  widget/CADCamera.cpp
  widget/KittiReader.cpp
  widget/TileSelectorWidget.cpp
  widget/waitingspinnerwidget.cpp
  widget/ImageViewer.cpp
)

target_link_libraries(labeler ${Boost_LIBRARIES} glow glow_util Qt5::Xml Qt5::OpenGL Qt5::Widgets
  pthread)
