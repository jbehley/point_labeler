PROJECT(point_labeler)

cmake_minimum_required(VERSION 2.6)

set(CMAKE_C_FLAGS "-Wall -O3 -g")
set(CMAKE_CXX_FLAGS "-Wall -O3 -g")

find_package(OpenGL REQUIRED)
find_package(Qt5Xml REQUIRED)
find_package(Qt5Gui REQUIRED) 
find_package(Qt5OpenGL REQUIRED)
find_package(GLEW REQUIRED)
find_package(Boost REQUIRED COMPONENTS filesystem system)

find_package(catkin COMPONENTS 
     glow)

if(catkin_FOUND)
  message("Found catkin")
  catkin_package(
    CATKIN_DEPENDS
      glow
    DEPENDS
      Boost
  )
endif()

include_directories(${catkin_INCLUDE_DIRS} src/ ${QT5_INCLUDE_DIRS} ${GLEW_INCLUDE_DIRS} /usr/include/eigen3)

set(CMAKE_INCLUDE_CURRENT_DIR ON) # needs to be activated for qt generated files in build directory.
set(CMAKE_AUTOMOC ON)
set(CMAKE_CXX_FLAGS "-std=c++11 -O3  -Wall ${CMAKE_CXX_FLAGS}")
set(CMAKE_CXX_FLAGS "-UDEBUG_GL -UDEBUG -DNDEBUG -g2 ${CMAKE_CXX_FLAGS}")

# configure_file(assets/labels.xml ${CMAKE_SOURCE_DIR}/bin/labels.xml COPYONLY)
ADD_CUSTOM_TARGET(link_labels ALL COMMAND ${CMAKE_COMMAND} -E create_symlink ${CMAKE_SOURCE_DIR}/assets/labels.xml ${CMAKE_SOURCE_DIR}/bin/labels.xml)

# ensure that custom configuration file is not overwritten.
if(NOT EXISTS "${CMAKE_SOURCE_DIR}/bin/settings.cfg")
  message("-- Copyied default settings into bin. See 'settings.cfg' to change labeler settings.")
  configure_file(${CMAKE_SOURCE_DIR}/assets/settings.cfg ${CMAKE_SOURCE_DIR}/bin/settings.cfg COPYONLY)
endif()

# since now everything resides in "bin", we have to copy some stuff.
  
SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin)


QT5_WRAP_UI(UI_HDRS 
  src/MainFrame.ui)

set(VIZ_SHADER_SRC ${CMAKE_BINARY_DIR}/visualization_shaders.cpp)

COMPILE_SHADERS(${VIZ_SHADER_SRC}
  src/shaders/color.glsl
  
  src/shaders/empty.vert
  src/shaders/empty.frag
  src/shaders/quad.geom
  src/shaders/passthrough.frag
  
  src/shaders/draw_pose.geom
  src/shaders/fill_tile_points.geom
  src/shaders/fill_tile_points.vert
  src/shaders/draw_points.vert
  src/shaders/draw_polygon.vert
  src/shaders/update_visibility.vert
  src/shaders/update_labels.vert
  src/shaders/gen_heightmap.vert
  src/shaders/gen_heightmap.frag
  src/shaders/draw_frustum.geom
  src/shaders/draw_heightmap.geom
  src/shaders/draw_heightmap.vert
  src/shaders/average_heightmap.frag
  src/shaders/draw_plane.geom
  src/shaders/draw_bbox.geom
  src/shaders/draw_bbox.vert
  src/shaders/draw_bbox_id.geom
  src/shaders/draw_bbox_id.frag
  src/shaders/selected_labels.vert
  src/shaders/selected_labels.geom
)

add_executable(labeler 
  ${UI_HDRS}
  ${VIZ_SHADER_SRC}
  
  src/data/label_utils.cpp
  src/data/kitti_utils.cpp
  src/rv/string_utils.cpp
  src/rv/Stopwatch.cpp
  src/data/Math.cpp
  src/data/misc.cpp
  src/labeler.cpp
  src/widget/LabelButton.h
  src/widget/Mainframe.h
  src/widget/Viewport.h
  src/widget/LabelButton.cpp
  src/widget/Mainframe.cpp
  src/widget/Viewport.cpp
  src/widget/CADCamera.h
  src/widget/CADCamera.cpp
  src/widget/KittiReader.cpp
  src/widget/TileSelectorWidget.cpp
  src/widget/waitingspinnerwidget.cpp
  src/widget/ImageViewer.cpp
)

if(NOT GTEST_FOUND)
 
  IF(IS_DIRECTORY "/usr/src/gtest/")
    MESSAGE(STATUS "Found google test sources in /usr/src/gtest/") 
    
    ADD_SUBDIRECTORY(/usr/src/gtest/ gtest)
    
    # mimick the behaviour of find_package(GTest)
    SET(GTEST_FOUND TRUE)
    SET(GTEST_BOTH_LIBRARIES gtest gtest_main)
    SET(GTEST_LIBRARIES gtest)
    SET(GTEST_MAIN_LIBRARIES gtest_main)
  ELSE()
    find_package(GTest)
  ENDIF()
endif()


if(GTEST_FOUND)
  enable_testing()
  add_subdirectory(src/test)
endif()

target_link_libraries(labeler ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${OPENGL_LIBRARIES} ${GLEW_LIBRARIES} pthread Qt5::Xml Qt5::OpenGL Qt5::Widgets)

