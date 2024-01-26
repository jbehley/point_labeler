cmake_minimum_required(VERSION 3.16...3.25)

project(point_labeler VERSION 1.0 DESCRIPTION "point labeling of KITTI point clouds" LANGUAGES C CXX)

set(OPENGL_VERSION "430" CACHE STRING "" FORCE)
set(ENABLE_NVIDIA_EXT On CACHE BOOL "" FORCE)

include(external/glow.cmake)

set(CMAKE_INCLUDE_CURRENT_DIR ON) # needs to be activated for qt generated files in build directory.
set(CMAKE_AUTOMOC ON)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS_DEBUG "-g3 -O0 -UDEBUG_GL -UDEBUG")
set(CMAKE_CXX_FLAGS_RELEASE "-g0 -O3 -DNDEBUG")

find_package(OpenGL REQUIRED)
find_package(Qt5Xml REQUIRED)
find_package(Qt5Gui REQUIRED) 
find_package(Qt5OpenGL REQUIRED)
find_package(GLEW REQUIRED)
find_package(Boost REQUIRED COMPONENTS filesystem system)

# since now everything resides in "bin", we have to copy some stuff.
set(CMAKE_BUILD_TYPE Release)
SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin)

include_directories(src)

set(CMAKE_INCLUDE_CURRENT_DIR ON) # needs to be activated for qt generated files in build directory.
set(CMAKE_AUTOMOC ON)
set(CMAKE_CXX_FLAGS "-std=c++14 -O3  -Wall ${CMAKE_CXX_FLAGS}")
set(CMAKE_CXX_FLAGS "-UDEBUG_GL -UDEBUG -DNDEBUG -g2 ${CMAKE_CXX_FLAGS}")

# ensure that custom configuration file is not overwritten.
if(NOT EXISTS "${CMAKE_SOURCE_DIR}/bin/settings.cfg")
  message("-- Copyied default settings into bin. See 'settings.cfg' to change labeler settings.")
  configure_file(${CMAKE_SOURCE_DIR}/assets/settings.cfg ${CMAKE_SOURCE_DIR}/bin/settings.cfg COPYONLY)
endif()

# Cofiguration Files
if(CMAKE_BUILD_TYPE STREQUAL "Debug")
  add_custom_target(link_labels ALL COMMAND ${CMAKE_COMMAND} -E create_symlink assets/labels.xml
                                            ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/labels.xml COMMENT "Symlinking labels.xml to ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}")
else()
  configure_file(assets/labels.xml ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/labels.xml COPYONLY)
  message(STATUS "Copied labels.xml to ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}")
endif()

add_subdirectory(src)

enable_testing()
add_subdirectory(src/test)

