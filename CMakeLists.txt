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

message("headers = ${QT5_INCLUDE_DIRS}")
include_directories(${catkin_INCLUDE_DIRS} src/ ${QT5_INCLUDE_DIRS} ${GLEW_INCLUDE_DIRS} /usr/include/eigen3)

set(CMAKE_INCLUDE_CURRENT_DIR ON) # needs to be activated for qt generated files in build directory.
set(CMAKE_AUTOMOC ON)
set(CMAKE_CXX_FLAGS "-std=c++11 -O3  -Wall ${CMAKE_CXX_FLAGS}")
set(CMAKE_CXX_FLAGS "-UDEBUG_GL -UDEBUG -DNDEBUG -g2 ${CMAKE_CXX_FLAGS}")

# since now everything resides in "bin", we have to copy some stuff.
  
SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin)


QT5_WRAP_UI(UI_HDRS 
  src/MainFrame.ui)

add_executable(labeler 
  ${UI_HDRS}
  src/data/ColorGL.cpp
  src/data/draw_utils.cpp
  src/data/label_utils.cpp
  src/data/Math.cpp
  src/data/misc.cpp
  src/data/transform.cpp
  src/data/ViewFrustum.cpp
  src/widget/CameraGL.cpp
  src/labeler.cpp
  src/widget/LabelButton.h
  src/widget/Mainframe.h
  src/widget/Viewport.h
  src/widget/LabelButton.cpp
  src/widget/Mainframe.cpp
  src/widget/Viewport.cpp
)

target_link_libraries(labeler ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${OPENGL_LIBRARIES} ${GLEW_LIBRARIES} pthread Qt5::Xml Qt5::OpenGL Qt5::Widgets)

