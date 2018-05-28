PROJECT(labeler)

cmake_minimum_required(VERSION 2.6)

set(CMAKE_C_FLAGS "-Wall -O3 -g")
set(CMAKE_CXX_FLAGS "-Wall -O3 -g")

find_library(PTHREAD pthread)
find_package(Threads REQUIRED)
find_package(OpenGL REQUIRED)
find_package(GLUT REQUIRED)
find_package(Qt4 REQUIRED) # find and setup Qt4 for this project

include_directories(${CMAKE_CURRENT_BINARY_DIR})
include_directories(${CMAKE_CURRENT_SOURCE_DIR} )


SET(QT_USE_QTOPENGL true)
SET(QT_USE_QTXML true)
include(${QT_USE_FILE})

set(labeler_HDRS
	src/data/CoordinateSystems.h
	src/data/ColorGL.h
	src/data/Cylinder.h
	src/data/draw_utils.h
	src/data/geometry.h
	src/data/label_utils.h
	src/data/Math.h
	src/data/misc.h
	src/data/transform.h
	src/data/ViewFrustum.h
	src/widget/CameraGL.h
)

set(labeler_SRCS
  src/data/ColorGL.cpp
  src/data/draw_utils.cpp
  src/data/label_utils.cpp
  src/data/Math.cpp
  src/data/misc.cpp
  src/data/transform.cpp
  src/data/ViewFrustum.cpp
  src/widget/CameraGL.cpp
  src/labeler.cpp
)

set(labeler_qt4_SRCS
	src/widget/LabelButton.cpp
	src/widget/Mainframe.cpp
	src/widget/Viewport.cpp
)

SET(labeler_qt4_UIS
	src/MainFrame.ui
)

QT4_WRAP_UI(labeler_qt4_UIS_H ${labeler_qt4_UIS})


SET(labeler_qt4_MOC_HDRS
	src/widget/LabelButton.h
	src/widget/Mainframe.h
	src/widget/Viewport.h
)

# After this call, foo_MOC_SRCS = moc_Class1.cxx moc_Class2.cxx moc_Class3.cxx.
QT4_WRAP_CPP(labeler_qt4_MOC_SRCS ${labeler_qt4_MOC_HDRS})

add_executable(labeler ${labeler_SRCS} ${labeler_HDRS} ${labeler_qt4_SRCS} ${labeler_qt4_HDRS} ${labeler_qt4_UIS_H} ${labeler_qt4_MOC_SRCS})

target_link_libraries(labeler GL GLU glut m ${QT_LIBRARIES})

