

add_executable(test_core
  ../data/label_utils.cpp
  ../data/kitti_utils.cpp
  ../rv/string_utils.cpp
  ../rv/Stopwatch.cpp
  ../data/Math.cpp
  ../data/misc.cpp

  triangulateTest.cpp
)


target_link_libraries(test_core PRIVATE ${GTEST_MAIN_LIBRARIES} ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${OPENGL_LIBRARIES} ${GLEW_LIBRARIES} pthread Qt5::Xml Qt5::OpenGL Qt5::Widgets)