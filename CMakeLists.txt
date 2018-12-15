cmake_minimum_required(VERSION 2.8.3)
project(package_identification_using_turtlebot)

add_compile_options(-std=c++11)


## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  gazebo_ros
  move_base_msgs 
  actionlib
  sensor_msgs
  cv_bridge
  image_transport
)
find_package( OpenCV REQUIRED )
find_package(PythonLibs 2.7)


catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS})
include_directories( ${OpenCV_INCLUDE_DIRS} )
include_directories(vendor/googletest)

add_executable(search src/main.cpp src/QReader.cpp src/PathPlanner.cpp)
target_include_directories(search PRIVATE ${PYTHON_INCLUDE_DIRS})
target_link_libraries(search ${catkin_LIBRARIES} ${OpenCV_LIBS} ${PYTHON_LIBRARIES})


## Build tests
if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  find_package(gtest)
  find_package(gmock)

  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -O0 --coverage")

  add_rostest_gtest(
  QbotTest
  test/Qbot.test
  test/main.cpp
  test/QReaderTest.cpp
  test/PathPlannerTest.cpp
  src/PathPlanner.cpp
  src/QReader.cpp
  )
 
  target_link_libraries(QbotTest ${catkin_LIBRARIES} ${OpenCV_LIBS} ${PYTHON_LIBRARIES})
  
  add_dependencies(
  search
  QbotTest
  ${catkin_EXPORTED_TARGETS}
  )
endif()
