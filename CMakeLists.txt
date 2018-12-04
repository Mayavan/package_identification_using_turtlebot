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
)
find_package( OpenCV REQUIRED )
find_package(PythonLibs 2.7)


catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS})
include_directories( ${OpenCV_INCLUDE_DIRS} )

add_executable(checkCode src/main.cpp src/QReader.cpp)
add_executable(sendGoals src/sendGoals.cpp)

target_include_directories(checkCode PRIVATE ${PYTHON_INCLUDE_DIRS})

target_link_libraries(checkCode ${catkin_LIBRARIES} ${OpenCV_LIBS} ${PYTHON_LIBRARIES})

target_link_libraries(sendGoals ${catkin_LIBRARIES})

## Build tests
if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)

  add_rostest_gtest(
  QbotTest
  test/Qbot.test
  test/main.cpp
  test/QReaderTest.cpp
  test/PathPlannerTest.cpp
  )

  target_include_directories(QbotTest PRIVATE ${PYTHON_INCLUDE_DIRS})

  target_link_libraries(QbotTest ${catkin_LIBRARIES} ${OpenCV_LIBS} ${PYTHON_LIBRARIES})
  
endif()
