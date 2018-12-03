cmake_minimum_required(VERSION 2.8.3)
project(Package_Identification_using_Turtlebot)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS roscpp)
find_package( OpenCV REQUIRED )
find_package(PythonLibs 2.7)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES project5
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

include_directories(include ${catkin_INCLUDE_DIRS})
include_directories( ${OpenCV_INCLUDE_DIRS} )

add_executable(checkCode src/main.cpp src/QReader.cpp)

target_include_directories(checkCode PRIVATE ${PYTHON_INCLUDE_DIRS})

target_link_libraries(checkCode ${catkin_LIBRARIES} ${OpenCV_LIBS} ${PYTHON_LIBRARIES})

