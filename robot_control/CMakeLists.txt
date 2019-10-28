cmake_minimum_required(VERSION 3.0)
project(robot_control)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)


find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS}) # Not needed for CMake >= 2.8.11

find_package(gazebo REQUIRED)
include_directories(${GAZEBO_PROTO_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})

find_library (FuzzyLiteLibrary fuzzylite)

#However, the file(GLOB...) allows for wildcard additions:
file(GLOB_RECURSE SOURCES "src/*.cpp")
SET(CMAKE_CXX_FLAGS -pthread)

add_executable(robot_control ${SOURCES})

target_link_libraries(robot_control ${OpenCV_LIBS} ${GAZEBO_PROTO_LIBRARIES} ${GAZEBO_LIBRARIES} ${FuzzyLiteLibrary})
