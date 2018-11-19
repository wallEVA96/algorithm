cmake_minimum_required (VERSION 2.8)

# projectname is the same as the main-executable
project(useEigen)

add_definitions('-g')
add_definitions('-Wall')
add_definitions('-std=c++11')
include_directories("/usr/include/eigen3")

add_executable(${PROJECT_NAME} ${PROJECT_NAME}.cpp)
add_executable(geometry geometry.cpp)
add_executable(c++11 c++11.cpp)
